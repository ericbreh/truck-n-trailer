"""Vision processing for truck-trailer state estimation."""

import math
import time
from typing import Optional

import numpy as np

try:
    import cv2
except ImportError:
    cv2 = None

from truck_n_trailer import params
from truck_n_trailer.kinematics import wrap_angle_rad
from truck_n_trailer.vision.camera import open_configured_camera
from truck_n_trailer.vision.config import (
    ARUCO_DICT,
    ARUCO_PARAMS,
    TRACKING_MARKER_IDS,
    REFERENCE_MARKER_IDS,
)
from truck_n_trailer.vision.detect import load_calibration, marker_pose_world, draw_world_plane
from truck_n_trailer.vision import overlay as _overlay


class VisionProcessor:
    """Encapsulates all vision processing from app.py."""

    def __init__(self):
        self.cap = None
        self.detector = None
        self.K = None
        self.dist = None
        self.H = None
        self.H_inv = None
        self.frame_counter = 0
        self.reference_marker_corners_cache: dict[int, np.ndarray] = {}
        self.vision_q: Optional[np.ndarray] = None
        self.goal_xy: Optional[np.ndarray] = None
        self.prev_vision_t: Optional[float] = None
        self.prev_pivot_xy: Optional[np.ndarray] = None
        self.prev_truck_heading: Optional[float] = None
        self.latest_hitch_vision_deg: Optional[float] = None
        self.vel_ema_cm_s = 0.0
        self.omega_ema_rad_s = 0.0
        self.axis_origin_px = None
        self.axis_x_hat_px = None
        self.axis_y_hat_px = None
        self.axis_px_per_cm: Optional[float] = None

    def connect(self, source) -> str:
        if cv2 is None:
            return "Camera: OpenCV not installed"
        if self.cap is not None:
            return "Camera: Already connected"
        if source == "configured" and open_configured_camera is not None:
            cap = open_configured_camera()
        elif source == "configured":
            return "Camera: vision module unavailable"
        else:
            cap = cv2.VideoCapture(int(source))
            if cap is not None and not cap.isOpened():
                cap.release()
                cap = None
        if cap is None:
            return "Camera: Failed to connect"
        self.cap = cap
        self._apply_focus_lock(cap)
        self.frame_counter = 0
        self.reference_marker_corners_cache.clear()
        self._init_detector()
        self.latest_hitch_vision_deg = None
        return "Camera: Connected"

    def disconnect(self):
        if self.cap is None:
            return
        self.cap.release()
        self.cap = None
        self.detector = None
        self.K = None
        self.dist = None
        self.H = None
        self.H_inv = None
        self.frame_counter = 0
        self.reference_marker_corners_cache.clear()
        self.vision_q = None
        self.goal_xy = None
        self.latest_hitch_vision_deg = None
        self.prev_vision_t = None
        self.prev_pivot_xy = None
        self.prev_truck_heading = None
        self.vel_ema_cm_s = 0.0
        self.omega_ema_rad_s = 0.0
        self.axis_origin_px = None
        self.axis_x_hat_px = None
        self.axis_y_hat_px = None
        self.axis_px_per_cm = None

    def _apply_focus_lock(self, cap):
        try:
            cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        except Exception:
            pass
        try:
            cap.set(cv2.CAP_PROP_FOCUS, 20)
        except Exception:
            pass

    def _init_detector(self):
        self.detector = None
        self.K = None
        self.dist = None
        self.H = None
        self.H_inv = None
        if cv2 is None or ARUCO_DICT is None or ARUCO_PARAMS is None:
            return
        try:
            self.detector = cv2.aruco.ArucoDetector(ARUCO_DICT, ARUCO_PARAMS)
        except Exception:
            return
        if load_calibration is None:
            return
        try:
            K, dist, H = load_calibration()
            if K is not None and dist is not None and H is not None:
                self.K = K
                self.dist = dist
                self.H = H
                self.H_inv = np.linalg.inv(H)
        except Exception:
            pass

    def tick(self, pred_path_xy_cm: list | None = None):
        if cv2 is None or self.cap is None or self.detector is None:
            return None, None, None
        ok, frame = self.cap.read()
        if not ok:
            self.disconnect()
            return None, None, None
        self.frame_counter += 1
        if self.frame_counter % params.CAMERA_FOCUS_REFRESH_INTERVAL_FRAMES == 0:
            self._apply_focus_lock(self.cap)
        view = frame
        if self.K is not None and self.dist is not None:
            try:
                view = cv2.undistort(view, self.K, self.dist)
            except Exception:
                pass
        if self.H_inv is not None and draw_world_plane is not None:
            try:
                draw_world_plane(view, self.H_inv)
            except Exception:
                pass
        found_count = 0
        hitch_vision_deg = None
        if self.detector is not None:
            try:
                corners_list, ids, _ = self.detector.detectMarkers(view)
            except Exception:
                corners_list, ids = None, None
            if ids is not None:
                id_to_corners = {int(ids[i][0]): corners_list[i][0] for i in range(len(ids))}
                required_ref_ids = (10, 11, 12, 13)
                for marker_id in required_ref_ids:
                    corners = id_to_corners.get(marker_id)
                    if corners is not None:
                        self.reference_marker_corners_cache[marker_id] = corners.copy()
                box_id_to_corners = dict(id_to_corners)
                for marker_id in required_ref_ids:
                    if marker_id not in box_id_to_corners:
                        cached = self.reference_marker_corners_cache.get(marker_id)
                        if cached is not None:
                            box_id_to_corners[marker_id] = cached
                ref_px_lengths = []
                for marker_id in required_ref_ids:
                    c = box_id_to_corners.get(marker_id)
                    if c is None:
                        continue
                    ref_px_lengths.extend([
                        float(np.linalg.norm(c[1] - c[0])),
                        float(np.linalg.norm(c[2] - c[1])),
                        float(np.linalg.norm(c[3] - c[2])),
                        float(np.linalg.norm(c[0] - c[3])),
                    ])
                if ref_px_lengths:
                    self.axis_px_per_cm = (sum(ref_px_lengths) / len(ref_px_lengths)) / params.MARKER_SIZE_CM
                else:
                    self.axis_px_per_cm = None
                truck_corners = id_to_corners.get(0)
                trailer_corners = id_to_corners.get(1)
                if truck_corners is not None and trailer_corners is not None:
                    try:
                        if self.H is not None and marker_pose_world is not None:
                            truck_pos_world, truck_heading_world = marker_pose_world(truck_corners, self.H)
                            trailer_pos_world, trailer_heading_world = marker_pose_world(trailer_corners, self.H)
                            hitch_vision_deg = float(np.degrees(wrap_angle_rad(float(truck_heading_world - trailer_heading_world))))
                            axis_tr = box_id_to_corners.get(11)
                            axis_br = box_id_to_corners.get(12)
                            if axis_tr is not None and axis_br is not None:
                                tr_center_px = axis_tr.mean(axis=0).astype(np.float32)
                                br_center_px = axis_br.mean(axis=0).astype(np.float32)
                                axis_pts_world = cv2.perspectiveTransform(
                                    np.array([[br_center_px, tr_center_px]], dtype=np.float32),
                                    self.H,
                                )[0]
                                origin = axis_pts_world[0].astype(float)
                                y_vec = axis_pts_world[1].astype(float) - origin
                                y_norm = float(np.linalg.norm(y_vec))
                                if y_norm > 1e-6:
                                    y_hat = y_vec / y_norm
                                    x_hat = np.array([y_hat[1], -y_hat[0]], dtype=float)
                                    self.axis_origin_px = br_center_px.astype(float)
                                    self.axis_y_hat_px = y_hat.astype(float)
                                    self.axis_x_hat_px = x_hat.astype(float)
                                    self.axis_px_per_cm = None
                                    ref_ids = (10, 11, 12, 13)
                                    if all(mid in box_id_to_corners for mid in ref_ids):
                                        r_tl = box_id_to_corners[10][2].astype(np.float32)
                                        r_tr = box_id_to_corners[11][3].astype(np.float32)
                                        r_br = box_id_to_corners[12][0].astype(np.float32)
                                        r_bl = box_id_to_corners[13][1].astype(np.float32)
                                        ref_poly_px = np.array([[r_tl, r_tr, r_br, r_bl]], dtype=np.float32)
                                        ref_poly_world = cv2.perspectiveTransform(ref_poly_px, self.H)[0]

                                        def _unit(v):
                                            n = float(np.linalg.norm(v))
                                            if n < 1e-6:
                                                return np.zeros(2, dtype=np.float32)
                                            return (v / n).astype(np.float32)

                                        inset_side = params.WORKSPACE_BOX_SIDE_INSET_CM
                                        extend_vertical = params.WORKSPACE_BOX_VERTICAL_EXTEND_CM
                                        w_tl, w_tr, w_br, w_bl = ref_poly_world
                                        in_tl = w_tl + inset_side * _unit(w_tr - w_tl) - extend_vertical * _unit(w_bl - w_tl)
                                        in_tr = w_tr + inset_side * _unit(w_tl - w_tr) - extend_vertical * _unit(w_br - w_tr)
                                        in_br = w_br + inset_side * _unit(w_bl - w_br) - extend_vertical * _unit(w_tr - w_br)
                                        in_bl = w_bl + inset_side * _unit(w_br - w_bl) - extend_vertical * _unit(w_tl - w_bl)
                                        box_center_world = np.array([in_tl, in_tr, in_br, in_bl], dtype=np.float32).mean(axis=0)
                                        rel_goal = box_center_world - origin
                                        self.goal_xy = np.array([
                                            float(np.dot(rel_goal, x_hat)),
                                            float(np.dot(rel_goal, y_hat)),
                                        ], dtype=float)
                                    truck_center_world = np.array(truck_pos_world, dtype=float)
                                    trailer_center_world = np.array(trailer_pos_world, dtype=float)
                                    truck_center_px = truck_corners.mean(axis=0).astype(np.float32)
                                    truck_top_mid_px = ((truck_corners[0] + truck_corners[1]) / 2.0).astype(np.float32)
                                    truck_pts_world = cv2.perspectiveTransform(
                                        np.array([[truck_center_px, truck_top_mid_px]], dtype=np.float32),
                                        self.H,
                                    )[0]
                                    truck_fwd_world = truck_pts_world[1].astype(float) - truck_pts_world[0].astype(float)
                                    truck_fwd_norm = float(np.linalg.norm(truck_fwd_world))
                                    if truck_fwd_norm > 1e-6:
                                        truck_fwd_world /= truck_fwd_norm
                                        back_offset_cm = params.MARKER_SIZE_CM * 0.5 + 4.0
                                        pivot_world = truck_center_world - truck_fwd_world * back_offset_cm
                                        rel_pivot = pivot_world - origin
                                        pivot_xy = np.array([
                                            float(np.dot(rel_pivot, x_hat)),
                                            float(np.dot(rel_pivot, y_hat)),
                                        ], dtype=float)
                                        truck_fwd_local = np.array([
                                            float(np.dot(truck_fwd_world, x_hat)),
                                            float(np.dot(truck_fwd_world, y_hat)),
                                        ], dtype=float)
                                        theta_t = float(wrap_angle_rad(math.atan2(truck_fwd_local[1], truck_fwd_local[0])))
                                        trailer_top_mid_px = ((trailer_corners[0] + trailer_corners[1]) / 2.0).astype(np.float32)
                                        trailer_pts_world = cv2.perspectiveTransform(
                                            np.array([[trailer_center_world.astype(np.float32), trailer_top_mid_px]], dtype=np.float32),
                                            self.H,
                                        )[0]
                                        trailer_fwd_world = trailer_pts_world[1].astype(float) - trailer_pts_world[0].astype(float)
                                        trailer_norm = float(np.linalg.norm(trailer_fwd_world))
                                        if trailer_norm > 1e-6:
                                            trailer_fwd_world /= trailer_norm
                                            trailer_fwd_local = np.array([
                                                float(np.dot(trailer_fwd_world, x_hat)),
                                                float(np.dot(trailer_fwd_world, y_hat)),
                                            ], dtype=float)
                                            theta_l = float(wrap_angle_rad(math.atan2(trailer_fwd_local[1], trailer_fwd_local[0])))
                                            now = time.monotonic()
                                            v_cm_s = 0.0
                                            omega_rad_s = 0.0
                                            if self.prev_vision_t is not None and self.prev_pivot_xy is not None and self.prev_truck_heading is not None:
                                                dt = now - self.prev_vision_t
                                                if 1e-3 < dt < 0.5:
                                                    dxy = pivot_xy - self.prev_pivot_xy
                                                    prev_theta = float(self.prev_truck_heading)
                                                    v_raw = float((dxy[0] * math.cos(prev_theta) + dxy[1] * math.sin(prev_theta)) / dt)
                                                    omega_raw = float(wrap_angle_rad(theta_t - prev_theta) / dt)
                                                    alpha = 0.35
                                                    self.vel_ema_cm_s = alpha * v_raw + (1.0 - alpha) * self.vel_ema_cm_s
                                                    self.omega_ema_rad_s = alpha * omega_raw + (1.0 - alpha) * self.omega_ema_rad_s
                                                    v_cm_s = self.vel_ema_cm_s
                                                    omega_rad_s = self.omega_ema_rad_s
                                            self.prev_vision_t = now
                                            self.prev_pivot_xy = pivot_xy.copy()
                                            self.prev_truck_heading = theta_t
                                            self.vision_q = np.array([
                                                float(pivot_xy[0]),
                                                float(pivot_xy[1]),
                                                float(theta_t),
                                                float(theta_l),
                                                float(v_cm_s),
                                                float(omega_rad_s),
                                            ], dtype=float)
                                        else:
                                            self.vision_q = None
                                    else:
                                        self.vision_q = None
                                else:
                                    self.vision_q = None
                            else:
                                self.vision_q = None
                        else:
                            axis_tr = box_id_to_corners.get(11)
                            axis_br = box_id_to_corners.get(12)
                            if axis_tr is None or axis_br is None:
                                self.vision_q = None
                            else:
                                px_lengths = []
                                for marker_id in required_ref_ids:
                                    c = box_id_to_corners.get(marker_id)
                                    if c is None:
                                        continue
                                    px_lengths.extend([
                                        float(np.linalg.norm(c[1] - c[0])),
                                        float(np.linalg.norm(c[2] - c[1])),
                                        float(np.linalg.norm(c[3] - c[2])),
                                        float(np.linalg.norm(c[0] - c[3])),
                                    ])
                                if not px_lengths:
                                    self.vision_q = None
                                else:
                                    px_per_cm = (sum(px_lengths) / len(px_lengths)) / params.MARKER_SIZE_CM
                                    if px_per_cm <= 1e-6:
                                        self.vision_q = None
                                    else:
                                        tr_center_px = axis_tr.mean(axis=0).astype(float)
                                        br_center_px = axis_br.mean(axis=0).astype(float)
                                        y_vec_px = tr_center_px - br_center_px
                                        y_norm = float(np.linalg.norm(y_vec_px))
                                        if y_norm <= 1e-6:
                                            self.vision_q = None
                                        else:
                                            y_hat_px = y_vec_px / y_norm
                                            x_hat_px = np.array([y_hat_px[1], -y_hat_px[0]], dtype=float)
                                            origin_px = br_center_px
                                            self.axis_origin_px = origin_px.astype(float)
                                            self.axis_x_hat_px = x_hat_px.astype(float)
                                            self.axis_y_hat_px = y_hat_px.astype(float)
                                            self.axis_px_per_cm = float(px_per_cm)

                                            def _unit(v):
                                                n = float(np.linalg.norm(v))
                                                if n < 1e-6:
                                                    return np.zeros(2, dtype=float)
                                                return v / n

                                            def _to_local_cm(pt_px):
                                                rel = pt_px.astype(float) - origin_px
                                                return np.array([
                                                    float(np.dot(rel, x_hat_px) / px_per_cm),
                                                    float(np.dot(rel, y_hat_px) / px_per_cm),
                                                ], dtype=float)

                                            if all(mid in box_id_to_corners for mid in required_ref_ids):
                                                p_tl = box_id_to_corners[10][2].astype(float)
                                                p_tr = box_id_to_corners[11][3].astype(float)
                                                p_br = box_id_to_corners[12][0].astype(float)
                                                p_bl = box_id_to_corners[13][1].astype(float)
                                                inset_side_px = params.WORKSPACE_BOX_SIDE_INSET_CM * px_per_cm
                                                extend_vertical_px = params.WORKSPACE_BOX_VERTICAL_EXTEND_CM * px_per_cm
                                                in_tl = p_tl + inset_side_px * _unit(p_tr - p_tl) - extend_vertical_px * _unit(p_bl - p_tl)
                                                in_tr = p_tr + inset_side_px * _unit(p_tl - p_tr) - extend_vertical_px * _unit(p_br - p_tr)
                                                in_br = p_br + inset_side_px * _unit(p_bl - p_br) - extend_vertical_px * _unit(p_tr - p_br)
                                                in_bl = p_bl + inset_side_px * _unit(p_br - p_bl) - extend_vertical_px * _unit(p_tl - p_bl)
                                                box_center_px = np.array([in_tl, in_tr, in_br, in_bl], dtype=float).mean(axis=0)
                                                self.goal_xy = _to_local_cm(box_center_px)
                                            truck_center_px = truck_corners.mean(axis=0).astype(float)
                                            truck_top_mid_px = ((truck_corners[0] + truck_corners[1]) / 2.0).astype(float)
                                            trailer_center_px = trailer_corners.mean(axis=0).astype(float)
                                            trailer_top_mid_px = ((trailer_corners[0] + trailer_corners[1]) / 2.0).astype(float)
                                            truck_fwd_px = truck_top_mid_px - truck_center_px
                                            trailer_fwd_px = trailer_top_mid_px - trailer_center_px
                                            tn = float(np.linalg.norm(truck_fwd_px))
                                            ln = float(np.linalg.norm(trailer_fwd_px))
                                            if tn <= 1e-6 or ln <= 1e-6:
                                                self.vision_q = None
                                            else:
                                                truck_fwd_px /= tn
                                                trailer_fwd_px /= ln
                                                theta_t = float(wrap_angle_rad(math.atan2(float(np.dot(truck_fwd_px, y_hat_px)), float(np.dot(truck_fwd_px, x_hat_px)))))
                                                theta_l = float(wrap_angle_rad(math.atan2(float(np.dot(trailer_fwd_px, y_hat_px)), float(np.dot(trailer_fwd_px, x_hat_px)))))
                                                hitch_vision_deg = float(np.degrees(wrap_angle_rad(theta_t - theta_l)))
                                                back_offset_px = (params.MARKER_SIZE_CM * 0.5 + 4.0) * px_per_cm
                                                pivot_px = truck_center_px - truck_fwd_px * back_offset_px
                                                pivot_xy = _to_local_cm(pivot_px)
                                                now = time.monotonic()
                                                v_cm_s = 0.0
                                                omega_rad_s = 0.0
                                                if self.prev_vision_t is not None and self.prev_pivot_xy is not None and self.prev_truck_heading is not None:
                                                    dt = now - self.prev_vision_t
                                                    if 1e-3 < dt < 0.5:
                                                        dxy = pivot_xy - self.prev_pivot_xy
                                                        prev_theta = float(self.prev_truck_heading)
                                                        v_raw = float((dxy[0] * math.cos(prev_theta) + dxy[1] * math.sin(prev_theta)) / dt)
                                                        omega_raw = float(wrap_angle_rad(theta_t - prev_theta) / dt)
                                                        alpha = 0.35
                                                        self.vel_ema_cm_s = alpha * v_raw + (1.0 - alpha) * self.vel_ema_cm_s
                                                        self.omega_ema_rad_s = alpha * omega_raw + (1.0 - alpha) * self.omega_ema_rad_s
                                                        v_cm_s = self.vel_ema_cm_s
                                                        omega_rad_s = self.omega_ema_rad_s
                                                self.prev_vision_t = now
                                                self.prev_pivot_xy = pivot_xy.copy()
                                                self.prev_truck_heading = theta_t
                                                self.vision_q = np.array([
                                                    float(pivot_xy[0]),
                                                    float(pivot_xy[1]),
                                                    float(theta_t),
                                                    float(theta_l),
                                                    float(v_cm_s),
                                                    float(omega_rad_s),
                                                ], dtype=float)
                    except Exception:
                        hitch_vision_deg = None
                        self.vision_q = None
                else:
                    self.vision_q = None
                all_names = {}
                all_names.update(REFERENCE_MARKER_IDS)
                all_names.update(TRACKING_MARKER_IDS)
                for marker_id, corners in id_to_corners.items():
                    pts = corners.astype(int)
                    cv2.polylines(view, [pts], isClosed=True, color=(0, 255, 0), thickness=2)
                    center = corners.mean(axis=0).astype(int)
                    label = all_names.get(marker_id, f"id {marker_id}")
                    text = f"{label} ({marker_id})"
                    cv2.putText(view, text, (int(pts[0][0]), int(pts[0][1]) - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2, cv2.LINE_AA)
                    cv2.circle(view, tuple(center), 4, (0, 255, 255), -1)
                    if label == "truck":
                        if _overlay is not None:
                            _overlay.draw_vehicle_box(view, corners, 0.0, front_extra_cm=7.0, rear_extra_cm=5.0, side_extra_cm=2.0, color=(255, 120, 0))
                            _overlay.draw_truck_pivot_x(view, corners, params.MARKER_SIZE_CM)
                    elif label == "trailer":
                        if _overlay is not None:
                            _overlay.draw_vehicle_box(view, corners, 0.0, front_extra_cm=9.0, rear_extra_cm=7.0, side_extra_cm=2.0, color=(180, 0, 255))
                    if self.H is not None and marker_pose_world is not None:
                        try:
                            world_pos, heading = marker_pose_world(corners, self.H)
                            deg = float(np.degrees(heading))
                            pose_text = f"({world_pos[0]:.1f},{world_pos[1]:.1f}) {deg:.1f}deg"
                            cv2.putText(view, pose_text, (int(pts[0][0]), int(pts[0][1]) + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2, cv2.LINE_AA)
                        except Exception:
                            pass
                    found_count += 1
            else:
                self.vision_q = None
                self.axis_origin_px = None
                self.axis_x_hat_px = None
                self.axis_y_hat_px = None
                self.axis_px_per_cm = None
        self.latest_hitch_vision_deg = hitch_vision_deg
        if _overlay is not None:
            path = pred_path_xy_cm if pred_path_xy_cm is not None else []
            _overlay.draw_prediction_path(
                view, path, self.axis_origin_px, self.axis_x_hat_px, self.axis_y_hat_px, self.axis_px_per_cm
            )
        return view, found_count, hitch_vision_deg
