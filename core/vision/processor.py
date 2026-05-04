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
    GOAL_AXIS_HANDEDNESS,
    TRACKING_MARKER_IDS,
    GOAL_MARKER_ID,
)
from truck_n_trailer.vision import overlay as _overlay

CAMERA_AUTO_SETTLE_SECONDS = 2.0


def _marker_edge_lengths_px(corners: np.ndarray) -> list[float]:
    c = corners.astype(float)
    return [
        float(np.linalg.norm(c[1] - c[0])),
        float(np.linalg.norm(c[2] - c[1])),
        float(np.linalg.norm(c[3] - c[2])),
        float(np.linalg.norm(c[0] - c[3])),
    ]


class VisionProcessor:
    """Encapsulates all vision processing from app.py."""

    def __init__(self):
        self.cap = None
        self.detector = None
        self.frame_counter = 0
        self.vision_q: Optional[np.ndarray] = None
        self.goal_xy: Optional[np.ndarray] = None
        self.latest_hitch_vision_deg: Optional[float] = None
        self.axis_origin_px = None
        self.axis_x_hat_px = None
        self.axis_y_hat_px = None
        self.axis_px_per_cm: Optional[float] = None
        self.goal_screen_px: Optional[tuple[int, int]] = None
        self._lock_controls_at: Optional[float] = None
        self._camera_controls_locked = False

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
        self._lock_controls_at = time.monotonic() + CAMERA_AUTO_SETTLE_SECONDS
        self._camera_controls_locked = False
        self.frame_counter = 0
        self._init_detector()
        self.latest_hitch_vision_deg = None
        return "Camera: Connected"

    def disconnect(self):
        if self.cap is None:
            return
        self.cap.release()
        self.cap = None
        self.detector = None
        self.frame_counter = 0
        self.vision_q = None
        self.goal_xy = None
        self.latest_hitch_vision_deg = None
        self.axis_origin_px = None
        self.axis_x_hat_px = None
        self.axis_y_hat_px = None
        self.axis_px_per_cm = None
        self.goal_screen_px = None
        self._lock_controls_at = None
        self._camera_controls_locked = False

    def _lock_camera_controls(self) -> None:
        if cv2 is None or self.cap is None:
            return
        cap = self.cap
        try:
            focus = cap.get(cv2.CAP_PROP_FOCUS)
            cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
            if focus > 0:
                cap.set(cv2.CAP_PROP_FOCUS, focus)
        except Exception:
            pass

        try:
            wb = cap.get(cv2.CAP_PROP_WB_TEMPERATURE)
            cap.set(cv2.CAP_PROP_AUTO_WB, 0)
            if wb > 0:
                cap.set(cv2.CAP_PROP_WB_TEMPERATURE, wb)
        except Exception:
            pass

        try:
            exposure = cap.get(cv2.CAP_PROP_EXPOSURE)
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
            cap.set(cv2.CAP_PROP_EXPOSURE, exposure)
        except Exception:
            pass

    def _init_detector(self):
        self.detector = None
        if cv2 is None or ARUCO_DICT is None or ARUCO_PARAMS is None:
            return
        try:
            self.detector = cv2.aruco.ArucoDetector(ARUCO_DICT, ARUCO_PARAMS)
        except Exception:
            pass

    def tick(self, pred_path_xy_cm: list | None = None):
        if cv2 is None or self.cap is None or self.detector is None:
            return None, None, None
        if (
            not self._camera_controls_locked
            and self._lock_controls_at is not None
            and time.monotonic() >= self._lock_controls_at
        ):
            self._lock_camera_controls()
            self._camera_controls_locked = True
        ok, frame = self.cap.read()
        if not ok:
            self.disconnect()
            return None, None, None
        self.frame_counter += 1
        view = frame
        found_count = 0
        hitch_vision_deg = None
        self.vision_q = None
        self.goal_xy = None
        self.axis_origin_px = None
        self.axis_x_hat_px = None
        self.axis_y_hat_px = None
        self.axis_px_per_cm = None
        self.goal_screen_px = None

        if self.detector is not None:
            try:
                corners_list, ids, _ = self.detector.detectMarkers(view)
            except Exception:
                corners_list, ids = None, None
            if ids is not None:
                id_to_corners = {int(ids[i][0]): corners_list[i][0] for i in range(len(ids))}
                goal_c = id_to_corners.get(GOAL_MARKER_ID)
                truck_c = id_to_corners.get(0)
                trailer_c = id_to_corners.get(1)

                if goal_c is not None and truck_c is not None and trailer_c is not None:
                    try:
                        px_lengths: list[float] = []
                        for c in (goal_c, truck_c, trailer_c):
                            px_lengths.extend(_marker_edge_lengths_px(c))
                        px_per_cm = (sum(px_lengths) / len(px_lengths)) / params.MARKER_SIZE_CM
                        if px_per_cm <= 1e-6:
                            raise ValueError("bad scale")

                        origin_px = goal_c.mean(axis=0).astype(float)
                        g_top = (goal_c[0] + goal_c[1]) / 2.0
                        y_vec = g_top.astype(float) - origin_px
                        y_norm = float(np.linalg.norm(y_vec))
                        if y_norm <= 1e-6:
                            raise ValueError("degenerate goal axis")
                        y_hat_px = y_vec / y_norm
                        # x̂ ⊥ ŷ in the image. CW vs CCW picks which side of ŷ is +x; that sets whether θ
                        # increases CCW with MPC when the camera uses y-down pixels (independent of how the
                        # paper marker is rotated on the floor). GOAL_AXIS_HANDEDNESS in config.py.
                        h = GOAL_AXIS_HANDEDNESS.strip().lower()
                        if h == "ccw":
                            x_hat_px = np.array([-y_hat_px[1], y_hat_px[0]], dtype=float)
                        else:
                            x_hat_px = np.array([y_hat_px[1], -y_hat_px[0]], dtype=float)

                        self.axis_origin_px = origin_px
                        self.axis_x_hat_px = x_hat_px
                        self.axis_y_hat_px = y_hat_px
                        self.axis_px_per_cm = float(px_per_cm)

                        gc = goal_c.mean(axis=0)
                        self.goal_screen_px = (int(round(float(gc[0]))), int(round(float(gc[1]))))
                        self.goal_xy = np.array([0.0, 0.0], dtype=float)

                        def _to_local_cm(pt_px: np.ndarray) -> np.ndarray:
                            rel = pt_px.astype(float) - origin_px
                            return np.array(
                                [
                                    float(np.dot(rel, x_hat_px) / px_per_cm),
                                    float(np.dot(rel, y_hat_px) / px_per_cm),
                                ],
                                dtype=float,
                            )

                        truck_center_px = truck_c.mean(axis=0).astype(float)
                        truck_top_mid_px = ((truck_c[0] + truck_c[1]) / 2.0).astype(float)
                        trailer_center_px = trailer_c.mean(axis=0).astype(float)
                        trailer_top_mid_px = ((trailer_c[0] + trailer_c[1]) / 2.0).astype(float)
                        truck_fwd_px = truck_top_mid_px - truck_center_px
                        trailer_fwd_px = trailer_top_mid_px - trailer_center_px
                        tn = float(np.linalg.norm(truck_fwd_px))
                        ln = float(np.linalg.norm(trailer_fwd_px))
                        if tn <= 1e-6 or ln <= 1e-6:
                            raise ValueError("degenerate heading")
                        truck_fwd_px /= tn
                        trailer_fwd_px /= ln
                        theta_t = float(
                            wrap_angle_rad(
                                math.atan2(
                                    float(np.dot(truck_fwd_px, y_hat_px)),
                                    float(np.dot(truck_fwd_px, x_hat_px)),
                                )
                            )
                        )
                        theta_l = float(
                            wrap_angle_rad(
                                math.atan2(
                                    float(np.dot(trailer_fwd_px, y_hat_px)),
                                    float(np.dot(trailer_fwd_px, x_hat_px)),
                                )
                            )
                        )
                        hitch_vision_deg = float(np.degrees(wrap_angle_rad(theta_t - theta_l)))

                        back_offset_px = (params.MARKER_SIZE_CM * 0.5 + 4.0) * px_per_cm
                        pivot_px = truck_center_px - truck_fwd_px * back_offset_px
                        pivot_xy = _to_local_cm(pivot_px)

                        self.vision_q = np.array(
                            [
                                float(pivot_xy[0]),
                                float(pivot_xy[1]),
                                float(theta_t),
                                float(theta_l),
                                0.0,
                                0.0,
                            ],
                            dtype=float,
                        )
                    except Exception:
                        self.vision_q = None
                        self.goal_xy = None
                        self.axis_origin_px = None
                        self.axis_x_hat_px = None
                        self.axis_y_hat_px = None
                        self.axis_px_per_cm = None
                        self.goal_screen_px = None

                all_names = {**TRACKING_MARKER_IDS, GOAL_MARKER_ID: "goal"}
                for marker_id, corners in id_to_corners.items():
                    pts = corners.astype(int)
                    cv2.polylines(view, [pts], isClosed=True, color=(0, 255, 0), thickness=2)
                    center = corners.mean(axis=0).astype(int)
                    label = all_names.get(marker_id, f"id {marker_id}")
                    text = f"{label} ({marker_id})"
                    cv2.putText(
                        view,
                        text,
                        (int(pts[0][0]), int(pts[0][1]) - 8),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.9,
                        (0, 255, 0),
                        2,
                        cv2.LINE_AA,
                    )
                    cv2.circle(view, tuple(center), 4, (0, 255, 255), -1)
                    if label == "truck":
                        if _overlay is not None:
                            _overlay.draw_vehicle_box(
                                view,
                                corners,
                                front_extra_cm=7.0,
                                rear_extra_cm=5.0,
                                side_extra_cm=2.0,
                                color=(255, 120, 0),
                                marker_size_cm=params.MARKER_SIZE_CM,
                            )
                            _overlay.draw_truck_pivot_x(view, corners, params.MARKER_SIZE_CM)
                    elif label == "trailer":
                        if _overlay is not None:
                            _overlay.draw_vehicle_box(
                                view,
                                corners,
                                front_extra_cm=9.0,
                                rear_extra_cm=7.0,
                                side_extra_cm=2.0,
                                color=(180, 0, 255),
                                marker_size_cm=params.MARKER_SIZE_CM,
                            )
                    found_count += 1

        self.latest_hitch_vision_deg = hitch_vision_deg
        if _overlay is not None:
            path = pred_path_xy_cm if pred_path_xy_cm is not None else []
            if len(path) >= 2:
                _overlay.draw_prediction_path(
                    view,
                    path,
                    self.axis_origin_px,
                    self.axis_x_hat_px,
                    self.axis_y_hat_px,
                    self.axis_px_per_cm,
                )
            if self.goal_screen_px is not None:
                _overlay.draw_parking_goal_marker(
                    view, self.goal_screen_px[0], self.goal_screen_px[1]
                )
        return view, found_count, hitch_vision_deg
