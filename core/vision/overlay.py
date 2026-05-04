"""OpenCV drawing helpers for camera-space overlays."""

import numpy as np

try:
    import cv2  # type: ignore
except ImportError:
    cv2 = None


def draw_vehicle_box(
    frame,
    corners: np.ndarray,
    front_extra_cm: float,
    rear_extra_cm: float,
    side_extra_cm: float,
    color: tuple[int, int, int],
    marker_size_cm: float,
) -> None:
    if cv2 is None:
        return
    center_px = corners.mean(axis=0).astype(float)
    top_mid = ((corners[0] + corners[1]) / 2.0).astype(float)
    fwd = top_mid - center_px
    norm = float(np.linalg.norm(fwd))
    if norm < 1e-6:
        return
    fwd /= norm
    right = np.array([-fwd[1], fwd[0]], dtype=float)

    edge_lengths = [
        np.linalg.norm(corners[1] - corners[0]),
        np.linalg.norm(corners[2] - corners[1]),
        np.linalg.norm(corners[3] - corners[2]),
        np.linalg.norm(corners[0] - corners[3]),
    ]
    marker_side_px = float(np.mean(edge_lengths))
    px_per_cm = marker_side_px / marker_size_cm

    half_marker = marker_size_cm * 0.5
    front = (half_marker + front_extra_cm) * px_per_cm
    rear = (half_marker + rear_extra_cm) * px_per_cm
    half_width = (half_marker + side_extra_cm) * px_per_cm

    p_fl = center_px + fwd * front - right * half_width
    p_fr = center_px + fwd * front + right * half_width
    p_rr = center_px - fwd * rear + right * half_width
    p_rl = center_px - fwd * rear - right * half_width
    box_px = np.array([p_fl, p_fr, p_rr, p_rl], dtype=np.int32)

    overlay = frame.copy()
    cv2.fillPoly(overlay, [box_px], color)
    cv2.addWeighted(overlay, 0.35, frame, 0.65, 0, frame)
    cv2.polylines(frame, [box_px], isClosed=True, color=color, thickness=2)


def draw_truck_pivot_x(frame, corners: np.ndarray, marker_size_cm: float) -> None:
    if cv2 is None:
        return
    center_px = corners.mean(axis=0).astype(float)
    top_mid = ((corners[0] + corners[1]) / 2.0).astype(float)
    fwd = top_mid - center_px
    norm = float(np.linalg.norm(fwd))
    if norm < 1e-6:
        return
    fwd /= norm
    back = -fwd

    edge_lengths = [
        np.linalg.norm(corners[1] - corners[0]),
        np.linalg.norm(corners[2] - corners[1]),
        np.linalg.norm(corners[3] - corners[2]),
        np.linalg.norm(corners[0] - corners[3]),
    ]
    marker_side_px = float(np.mean(edge_lengths))
    px_per_cm = marker_side_px / marker_size_cm

    back_offset_px = (marker_size_cm * 0.5 + 4.0) * px_per_cm
    pivot = center_px + back * back_offset_px
    px, py = int(round(float(pivot[0]))), int(round(float(pivot[1])))
    arm = max(10, int(round(1.2 * px_per_cm)))

    cv2.line(frame, (px - arm, py - arm), (px + arm, py + arm), (0, 0, 0), 8, cv2.LINE_AA)
    cv2.line(frame, (px - arm, py + arm), (px + arm, py - arm), (0, 0, 0), 8, cv2.LINE_AA)
    cv2.line(frame, (px - arm, py - arm), (px + arm, py + arm), (0, 255, 255), 4, cv2.LINE_AA)
    cv2.line(frame, (px - arm, py + arm), (px + arm, py - arm), (0, 255, 255), 4, cv2.LINE_AA)
    cv2.putText(frame, "PIVOT", (px + arm + 6, py - arm - 4),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(frame, "PIVOT", (px + arm + 6, py - arm - 4),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2, cv2.LINE_AA)


def draw_workspace_box(
    frame,
    id_to_corners: dict,
    camera_H,
    camera_H_inv,
    workspace_side_inset_cm: float,
    workspace_vertical_extend_cm: float,
    marker_size_cm: float,
) -> None:
    if cv2 is None:
        return
    required_ids = (10, 11, 12, 13)
    if any(marker_id not in id_to_corners for marker_id in required_ids):
        return

    p_tl = id_to_corners[10][2]
    p_tr = id_to_corners[11][3]
    p_br = id_to_corners[12][0]
    p_bl = id_to_corners[13][1]
    poly = np.array([p_tl, p_tr, p_br, p_bl], dtype=np.float32)

    def _unit(v: np.ndarray) -> np.ndarray:
        norm = float(np.linalg.norm(v))
        if norm < 1e-6:
            return np.zeros(2, dtype=np.float32)
        return (v / norm).astype(np.float32)

    if camera_H is not None and camera_H_inv is not None:
        world_poly = cv2.perspectiveTransform(poly.reshape(1, -1, 2), camera_H)[0]
        w_tl, w_tr, w_br, w_bl = world_poly
        in_tl = w_tl + workspace_side_inset_cm * _unit(w_tr - w_tl) - workspace_vertical_extend_cm * _unit(w_bl - w_tl)
        in_tr = w_tr + workspace_side_inset_cm * _unit(w_tl - w_tr) - workspace_vertical_extend_cm * _unit(w_br - w_tr)
        in_br = w_br + workspace_side_inset_cm * _unit(w_bl - w_br) - workspace_vertical_extend_cm * _unit(w_tr - w_br)
        in_bl = w_bl + workspace_side_inset_cm * _unit(w_br - w_bl) - workspace_vertical_extend_cm * _unit(w_tl - w_bl)
        inset_world_poly = np.array([in_tl, in_tr, in_br, in_bl], dtype=np.float32)
        poly = cv2.perspectiveTransform(inset_world_poly.reshape(1, -1, 2), camera_H_inv)[0]
    else:
        px_lengths: list[float] = []
        for marker_id in required_ids:
            c = id_to_corners[marker_id]
            px_lengths.extend([
                float(np.linalg.norm(c[1] - c[0])),
                float(np.linalg.norm(c[2] - c[1])),
                float(np.linalg.norm(c[3] - c[2])),
                float(np.linalg.norm(c[0] - c[3])),
            ])
        px_per_cm = (sum(px_lengths) / len(px_lengths)) / marker_size_cm if px_lengths else 0.0
        inset_side_px = workspace_side_inset_cm * px_per_cm
        extend_vertical_px = workspace_vertical_extend_cm * px_per_cm

        p_tl_f, p_tr_f, p_br_f, p_bl_f = poly
        in_tl = p_tl_f + inset_side_px * _unit(p_tr_f - p_tl_f) - extend_vertical_px * _unit(p_bl_f - p_tl_f)
        in_tr = p_tr_f + inset_side_px * _unit(p_tl_f - p_tr_f) - extend_vertical_px * _unit(p_br_f - p_tr_f)
        in_br = p_br_f + inset_side_px * _unit(p_bl_f - p_br_f) - extend_vertical_px * _unit(p_tr_f - p_br_f)
        in_bl = p_bl_f + inset_side_px * _unit(p_br_f - p_bl_f) - extend_vertical_px * _unit(p_tl_f - p_bl_f)
        poly = np.array([in_tl, in_tr, in_br, in_bl], dtype=np.float32)

    poly_int = poly.astype(np.int32)
    tmp = frame.copy()
    cv2.fillPoly(tmp, [poly_int], (255, 200, 40))
    cv2.addWeighted(tmp, 0.25, frame, 0.75, 0.0, frame)
    cv2.polylines(frame, [poly_int], isClosed=True, color=(255, 180, 0), thickness=3)


def draw_parking_goal_marker(
    frame,
    u: int,
    v: int,
    *,
    color: tuple[int, int, int] = (0, 255, 120),
) -> None:
    if cv2 is None:
        return
    cv2.drawMarker(
        frame,
        (u, v),
        color,
        markerType=cv2.MARKER_CROSS,
        markerSize=22,
        thickness=2,
        line_type=cv2.LINE_AA,
    )
    cv2.circle(frame, (u, v), 10, color, 2, cv2.LINE_AA)


def _dotted_polyline_px(frame, path_px: list[np.ndarray], color: tuple[int, int, int]) -> None:
    if cv2 is None or len(path_px) < 2:
        return

    def _dotted_segment(p0: np.ndarray, p1: np.ndarray) -> None:
        vec = p1 - p0
        dist = float(np.linalg.norm(vec))
        if dist < 1e-6:
            return
        direction = vec / dist
        on_px, off_px, pos = 8.0, 6.0, 0.0
        while pos < dist:
            seg_start = p0 + direction * pos
            seg_end = p0 + direction * min(dist, pos + on_px)
            cv2.line(
                frame,
                (int(round(float(seg_start[0]))), int(round(float(seg_start[1])))),
                (int(round(float(seg_end[0]))), int(round(float(seg_end[1])))),
                color,
                2,
                cv2.LINE_AA,
            )
            pos += on_px + off_px

    for i in range(len(path_px) - 1):
        _dotted_segment(path_px[i], path_px[i + 1])


def draw_local_path_homography(
    frame,
    pred_xy_cm: list,
    origin_world: np.ndarray,
    ex_world: np.ndarray,
    ey_world: np.ndarray,
    H_inv: np.ndarray,
) -> None:
    """Map local cm path (same frame as vision_q x,y) to pixels via world + inverse homography."""
    if cv2 is None:
        return
    if len(pred_xy_cm) < 2:
        return
    ow = origin_world.astype(np.float32)
    ex = ex_world.astype(np.float32)
    ey = ey_world.astype(np.float32)
    path_px: list[np.ndarray] = []
    for p in pred_xy_cm:
        gx = float(ow[0] + float(p[0]) * float(ex[0]) + float(p[1]) * float(ey[0]))
        gy = float(ow[1] + float(p[0]) * float(ex[1]) + float(p[1]) * float(ey[1]))
        pt = cv2.perspectiveTransform(
            np.array([[[gx, gy]]], dtype=np.float32),
            H_inv,
        )[0][0]
        path_px.append(pt.astype(np.float64))
    _dotted_polyline_px(frame, path_px, (0, 255, 255))


def draw_prediction_path(
    frame,
    pred_xy_cm: list,
    origin_px: np.ndarray | None,
    x_hat_px: np.ndarray | None,
    y_hat_px: np.ndarray | None,
    px_per_cm: float | None,
) -> None:
    if cv2 is None:
        return
    if (
        len(pred_xy_cm) < 2
        or origin_px is None
        or x_hat_px is None
        or y_hat_px is None
        or px_per_cm is None
        or px_per_cm <= 1e-6
    ):
        return

    path_px = [
        origin_px + x_hat_px * (float(p[0]) * px_per_cm) + y_hat_px * (float(p[1]) * px_per_cm)
        for p in pred_xy_cm
    ]
    _dotted_polyline_px(frame, path_px, (0, 255, 255))
