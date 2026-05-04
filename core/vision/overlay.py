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
    cv2.putText(
        frame,
        "PIVOT",
        (px + arm + 6, py - arm - 4),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 0, 0),
        3,
        cv2.LINE_AA,
    )
    cv2.putText(
        frame,
        "PIVOT",
        (px + arm + 6, py - arm - 4),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )


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
