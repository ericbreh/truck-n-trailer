import cv2
import numpy as np

from vision.camera import open_configured_camera
from vision.config import (
    CAMERA_PARAMS_PATH,
    HOMOGRAPHY_PATH,
    ARUCO_DICT,
    ARUCO_PARAMS,
    TRACKING_MARKER_IDS,
    WORKSPACE_WIDTH_CM,
    WORKSPACE_HEIGHT_CM,
)


def load_calibration():
    if not CAMERA_PARAMS_PATH.exists():
        print("Error: camera params not found. Run calibrate_camera.py first.")
        return None, None, None
    if not HOMOGRAPHY_PATH.exists():
        print("Error: homography not found. Run calibrate_homography.py first.")
        return None, None, None
    cam = np.load(str(CAMERA_PARAMS_PATH))
    hom = np.load(str(HOMOGRAPHY_PATH))
    return cam["K"], cam["dist"], hom["H"]


def wrap_angle_rad(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def pixel_to_world(pt, H):
    p = np.array([[[pt[0], pt[1]]]], dtype=np.float32)
    w = cv2.perspectiveTransform(p, H)
    return w[0][0]


def world_to_pixel(pt, H_inv):
    p = np.array([[[pt[0], pt[1]]]], dtype=np.float32)
    px = cv2.perspectiveTransform(p, H_inv)
    return px[0][0]


def marker_pose_world(corners, H):
    center_px = corners.mean(axis=0)
    top_mid_px = (corners[0] + corners[1]) / 2
    center_world = pixel_to_world(center_px, H)
    top_mid_world = pixel_to_world(top_mid_px, H)
    heading_rad = wrap_angle_rad(np.arctan2(
        top_mid_world[1] - center_world[1],
        top_mid_world[0] - center_world[0],
    ))
    return center_world, heading_rad


def draw_world_plane(frame, H_inv):
    world_corners = np.array([
        [0.0, 0.0],
        [WORKSPACE_WIDTH_CM, 0.0],
        [WORKSPACE_WIDTH_CM, WORKSPACE_HEIGHT_CM],
        [0.0, WORKSPACE_HEIGHT_CM],
    ], dtype=np.float32)

    plane_px = np.array([world_to_pixel(pt, H_inv) for pt in world_corners], dtype=np.float32)
    plane_pts = plane_px.astype(int)
    cv2.polylines(frame, [plane_pts], isClosed=True, color=(255, 0, 0), thickness=2)


def draw_overlay(frame, corners, name, world_pos, heading_rad):
    pts = corners.astype(int)
    cv2.polylines(frame, [pts], isClosed=True, color=(0, 255, 0), thickness=2)
    center = corners.mean(axis=0).astype(int)
    top_mid = ((corners[0] + corners[1]) / 2).astype(int)
    cv2.arrowedLine(frame, tuple(center), tuple(top_mid), (0, 255, 255), 2, tipLength=0.4)
    heading_deg = np.degrees(heading_rad)
    label = f"{name} ({world_pos[0]:.1f}, {world_pos[1]:.1f}) cm  {heading_deg:.1f}deg"
    cv2.putText(frame, label, (pts[0][0], pts[0][1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1, cv2.LINE_AA)


def main():
    K, dist, H = load_calibration()
    if K is None or dist is None or H is None:
        return

    H_inv = np.linalg.inv(H)

    cap = open_configured_camera()
    if cap is None:
        return

    detector = cv2.aruco.ArucoDetector(ARUCO_DICT, ARUCO_PARAMS)

    print("Running detection. Press [q] to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        undistorted = cv2.undistort(frame, K, dist)
        draw_world_plane(undistorted, H_inv)
        corners_list, ids, _ = detector.detectMarkers(undistorted)

        if ids is not None:
            id_to_corners = {int(ids[i][0]): corners_list[i][0] for i in range(len(ids))}

            for marker_id, name in TRACKING_MARKER_IDS.items():
                if marker_id not in id_to_corners:
                    continue
                corners = id_to_corners[marker_id]
                world_pos, heading_rad = marker_pose_world(corners, H)
                heading_deg = np.degrees(heading_rad)
                draw_overlay(undistorted, corners, name, world_pos, heading_rad)
                print(
                    f"{name}: pos=({world_pos[0]:.1f}, {world_pos[1]:.1f}) cm  "
                    f"heading={heading_rad:.3f} rad ({heading_deg:.1f} deg)"
                )

        cv2.imshow("Detection", undistorted)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
