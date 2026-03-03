import sys
from pathlib import Path
import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))

from config import (
    CAMERA_INDEX,
    CAMERA_PARAMS_PATH,
    HOMOGRAPHY_PATH,
    ARUCO_DICT,
    ARUCO_PARAMS,
    TRACKING_MARKER_IDS,
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


def marker_pose(corners):
    center = corners.mean(axis=0)
    top_mid = (corners[0] + corners[1]) / 2
    angle = np.degrees(np.arctan2(top_mid[1] - center[1], top_mid[0] - center[0]))
    return center, angle


def pixel_to_world(pt, H):
    p = np.array([[[pt[0], pt[1]]]], dtype=np.float32)
    w = cv2.perspectiveTransform(p, H)
    return w[0][0]


def draw_overlay(frame, corners, name, world_pos, angle):
    pts = corners.astype(int)
    cv2.polylines(frame, [pts], isClosed=True, color=(0, 255, 0), thickness=2)
    center = corners.mean(axis=0).astype(int)
    top_mid = ((corners[0] + corners[1]) / 2).astype(int)
    cv2.arrowedLine(frame, tuple(center), tuple(top_mid), (0, 255, 255), 2, tipLength=0.4)
    label = f"{name} ({world_pos[0]:.1f}, {world_pos[1]:.1f}) cm  {angle:.1f}deg"
    cv2.putText(frame, label, (pts[0][0], pts[0][1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1, cv2.LINE_AA)


def main():
    K, dist, H = load_calibration()
    if K is None:
        return

    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print("Error: could not open camera.")
        return
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    detector = cv2.aruco.ArucoDetector(ARUCO_DICT, ARUCO_PARAMS)

    print("Running detection. Press [q] to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        undistorted = cv2.undistort(frame, K, dist)
        corners_list, ids, _ = detector.detectMarkers(undistorted)

        if ids is not None:
            id_to_corners = {int(ids[i][0]): corners_list[i][0] for i in range(len(ids))}

            for marker_id, name in TRACKING_MARKER_IDS.items():
                if marker_id not in id_to_corners:
                    continue
                corners = id_to_corners[marker_id]
                center, angle = marker_pose(corners)
                world_pos = pixel_to_world(center, H)
                draw_overlay(undistorted, corners, name, world_pos, angle)
                print(f"{name}: pos=({world_pos[0]:.1f}, {world_pos[1]:.1f}) cm  heading={angle:.1f} deg")

        cv2.imshow("Detection", undistorted)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
