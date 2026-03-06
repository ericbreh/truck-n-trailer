import sys
from pathlib import Path
import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))

from config import (
    CAMERA_PARAMS_PATH,
    HOMOGRAPHY_PATH,
    ARUCO_DICT,
    ARUCO_PARAMS,
    REFERENCE_MARKER_IDS,
    WORKSPACE_WIDTH_CM,
    WORKSPACE_HEIGHT_CM,
)
from camera import open_configured_camera

REFERENCE_WORLD_COORDS_CM = {
    10: (0.0,               0.0),
    11: (WORKSPACE_WIDTH_CM, 0.0),
    12: (WORKSPACE_WIDTH_CM, WORKSPACE_HEIGHT_CM),
    13: (0.0,               WORKSPACE_HEIGHT_CM),
}


def load_camera_params():
    if not CAMERA_PARAMS_PATH.exists():
        print(f"Error: camera params not found at {CAMERA_PARAMS_PATH}")
        return None, None
    data = np.load(str(CAMERA_PARAMS_PATH))
    return data["K"], data["dist"]


def main():
    K, dist = load_camera_params()
    if K is None:
        return

    cap = open_configured_camera()
    if cap is None:
        return

    print("Place the 4 reference markers at the workspace corners.")
    print("Controls: [s] capture and compute  [q] quit")

    detector = cv2.aruco.ArucoDetector(ARUCO_DICT, ARUCO_PARAMS)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        undistorted = cv2.undistort(frame, K, dist)
        corners, ids, _ = detector.detectMarkers(undistorted)

        display = undistorted.copy()
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(display, corners, ids)
            detected = set(ids.flatten().tolist())
            needed = set(REFERENCE_MARKER_IDS.keys())
            found = detected & needed
            cv2.putText(display, f"Reference markers: {len(found)}/4", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0) if len(found) == 4 else (0, 165, 255), 2)
        else:
            cv2.putText(display, "No markers detected", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        cv2.imshow("Homography Calibration", display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("s"):
            if ids is None:
                print("No markers detected.")
                continue

            id_to_corners = {int(ids[i][0]): corners[i][0] for i in range(len(ids))}
            missing = [mid for mid in REFERENCE_MARKER_IDS if mid not in id_to_corners]
            if missing:
                print(f"Missing reference markers: {missing}")
                continue

            # pixel centers of each reference marker
            pixel_pts = np.array([
                id_to_corners[mid].mean(axis=0) for mid in REFERENCE_MARKER_IDS
            ], dtype=np.float32)

            world_pts = np.array(
                [REFERENCE_WORLD_COORDS_CM[mid] for mid in REFERENCE_MARKER_IDS],
                dtype=np.float32
            )

            H, _ = cv2.findHomography(pixel_pts, world_pts)

            HOMOGRAPHY_PATH.parent.mkdir(parents=True, exist_ok=True)
            np.savez(str(HOMOGRAPHY_PATH), H=H)
            print(f"Homography saved to {HOMOGRAPHY_PATH}")
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
