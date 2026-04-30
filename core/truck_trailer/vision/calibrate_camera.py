import cv2
import numpy as np
from pathlib import Path

from truck_trailer.vision.camera import open_configured_camera
from truck_trailer.vision.config import (
    CAMERA_PARAMS_PATH,
)

CALIBRATION_IMAGES_DIR = Path(__file__).parent / "calibration" / "images"

CHESSBOARD_SIZE = (9, 6)
CHESSBOARD_SQUARE_SIZE_CM = 2.5
MIN_CALIBRATION_FRAMES = 15

# 3D object points for one chessboard frame (z=0, flat plane)
objp = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)
objp *= CHESSBOARD_SQUARE_SIZE_CM

def main():
    CALIBRATION_IMAGES_DIR.mkdir(parents=True, exist_ok=True)

    cap = open_configured_camera()
    if cap is None:
        return

    obj_points = []  # 3D points in real world space
    img_points = []  # 2D points in image plane
    frame_count = 0
    image_size = None

    print("Controls: [s] capture frame  [q] quit and compute")
    print(f"Need at least {MIN_CALIBRATION_FRAMES} frames. Hold the chessboard at varied angles.")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, None)

        display = frame.copy()
        if found:
            cv2.drawChessboardCorners(display, CHESSBOARD_SIZE, corners, found)
            cv2.putText(display, "Chessboard detected", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        else:
            cv2.putText(display, "No chessboard", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        cv2.putText(display, f"Captured: {frame_count}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.imshow("Camera Calibration", display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("s") and found:
            corners_refined = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            )
            obj_points.append(objp)
            img_points.append(corners_refined)
            frame_count += 1
            image_size = gray.shape[::-1]

            img_path = CALIBRATION_IMAGES_DIR / f"frame_{frame_count:03d}.png"
            cv2.imwrite(str(img_path), frame)
            print(f"  Captured frame {frame_count}")

    cap.release()
    cv2.destroyAllWindows()

    if frame_count < MIN_CALIBRATION_FRAMES:
        print(f"Not enough frames ({frame_count}/{MIN_CALIBRATION_FRAMES}). Aborting.")
        return

    print(f"Computing intrinsics from {frame_count} frames...")
    assert image_size is not None
    ret, K, dist, _, _ = cv2.calibrateCamera(
        obj_points, img_points, image_size,
        np.zeros((3, 3), dtype=np.float64),
        np.zeros((5, 1), dtype=np.float64),
    )
    print(f"  RMS reprojection error: {ret:.4f} px")

    CAMERA_PARAMS_PATH.parent.mkdir(parents=True, exist_ok=True)
    np.savez(str(CAMERA_PARAMS_PATH), K=K, dist=dist)
    print(f"  Saved to {CAMERA_PARAMS_PATH}")

if __name__ == "__main__":
    main()
