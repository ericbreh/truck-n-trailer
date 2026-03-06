import cv2
from pathlib import Path

# Paths
CV_DIR = Path(__file__).parent
MARKERS_DIR = CV_DIR / "markers"
CALIBRATION_DIR = CV_DIR / "calibration"
CALIBRATION_IMAGES_DIR = CALIBRATION_DIR / "images"
CAMERA_PARAMS_PATH = CALIBRATION_DIR / "camera_params.npz"
HOMOGRAPHY_PATH = CALIBRATION_DIR / "homography.npz"

# ArUco dictionary
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
ARUCO_PARAMS = cv2.aruco.DetectorParameters()

# Marker IDs
TRACKING_MARKER_IDS = {
    0: "truck",
    1: "trailer",
}
REFERENCE_MARKER_IDS = {
    10: "ref top-left",
    11: "ref top-right",
    12: "ref bottom-right",
    13: "ref bottom-left",
}

# Real-world positions of the reference marker CENTERS in cm.
# Edit these to match your measured workspace dimensions before running
# calibrate_homography.py.
WORKSPACE_WIDTH_CM = 100.0
WORKSPACE_HEIGHT_CM = 100.0

# Camera
CAMERA_INDEX = 4
