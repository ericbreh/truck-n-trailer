import cv2
from pathlib import Path

# Paths
CALIBRATION_DIR = Path(__file__).parent / "calibration"
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

