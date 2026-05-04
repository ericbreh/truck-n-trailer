import cv2

# ArUco dictionary
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
ARUCO_PARAMS = cv2.aruco.DetectorParameters()

# Marker IDs (DICT_4X4_50)
TRACKING_MARKER_IDS = {
    0: "truck",
    1: "trailer",
}

# Single floor marker: origin + axes for state; desired pivot is marker center (goal_xy = 0,0).
# ID 10 matches the legacy top-left reference marker so existing prints stay usable.
GOAL_MARKER_ID = 10

# How x̂ is built from ŷ in image pixels: OpenCV y grows downward; MPC uses CCW-from-+x on the floor.
# "ccw" matches physical turn sense vs camera for most rigs; set "cw" if left/right is still reversed.
GOAL_AXIS_HANDEDNESS: str = "ccw"
