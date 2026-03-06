# CV

This folder contains the computer vision pipeline for tracking a truck and trailer using ArUco markers from an overhead camera.

It outputs each object's:

- position `(x, y)` in world centimeters
- heading angle in the world frame

## To run

1. Generate markers:
   - `python cv/generate_markers.py`
2. Calibrate camera intrinsics (one-time per camera):
   - `python cv/calibrate_camera.py`
3. Calibrate workspace homography (redo if camera/workspace moves):
   - `python cv/calibrate_homography.py`
4. Run detection:
   - `python cv/detect.py`
