# CV

This folder contains the computer vision pipeline for tracking a truck and trailer using ArUco markers from an overhead camera.

It outputs each object's:

- position `(x, y)` in world centimeters
- heading angle in the world frame

## To run

1. Generate markers:
2. Calibrate camera intrinsics (one-time per camera):
3. Calibrate workspace homography (redo if camera/workspace moves):
4. Run detection:

```sh
# Run from repo root
python -m cv.generate_markers
python -m cv.calibrate_camera
python -m cv.calibrate_homography
python -m cv.detect
```
