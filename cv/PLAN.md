# CV Plan: ArUco-based 2D Pose Detection (Truck & Trailer)

## Goal

Detect the 2D position (x, y in cm) and heading angle of two physical objects —
a truck and a trailer — using a fixed overhead USB webcam and ArUco markers.

---

## Directory Structure

```
cv/
├── PLAN.md                      # this file
├── config.py                    # shared constants (marker IDs, dictionary, paths, etc.)
├── generate_markers.py          # Step 1: generate and save marker PNG images
├── calibrate_camera.py          # Step 2a: compute and save camera intrinsics
├── calibrate_homography.py      # Step 2b: compute and save pixel -> world homography
├── detect.py                    # Step 3: main detection loop
├── markers/                     # output: generated marker PNG files
└── calibration/
    ├── images/                  # chessboard frames captured during intrinsic calibration
    ├── camera_params.npz        # saved intrinsic matrix K and distortion coefficients
    └── homography.npz           # saved 3x3 homography matrix H (pixel -> cm)
```

---

## Dependencies

- `opencv-contrib-python` — required for `cv2.aruco` (ArUco is a contrib module,
  NOT included in plain `opencv-python`)
- `numpy`

Managed via `pyproject.toml` / `uv`.

---

## Step 1 — Generate Markers (`generate_markers.py`)

- Dictionary: `DICT_4X4_50` (small, fast, unambiguous for close-range detection)
- **ID 0** → truck
- **ID 1** → trailer
- **IDs 10–13** → fixed workspace corner reference markers (used for homography calibration)
- Output: PNG files saved to `cv/markers/`
- Recommended physical print size: 5–10 cm per marker side so the camera can
  reliably detect them at your working distance.

---

## Step 2a — Camera Intrinsic Calibration (`calibrate_camera.py`)

**When to run:** Once per camera (or if you swap to a different lens/webcam).
Saved results are reloaded on subsequent runs — no need to redo unless the
camera hardware changes.

**Process:**

1. Print a standard chessboard pattern (9x6 inner corners recommended).
2. Run the script. A live feed opens.
3. Hold the chessboard in front of the camera at varied positions and angles.
4. Press `s` to capture a calibration frame (aim for 15-20 frames minimum).
5. Press `q` when done. The script computes intrinsics and saves to
   `calibration/camera_params.npz`.

**Output:** Intrinsic matrix `K` (3x3) and distortion coefficients `dist`.

---

## Step 2b — Homography Calibration (`calibrate_homography.py`)

**When to run:** Any time the camera is physically moved or the workspace
is rearranged. Quick — single-shot capture.

**Process:**

1. Place the four reference ArUco markers (IDs 10–13) at the known corners of
   your workspace. Measure their real-world positions in cm.
2. Run the script. It detects the four reference markers in one frame and
   computes the homography from pixel coordinates to world coordinates (cm).
3. Saves the 3x3 matrix `H` to `calibration/homography.npz`.

**Output:** Homography matrix `H` mapping pixel (u, v) -> world (x, y) in cm.

---

## Step 3 — Detection Loop (`detect.py`)

**Per frame pipeline:**

1. Capture frame from webcam.
2. Undistort using `K` / `dist` from camera calibration.
3. Detect all ArUco markers via `cv2.aruco.detectMarkers`.
4. For each detected tracking marker (ID 0 = truck, ID 1 = trailer):
   - Compute pixel center: mean of the 4 corner points.
   - Compute heading angle: `atan2` of the vector from marker center to the
     midpoint of the top edge (consistent with marker orientation encoding).
   - Apply homography `H` to map pixel center -> real-world (x, y) in cm.
5. Draw overlays: marker outline, ID label, position, heading arrow.
6. Print/log output each frame:

   ```
   Truck:   pos=(12.3, 45.6) cm,  heading=27.4 deg
   Trailer: pos=(18.1, 42.0) cm,  heading=31.2 deg
   ```

7. Press `q` to quit.
