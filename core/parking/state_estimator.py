"""
truck_n_trailer/parking/state_estimator.py
─────────────────────────
Wraps the vision pipeline and produces a full 6-element state vector
    q = [x, y, theta_t, theta_l, v, omega]
that the MPC can consume directly.

The camera only gives pose (x, y, theta_t, theta_l) each frame.
Velocity (v, omega) is estimated by finite-differencing consecutive poses,
smoothed with a simple exponential moving average (EMA) to reduce noise.

Units
-----
  x, y        : centimetres  (matches workspace calibration)
  theta_t/l   : radians
  v           : cm / s
  omega       : rad / s

Usage
-----
    estimator = StateEstimator()
    estimator.open()
    q = estimator.get_state()   # returns np.ndarray shape (6,) or None
    estimator.close()
"""

import math
import time
from dataclasses import dataclass, field
from typing import Optional

import cv2
import numpy as np

from truck_n_trailer.vision.camera import open_configured_camera
from truck_n_trailer.vision.config import (
    ARUCO_DICT,
    ARUCO_PARAMS,
    CAMERA_PARAMS_PATH,
    HOMOGRAPHY_PATH,
)
from truck_n_trailer.vision.detect import load_calibration, marker_pose_world


# Marker IDs for truck and trailer (matches vision/config.py)
_TRUCK_ID = 0
_TRAILER_ID = 1


def _wrap(angle: float) -> float:
    """Wrap angle to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass
class _PoseStamp:
    x: float
    y: float
    theta_t: float
    theta_l: float
    t: float  # monotonic seconds


@dataclass
class StateEstimator:
    """
    Opens the overhead camera, runs ArUco detection on every call to
    get_state(), and returns a smoothed 6-element state vector.

    Parameters
    ----------
    ema_alpha : float
        Smoothing factor for the velocity EMA.  0 < alpha <= 1.
        Higher = less smoothing (tracks faster changes but noisier).
        Lower  = more smoothing (lags behind fast moves).
        Default 0.3 works well at ~10–15 Hz camera rates.
    max_dt : float
        Discard velocity estimates when the frame gap exceeds this
        threshold (seconds).  Guards against stale-frame spikes.
    show_overlay : bool
        If True, opens an OpenCV window showing the detection overlay.
        Set False when running headless.
    """

    ema_alpha: float = 0.3
    max_dt: float = 0.5
    show_overlay: bool = True

    _cap: Optional[object] = field(default=None, init=False, repr=False)
    _K: Optional[np.ndarray] = field(default=None, init=False, repr=False)
    _dist: Optional[np.ndarray] = field(default=None, init=False, repr=False)
    _H: Optional[np.ndarray] = field(default=None, init=False, repr=False)
    _H_inv: Optional[np.ndarray] = field(default=None, init=False, repr=False)
    _detector: Optional[object] = field(default=None, init=False, repr=False)
    _prev: Optional[_PoseStamp] = field(default=None, init=False, repr=False)
    _v_smooth: float = field(default=0.0, init=False, repr=False)
    _omega_smooth: float = field(default=0.0, init=False, repr=False)

    # ------------------------------------------------------------------ #
    # Lifecycle                                                            #
    # ------------------------------------------------------------------ #

    def open(self) -> None:
        """Load calibration and open the camera.  Call once before use."""
        self._K, self._dist, self._H = load_calibration()
        if self._K is None:
            raise RuntimeError(
                "Camera calibration not found. "
                "Run: python -m truck_n_trailer.vision.calibrate_camera"
            )
        if self._H is None:
            raise RuntimeError(
                "Homography calibration not found. "
                "Run: python -m truck_n_trailer.vision.calibrate_homography"
            )
        self._H_inv = np.linalg.inv(self._H)
        self._detector = cv2.aruco.ArucoDetector(ARUCO_DICT, ARUCO_PARAMS)
        self._cap = open_configured_camera()
        if self._cap is None:
            raise RuntimeError("Could not open camera.")

    def close(self) -> None:
        """Release camera and destroy any OpenCV windows."""
        if self._cap is not None:
            self._cap.release()
            self._cap = None
        if self.show_overlay:
            cv2.destroyAllWindows()

    # ------------------------------------------------------------------ #
    # Main API                                                             #
    # ------------------------------------------------------------------ #

    def get_state(self) -> Optional[np.ndarray]:
        """
        Grab one camera frame, detect truck + trailer markers, estimate
        velocity, and return the state vector q = [x, y, θ_t, θ_l, v, ω].

        Returns None if either marker is not visible in the current frame.
        """
        assert self._cap is not None, "Call open() first."

        ret, frame = self._cap.read()
        if not ret:
            return None

        undistorted = cv2.undistort(frame, self._K, self._dist)
        corners_list, ids, _ = self._detector.detectMarkers(undistorted)

        if ids is None:
            return None

        id_to_corners = {
            int(ids[i][0]): corners_list[i][0] for i in range(len(ids))
        }

        if _TRUCK_ID not in id_to_corners or _TRAILER_ID not in id_to_corners:
            return None

        truck_pos, theta_t = marker_pose_world(
            id_to_corners[_TRUCK_ID], self._H
        )
        trailer_pos, theta_l = marker_pose_world(
            id_to_corners[_TRAILER_ID], self._H
        )

        x, y = float(truck_pos[0]), float(truck_pos[1])
        theta_t = _wrap(float(theta_t))
        theta_l = _wrap(float(theta_l))
        now = time.monotonic()

        v, omega = self._estimate_velocity(x, y, theta_t, now)

        if self.show_overlay:
            self._draw(undistorted, id_to_corners, truck_pos, theta_t,
                       trailer_pos, theta_l, v, omega)

        return np.array([x, y, theta_t, theta_l, v, omega], dtype=float)

    # ------------------------------------------------------------------ #
    # Internal helpers                                                     #
    # ------------------------------------------------------------------ #

    def _estimate_velocity(
        self, x: float, y: float, theta_t: float, now: float
    ) -> tuple[float, float]:
        """
        Finite-difference velocity from consecutive poses, then EMA-smooth.
        Returns (v_cm_s, omega_rad_s).
        """
        if self._prev is None:
            self._prev = _PoseStamp(x, y, theta_t, theta_t, now)
            return 0.0, 0.0

        dt = now - self._prev.t
        if dt <= 0 or dt > self.max_dt:
            # Frame gap too large — reset and trust nothing
            self._prev = _PoseStamp(x, y, theta_t, theta_t, now)
            self._v_smooth = 0.0
            self._omega_smooth = 0.0
            return 0.0, 0.0

        dx = x - self._prev.x
        dy = y - self._prev.y
        # Project displacement onto truck heading to get signed speed
        v_raw = (dx * math.cos(self._prev.theta_t)
                 + dy * math.sin(self._prev.theta_t)) / dt
        omega_raw = _wrap(theta_t - self._prev.theta_t) / dt

        # EMA smoothing
        a = self.ema_alpha
        self._v_smooth = a * v_raw + (1 - a) * self._v_smooth
        self._omega_smooth = a * omega_raw + (1 - a) * self._omega_smooth

        self._prev = _PoseStamp(x, y, theta_t, theta_t, now)
        return self._v_smooth, self._omega_smooth

    def _draw(
        self,
        frame: np.ndarray,
        id_to_corners: dict,
        truck_pos: np.ndarray,
        theta_t: float,
        trailer_pos: np.ndarray,
        theta_l: float,
        v: float,
        omega: float,
    ) -> None:
        for marker_id, (pos, heading, label) in {
            _TRUCK_ID:   (truck_pos,   theta_t, "truck"),
            _TRAILER_ID: (trailer_pos, theta_l, "trailer"),
        }.items():
            corners = id_to_corners[marker_id]
            pts = corners.astype(int)
            cv2.polylines(frame, [pts], True, (0, 255, 0), 2)
            center = corners.mean(axis=0).astype(int)
            top_mid = ((corners[0] + corners[1]) / 2).astype(int)
            cv2.arrowedLine(frame, tuple(center), tuple(top_mid),
                            (0, 255, 255), 2, tipLength=0.4)
            cv2.putText(
                frame,
                f"{label} ({pos[0]:.1f},{pos[1]:.1f}) h={math.degrees(heading):.1f}d",
                (pts[0][0], pts[0][1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1,
            )

        cv2.putText(
            frame,
            f"v={v:.1f} cm/s  omega={math.degrees(omega):.1f} deg/s",
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1,
        )
        cv2.imshow("State estimator", frame)
        cv2.waitKey(1)
