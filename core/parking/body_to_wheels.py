"""
bridge/body_to_wheels.py
────────────────────────
Converts MPC body-frame outputs  [a, α]  →  left/right wheel RPM targets,
following the equations from the project README:

    v_desired(k)  = v(k-1) + a(k) * dt
    ω_desired(k)  = ω(k-1) + α(k) * dt

    v_L = v_desired - (ω_desired * W) / 2
    v_R = v_desired + (ω_desired * W) / 2

    RPM_L = v_L / (2π * r_wheel) * 60
    RPM_R = v_R / (2π * r_wheel) * 60

Units
-----
  a       : cm/s²   (linear acceleration, from MPC)
  alpha   : rad/s²  (angular acceleration, from MPC)
  v       : cm/s    (linear velocity, maintained internally)
  omega   : rad/s   (angular velocity, maintained internally)
  W       : cm      (wheel track width — distance between left and right contact patches)
  r_wheel : cm      (wheel radius)
  RPM     : rev/min (what the firmware PID loop expects)

Tuning note
-----------
W and r_wheel must match the physical model.  Neither is in the existing
codebase, so they are exposed as constructor parameters with placeholder
defaults.  Measure your model truck and set them in runner.py before
running.
"""

import math
from dataclasses import dataclass


@dataclass
class BodyToWheels:
    """
    Stateful converter that integrates MPC accelerations into velocities
    and maps those to per-wheel RPM targets.

    Parameters
    ----------
    wheel_track_cm : float
        W — lateral distance (cm) between the two rear wheel contact patches.
        Measure from tyre centreline to tyre centreline.
    wheel_radius_cm : float
        r — radius of the rear wheels in cm.
    rpm_limit : float
        Hard clamp applied to both output RPMs.  Should match the motor's
        safe operating range.  The firmware also clamps, but it's cleaner
        to clip here so the MPC never commands impossible targets.
    invert_differential : bool
        If True, use v_L = v + ωW/2 and v_R = v − ωW/2 instead of the default
        minus/plus split. Use when the truck’s wheel mixing matches the MPC ω
        sign but the commanded left/right wheel speeds are reversed (common with
        camera frame vs. standard diff-drive convention). Does not change how
        ω is integrated from α — only the wheel mapping.
    """

    wheel_track_cm: float = 12.0   # ← MEASURE YOUR TRUCK and update in runner.py
    wheel_radius_cm: float = 3.0   # ← MEASURE YOUR TRUCK and update in runner.py
    rpm_limit: float = 120.0
    invert_differential: bool = False

    # Internal velocity state — kept in sync with what we've commanded
    _v: float = 0.0
    _omega: float = 0.0

    def reset(self, v: float = 0.0, omega: float = 0.0) -> None:
        """
        Seed the integrator with known velocity values.  Call this after
        the state estimator gives a fresh reading so the integrator doesn't
        drift from reality.
        """
        self._v = v
        self._omega = omega

    def convert(self, a: float, alpha: float, dt: float) -> tuple[float, float]:
        """
        Integrate one MPC step and return (rpm_left, rpm_right).

        Parameters
        ----------
        a     : linear acceleration (cm/s²)
        alpha : angular acceleration (rad/s²)
        dt    : timestep (seconds) — should match MPCConfig.dt
        """
        # Integrate accelerations → desired velocities
        self._v     += a     * dt
        self._omega += alpha * dt

        # Differential drive: split body velocity into per-wheel tangential speeds
        w = self.wheel_track_cm
        o = self._omega
        if self.invert_differential:
            v_l = self._v + (o * w) / 2.0
            v_r = self._v - (o * w) / 2.0
        else:
            v_l = self._v - (o * w) / 2.0
            v_r = self._v + (o * w) / 2.0

        # Convert cm/s → RPM:  RPM = (v / circumference) * 60
        circumference = 2.0 * math.pi * self.wheel_radius_cm
        rpm_l = (v_l / circumference) * 60.0
        rpm_r = (v_r / circumference) * 60.0

        # Hard clamp — never send the firmware an impossible target
        rpm_l = max(-self.rpm_limit, min(self.rpm_limit, rpm_l))
        rpm_r = max(-self.rpm_limit, min(self.rpm_limit, rpm_r))

        return rpm_l, rpm_r

    # ------------------------------------------------------------------ #
    # Convenience read-only properties                                     #
    # ------------------------------------------------------------------ #

    @property
    def v(self) -> float:
        """Current integrated linear velocity (cm/s)."""
        return self._v

    @property
    def omega(self) -> float:
        """Current integrated angular velocity (rad/s)."""
        return self._omega
