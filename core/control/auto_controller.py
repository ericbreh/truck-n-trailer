"""Automatic MPC control for truck-trailer system."""

import math
from typing import Optional, Tuple

import numpy as np

from truck_n_trailer.control.mpc import MPCConfig, TruckTrailerMPC
from truck_n_trailer.control.body_to_wheels import BodyToWheels
from truck_n_trailer.gui.constants import (
    AUTO_WHEEL_RADIUS_CM,
    AUTO_WHEEL_TRACK_CM,
    AUTO_TRUCK_LENGTH_CM,
    AUTO_TRAILER_LENGTH_CM,
    AUTO_START_YAW_RAD,
    AUTO_HITCH_HARD_LIMIT_DEG,
    AUTO_HITCH_RELEASE_DEG,
    AUTO_HITCH_RECOVERY_RPM,
    AUTO_MIN_EFFECTIVE_RPM,
    DEFAULT_HZ,
)


class AutoController:
    """Encapsulates MPC control loop and state."""

    def __init__(self):
        self.mpc = self._build_mpc()
        self.wheels = BodyToWheels(
            wheel_track_cm=AUTO_WHEEL_TRACK_CM,
            wheel_radius_cm=AUTO_WHEEL_RADIUS_CM,
            rpm_limit=120.0,
        )
        self.running = False
        self.q = self.mpc.cfg.q0.copy().astype(float)
        self.u_guess = np.zeros((2, self.mpc.cfg.N), dtype=float)
        self.solver_fail_count = 0
        self.hitch_recovery_active = False
        self.last_meas_rpms: Optional[Tuple[float, float]] = None
        self.last_meas_hitch_deg: Optional[float] = None
        self.prev_vision_t: Optional[float] = None
        self.prev_pivot_xy: Optional[np.ndarray] = None
        self.prev_truck_heading: Optional[float] = None
        self.vel_ema_cm_s = 0.0
        self.omega_ema_rad_s = 0.0
        self.pred_path_xy_cm: list[np.ndarray] = []

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def _build_mpc(self) -> TruckTrailerMPC:
        q0 = np.array([20.0, 155.0, AUTO_START_YAW_RAD, AUTO_START_YAW_RAD, 0.0, 0.0], dtype=float)
        q_des = np.array([20.0, 20.0, AUTO_START_YAW_RAD, AUTO_START_YAW_RAD, 0.0, 0.0], dtype=float)
        cfg = MPCConfig(
            L=AUTO_TRUCK_LENGTH_CM,
            d=AUTO_TRAILER_LENGTH_CM,
            dt=1.0 / DEFAULT_HZ,
            N=30,
            max_steps=500,
            target_tol=2.0,
            angle_tol=0.15,
            a_min=-10.0,
            a_max=10.0,
            v_min=-20.0,
            v_max=20.0,
            max_jackknife_angle=math.radians(45.0),
            q0=q0,
            q_des=q_des,
            w_theta_t=20.0,
            w_theta_l=40.0,
            w_v=15.0,
            w_omega=15.0,
            w_pos_stage=0.05,
        )
        return TruckTrailerMPC(cfg)

    def reset_state(self) -> None:
        self.q = self.mpc.cfg.q0.copy().astype(float)
        self.u_guess = np.zeros((2, self.mpc.cfg.N), dtype=float)
        self.wheels.reset(v=float(self.q[4]), omega=float(self.q[5]))
        self.solver_fail_count = 0
        self.hitch_recovery_active = False
        self.last_meas_rpms = None
        self.last_meas_hitch_deg = None
        self.prev_vision_t = None
        self.prev_pivot_xy = None
        self.prev_truck_heading = None
        self.vel_ema_cm_s = 0.0
        self.omega_ema_rad_s = 0.0
        self.pred_path_xy_cm = []

    def reached_goal(self) -> bool:
        cfg = self.mpc.cfg
        pos_err = float(np.linalg.norm(self.q[:2] - cfg.q_des[:2]))
        ang_t = abs(self._wrap_angle(float(self.q[2] - cfg.q_des[2])))
        ang_l = abs(self._wrap_angle(float(self.q[3] - cfg.q_des[3])))
        return pos_err <= cfg.target_tol and ang_t <= cfg.angle_tol and ang_l <= cfg.angle_tol

    def simulate_step(self, q: np.ndarray, u: np.ndarray, dt: float, d: float) -> np.ndarray:
        x, y, theta_t, theta_l, v, omega = [float(val) for val in q]
        a, alpha = [float(val) for val in u]
        q_next = np.array(
            [
                x + dt * v * math.cos(theta_t),
                y + dt * v * math.sin(theta_t),
                theta_t + dt * omega,
                theta_l + dt * (v / d) * math.sin(theta_t - theta_l),
                v + dt * a,
                omega + dt * alpha,
            ],
            dtype=float,
        )
        q_next[2] = self._wrap_angle(float(q_next[2]))
        q_next[3] = self._wrap_angle(float(q_next[3]))
        return q_next

    def tick(self, vision_q: Optional[np.ndarray], goal_xy: Optional[np.ndarray], hitch_deg: float) -> Tuple[float, float]:
        if not self.running or vision_q is None or goal_xy is None:
            return 0.0, 0.0

        self.q = vision_q.copy()
        q_des = self.mpc.cfg.q_des.copy()
        q_des[0] = float(goal_xy[0])
        q_des[1] = float(goal_xy[1])
        q_des[2] = float(self.q[2])
        q_des[3] = float(self.q[3])
        q_des[4] = 0.0
        q_des[5] = 0.0
        self.mpc.cfg.q_des = q_des

        if self.reached_goal():
            self.running = False
            self.pred_path_xy_cm = []
            return 0.0, 0.0

        cfg = self.mpc.cfg
        if float(self.q[4]) <= cfg.v_min + 1e-6:
            self.u_guess[0, :] = np.maximum(self.u_guess[0, :], 0.0)
        elif float(self.q[4]) >= cfg.v_max - 1e-6:
            self.u_guess[0, :] = np.minimum(self.u_guess[0, :], 0.0)

        u_opt = self.mpc.solve(self.q, self.u_guess)
        if u_opt is None:
            retry_guess = np.zeros_like(self.u_guess)
            u_opt = self.mpc.solve(self.q, retry_guess)
            if u_opt is None:
                self.solver_fail_count += 1
                fallback_u = np.array([0.0, 0.0], dtype=float)
                self.q = self.simulate_step(self.q, fallback_u, cfg.dt, cfg.d)
                self.u_guess[:, :] = 0.0
                rpm_l, rpm_r = self.wheels.convert(a=0.0, alpha=0.0, dt=cfg.dt)
                self.pred_path_xy_cm = []
                return rpm_l, rpm_r
            self.u_guess = retry_guess

        u0 = u_opt[:, 0]
        self.solver_fail_count = 0
        rpm_l, rpm_r = self.wheels.convert(a=float(u0[0]), alpha=float(u0[1]), dt=cfg.dt)
        pred_steps = max(1, min(u_opt.shape[1], int(round(3.0 / cfg.dt))))
        q_pred = self.q.copy()
        pred_xy_cm = [q_pred[:2].copy()]
        for k in range(pred_steps):
            q_pred = self.simulate_step(q_pred, u_opt[:, k], cfg.dt, cfg.d)
            pred_xy_cm.append(q_pred[:2].copy())
        self.pred_path_xy_cm = pred_xy_cm
        self.u_guess = np.hstack([u_opt[:, 1:], u_opt[:, -1:]])
        return rpm_l, rpm_r

    def start(self, vision_q: Optional[np.ndarray], goal_xy: Optional[np.ndarray]) -> bool:
        if vision_q is None or goal_xy is None:
            return False
        self.reset_state()
        self.q = vision_q.copy()
        self.running = True
        return True

    def stop(self) -> None:
        self.running = False
        self.hitch_recovery_active = False
        self.wheels.reset(v=0.0, omega=0.0)
        self.pred_path_xy_cm = []

    def update_state_from_measurements(self, motor_rpms: Tuple[float, float], hitch_deg_display: float) -> None:
        cfg = self.mpc.cfg
        dt = cfg.dt
        rpm_l, rpm_r = motor_rpms
        v_l = (float(rpm_l) * (2.0 * math.pi * AUTO_WHEEL_RADIUS_CM)) / 60.0
        v_r = (float(rpm_r) * (2.0 * math.pi * AUTO_WHEEL_RADIUS_CM)) / 60.0
        v = 0.5 * (v_l + v_r)
        omega = (v_r - v_l) / AUTO_WHEEL_TRACK_CM
        theta_t = float(self.q[2]) + dt * omega
        theta_t = self._wrap_angle(theta_t)
        x = float(self.q[0]) + dt * v * math.cos(theta_t)
        y = float(self.q[1]) + dt * v * math.sin(theta_t)
        hitch_rad = math.radians(float(hitch_deg_display))
        theta_l = self._wrap_angle(theta_t - hitch_rad)
        self.q = np.array([x, y, theta_t, theta_l, v, omega], dtype=float)

    def car_like_recovery_rpms(self, hitch_display_deg: float) -> Tuple[float, float]:
        base_v = -((AUTO_HITCH_RECOVERY_RPM * (2.0 * math.pi * AUTO_WHEEL_RADIUS_CM)) / 60.0)
        desired_omega = -1.0 if hitch_display_deg > 0.0 else 1.0
        max_omega = (2.0 * abs(base_v) / AUTO_WHEEL_TRACK_CM) * 0.9
        omega = desired_omega * max_omega
        v_l = float(base_v) - (float(omega) * AUTO_WHEEL_TRACK_CM) / 2.0
        v_r = float(base_v) + (float(omega) * AUTO_WHEEL_TRACK_CM) / 2.0
        circumference = 2.0 * math.pi * AUTO_WHEEL_RADIUS_CM
        rpm_l = (v_l / circumference) * 60.0
        rpm_r = (v_r / circumference) * 60.0
        return rpm_l, rpm_r

    def boost_rpms(self, rpm_l: float, rpm_r: float) -> Tuple[float, float]:
        peak = max(abs(rpm_l), abs(rpm_r))
        if peak < 1e-6:
            return 0.0, 0.0
        if peak >= AUTO_MIN_EFFECTIVE_RPM:
            return rpm_l, rpm_r
        scale = AUTO_MIN_EFFECTIVE_RPM / peak
        return rpm_l * scale, rpm_r * scale

    def check_jackknife(self, hitch_display_deg: float, hitch_vision_deg: Optional[float]) -> None:
        hitch_abs = abs(hitch_vision_deg) if hitch_vision_deg is not None else 0.0
        if self.hitch_recovery_active and hitch_abs <= AUTO_HITCH_RELEASE_DEG:
            self.hitch_recovery_active = False
        if not self.hitch_recovery_active and hitch_abs > AUTO_HITCH_HARD_LIMIT_DEG:
            self.hitch_recovery_active = True
