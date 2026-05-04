"""Automatic MPC control for truck-trailer system."""

from typing import Optional, Tuple

import numpy as np

from truck_n_trailer import params
from truck_n_trailer.kinematics import rpm_to_cm_s, wrap_angle_rad
from truck_n_trailer.control.body_to_wheels import BodyToWheels
from truck_n_trailer.control.mpc import TruckTrailerMPC
from truck_n_trailer.control.mpc_config import make_mpc_config
from truck_n_trailer.control.simulation import discrete_step


class AutoController:
    """Encapsulates MPC control loop and state."""

    def __init__(self):
        self.mpc = self._build_mpc()
        self.wheels = BodyToWheels(
            wheel_track_cm=params.WHEEL_TRACK_CM,
            wheel_radius_cm=params.WHEEL_RADIUS_CM,
            rpm_limit=120.0,
        )
        self.running = False
        self.q = self.mpc.cfg.q0.copy().astype(float)
        self.u_guess = np.zeros((2, self.mpc.cfg.N), dtype=float)
        self.solver_fail_count = 0
        self.pred_path_xy_cm: list[np.ndarray] = []
        self.preview_pred_path_xy_cm: list[np.ndarray] = []

    def _build_mpc(self) -> TruckTrailerMPC:
        return TruckTrailerMPC(make_mpc_config())

    def reset_state(self) -> None:
        self.q = self.mpc.cfg.q0.copy().astype(float)
        self.u_guess = np.zeros((2, self.mpc.cfg.N), dtype=float)
        self.wheels.reset(v=float(self.q[4]), omega=float(self.q[5]))
        self.solver_fail_count = 0
        self.pred_path_xy_cm = []
        self.preview_pred_path_xy_cm = []

    def reached_goal(self) -> bool:
        cfg = self.mpc.cfg
        pos_err = float(np.linalg.norm(self.q[:2] - cfg.q_des[:2]))
        ang_t = abs(wrap_angle_rad(float(self.q[2] - cfg.q_des[2])))
        ang_l = abs(wrap_angle_rad(float(self.q[3] - cfg.q_des[3])))
        return pos_err <= cfg.target_tol and ang_t <= cfg.angle_tol and ang_l <= cfg.angle_tol

    def preview_parking_path(
        self, vision_q: Optional[np.ndarray], goal_xy: Optional[np.ndarray]
    ) -> None:
        """Compute MPC horizon path for overlay without running auto or mutating q/u_guess."""
        self.preview_pred_path_xy_cm = []
        if vision_q is None or goal_xy is None:
            return
        saved_q_des = self.mpc.cfg.q_des.copy()
        try:
            q = vision_q.copy()
            q_des = saved_q_des.copy()
            q_des[0] = float(goal_xy[0])
            q_des[1] = float(goal_xy[1])
            q_des[2] = float(q[2])
            q_des[3] = float(q[3])
            q_des[4] = 0.0
            q_des[5] = 0.0
            self.mpc.cfg.q_des = q_des

            cfg = self.mpc.cfg
            pos_err = float(np.linalg.norm(q[:2] - q_des[:2]))
            ang_t = abs(wrap_angle_rad(float(q[2] - q_des[2])))
            ang_l = abs(wrap_angle_rad(float(q[3] - q_des[3])))
            if (
                pos_err <= cfg.target_tol
                and ang_t <= cfg.angle_tol
                and ang_l <= cfg.angle_tol
            ):
                return

            u_guess = np.zeros((2, cfg.N), dtype=float)
            if float(q[4]) <= cfg.v_min + 1e-6:
                u_guess[0, :] = np.maximum(u_guess[0, :], 0.0)
            elif float(q[4]) >= cfg.v_max - 1e-6:
                u_guess[0, :] = np.minimum(u_guess[0, :], 0.0)

            u_opt = self.mpc.solve(q, u_guess)
            if u_opt is None:
                retry_guess = np.zeros_like(u_guess)
                u_opt = self.mpc.solve(q, retry_guess)
            if u_opt is None:
                return

            pred_steps = max(1, min(u_opt.shape[1], int(round(3.0 / cfg.dt))))
            q_pred = q.copy()
            pred_xy_cm = [q_pred[:2].copy()]
            for k in range(pred_steps):
                q_pred = discrete_step(q_pred, u_opt[:, k], cfg.dt, cfg.d)
                pred_xy_cm.append(q_pred[:2].copy())
            self.preview_pred_path_xy_cm = pred_xy_cm
        finally:
            self.mpc.cfg.q_des = saved_q_des

    def tick(self, vision_q: Optional[np.ndarray], goal_xy: Optional[np.ndarray], motor_rpms: Optional[Tuple[float, float]] = None) -> Tuple[float, float]:
        if not self.running or vision_q is None or goal_xy is None:
            return 0.0, 0.0

        # Pose from vision (camera)
        x = float(vision_q[0])
        y = float(vision_q[1])
        theta_t = float(vision_q[2])
        theta_l = float(vision_q[3])

        # Velocities from encoders
        v = 0.0
        omega = 0.0
        if motor_rpms is not None:
            rpm_l, rpm_r = motor_rpms
            v_l = rpm_to_cm_s(float(rpm_l), params.WHEEL_RADIUS_CM)
            v_r = rpm_to_cm_s(float(rpm_r), params.WHEEL_RADIUS_CM)
            v = 0.5 * (v_l + v_r)
            omega = (v_r - v_l) / params.WHEEL_TRACK_CM

        self.q = np.array([x, y, theta_t, theta_l, v, omega], dtype=float)
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
                self.q = discrete_step(self.q, fallback_u, cfg.dt, cfg.d)
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
            q_pred = discrete_step(q_pred, u_opt[:, k], cfg.dt, cfg.d)
            pred_xy_cm.append(q_pred[:2].copy())
        self.pred_path_xy_cm = pred_xy_cm
        self.u_guess = np.hstack([u_opt[:, 1:], u_opt[:, -1:]])
        return rpm_l, rpm_r

    def start(self, vision_q: Optional[np.ndarray], goal_xy: Optional[np.ndarray], motor_rpms: Optional[Tuple[float, float]] = None) -> bool:
        if vision_q is None or goal_xy is None:
            return False
        self.reset_state()
        # Pose from vision
        x = float(vision_q[0])
        y = float(vision_q[1])
        theta_t = float(vision_q[2])
        theta_l = float(vision_q[3])
        # Velocities from encoders
        v = 0.0
        omega = 0.0
        if motor_rpms is not None:
            rpm_l, rpm_r = motor_rpms
            v_l = rpm_to_cm_s(float(rpm_l), params.WHEEL_RADIUS_CM)
            v_r = rpm_to_cm_s(float(rpm_r), params.WHEEL_RADIUS_CM)
            v = 0.5 * (v_l + v_r)
            omega = (v_r - v_l) / params.WHEEL_TRACK_CM
        self.q = np.array([x, y, theta_t, theta_l, v, omega], dtype=float)
        self.running = True
        return True

    def stop(self) -> None:
        self.running = False
        self.wheels.reset(v=0.0, omega=0.0)
        self.pred_path_xy_cm = []
        self.preview_pred_path_xy_cm = []
