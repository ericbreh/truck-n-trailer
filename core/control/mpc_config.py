"""Single source of truth for :class:`MPCConfig` (vehicle + offline simulation)."""

import math

import numpy as np

from truck_n_trailer import params

from .mpc import MPCConfig


def make_mpc_config() -> MPCConfig:
    """MPC settings shared by AutoController and :mod:`simulation` (state in cm)."""
    q0 = np.array(
        [20.0, 155.0, params.START_YAW_RAD, params.START_YAW_RAD, 0.0, 0.0],
        dtype=float,
    )
    q_des = np.array(
        [20.0, 20.0, params.START_YAW_RAD, params.START_YAW_RAD, 0.0, 0.0],
        dtype=float,
    )
    return MPCConfig(
        L=params.TRUCK_LENGTH_CM,
        d=params.TRAILER_LENGTH_CM,
        dt=1.0 / params.DEFAULT_HZ,
        N=30,
        max_steps=500,
        target_tol=2.0,
        angle_tol=0.15,
        a_min=-10.0,
        a_max=10.0,
        alpha_min=-0.8,
        alpha_max=0.8,
        v_min=-20.0,
        v_max=20.0,
        omega_min=-0.6,
        omega_max=0.6,
        max_jackknife_angle=math.radians(45.0),
        w_pos=1.0,
        w_theta_t=20.0,
        w_theta_l=40.0,
        w_v=15.0,
        w_omega=15.0,
        w_pos_stage=0.05,
        w_a=0.1,
        w_alpha=0.1,
        q0=q0,
        q_des=q_des,
        solver_max_iter=600,
        solver_tol=1e-6,
    )
