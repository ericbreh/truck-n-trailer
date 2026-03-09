import math
from dataclasses import dataclass, field

import numpy as np


@dataclass
class MPCConfig:
    # Vehicle parameters
    L: float = 1.0
    d: float = 1.0

    # Simulation settings
    dt: float = 0.1
    N: int = 50
    max_steps: int = 500
    target_tol: float = 0.05
    angle_tol: float = 0.1

    # State and control bounds
    a_min: float = -1.5
    a_max: float = 1.5
    phi_dot_min: float = -0.8
    phi_dot_max: float = 0.8
    v_min: float = -2.0
    v_max: float = 2.0
    phi_min: float = -0.6
    phi_max: float = 0.6
    max_jackknife_angle: float = math.pi / 2.0

    # Costs
    w_pos: float = 1.0
    w_theta_t: float = 10.0
    w_theta_l: float = 100.0
    w_v: float = 10.0
    w_phi: float = 10.0
    w_a: float = 0.1
    w_phi_dot: float = 0.1

    # Initial and desired states
    q0: np.ndarray = field(default_factory=lambda: np.array([2, 4, 1.57, 1.57, 0, 0]))
    q_des: np.ndarray = field(default_factory=lambda: np.array([0, 0, 0, 0, 0, 0]))

    # Solver settings
    solver_max_iter: int = 600
    solver_tol: float = 1e-6
