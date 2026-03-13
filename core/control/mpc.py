import math
from dataclasses import dataclass, field
from typing import Any

import numpy as np
import pyomo.environ as pyo


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


class TruckTrailerMPC:
    def __init__(self, cfg: MPCConfig):
        self.cfg = cfg
        self.model: Any = self._build_model()
        self.solver = pyo.SolverFactory("ipopt")
        self.solver.options["max_iter"] = self.cfg.solver_max_iter
        self.solver.options["tol"] = self.cfg.solver_tol

    def _build_model(self) -> pyo.ConcreteModel:
        c = self.cfg
        m = pyo.ConcreteModel()
        m.T = pyo.RangeSet(0, c.N)
        m.K = pyo.RangeSet(0, c.N - 1)
        m.S = pyo.RangeSet(0, 5)

        m.x = pyo.Var(m.S, m.T, domain=pyo.Reals)
        m.a = pyo.Var(m.K, bounds=(c.a_min, c.a_max))
        m.phi_dot = pyo.Var(m.K, bounds=(c.phi_dot_min, c.phi_dot_max))

        def dyn_rule(model: Any, s: int, k: int):
            if s == 0:
                return model.x[0, k + 1] == model.x[0, k] + c.dt * model.x[4, k] * pyo.cos(model.x[2, k])
            if s == 1:
                return model.x[1, k + 1] == model.x[1, k] + c.dt * model.x[4, k] * pyo.sin(model.x[2, k])
            if s == 2:
                return model.x[2, k + 1] == model.x[2, k] + c.dt * (model.x[4, k] / c.L) * pyo.tan(model.x[5, k])
            if s == 3:
                return model.x[3, k + 1] == model.x[3, k] + c.dt * (model.x[4, k] / c.d) * pyo.sin(model.x[2, k] - model.x[3, k])
            if s == 4:
                return model.x[4, k + 1] == model.x[4, k] + c.dt * model.a[k]
            return model.x[5, k + 1] == model.x[5, k] + c.dt * model.phi_dot[k]

        m.dyn = pyo.Constraint(m.S, m.K, rule=dyn_rule)
        m.vel_bounds = pyo.Constraint(m.T, rule=lambda model, t: pyo.inequality(c.v_min, model.x[4, t], c.v_max))
        m.steer_bounds = pyo.Constraint(m.T, rule=lambda model, t: pyo.inequality(c.phi_min, model.x[5, t], c.phi_max))
        m.jackknife = pyo.Constraint(
            m.T,
            rule=lambda model, t: pyo.inequality(-c.max_jackknife_angle, model.x[2, t] - model.x[3, t], c.max_jackknife_angle),
        )

        def obj_rule(model: Any):
            cost = (
                c.w_pos * ((model.x[0, c.N] - c.q_des[0]) ** 2 + (model.x[1, c.N] - c.q_des[1]) ** 2)
                + c.w_theta_t * (model.x[2, c.N] - c.q_des[2]) ** 2
                + c.w_theta_l * (model.x[3, c.N] - c.q_des[3]) ** 2
                + c.w_v * (model.x[4, c.N] - c.q_des[4]) ** 2
                + c.w_phi * (model.x[5, c.N] - c.q_des[5]) ** 2
            )
            cost += sum(c.w_a * model.a[k] ** 2 + c.w_phi_dot * model.phi_dot[k] ** 2 for k in range(c.N))
            return cost

        m.obj = pyo.Objective(rule=obj_rule, sense=pyo.minimize)
        return m

    def _set_guess(self, u_guess: np.ndarray) -> None:
        m = self.model
        for k in range(u_guess.shape[1]):
            m.a[k].set_value(float(u_guess[0, k]))
            m.phi_dot[k].set_value(float(u_guess[1, k]))

    def _extract_u(self) -> np.ndarray:
        m = self.model
        u = np.zeros((2, self.cfg.N), dtype=float)
        for k in range(self.cfg.N):
            u[0, k] = pyo.value(m.a[k])
            u[1, k] = pyo.value(m.phi_dot[k])
        return u

    def solve(self, q: np.ndarray, u_guess: np.ndarray) -> np.ndarray | None:
        m = self.model
        for s in range(6):
            m.x[s, 0].fix(float(q[s]))

        u_guess[0, :] = np.clip(u_guess[0, :], self.cfg.a_min, self.cfg.a_max)
        u_guess[1, :] = np.clip(u_guess[1, :], self.cfg.phi_dot_min, self.cfg.phi_dot_max)
        self._set_guess(u_guess)

        try:
            result = self.solver.solve(m, tee=False, load_solutions=False)
            if result.solver.status not in (pyo.SolverStatus.ok, pyo.SolverStatus.warning):
                return None
            if result.solver.termination_condition not in (
                pyo.TerminationCondition.optimal,
                pyo.TerminationCondition.locallyOptimal,
            ):
                return None
            m.solutions.load_from(result)
            return self._extract_u()
        except Exception as exc:
            print(f"Solver exception: {exc}")
            return None
