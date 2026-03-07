#!/usr/bin/env python3

import math
from dataclasses import dataclass, field
from typing import Any

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
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
    
    # State and Control bounds
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
    
    # Initial and Desired states
    q0: np.ndarray = field(default_factory=lambda: np.array([2, 4, 1.57, 1.57, 0, 0]))
    q_des: np.ndarray = field(default_factory=lambda: np.array([0, 0, 0, 0, 0, 0]))
    
    # Solver settings
    solver_max_iter: int = 600
    solver_tol: float = 1e-6


def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def dynamics(q: np.ndarray, u: np.ndarray, L: float, d: float) -> np.ndarray:
    x, y, theta_t, theta_l, v, phi = q
    a, phi_dot = u
    return np.array(
        [
            v * math.cos(theta_t),
            v * math.sin(theta_t),
            (v / L) * math.tan(phi),
            (v / d) * math.sin(theta_t - theta_l),
            a,
            phi_dot,
        ],
        dtype=float,
    )


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

        # Dynamic Constraints
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

        # Other Constraints
        m.vel_bounds = pyo.Constraint(m.T, rule=lambda model, t: pyo.inequality(c.v_min, model.x[4, t], c.v_max))
        m.steer_bounds = pyo.Constraint(m.T, rule=lambda model, t: pyo.inequality(c.phi_min, model.x[5, t], c.phi_max))
        m.jackknife = pyo.Constraint(
            m.T,
            rule=lambda model, t: pyo.inequality(-c.max_jackknife_angle, model.x[2, t] - model.x[3, t], c.max_jackknife_angle),
        )

        # Objective
        def obj_rule(model: Any):
            cost = (
                c.w_pos * ((model.x[0, c.N] - c.q_des[0]) ** 2 + (model.x[1, c.N] - c.q_des[1]) ** 2)
                + c.w_theta_t * (model.x[2, c.N] - c.q_des[2]) ** 2
                + c.w_theta_l * (model.x[3, c.N] - c.q_des[3]) ** 2
                + c.w_v * (model.x[4, c.N] - c.q_des[4]) ** 2
                + c.w_phi * (model.x[5, c.N] - c.q_des[5]) ** 2
            )
            # Control effort
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
            if result.solver.termination_condition not in (pyo.TerminationCondition.optimal, pyo.TerminationCondition.locallyOptimal):
                return None
            m.solutions.load_from(result)
            return self._extract_u()
        except Exception as exc:
            print(f"Solver exception: {exc}")
            return None

    def simulate(self) -> tuple[np.ndarray, np.ndarray]:
        q = self.cfg.q0.copy().astype(float)
        u_guess = np.zeros((2, self.cfg.N), dtype=float)
        q_hist = [q.copy()]
        u_hist: list[np.ndarray] = []

        for step in range(self.cfg.max_steps):
            pos_err = float(np.linalg.norm(q[:2] - self.cfg.q_des[:2]))
            ang_err_t = abs(wrap_angle(q[2] - self.cfg.q_des[2]))
            ang_err_l = abs(wrap_angle(q[3] - self.cfg.q_des[3]))

            if (
                pos_err <= self.cfg.target_tol
                and ang_err_t <= self.cfg.angle_tol
                and ang_err_l <= self.cfg.angle_tol
            ):
                break

            u_opt = self.solve(q, u_guess)
            if u_opt is None:
                print(f"Solver failed at step {step}")
                break

            u0 = u_opt[:, 0]
            q = q + self.cfg.dt * dynamics(q, u0, self.cfg.L, self.cfg.d)
            q[2] = wrap_angle(q[2])
            q[3] = wrap_angle(q[3])

            q_hist.append(q.copy())
            u_hist.append(u0.copy())
            u_guess = np.hstack([u_opt[:, 1:], u_opt[:, -1:]])

            print(
                f"step {step:03d} | pos_err={pos_err: .3f} | ang_err_t={ang_err_t: .3f} | ang_err_l={ang_err_l: .3f}"
            )

        return np.array(q_hist), np.array(u_hist)


def animate_result(q_hist: np.ndarray, cfg: MPCConfig) -> None:
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")

    all_x = np.r_[q_hist[:, 0], cfg.q_des[0]]
    all_y = np.r_[q_hist[:, 1], cfg.q_des[1]]
    pad = 1.5
    ax.set_xlim(np.min(all_x) - pad, np.max(all_x) + pad)
    ax.set_ylim(np.min(all_y) - pad, np.max(all_y) + pad)
    ax.plot(cfg.q_des[0], cfg.q_des[1], "rx", markersize=10, mew=2, label="target")

    path_line, = ax.plot([], [], "b-", linewidth=1.5, alpha=0.6, label="path")
    truck_line, = ax.plot([], [], "k-", linewidth=3.0, label="truck")
    trailer_line, = ax.plot([], [], "r-", linewidth=3.0, label="trailer")
    hitch_point, = ax.plot([], [], "ko", markersize=5)
    title_text = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top")
    ax.legend(loc="best")

    def init():
        path_line.set_data([], [])
        truck_line.set_data([], [])
        trailer_line.set_data([], [])
        hitch_point.set_data([], [])
        title_text.set_text("")
        return path_line, truck_line, trailer_line, hitch_point, title_text

    def update(frame: int):
        x, y, theta_t, theta_l = q_hist[frame, 0], q_hist[frame, 1], q_hist[frame, 2], q_hist[frame, 3]
        x_front = x + cfg.L * math.cos(theta_t)
        y_front = y + cfg.L * math.sin(theta_t)
        x_trailer = x - cfg.d * math.cos(theta_l)
        y_trailer = y - cfg.d * math.sin(theta_l)

        path_line.set_data(q_hist[: frame + 1, 0], q_hist[: frame + 1, 1])
        truck_line.set_data([x, x_front], [y, y_front])
        trailer_line.set_data([x, x_trailer], [y, y_trailer])
        hitch_point.set_data([x], [y])
        title_text.set_text(f"Step {frame + 1}/{len(q_hist)}")
        return path_line, truck_line, trailer_line, hitch_point, title_text

    _anim = FuncAnimation(fig, update, init_func=init, frames=len(q_hist), interval=80, blit=True)

    if plt.get_backend().lower() == "agg":
        _anim.save("parking.gif", writer=PillowWriter(fps=12))
        print("Saved animation to parking.gif")
    else:
        plt.show()
    plt.close(fig)


def main() -> None:
    cfg = MPCConfig()
    mpc = TruckTrailerMPC(cfg)
    
    q_hist, u_hist = mpc.simulate()
    
    final_err = float(np.linalg.norm(q_hist[-1, :2] - cfg.q_des[:2]))
    final_ang_t = abs(wrap_angle(q_hist[-1, 2] - cfg.q_des[2]))
    final_ang_l = abs(wrap_angle(q_hist[-1, 3] - cfg.q_des[3]))

    print(f"Closed-loop steps: {len(q_hist) - 1}")
    print(f"Final pos error: {final_err:.3f} m")
    print(f"Final ang error (truck): {final_ang_t:.3f} rad")
    print(f"Final ang error (trailer): {final_ang_l:.3f} rad")

    if q_hist.size:
        animate_result(q_hist, cfg)


if __name__ == "__main__":
    main()
