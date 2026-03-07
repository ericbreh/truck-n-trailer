#!/usr/bin/env python3

import math
import os
from importlib import import_module
from typing import Any, cast

import matplotlib


def select_backend() -> str:
    def backend_available(name: str) -> bool:
        try:
            if name == "QtAgg":
                import_module("matplotlib.backends.backend_qtagg")
                for qt_mod in ("PyQt6", "PySide6", "PyQt5", "PySide2"):
                    try:
                        import_module(qt_mod)
                        return True
                    except Exception:
                        continue
                return False
            if name == "TkAgg":
                import_module("tkinter")
                import_module("matplotlib.backends.backend_tkagg")
                return True
        except Exception:
            return False
        return False

    has_gui = bool(os.environ.get("WAYLAND_DISPLAY") or os.environ.get("DISPLAY"))
    if has_gui:
        for backend in ("QtAgg", "TkAgg"):
            if backend_available(backend):
                matplotlib.use(backend, force=True)
                return backend
    matplotlib.use("Agg", force=True)
    return "Agg"


SELECTED_BACKEND = select_backend()
NON_INTERACTIVE_BACKENDS = {"agg", "pdf", "svg", "ps", "cairo", "template"}

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
import numpy as np
import pyomo.environ as pyo


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


def build_model(p: dict) -> pyo.ConcreteModel:
    N, dt, L, d = p["N"], p["dt"], p["L"], p["d"]
    q_des = p["q_des"]

    m = pyo.ConcreteModel()
    m.T = pyo.RangeSet(0, N)
    m.K = pyo.RangeSet(0, N - 1)
    m.S = pyo.RangeSet(0, 5)

    m.x = pyo.Var(m.S, m.T, domain=pyo.Reals)
    m.a = pyo.Var(m.K, bounds=(p["a_min"], p["a_max"]))
    m.phi_dot = pyo.Var(m.K, bounds=(p["phi_dot_min"], p["phi_dot_max"]))

    def dyn_rule(model: pyo.ConcreteModel, s: int, k: int):
        x = cast(Any, model.x)
        a = cast(Any, model.a)
        phi_dot = cast(Any, model.phi_dot)
        if s == 0:
            return x[0, k + 1] == x[0, k] + dt * x[4, k] * pyo.cos(x[2, k])
        if s == 1:
            return x[1, k + 1] == x[1, k] + dt * x[4, k] * pyo.sin(x[2, k])
        if s == 2:
            return x[2, k + 1] == x[2, k] + dt * (x[4, k] / L) * pyo.tan(x[5, k])
        if s == 3:
            return x[3, k + 1] == x[3, k] + dt * (x[4, k] / d) * pyo.sin(x[2, k] - x[3, k])
        if s == 4:
            return x[4, k + 1] == x[4, k] + dt * a[k]
        return x[5, k + 1] == x[5, k] + dt * phi_dot[k]

    m.dyn = pyo.Constraint(m.S, m.K, rule=dyn_rule)

    m.vel_bounds = pyo.Constraint(m.T, rule=lambda model, t: pyo.inequality(p["v_min"], model.x[4, t], p["v_max"]))
    m.steer_bounds = pyo.Constraint(m.T, rule=lambda model, t: pyo.inequality(p["phi_min"], model.x[5, t], p["phi_max"]))
    m.jackknife = pyo.Constraint(
        m.T,
        rule=lambda model, t: pyo.inequality(-p["max_jackknife_angle"], model.x[2, t] - model.x[3, t], p["max_jackknife_angle"]),
    )

    def state_cost(model: pyo.ConcreteModel, t: int):
        x = cast(Any, model.x)
        return (
            p["w_pos"] * ((x[0, t] - q_des[0]) ** 2 + (x[1, t] - q_des[1]) ** 2)
            + p["w_theta_t"] * (x[2, t] - q_des[2]) ** 2
            + p["w_theta_l"] * (x[3, t] - q_des[3]) ** 2
            + p["w_v"] * (x[4, t] - q_des[4]) ** 2
            + p["w_phi"] * (x[5, t] - q_des[5]) ** 2
        )

    def obj_rule(model: pyo.ConcreteModel):
        a = cast(Any, model.a)
        phi_dot = cast(Any, model.phi_dot)
        terminal = state_cost(model, N)
        effort = sum(p["w_a"] * a[k] ** 2 + p["w_phi_dot"] * phi_dot[k] ** 2 for k in range(N))
        return terminal + effort

    m.obj = pyo.Objective(rule=obj_rule, sense=pyo.minimize)
    return m


def set_guess(m: pyo.ConcreteModel, u_guess: np.ndarray) -> None:
    a = cast(Any, m.a)
    phi_dot = cast(Any, m.phi_dot)
    for k in range(u_guess.shape[1]):
        a[k].set_value(float(u_guess[0, k]))
        phi_dot[k].set_value(float(u_guess[1, k]))


def extract_u(m: pyo.ConcreteModel, N: int) -> np.ndarray:
    a = cast(Any, m.a)
    phi_dot = cast(Any, m.phi_dot)
    u = np.zeros((2, N), dtype=float)
    for k in range(N):
        u[0, k] = pyo.value(a[k])
        u[1, k] = pyo.value(phi_dot[k])
    return u


def run_mpc(p: dict) -> tuple[np.ndarray, np.ndarray]:
    N, dt = p["N"], p["dt"]
    q = p["q0"].copy().astype(float)

    m = build_model(p)
    x = cast(Any, m.x)
    solver = pyo.SolverFactory("ipopt")
    solver.options["max_iter"] = p.get("solver_max_iter", 500)
    solver.options["tol"] = p.get("solver_tol", 1e-6)

    u_guess = np.zeros((2, N), dtype=float)
    q_hist = [q.copy()]
    u_hist: list[np.ndarray] = []

    for step in range(p["max_steps"]):
        pos_err = float(np.linalg.norm(q[:2] - p["q_des"][:2]))
        if pos_err <= p["target_tol"]:
            break

        for s in range(6):
            x[s, 0].fix(float(q[s]))

        u_guess[0, :] = np.clip(u_guess[0, :], p["a_min"], p["a_max"])
        u_guess[1, :] = np.clip(u_guess[1, :], p["phi_dot_min"], p["phi_dot_max"])
        set_guess(m, u_guess)
        try:
            result = solver.solve(m, tee=False, load_solutions=False)
        except Exception as exc:
            print(f"Solver failed at step {step}: {exc}")
            break

        status = result.solver.status
        term = result.solver.termination_condition
        if status not in (pyo.SolverStatus.ok, pyo.SolverStatus.warning):
            print(f"Solver status {status} at step {step}")
            break
        if term not in (pyo.TerminationCondition.optimal, pyo.TerminationCondition.locallyOptimal):
            print(f"Solver stopped with {term} at step {step}")
            break

        m.solutions.load_from(result)

        u_opt = extract_u(m, N)
        u0 = u_opt[:, 0]

        q = q + dt * dynamics(q, u0, p["L"], p["d"])
        q[2] = wrap_angle(q[2])
        q[3] = wrap_angle(q[3])

        q_hist.append(q.copy())
        u_hist.append(u0.copy())
        u_guess = np.hstack([u_opt[:, 1:], u_opt[:, -1:]])

        print(f"step {step:03d} | a={u0[0]: .3f} | phi_dot={u0[1]: .3f} | pos_err={pos_err: .3f}")

    return np.array(q_hist), np.array(u_hist)


def plot_result(q_hist: np.ndarray, p: dict, output_path: str) -> None:
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot(q_hist[:, 0], q_hist[:, 1], "b-", linewidth=2, label="trajectory")
    ax.plot(q_hist[0, 0], q_hist[0, 1], "go", label="start")
    ax.plot(p["q_des"][0], p["q_des"][1], "rx", markersize=10, mew=2, label="target")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("6-state MPC point navigation")
    ax.legend(loc="best")
    plt.tight_layout()
    fig.savefig(output_path, dpi=150)

    if matplotlib.get_backend().lower() not in NON_INTERACTIVE_BACKENDS:
        plt.show()
    plt.close(fig)


def animate_result(q_hist: np.ndarray, p: dict, output_path: str) -> None:
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")

    all_x = np.r_[q_hist[:, 0], p["q_des"][0]]
    all_y = np.r_[q_hist[:, 1], p["q_des"][1]]
    pad = 1.5
    ax.set_xlim(np.min(all_x) - pad, np.max(all_x) + pad)
    ax.set_ylim(np.min(all_y) - pad, np.max(all_y) + pad)
    ax.plot(p["q_des"][0], p["q_des"][1], "rx", markersize=10, mew=2, label="target")

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
        x_front = x + p["L"] * math.cos(theta_t)
        y_front = y + p["L"] * math.sin(theta_t)
        x_trailer = x - p["d"] * math.cos(theta_l)
        y_trailer = y - p["d"] * math.sin(theta_l)

        path_line.set_data(q_hist[: frame + 1, 0], q_hist[: frame + 1, 1])
        truck_line.set_data([x, x_front], [y, y_front])
        trailer_line.set_data([x, x_trailer], [y, y_trailer])
        hitch_point.set_data([x], [y])
        title_text.set_text(f"Step {frame + 1}/{len(q_hist)}")
        return path_line, truck_line, trailer_line, hitch_point, title_text

    _anim = FuncAnimation(fig, update, init_func=init, frames=len(q_hist), interval=80, blit=True)

    if SELECTED_BACKEND.lower() in NON_INTERACTIVE_BACKENDS:
        _anim.save(output_path, writer=PillowWriter(fps=12))
        plt.close(fig)
        print(f"Saved animation to {output_path}")
    else:
        plt.show()
        plt.close(fig)


def main() -> None:
    params = {
        "L": 1.0,
        "d": 5.0,
        "dt": 0.1,
        "N": 20,
        "q0": np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=float),
        "q_des": np.array([8.0, 3.0, 0.0, 0.0, 0.0, 0.0], dtype=float),
        "a_min": -1.5,
        "a_max": 1.5,
        "phi_dot_min": -0.8,
        "phi_dot_max": 0.8,
        "v_min": -2.0,
        "v_max": 2.0,
        "phi_min": -0.6,
        "phi_max": 0.6,
        "max_jackknife_angle": math.pi / 2.0,
        "w_pos": 8.0,
        "w_theta_t": 0.5,
        "w_theta_l": 0.5,
        "w_v": 0.3,
        "w_phi": 0.3,
        "w_a": 0.05,
        "w_phi_dot": 0.05,
        "max_steps": 80,
        "target_tol": 0.1,
        "solver_max_iter": 600,
        "solver_tol": 1e-6,
    }

    q_hist, u_hist = run_mpc(params)
    final_err = float(np.linalg.norm(q_hist[-1, :2] - params["q_des"][:2]))

    print(f"Closed-loop steps: {len(q_hist) - 1}")
    print(f"Final position error: {final_err:.3f} m")
    if u_hist.size:
        print(f"Last control: a={u_hist[-1, 0]:.3f}, phi_dot={u_hist[-1, 1]:.3f}")

    out_file = "parking.gif"
    try:
        animate_result(q_hist, params, out_file)
    except Exception as exc:
        print(f"Animation failed ({exc})")
        png_file = "parking_result.png"
        plot_result(q_hist, params, png_file)
        print(f"Saved plot to {png_file}")


if __name__ == "__main__":
    main()
