#!/usr/bin/env python3

import math

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
import numpy as np

from .mpc import MPCConfig, TruckTrailerMPC


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


def run_simulation(cfg: MPCConfig, mpc: TruckTrailerMPC) -> tuple[np.ndarray, np.ndarray]:
    q = cfg.q0.copy().astype(float)
    u_guess = np.zeros((2, cfg.N), dtype=float)
    q_hist = [q.copy()]
    u_hist: list[np.ndarray] = []

    for step in range(cfg.max_steps):
        pos_err = float(np.linalg.norm(q[:2] - cfg.q_des[:2]))
        ang_err_t = abs(wrap_angle(q[2] - cfg.q_des[2]))
        ang_err_l = abs(wrap_angle(q[3] - cfg.q_des[3]))

        if pos_err <= cfg.target_tol and ang_err_t <= cfg.angle_tol and ang_err_l <= cfg.angle_tol:
            break

        u_opt = mpc.solve(q, u_guess)
        if u_opt is None:
            print(f"Solver failed at step {step}")
            break

        u0 = u_opt[:, 0]
        q = q + cfg.dt * dynamics(q, u0, cfg.L, cfg.d)
        q[2] = wrap_angle(q[2])
        q[3] = wrap_angle(q[3])

        q_hist.append(q.copy())
        u_hist.append(u0.copy())
        u_guess = np.hstack([u_opt[:, 1:], u_opt[:, -1:]])

        print(f"step {step:03d} | pos_err={pos_err: .3f} | ang_err_t={ang_err_t: .3f} | ang_err_l={ang_err_l: .3f}")

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
    cfg = MPCConfig(q0=np.array([2, 4, 1.57, 1.57, 0, 0]), q_des=np.array([0, 0, 0, 0, 0, 0]))
    mpc = TruckTrailerMPC(cfg)

    q_hist, _u_hist = run_simulation(cfg, mpc)

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
