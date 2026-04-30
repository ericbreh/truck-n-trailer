"""
truck_trailer/parking/runner.py
────────────────
Closed-loop autonomous parking controller.

Data flow every tick
────────────────────
  Camera frame
    → StateEstimator        : ArUco detect  →  q = [x, y, θ_t, θ_l, v, ω]
    → TruckTrailerMPC.solve : IPOPT         →  u = [a, α]   (first step)
    → BodyToWheels.convert  : kinematics    →  (rpm_l, rpm_r)
    → UartPacketSender      : serial UART   →  ESP32 firmware PID loop

Usage
─────
    cd core/
    python -m truck_trailer.parking.runner --port /dev/ttyACM0 --goal "50,50,0,0"

    # Dry-run (no hardware attached — prints commands instead of sending)
    python -m truck_trailer.parking.runner --dry-run

Tuning checklist before first run
──────────────────────────────────
  1. Set WHEEL_TRACK_CM and WHEEL_RADIUS_CM to your model's measurements.
  2. Set MPC_DT to match your expected loop rate (default 0.1 s = 10 Hz).
     The IPOPT solver typically takes 50–300 ms, so 0.1–0.2 s is realistic.
  3. Set GOAL_STATE to the desired parking pose in workspace centimetres.
  4. Set MPC_N (horizon length).  Longer = smoother but slower to solve.
  5. Verify units: MPC works in cm and radians; camera outputs cm; firmware
     expects RPM.  BodyToWheels handles the cm/s → RPM conversion.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from typing import Optional

import numpy as np

# ── project imports ──────────────────────────────────────────────────────── #
from truck_trailer.parking.body_to_wheels import BodyToWheels
from truck_trailer.parking.state_estimator import StateEstimator
from truck_trailer.control.mpc import MPCConfig, TruckTrailerMPC
from truck_trailer.uart import UartPacketSender

# ═══════════════════════════════════════════════════════════════════════════ #
#  CONFIG — edit these before running                                         #
# ═══════════════════════════════════════════════════════════════════════════ #

# Physical model measurements (centimetres)
WHEEL_TRACK_CM: float = 12.0    # left–right distance between wheel centres
WHEEL_RADIUS_CM: float = 3.0    # rear wheel radius
TRAILER_HITCH_CM: float = 10.0  # hitch-to-trailer-axle distance (MPC param d)
TRUCK_LENGTH_CM: float = 15.0   # rear-axle-to-front distance  (MPC param L)

# MPC settings
MPC_DT: float = 0.1          # seconds per step (also sets UART send rate)
MPC_N: int = 30              # planning horizon (steps)
MPC_MAX_STEPS: int = 500     # give up after this many closed-loop iterations

# Goal state: [x_cm, y_cm, theta_truck_rad, theta_trailer_rad, v=0, omega=0]
GOAL_STATE = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

# Convergence thresholds
GOAL_POS_TOL_CM: float = 2.0    # stop when position error < this (cm)
GOAL_ANG_TOL_RAD: float = 0.15  # stop when heading error < this (rad)

# How many consecutive frames both markers must be missing before we E-stop
MISSING_FRAME_LIMIT: int = 5

# ═══════════════════════════════════════════════════════════════════════════ #


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _reached_goal(q: np.ndarray, q_des: np.ndarray) -> bool:
    pos_err = float(np.linalg.norm(q[:2] - q_des[:2]))
    ang_t = abs(_wrap(q[2] - q_des[2]))
    ang_l = abs(_wrap(q[3] - q_des[3]))
    return (pos_err < GOAL_POS_TOL_CM
            and ang_t < GOAL_ANG_TOL_RAD
            and ang_l < GOAL_ANG_TOL_RAD)


def _build_mpc(goal: np.ndarray) -> TruckTrailerMPC:
    cfg = MPCConfig(
        L=TRUCK_LENGTH_CM,
        d=TRAILER_HITCH_CM,
        dt=MPC_DT,
        N=MPC_N,
        max_steps=MPC_MAX_STEPS,
        q_des=goal,
        # tighten tolerances slightly for real hardware
        target_tol=GOAL_POS_TOL_CM / 100.0,  # MPC internal (metres-equivalent)
        angle_tol=GOAL_ANG_TOL_RAD,
    )
    return TruckTrailerMPC(cfg)


def run(
    port: Optional[str],
    baud: int,
    goal: np.ndarray,
    dry_run: bool,
    show_overlay: bool,
) -> None:
    print("=== Truck-N-Trailer autonomous runner ===")
    print(f"  Goal : {goal}")
    print(f"  Port : {'[dry-run]' if dry_run else port}")
    print(f"  dt   : {MPC_DT} s  |  horizon N={MPC_N}")
    print()

    # ── initialise subsystems ────────────────────────────────────────── #
    estimator = StateEstimator(show_overlay=show_overlay)
    estimator.open()
    print("[vision] Camera opened.")

    converter = BodyToWheels(
        wheel_track_cm=WHEEL_TRACK_CM,
        wheel_radius_cm=WHEEL_RADIUS_CM,
    )

    mpc = _build_mpc(goal)
    print("[mpc]    Model built, solver ready.")

    sender: Optional[UartPacketSender] = None
    if not dry_run:
        assert port is not None, "--port is required unless --dry-run"
        sender = UartPacketSender(port=port, baud=baud)
        sender.connect()
        print(f"[uart]   Connected to {port} @ {baud} baud.")
    else:
        print("[uart]   Dry-run mode — commands will be printed, not sent.")

    # ── wait for first valid frame ────────────────────────────────────── #
    print("\n[runner] Waiting for both markers to become visible...")
    q = None
    while q is None:
        q = estimator.get_state()
        time.sleep(0.05)
    print(f"[runner] Initial state: {np.round(q, 3)}")

    # Seed the velocity integrator with measured initial velocities
    converter.reset(v=q[4], omega=q[5])

    # ── main control loop ─────────────────────────────────────────────── #
    u_guess = np.zeros((2, MPC_N), dtype=float)
    missing_count = 0
    step = 0

    try:
        while step < MPC_MAX_STEPS:
            t_start = time.monotonic()

            # 1. Get current state from vision
            q_new = estimator.get_state()

            if q_new is None:
                missing_count += 1
                print(f"[vision] Marker(s) missing ({missing_count}/{MISSING_FRAME_LIMIT})")
                if missing_count >= MISSING_FRAME_LIMIT:
                    print("[runner] E-STOP: markers lost for too long.")
                    _send_stop(sender, dry_run)
                    break
                # Reuse last known state but continue — don't advance step
                time.sleep(MPC_DT)
                continue

            missing_count = 0
            q = q_new

            # 2. Check convergence
            if _reached_goal(q, goal):
                print(f"\n[runner] Goal reached at step {step}!")
                _send_stop(sender, dry_run)
                break

            # 3. Sync velocity integrator with measured state
            #    (prevents integrator drift diverging from reality)
            converter.reset(v=q[4], omega=q[5])

            # 4. Solve MPC
            t_solve = time.monotonic()
            u_opt = mpc.solve(q, u_guess)
            solve_ms = (time.monotonic() - t_solve) * 1000.0

            if u_opt is None:
                print(f"[mpc]    Solver failed at step {step} — sending stop.")
                _send_stop(sender, dry_run)
                break

            # 5. Extract first action and convert to wheel RPMs
            a, alpha = float(u_opt[0, 0]), float(u_opt[1, 0])
            rpm_l, rpm_r = converter.convert(a, alpha, MPC_DT)

            # 6. Send to firmware
            _send(sender, rpm_l, rpm_r, dry_run)

            # 7. Warm-start next solve
            u_guess = np.hstack([u_opt[:, 1:], u_opt[:, -1:]])

            # 8. Logging
            pos_err = float(np.linalg.norm(q[:2] - goal[:2]))
            ang_t = abs(_wrap(q[2] - goal[2]))
            ang_l = abs(_wrap(q[3] - goal[3]))
            elapsed_ms = (time.monotonic() - t_start) * 1000.0

            print(
                f"[{step:04d}] "
                f"pos={pos_err:6.1f}cm  "
                f"θt={math.degrees(ang_t):5.1f}°  "
                f"θl={math.degrees(ang_l):5.1f}°  "
                f"a={a:+.3f}  α={alpha:+.3f}  "
                f"rpm=({rpm_l:+.1f},{rpm_r:+.1f})  "
                f"solve={solve_ms:.0f}ms  loop={elapsed_ms:.0f}ms"
            )

            step += 1

            # 9. Pace the loop — sleep any leftover time within the MPC period
            elapsed = time.monotonic() - t_start
            sleep_s = MPC_DT - elapsed
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                print(
                    f"[runner] WARNING: loop overran by {-sleep_s*1000:.0f} ms "
                    f"at step {step}. Consider increasing MPC_DT or reducing N."
                )

    except KeyboardInterrupt:
        print("\n[runner] Interrupted by user.")
    finally:
        print("[runner] Stopping motors and releasing resources.")
        _send_stop(sender, dry_run)
        if sender is not None:
            sender.close()
        estimator.close()


# ── helpers ──────────────────────────────────────────────────────────────── #

def _send(
    sender: Optional[UartPacketSender],
    rpm_l: float,
    rpm_r: float,
    dry_run: bool,
) -> None:
    if dry_run:
        print(f"  [dry] CMD  rpm_l={rpm_l:+.2f}  rpm_r={rpm_r:+.2f}")
        return
    assert sender is not None
    sender.send_targets(rpm_l, rpm_r)


def _send_stop(sender: Optional[UartPacketSender], dry_run: bool) -> None:
    _send(sender, 0.0, 0.0, dry_run)


# ── CLI ──────────────────────────────────────────────────────────────────── #

def _parse_goal(s: str) -> np.ndarray:
    """Parse '50,50,0,0' into a 6-element goal array (v=0, omega=0 appended)."""
    parts = [float(x.strip()) for x in s.split(",")]
    if len(parts) == 4:
        parts += [0.0, 0.0]
    if len(parts) != 6:
        raise argparse.ArgumentTypeError(
            "goal must be 4 values (x,y,θt,θl) or 6 values (x,y,θt,θl,v,ω)"
        )
    return np.array(parts, dtype=float)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Closed-loop truck-trailer autonomous parking runner"
    )
    parser.add_argument(
        "--port", default=None,
        help="Serial port for UART (e.g. /dev/ttyACM0).  Required unless --dry-run."
    )
    parser.add_argument(
        "--baud", type=int, default=115200,
        help="UART baud rate (default 115200)"
    )
    parser.add_argument(
        "--goal", type=_parse_goal,
        default="0,0,0,0",
        help="Goal pose as 'x,y,theta_t_deg,theta_l_deg' in cm/degrees "
             "(degrees are converted to radians internally). "
             "Default: '0,0,0,0'"
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help="Run without UART hardware — prints RPM commands to stdout."
    )
    parser.add_argument(
        "--no-overlay", action="store_true",
        help="Disable the OpenCV camera overlay window (headless mode)."
    )
    args = parser.parse_args()

    if not args.dry_run and args.port is None:
        parser.error("--port is required unless --dry-run is set.")

    # Convert theta values from degrees to radians for convenience
    goal = args.goal.copy()
    goal[2] = math.radians(goal[2])
    goal[3] = math.radians(goal[3])

    run(
        port=args.port,
        baud=args.baud,
        goal=goal,
        dry_run=args.dry_run,
        show_overlay=not args.no_overlay,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
