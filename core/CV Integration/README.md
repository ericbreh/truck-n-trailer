# bridge/

This directory contains the three files that connect the vision, MPC, and
firmware subsystems into a single closed-loop autonomous parking controller.

## Files

| File | Role |
|---|---|
| `state_estimator.py` | Wraps the vision pipeline. Opens the camera, detects ArUco markers every frame, and returns a 6-element state vector `q = [x, y, θ_t, θ_l, v, ω]` ready for the MPC. Velocity `v` and `ω` are estimated by finite-differencing consecutive poses and smoothed with an EMA filter. |
| `body_to_wheels.py` | Converts MPC body-frame outputs `[a, α]` (linear and angular acceleration) into left/right wheel RPM targets using the differential drive equations from the README. |
| `runner.py` | The main closed-loop script. Ties everything together: vision → MPC → wheel conversion → UART → ESP32. |

## Before running

**1. Measure your truck and set physical constants in `runner.py`:**

```python
WHEEL_TRACK_CM  = 12.0   # distance between rear wheel centrelines
WHEEL_RADIUS_CM =  3.0   # rear wheel radius
TRAILER_HITCH_CM = 10.0  # hitch to trailer axle
TRUCK_LENGTH_CM  = 15.0  # rear axle to front
```

**2. Run camera calibration (one-time per camera):**

```sh
cd core/
python -m vision.calibrate_camera
```

**3. Run homography calibration (redo if camera or workspace moves):**

```sh
python -m vision.calibrate_homography
```

**4. Test in dry-run mode first (no hardware needed):**

```sh
python -m bridge.runner --dry-run
```

This runs the full loop — camera, MPC solve, body-to-wheels conversion —
and prints the RPM commands to stdout instead of sending them to the ESP32.

**5. Run with hardware:**

```sh
python -m bridge.runner --port /dev/ttyACM0 --goal "50,50,0,0"
```

`--goal` takes `x,y,theta_truck,theta_trailer` in centimetres and degrees.
Degrees are converted to radians internally.

## Loop timing

The MPC solve (IPOPT) typically takes 50–300 ms depending on horizon length
and initial conditions.  The default `MPC_DT = 0.1 s` is a reasonable
starting point — if the loop overruns you will see a warning printed each
step.  Either increase `MPC_DT` or reduce `MPC_N` to bring it back in budget.

## Tuning the velocity estimator

`StateEstimator.ema_alpha` controls how aggressively velocity estimates are
smoothed.  The default is `0.3` (moderate smoothing).

- Increase toward `1.0` if the MPC seems to react too slowly to speed changes.
- Decrease toward `0.1` if estimated velocity is noisy and causing jitter.

## Known limitations / next steps

- The velocity estimator is purely vision-based (no IMU or wheel odometry
  fusion).  It will be noisy at low speeds and degrade if the camera framerate
  drops.
- MPC unit assumptions: positions in **cm**, velocities in **cm/s**,
  angles in **radians**.  If you change the workspace calibration units,
  update `BodyToWheels` accordingly.
- No obstacle avoidance or parking-spot boundary constraints yet (noted as
  a "Possible addition" in the main README).
