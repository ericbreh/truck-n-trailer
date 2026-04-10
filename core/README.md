# Core Logic

## Vision

The vision system tracks the truck and trailer using an overhead camera. It outputs position `(x, y)` in centimeters and heading angles.

### Usage

Run from the `core/` directory:

1. **Generate markers**
2. **Calibrate camera intrinsics** (one-time per camera)
3. **Calibrate workspace homography** (redo if camera/workspace moves)
4. **Run detection**

```sh
python -m vision.generate_markers
python -m vision.calibrate_camera
python -m vision.calibrate_homography
python -m vision.detect
```

## Control

The control system calculates and outputs steering and acceleration.

### Usage

Run from the `core/` directory:

```sh
python -m control.simulation
```

### UART Sender

```sh
python -m control.uart_sender --port /dev/ttyACM0 --mode step --hz 10 --low 20 --high 60 --period-s 5
python -m control.uart_sender --port /dev/ttyACM0 --mode constant --left 30 --right 30 --hz 10
```
