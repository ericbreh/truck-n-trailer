# Core Logic

## Vision

The vision system tracks the truck and trailer using an overhead camera. It outputs position `(x, y)` in centimeters and heading angles.

### Usage

Run all commands from the `core/` directory:

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

The control system calculates and outputs steering and acceleration using MPC.

### Usage

```sh
python -m control.simulation
```

## GUI

The GUI provides a interface for sending commands via UART.

### Usage

```sh
python -m gui.uart_gui --port /dev/ttyACM0 --rpm 30
```
