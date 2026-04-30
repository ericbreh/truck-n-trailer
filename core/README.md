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
python -m truck_trailer.vision.generate_markers
python -m truck_trailer.vision.calibrate_camera
python -m truck_trailer.vision.calibrate_homography
python -m truck_trailer.vision.detect
```

## Control

The control system calculates and outputs steering and acceleration using MPC.

### Usage

```sh
python -m truck_trailer.control.simulation
```

## GUI

The GUI provides a interface for sending commands via UART.

### Usage

```sh
python -m truck_trailer.gui.uart_gui --port /dev/ttyACM0 --rpm 30
```

## Parking

The autonomous parking module orchestrates vision, control, and UART communication.

### Usage

```sh
# Dry-run (no hardware)
python -m truck_trailer.parking.runner --dry-run

# With UART hardware
python -m truck_trailer.parking.runner --port /dev/ttyACM0 --goal "50,50,0,0"
```
