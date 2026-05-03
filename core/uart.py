#!/usr/bin/env python3

import argparse
import sys
import time
from dataclasses import dataclass


try:
    import serial  # type: ignore
except ImportError:  # pragma: no cover
    serial = None


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def format_packet(rpm_l: float, rpm_r: float) -> str:
    payload = f"CMD,{rpm_l:.3f},{rpm_r:.3f}"
    crc = crc16_modbus(payload.encode("ascii"))
    return f"{payload},{crc:04X}\n"


def validate_sender_config(*, hz: float, period_s: float | None = None, mode: str | None = None) -> None:
    if hz <= 0:
        raise ValueError("hz must be > 0")
    if mode == "step" and (period_s is None or period_s <= 0):
        raise ValueError("period_s must be > 0 for step mode")


def open_serial(port: str, baud: int):
    if serial is None:
        raise RuntimeError("pyserial is required. Install with: pip install pyserial")
    return serial.Serial(port, baud, timeout=0)


@dataclass
class UartPacketSender:
    port: str
    baud: int = 115200

    def __post_init__(self):
        self.ser = None
        self._rx_buffer = ""
        self._latest_pot_angle_deg: float | None = None
        self._latest_motor_rpms: tuple[float, float] | None = None
        validate_sender_config(hz=1.0)

    def connect(self) -> None:
        self.ser = open_serial(self.port, self.baud)

    def send_targets(self, rpm_l: float, rpm_r: float) -> str:
        if self.ser is None:
            raise RuntimeError("serial port is not connected")
        packet = format_packet(rpm_l, rpm_r)
        self.ser.write(packet.encode("ascii"))
        return packet

    def _poll_rx(self) -> None:
        if self.ser is None:
            return
        waiting = int(getattr(self.ser, "in_waiting", 0))
        if waiting <= 0:
            return

        raw = self.ser.read(waiting)
        if not raw:
            return

        self._rx_buffer += raw.decode("ascii", errors="ignore")
        parts = self._rx_buffer.split("\n")
        self._rx_buffer = parts[-1]

        # Parse completed lines - format: L_rpm,R_rpm,POT_angle
        for line in parts[:-1]:
            values = line.strip().split(",")
            if len(values) == 3:
                try:
                    self._latest_motor_rpms = (float(values[0]), float(values[1]))
                    self._latest_pot_angle_deg = float(values[2])
                except ValueError:
                    pass

    def read_pot_angle_deg(self) -> float | None:
        """Read latest POT telemetry from format: L_rpm,R_rpm,POT_angle"""
        self._poll_rx()
        return self._latest_pot_angle_deg

    def read_motor_rpms(self) -> tuple[float, float] | None:
        """Read latest measured motor RPMs from format: L_rpm,R_rpm,POT_angle"""
        self._poll_rx()
        return self._latest_motor_rpms

    def close(self) -> None:
        if self.ser is not None:
            self.ser.close()
            self.ser = None


def get_targets(mode: str, now_s: float, args: argparse.Namespace) -> tuple[float, float]:
    if mode == "constant":
        return args.left, args.right

    if mode == "step":
        phase = int(now_s / args.period_s) % 2
        value = args.low if phase == 0 else args.high
        return value, value

    raise ValueError(f"unsupported mode: {mode}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Send RPM commands to ESP32 motor firmware over UART")
    parser.add_argument("--port", required=True, help="Serial port (e.g. /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=115200, help="UART baud rate")
    parser.add_argument("--hz", type=float, default=10.0, help="Send rate in Hz")
    parser.add_argument("--mode", choices=["constant", "step"], default="step", help="Command profile mode")
    parser.add_argument("--left", type=float, default=30.0, help="Constant left RPM")
    parser.add_argument("--right", type=float, default=30.0, help="Constant right RPM")
    parser.add_argument("--low", type=float, default=20.0, help="Step low RPM")
    parser.add_argument("--high", type=float, default=60.0, help="Step high RPM")
    parser.add_argument("--period-s", type=float, default=5.0, help="Step half-period in seconds")
    parser.add_argument("--duration-s", type=float, default=0.0, help="Run duration; 0 means forever")
    args = parser.parse_args()

    try:
        validate_sender_config(hz=args.hz, period_s=args.period_s, mode=args.mode)
    except ValueError as exc:
        print(f"--{str(exc)}", file=sys.stderr)
        return 2

    period_s = 1.0 / args.hz
    start = time.monotonic()
    next_tick = start
    sender = UartPacketSender(port=args.port, baud=args.baud)
    packets_sent = 0

    try:
        sender.connect()
        try:
            print(f"Sending on {args.port} @ {args.baud} baud, mode={args.mode}, rate={args.hz:.2f} Hz")
            while True:
                now = time.monotonic()
                elapsed = now - start
                if args.duration_s > 0 and elapsed >= args.duration_s:
                    break

                rpm_l, rpm_r = get_targets(args.mode, elapsed, args)
                packet = sender.send_targets(rpm_l, rpm_r)
                packets_sent += 1

                if packets_sent % int(max(1, args.hz)) == 0:
                    print(packet.strip())

                next_tick += period_s
                sleep_s = next_tick - time.monotonic()
                if sleep_s > 0:
                    time.sleep(sleep_s)
                else:
                    next_tick = time.monotonic()
        finally:
            sender.close()
    except KeyboardInterrupt:
        print("Stopped")
        return 0
    except Exception as exc:
        print(f"UART sender failed: {exc}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
