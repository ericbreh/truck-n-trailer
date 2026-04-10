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


def format_packet(seq: int, timestamp_ms: int, rpm_l: float, rpm_r: float, ttl_ms: int) -> str:
    payload = f"CMD,{seq},{timestamp_ms},{rpm_l:.3f},{rpm_r:.3f},{ttl_ms}"
    crc = crc16_modbus(payload.encode("ascii"))
    return f"{payload},{crc:04X}\n"


def next_seq(seq: int) -> int:
    value = (seq + 1) & 0xFFFF
    return 1 if value == 0 else value


def validate_sender_config(*, hz: float, ttl_ms: int, period_s: float | None = None, mode: str | None = None) -> None:
    if hz <= 0:
        raise ValueError("hz must be > 0")
    if ttl_ms <= 0 or ttl_ms > 65535:
        raise ValueError("ttl_ms must be in 1..65535")
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
    ttl_ms: int = 500
    seq: int = 1

    def __post_init__(self):
        self.ser = None
        validate_sender_config(hz=1.0, ttl_ms=self.ttl_ms)

    def connect(self) -> None:
        self.ser = open_serial(self.port, self.baud)

    def send_targets(self, rpm_l: float, rpm_r: float, *, timestamp_ms: int | None = None) -> str:
        if self.ser is None:
            raise RuntimeError("serial port is not connected")
        now_ms = int(time.time() * 1000) if timestamp_ms is None else timestamp_ms
        packet = format_packet(self.seq, now_ms, rpm_l, rpm_r, self.ttl_ms)
        self.ser.write(packet.encode("ascii"))
        self.seq = next_seq(self.seq)
        return packet

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
    parser.add_argument("--ttl-ms", type=int, default=500, help="Packet TTL in milliseconds")
    parser.add_argument("--mode", choices=["constant", "step"], default="step", help="Command profile mode")
    parser.add_argument("--left", type=float, default=30.0, help="Constant left RPM")
    parser.add_argument("--right", type=float, default=30.0, help="Constant right RPM")
    parser.add_argument("--low", type=float, default=20.0, help="Step low RPM")
    parser.add_argument("--high", type=float, default=60.0, help="Step high RPM")
    parser.add_argument("--period-s", type=float, default=5.0, help="Step half-period in seconds")
    parser.add_argument("--duration-s", type=float, default=0.0, help="Run duration; 0 means forever")
    args = parser.parse_args()

    try:
        validate_sender_config(hz=args.hz, ttl_ms=args.ttl_ms, period_s=args.period_s, mode=args.mode)
    except ValueError as exc:
        print(f"--{str(exc)}", file=sys.stderr)
        return 2

    period_s = 1.0 / args.hz
    start = time.monotonic()
    next_tick = start
    sender = UartPacketSender(port=args.port, baud=args.baud, ttl_ms=args.ttl_ms)

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
                sent_seq = sender.seq
                packet = sender.send_targets(rpm_l, rpm_r)

                if sent_seq % int(max(1, args.hz)) == 0:
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
