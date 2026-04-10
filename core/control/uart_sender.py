#!/usr/bin/env python3

import argparse
import sys
import time


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

    if args.hz <= 0:
        print("--hz must be > 0", file=sys.stderr)
        return 2
    if args.ttl_ms <= 0 or args.ttl_ms > 65535:
        print("--ttl-ms must be in 1..65535", file=sys.stderr)
        return 2
    if args.mode == "step" and args.period_s <= 0:
        print("--period-s must be > 0 for step mode", file=sys.stderr)
        return 2

    try:
        import serial  # type: ignore
    except ImportError:
        print("pyserial is required. Install with: pip install pyserial", file=sys.stderr)
        return 2

    period_s = 1.0 / args.hz
    seq = 1
    start = time.monotonic()
    next_tick = start

    try:
        with serial.Serial(args.port, args.baud, timeout=0) as ser:
            print(f"Sending on {args.port} @ {args.baud} baud, mode={args.mode}, rate={args.hz:.2f} Hz")
            while True:
                now = time.monotonic()
                elapsed = now - start
                if args.duration_s > 0 and elapsed >= args.duration_s:
                    break

                rpm_l, rpm_r = get_targets(args.mode, elapsed, args)
                timestamp_ms = int(time.time() * 1000)
                packet = format_packet(seq, timestamp_ms, rpm_l, rpm_r, args.ttl_ms)
                ser.write(packet.encode("ascii"))

                if seq % int(max(1, args.hz)) == 0:
                    print(packet.strip())

                seq = (seq + 1) & 0xFFFF
                if seq == 0:
                    seq = 1

                next_tick += period_s
                sleep_s = next_tick - time.monotonic()
                if sleep_s > 0:
                    time.sleep(sleep_s)
                else:
                    next_tick = time.monotonic()
    except KeyboardInterrupt:
        print("Stopped")
        return 0

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
