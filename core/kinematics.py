"""Shared wheel and angle helpers used by control, vision, and telemetry."""

import math


def wrap_angle_rad(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def rpm_to_cm_s(rpm: float, wheel_radius_cm: float) -> float:
    return (float(rpm) * (2.0 * math.pi * wheel_radius_cm)) / 60.0


def cm_s_to_rpm(v_cm_s: float, wheel_radius_cm: float) -> float:
    circumference = 2.0 * math.pi * wheel_radius_cm
    return (float(v_cm_s) / circumference) * 60.0



