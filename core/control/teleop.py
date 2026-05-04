"""Manual driving: discrete motion commands to wheel RPM targets."""


def motion_to_rpms(motion: str, base_rpm: float) -> tuple[float, float]:
    b = float(base_rpm)
    if motion == "FORWARD":
        return b, b
    if motion == "BACK":
        return -b, -b
    if motion == "LEFT":
        return -b, b
    if motion == "RIGHT":
        return b, -b
    return 0.0, 0.0
