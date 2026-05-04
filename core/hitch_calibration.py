"""POT hitch: piecewise linear map from raw reading to physical degrees."""

from dataclasses import dataclass

from truck_n_trailer import params


@dataclass
class HitchPotCalibration:
    meas_neg: float
    meas_zero: float
    meas_pos: float

    def raw_to_physical_deg(self, raw: float) -> float:
        a = float(raw)
        z = self.meas_zero
        if a <= z:
            denom = self.meas_neg - z
            if abs(denom) < 1e-9:
                return a
            t = (a - z) / denom
            return params.HITCH_REAL_ZERO + t * (params.HITCH_REAL_NEG_WORKING - params.HITCH_REAL_ZERO)
        denom = self.meas_pos - z
        if abs(denom) < 1e-9:
            return a
        t = (a - z) / denom
        return params.HITCH_REAL_ZERO + t * (params.HITCH_REAL_POS_WORKING - params.HITCH_REAL_ZERO)


def default_hitch_calibration() -> HitchPotCalibration:
    return HitchPotCalibration(
        meas_neg=params.HITCH_MEAS_NEG_WORKING,
        meas_zero=params.HITCH_MEAS_ZERO,
        meas_pos=params.HITCH_MEAS_POS_WORKING,
    )


def hitch_calibration_prompts() -> list[str]:
    return [
        (
            f"Move hitch to {params.HITCH_REAL_NEG_WORKING:+.1f} deg "
            "(truck rotated clockwise/right relative to trailer), then click OK."
        ),
        f"Move hitch to {params.HITCH_REAL_ZERO:+.1f} deg, then click OK.",
        (
            f"Move hitch to {params.HITCH_REAL_POS_WORKING:+.1f} deg "
            "(truck rotated counterclockwise/left relative to trailer), then click OK."
        ),
        (
            f"Move hitch back to {params.HITCH_REAL_ZERO:+.1f} deg, then click OK. "
            "This second zero reading is used to reduce hysteresis."
        ),
    ]


def fit_hitch_calibration(
    neg: float, zero_first: float, pos: float, zero_return: float
) -> HitchPotCalibration:
    zero = 0.5 * (float(zero_first) + float(zero_return))
    n, p = float(neg), float(pos)
    if not (n < zero < p):
        raise ValueError("Captured points are not ordered (need neg < zero < pos).")
    return HitchPotCalibration(meas_neg=n, meas_zero=zero, meas_pos=p)
