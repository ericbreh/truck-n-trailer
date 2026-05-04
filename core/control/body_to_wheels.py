from dataclasses import dataclass

from truck_n_trailer import params
from truck_n_trailer.kinematics import cm_s_to_rpm


@dataclass
class BodyToWheels:
    wheel_track_cm: float = params.WHEEL_TRACK_CM
    wheel_radius_cm: float = params.WHEEL_RADIUS_CM
    rpm_limit: float = 100.0
    invert_differential: bool = False

    # Internal velocity state — kept in sync with what we've commanded
    _v: float = 0.0
    _omega: float = 0.0

    def reset(self, v: float = 0.0, omega: float = 0.0) -> None:
        self._v = v
        self._omega = omega

    def convert(self, a: float, alpha: float, dt: float) -> tuple[float, float]:
        # Integrate accelerations → desired velocities
        self._v     += a     * dt
        self._omega += alpha * dt

        # Differential drive: split body velocity into per-wheel tangential speeds
        w = self.wheel_track_cm
        o = self._omega
        if self.invert_differential:
            v_l = self._v + (o * w) / 2.0
            v_r = self._v - (o * w) / 2.0
        else:
            v_l = self._v - (o * w) / 2.0
            v_r = self._v + (o * w) / 2.0

        rpm_l = cm_s_to_rpm(v_l, self.wheel_radius_cm)
        rpm_r = cm_s_to_rpm(v_r, self.wheel_radius_cm)

        # Hard clamp — never send the firmware an impossible target
        rpm_l = max(-self.rpm_limit, min(self.rpm_limit, rpm_l))
        rpm_r = max(-self.rpm_limit, min(self.rpm_limit, rpm_r))

        return rpm_l, rpm_r
