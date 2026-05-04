"""Vehicle layout in the hitch / pivot frame (cm, world axes aligned with state vector)."""

import math

import numpy as np


def body_centers_cm(
    q_pivot: np.ndarray,
    truck_len_cm: float,
    trailer_len_cm: float,
) -> tuple[tuple[float, float], tuple[float, float]]:
    """Return trailer center and truck center given hitch pivot (x,y,theta_truck,theta_trailer,...)."""
    x, y = float(q_pivot[0]), float(q_pivot[1])
    theta_t, theta_l = float(q_pivot[2]), float(q_pivot[3])
    tmx = x - (trailer_len_cm / 2.0) * math.cos(theta_l)
    tmy = y - (trailer_len_cm / 2.0) * math.sin(theta_l)
    trx = x + (truck_len_cm / 2.0) * math.cos(theta_t)
    tr_y = y + (truck_len_cm / 2.0) * math.sin(theta_t)
    return (tmx, tmy), (trx, tr_y)
