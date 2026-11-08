"""Phone joystick (x, y in [-1, 1]) -> base velocity (v m/s, w rad/s).

Reproduces the feel of xle_wheel_joystick_server_v4.py exactly: same rotation,
dead zone, response curve and left/right mixing, only expressed in physical units
so the joystick and navigation share one interface (DiffDriveBase.set_velocity).
"""

from __future__ import annotations

import math


def shape(x: float, y: float, js: dict) -> tuple[float, float]:
    """Rotation, dead zone and response curve, as in v4."""
    th = math.radians(js["rotate_degrees"])
    x, y = x * math.cos(th) - y * math.sin(th), x * math.sin(th) + y * math.cos(th)
    if abs(x) < js["deadzone"]:
        x = 0.0
    if abs(y) < js["deadzone"]:
        y = 0.0

    def curve(v: float) -> float:
        return math.copysign(abs(v) ** js["curve_exponent"], v)
    return curve(x), curve(y)


def joystick_to_velocity(x: float, y: float, cfg: dict) -> tuple[float, float]:
    """v4 mixing was left = (y+x)*cap, right = (y-x)*cap (speed units, before inversion)."""
    x = max(-1.0, min(1.0, float(x))) if math.isfinite(float(x)) else 0.0
    y = max(-1.0, min(1.0, float(y))) if math.isfinite(float(y)) else 0.0
    xc, yc = shape(x, y, cfg["joystick"])
    cap = float(cfg["wheels"]["max_units"])
    k = float(cfg["calibration"]["speed_units_per_mps"])
    b = float(cfg["geometry"]["track_width_m"])
    v = yc * cap / k
    w = -2.0 * xc * cap / (k * b)        # pushing right turns right = clockwise = negative w
    return v, w


def v4_wheel_targets(x: float, y: float, cfg: dict) -> tuple[float, float]:
    """The original v4 formula, kept only so tests can prove the new path is equivalent."""
    xc, yc = shape(x, y, cfg["joystick"])
    cap = float(cfg["wheels"]["max_units"])
    left, right = (yc + xc) * cap, (yc - xc) * cap
    if cfg["wheels"]["invert_left"]:
        left = -left
    if cfg["wheels"]["invert_right"]:
        right = -right
    return max(-cap, min(cap, left)), max(-cap, min(cap, right))
