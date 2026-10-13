"""Pure control functions shared by the ball-follow controller and tests."""

from __future__ import annotations

import math


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def pixel_center_to_bearing(
    normalized_center_x: float, horizontal_fov_rad: float, reversed_axis: bool = False
) -> float:
    """Convert normalized image x coordinate to pinhole-camera bearing angle."""
    x = clamp(float(normalized_center_x), 0.0, 1.0)
    tangent = (2.0 * x - 1.0) * math.tan(horizontal_fov_rad * 0.5)
    bearing = math.atan(tangent)
    return -bearing if reversed_axis else bearing


def desired_yaw_rate(
    bearing: float, deadband: float, gain: float, maximum_rate: float
) -> float:
    if abs(bearing) <= deadband:
        return 0.0
    effective = math.copysign(abs(bearing) - deadband, bearing)
    return clamp(gain * effective, -maximum_rate, maximum_rate)


def scheduled_throttle(
    cruise: float,
    absolute_bearing: float,
    slowdown_angle: float,
    area_ratio: float,
    stop_area_ratio: float,
) -> float:
    if area_ratio >= stop_area_ratio:
        return 0.0
    factor = clamp(1.0 - absolute_bearing / slowdown_angle, 0.0, 1.0)
    return cruise * factor


def mix_same_direction_or_pivot(throttle: float, steering: float) -> tuple[float, float]:
    """Differential mix without reversing one motor while translating.

    At zero throttle, opposite signs permit an in-place pivot. During forward or
    reverse translation, the inner motor may stop but cannot reverse direction.
    """
    if throttle > 0.0:
        return max(0.0, throttle + steering), max(0.0, throttle - steering)
    if throttle < 0.0:
        return min(0.0, throttle + steering), min(0.0, throttle - steering)
    return steering, -steering


def slew(current: float, target: float, maximum_delta: float) -> float:
    return current + clamp(target - current, -maximum_delta, maximum_delta)
