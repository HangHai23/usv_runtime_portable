"""Conversion and UART framing for the 16-channel PWM controller."""

from __future__ import annotations

import math


PERCENT_MIN = -100.0
PERCENT_MAX = 100.0
PULSE_MIN_US = 1000
PULSE_NEUTRAL_US = 1500
PULSE_MAX_US = 2000
ANGLE_MIN_DEG = 44.0
ANGLE_NEUTRAL_DEG = 87.0
ANGLE_MAX_DEG = 130.0


def clamp_percent(value: float) -> float:
    if not math.isfinite(value):
        raise ValueError("propulsion percentage must be finite")
    return max(PERCENT_MIN, min(PERCENT_MAX, float(value)))


def apply_output_limit_and_trim(
    requested_percent: float,
    output_limit_percent: float,
    trim_percent: float,
) -> float:
    """Scale demand by a global limit, apply channel gain trim, then limit safely.

    Trim is multiplicative so a zero demand always remains zero. The final clamp
    guarantees that channel trim can never exceed the configured global limit.
    """
    requested = clamp_percent(requested_percent)
    if not math.isfinite(output_limit_percent) or not 0.0 <= output_limit_percent <= 100.0:
        raise ValueError("output_limit_percent must be finite and in the range 0..100")
    if not math.isfinite(trim_percent) or not -100.0 <= trim_percent <= 100.0:
        raise ValueError("channel trim must be finite and in the range -100..100")
    scaled = requested * output_limit_percent / 100.0
    trimmed = scaled * (1.0 + trim_percent / 100.0)
    return max(-output_limit_percent, min(output_limit_percent, trimmed))


def percent_to_pulse_us(percent: float) -> int:
    """Map -100..100 percent linearly to 1000..2000 microseconds."""
    value = clamp_percent(percent)
    return round(PULSE_NEUTRAL_US + value * 5.0)


def percent_to_compensated_pulse_us(
    percent: float,
    output_limit_percent: float,
    forward_start_us: int,
    reverse_start_us: int,
) -> int:
    """Map applied demand with independent ESC start-deadband compensation.

    The configured global limit remains a hard pulse bound. At a 50% limit,
    for example, compensation can never command above 1750 us or below 1250 us.
    A start threshold outside that limited range is clipped to the limit, which
    means a very low global limit may intentionally be unable to start an ESC.
    """
    value = clamp_percent(percent)
    limit = float(output_limit_percent)
    if not math.isfinite(limit) or not 0.0 <= limit <= 100.0:
        raise ValueError("output_limit_percent must be finite and in the range 0..100")
    if not PULSE_NEUTRAL_US <= int(forward_start_us) <= PULSE_MAX_US:
        raise ValueError("forward_start_us must be in the range 1500..2000")
    if not PULSE_MIN_US <= int(reverse_start_us) <= PULSE_NEUTRAL_US:
        raise ValueError("reverse_start_us must be in the range 1000..1500")
    if value == 0.0 or limit == 0.0:
        return PULSE_NEUTRAL_US

    fraction = min(1.0, abs(value) / limit)
    if value > 0.0:
        upper_bound = PULSE_NEUTRAL_US + limit * 5.0
        start = min(float(forward_start_us), upper_bound)
        return round(start + fraction * (upper_bound - start))

    lower_bound = PULSE_NEUTRAL_US - limit * 5.0
    start = max(float(reverse_start_us), lower_bound)
    return round(start - fraction * (start - lower_bound))


def pulse_us_to_controller_angle(pulse_us: int) -> int:
    """Map measured PWM endpoints to the controller's integer angle command."""
    pulse = max(PULSE_MIN_US, min(PULSE_MAX_US, int(pulse_us)))
    if pulse <= PULSE_NEUTRAL_US:
        ratio = (pulse - PULSE_MIN_US) / (PULSE_NEUTRAL_US - PULSE_MIN_US)
        angle = ANGLE_MIN_DEG + ratio * (ANGLE_NEUTRAL_DEG - ANGLE_MIN_DEG)
    else:
        ratio = (pulse - PULSE_NEUTRAL_US) / (PULSE_MAX_US - PULSE_NEUTRAL_US)
        angle = ANGLE_NEUTRAL_DEG + ratio * (ANGLE_MAX_DEG - ANGLE_NEUTRAL_DEG)
    return round(angle)


def percent_to_controller_angle(percent: float) -> int:
    return pulse_us_to_controller_angle(percent_to_pulse_us(percent))


def build_uart_command(channel: int, angle_deg: int) -> bytes:
    """Build `$<channel><angle:03d>#` as required by the controller."""
    if not 1 <= channel <= 16:
        raise ValueError("PWM controller channel must be in the range 1..16")
    if not 0 <= angle_deg <= 999:
        raise ValueError("controller angle must be in the range 0..999")
    return bytes([ord("$"), 64 + channel]) + f"{angle_deg:03d}".encode("ascii") + bytes([ord("#")])
