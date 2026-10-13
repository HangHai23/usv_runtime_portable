"""Pure calculations for delay compensation and forward-only goal control."""
from __future__ import annotations

import math


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def goal_center(ends) -> tuple[float, float]:
    if not isinstance(ends, list) or len(ends) != 2:
        raise ValueError("goal ends must contain two points")
    values = [float(v) for point in ends for v in point]
    if len(values) != 4 or not all(math.isfinite(v) for v in values):
        raise ValueError("goal endpoints must be finite 2D points")
    return (values[0] + values[2]) / 2.0, (values[1] + values[3]) / 2.0


def predict_pose(x, y, heading, vx, vy, yaw_rate, delay):
    return (
        x + vx * delay,
        y + vy * delay,
        wrap_angle(heading + yaw_rate * delay),
    )


def compute_forward_only_command(
    x,
    y,
    heading,
    target_x,
    target_y,
    yaw_rate,
    arrival_radius,
    cruise,
    heading_kp,
    maximum_yaw_rate,
    yaw_rate_kp,
    maximum_steering,
    slowdown_angle,
):
    dx, dy = target_x - x, target_y - y
    distance = math.hypot(dx, dy)
    if distance <= arrival_radius:
        return 0.0, 0.0, distance, 0.0, 0.0
    error = wrap_angle(math.atan2(dy, dx) - heading)
    desired_rate = clamp(heading_kp * error, -maximum_yaw_rate, maximum_yaw_rate)
    steering = clamp(
        yaw_rate_kp * (desired_rate - yaw_rate),
        -maximum_steering,
        maximum_steering,
    )
    throttle = cruise * clamp(1.0 - abs(error) / slowdown_angle, 0.0, 1.0)
    # At large heading error throttle becomes zero: one motor advances and the
    # other remains stopped. Negative commands are never emitted.
    left = clamp(throttle - steering, 0.0, 100.0)
    right = clamp(throttle + steering, 0.0, 100.0)
    return left, right, distance, error, desired_rate


class TurnPhase:
    """Hysteresis prevents resuming forward thrust while still rotating quickly."""
    def __init__(self):
        self.aligning = False

    def reset(self):
        self.aligning = False

    def update(self, error, yaw_rate, c):
        if abs(error) >= c['turn_enter_angle_rad']:
            self.aligning = True
        elif (abs(error) <= c['turn_exit_angle_rad'] and
              abs(yaw_rate) <= c['turn_exit_yaw_rate_rad_s']):
            self.aligning = False
        return self.aligning


def route_command(state, start, target, yaw_rate, c, phase=None, guidance=None):
    """Straight segment lookahead, yaw-rate feedback and PWM model inversion."""
    x, y, heading, speed = state
    dx, dy = target[0] - start[0], target[1] - start[1]
    length = math.hypot(dx, dy)
    ux, uy = (dx / length, dy / length) if length > 1e-6 else (1.0, 0.0)
    along = (x - start[0]) * ux + (y - start[1]) * uy
    cross = -(x - start[0]) * uy + (y - start[1]) * ux
    aim = clamp(along + c['lookahead_m'], 0.0, length)
    ax, ay = start[0] + ux * aim, start[1] + uy * aim
    if guidance is not None:
        ax, ay = guidance['reference_xy_m']
        cross, along = guidance['cross_track_m'], guidance['along_track_m']
    error = wrap_angle(math.atan2(ay - y, ax - x) - heading)
    distance = math.hypot(target[0] - x, target[1] - y)
    # Anticipate the remaining rotation before neutral differential takes effect.
    aligning = phase.update(error, yaw_rate, c) if phase is not None else False
    # Extra anticipation is for the turn-around phase; retain the validated
    # rate loop in cruise instead of making small path corrections sluggish.
    preview = c.get('yaw_brake_preview_sec', 0.0) if aligning else 0.0
    braking_error = error - preview * yaw_rate
    desired_rate = clamp(c['heading_kp'] * braking_error, -c['maximum_yaw_rate_rad_s'], c['maximum_yaw_rate_rad_s'])
    alignment_rate = c.get('goal_alignment_yaw_rate_rad_s', 0.0)
    desired_rate = clamp(desired_rate + alignment_rate, -c['maximum_yaw_rate_rad_s'], c['maximum_yaw_rate_rad_s'])
    desired_speed = c['cruise_speed_m_s'] * clamp(distance / c['slowdown_distance_m'], 0, 1)
    if c.get('goal_runthrough', False):
        desired_speed = c['cruise_speed_m_s']
    if guidance is not None:
        # Slow for end-of-route only, not at every waypoint or a nearby end
        # reached prematurely by a loop. Curvature imposes a feasibility cap.
        desired_speed = c['cruise_speed_m_s'] * clamp(guidance['remaining_path_m']/c['slowdown_distance_m'],0,1)
        curvature = guidance['curvature_abs_m_inv']
        if curvature > 1e-6:
            desired_speed = min(desired_speed,
                c['maximum_yaw_rate_rad_s']*c['route_curve_speed_factor']/curvature,
                math.sqrt(c['route_lateral_acceleration_m_s2']/curvature))
    desired_speed *= max(0.0, math.cos(error)) ** 2
    if aligning:
        desired_speed = min(desired_speed, c['turn_speed_cap_m_s'])
    common_us = max(0.0, (desired_speed + c['speed_feedback_gain'] * (desired_speed - speed)) / c['speed_gain_m_s_per_us'])
    difference_us = (desired_rate + c['yaw_feedback_gain'] * (desired_rate - yaw_rate)) / c['turn_gain_rad_s_per_us']
    difference_us *= c['motor_turn_sign']
    difference_us = clamp(difference_us, -c['maximum_differential_us'], c['maximum_differential_us'])
    left_us = max(0.0, common_us - difference_us / 2)
    right_us = max(0.0, common_us + difference_us / 2)
    signed_output = c.get('bidirectional_enabled', False) and not c.get('command_forward_only', False)
    if signed_output:
        left_us, right_us = common_us-difference_us/2, common_us+difference_us/2
    turn_blend = 0.0
    if aligning and c.get('single_motor_turn_enabled', False) and not signed_output:
        turn_blend = clamp((abs(error)-c['turn_exit_angle_rad']) /
                           (c['turn_enter_angle_rad']-c['turn_exit_angle_rad']), 0., 1.)
        turn_blend = turn_blend*turn_blend*(3-2*turn_blend)
    single_motor = turn_blend >= 1.0
    if turn_blend > 0.0:
        # Keep the full requested differential with one motor stopped. The rate
        # feedback may select the opposite side to brake an existing rotation.
        magnitude = min(abs(difference_us),
                        2 * min(c['turn_speed_cap_m_s'], c['cruise_speed_m_s']) / c['speed_gain_m_s_per_us'])
        pivot_left, pivot_right = (0., magnitude) if difference_us >= 0 else (magnitude, 0.)
        left_us = (1-turn_blend)*left_us + turn_blend*pivot_left
        right_us = (1-turn_blend)*right_us + turn_blend*pivot_right
    lower = -c['maximum_command_percent'] if signed_output else 0
    left = clamp(left_us / c['command_span_us'] * 100, lower, c['maximum_command_percent'])
    right = clamp(right_us / c['command_span_us'] * 100, lower, c['maximum_command_percent'])
    if single_motor:
        desired_speed = (left + right) / 200 * c['command_span_us'] * c['speed_gain_m_s_per_us']
    return left, right, dict(distance_m=distance, cross_track_m=cross, along_track_m=along,
                            heading_error_rad=error, desired_speed_m_s=desired_speed,
                            desired_yaw_rate_rad_s=desired_rate,
                            desired_differential_us=difference_us, reference_xy_m=[ax, ay],
                            motion_phase='aligning' if aligning else 'cruising',
                            single_motor_turn=single_motor,
                            single_motor_blend=turn_blend,
                            braking_heading_error_rad=braking_error)
