"""Only this node sends automatic propulsion in the integrated mission stack."""
import json
import math
import time
import faulthandler
import signal

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from usv_interfaces.msg import BallTargetState, PropulsionCommand, PwmOutputState, CrsfChannels
from usv_goal_navigation.control import TurnPhase, route_command
from usv_ball_follow.control_math import alignment_active, desired_yaw_rate, clamp, slew
from usv_goal_navigation.navigation_node import DEFAULTS as NAV_DEFAULTS
from usv_goal_navigation.recording import Recorder
from .state_machine import MissionState, PossessionGuard, forward_floor, HeldVote, StuckRecovery, ManualEscape

DEFAULTS = dict(camera_confidence_threshold=0.7, camera_confidence_hold_sec=0.3,
    continuous_camera_search_enabled=False, search_forward_percent=25.0,
    search_steering_percent=15.0, search_turn_period_sec=4.0,
    camera_independent_capture_enabled=False,
    historical_ball_search_enabled=True, historical_ball_search_timeout_sec=10.0,
    historical_ball_prediction_sec=2.0, historical_ball_max_speed_m_s=.5,
    historical_ball_search_speed_m_s=.35,
    alignment_pivot_enabled=False, alignment_enter_angle_deg=15.0,
    alignment_exit_angle_deg=7.0, alignment_pivot_percent=35.0,
    alignment_bearing_kp=3.8, alignment_yaw_rate_kp=66.0,
    alignment_yaw_preview_sec=.45, alignment_maximum_yaw_rate=1.2,
    alignment_imu_yaw_rate_reversed=True,
    camera_detection_max_age_sec=0.25, camera_return_confidence=0.55,
    camera_loss_return_sec=0.8, handover_radius_m=3.0,
    ball_approach_stop_radius_m=1.0, ball_position_timeout_sec=2.0,
    target_update_smoothing_gain=0.3, command_timeout_sec=0.2, status_timeout_sec=0.3,
    accept_drone_held=True, drone_held_confirm_sec=1.0, capture_area_ratio=0.18, capture_hold_sec=1.0,
    ball_loss_confirm_sec=3.0, possession_timeout_sec=3.0,
    capture_min_forward_percent=20.0, capture_handover_timeout_sec=1.0,
    capture_creep_area_ratio=0.10, reacquire_speed_m_s=0.35,
    stuck_recovery_enabled=False, stuck_detection_sec=1.0, stuck_position_tolerance_m=0.10,
    stuck_min_forward_percent=10.0, stuck_position_max_age_sec=0.5,
    stuck_turn_percent=35.0, stuck_turn_duration_sec=1.0, stuck_turn_angle_rad=0.8,
    stuck_cooldown_sec=2.0, stuck_turn_direction='right',
    escape_rc_channel=0, escape_direction='right', escape_rotation_deg=45.0,
    escape_rotation_percent=35.0, escape_rotation_timeout_sec=5.0,
    escape_forward_percent=35.0, escape_arc_steering_percent=15.0, escape_arc_duration_sec=1.5,
    drone_vote_enabled=True, drone_vote_duration_sec=1.0, drone_vote_ratio=0.8,
    drone_vote_window_sec=2.0, drone_vote_min_frames=5,
    capture_topic='/mission/ball_captured', external_capture_override_enabled=False,
    log_directory='~/Downloads/usv_mission_logs',
    output_rate=20.0)


def decode(raw):
    value = json.loads(raw)
    json.dumps(value, allow_nan=False)
    if not isinstance(value, dict):
        raise ValueError('expected object')
    return value


def point(value):
    return isinstance(value, (list, tuple)) and len(value) == 2 and all(
        type(v) in (int, float) and math.isfinite(v) for v in value)


class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        # The launch loads current navigation YAML as a dictionary, so model
        # tuning has one source and automatically reaches approach control.
        for key, value in dict(NAV_DEFAULTS, **DEFAULTS).items():
            self.declare_parameter(key, value)
        self.c = {key: self.get_parameter(key).value for key in dict(NAV_DEFAULTS, **DEFAULTS)}
        c = self.c
        if not (0 < c['search_forward_percent'] <= 100 and
                0 < c['search_steering_percent'] < c['search_forward_percent'] and
                c['search_forward_percent']+c['search_steering_percent'] <= 100 and
                math.isfinite(c['search_turn_period_sec']) and c['search_turn_period_sec'] > 0):
            raise ValueError('invalid continuous search power/period')
        for key in ('historical_ball_search_timeout_sec','historical_ball_prediction_sec',
                    'historical_ball_max_speed_m_s','historical_ball_search_speed_m_s'):
            if not math.isfinite(c[key]) or c[key] <= 0:
                raise ValueError(f'{key} must be positive and finite')
        if not (0 < c['alignment_exit_angle_deg'] < c['alignment_enter_angle_deg'] <= 180 and
                0 < c['alignment_pivot_percent'] <= 100):
            raise ValueError('invalid alignment angles/power')
        for key in ('camera_confidence_hold_sec', 'camera_detection_max_age_sec',
                    'camera_loss_return_sec', 'handover_radius_m', 'ball_approach_stop_radius_m',
                    'ball_position_timeout_sec', 'command_timeout_sec', 'status_timeout_sec',
                    'drone_held_confirm_sec', 'capture_hold_sec', 'ball_loss_confirm_sec',
                    'possession_timeout_sec', 'capture_handover_timeout_sec', 'reacquire_speed_m_s', 'output_rate', 'speed_gain_m_s_per_us',
                    'turn_gain_rad_s_per_us', 'command_span_us', 'lookahead_m',
                    'output_slew_rate_percent_s'):
            if not math.isfinite(c[key]) or c[key] <= 0:
                raise ValueError(f'{key} must be positive and finite')
        if not 0 <= c['camera_return_confidence'] < c['camera_confidence_threshold'] <= 1:
            raise ValueError('require return confidence < handover confidence in 0..1')
        if not 0 < c['target_update_smoothing_gain'] <= 1:
            raise ValueError('invalid ball smoothing gain')
        if not 0 < c['capture_area_ratio'] < 1:
            raise ValueError('capture_area_ratio must be in (0,1)')
        if not 0 < c['capture_creep_area_ratio'] <= c['capture_area_ratio']:
            raise ValueError('capture creep area must be positive and <= capture area')
        if not math.isfinite(c['capture_min_forward_percent']) or not 0 <= c['capture_min_forward_percent'] <= 100:
            raise ValueError('capture_min_forward_percent must be in 0..100')
        if not c['ball_approach_stop_radius_m'] < c['handover_radius_m']:
            raise ValueError('stop radius must be less than handover radius')
        self.rec = Recorder(c['log_directory'], c)
        self.machine = MissionState()
        self.possession_guard = PossessionGuard()
        self.held_vote = HeldVote()
        self.stuck = StuckRecovery()
        self.escape = ManualEscape()
        self.escape_rc_low_seen = False
        self.escape_rc_high = False
        self.escape_rc_frame = None
        for key in ('escape_rotation_deg', 'escape_rotation_percent', 'escape_rotation_timeout_sec',
                    'escape_forward_percent', 'escape_arc_duration_sec'):
            if not math.isfinite(c[key]) or c[key] <= 0:
                raise ValueError(f'{key} must be positive and finite')
        if (c['escape_direction'] not in ('left', 'right') or
                c['escape_rotation_percent'] > 100 or c['escape_forward_percent'] > 100 or
                not math.isfinite(c['escape_arc_steering_percent']) or
                not 0 <= c['escape_arc_steering_percent'] <= c['escape_forward_percent'] or
                type(c['escape_rc_channel']) is not int or c['escape_rc_channel'] not in (0, 6, 7)):
            raise ValueError('invalid manual escape parameters; RC channel may be 0(disabled),6,7')
        self.stuck_position = None
        self.stuck_position_at = None
        self.stuck_yaw_rate = 0.
        for key in ('stuck_detection_sec', 'stuck_position_tolerance_m', 'stuck_min_forward_percent',
                    'stuck_position_max_age_sec', 'stuck_turn_percent', 'stuck_turn_duration_sec',
                    'stuck_turn_angle_rad', 'stuck_cooldown_sec'):
            if not math.isfinite(c[key]) or c[key] <= 0:
                raise ValueError(f'{key} must be positive and finite')
        if c['stuck_turn_percent'] > 100 or c['stuck_min_forward_percent'] > 100 or c['stuck_turn_direction'] not in ('left', 'right'):
            raise ValueError('invalid stuck recovery power/direction')
        if (not 0 < c['drone_vote_ratio'] <= 1 or
                not 0 < c['drone_vote_duration_sec'] < c['drone_vote_window_sec'] or
                not math.isfinite(c['drone_vote_window_sec']) or
                type(c['drone_vote_min_frames']) is not int or c['drone_vote_min_frames'] < 2):
            raise ValueError('invalid drone vote ratio/duration/window/minimum frames')
        self.auto = False
        self.mode_at = self.imu_at = self.pwm_at = self.nav_at = 0.0
        self.nav = {}
        self.relocation_key = None
        self.pwm = None
        self.target = None
        self.target_at = self.camera_good_since = self.camera_last_good = 0.0
        self.ball = None
        self.ball_at = 0.0
        self.ball_velocity = [0.,0.]
        self.guidance_ball = None
        self.history_search = False
        self.search_started = None
        self.identity = None
        self.frame = -1
        self.held_since = self.held_at = 0.0
        self.joint_since = 0.0
        self.drone_possession, self.possession_at = 'unknown', 0.0
        self.capture_high = False
        self.candidates = {}
        self.start = None
        self.outputs = [0., 0.]
        self.alignment = False
        self.capture_bridge_outputs = [0., 0.]
        self.phase = TurnPhase()
        self.tick_at = self.stage_at = time.monotonic()
        self.run_id = 0
        self.last_block = None
        self.command_pub = self.create_publisher(PropulsionCommand, '/propulsion/automatic_command', 10)
        self.enable_pub = self.create_publisher(Bool, '/mission/navigation_enabled', 10)
        self.visual_enable_pub = self.create_publisher(Bool, '/mission/visual_enabled', 10)
        self.status_pub = self.create_publisher(String, '/mission/status', 10)
        self.goal_pub = self.create_publisher(String, '/mission/navigation_goal_request', 10)
        self.reply_pub = self.create_publisher(String, '/navigation/goal_reply', 10)
        self.create_subscription(String, '/navigation/goal_request', self.on_goal, 10)
        self.create_subscription(String, '/navigation/status', self.on_navigation, 10)
        self.create_subscription(String, '/vision/global_observation', self.on_observation, 100)
        self.create_subscription(BallTargetState, '/ball/target_state', self.on_target, 10)
        self.create_subscription(Bool, '/control/automatic_mode', self.on_mode, 10)
        self.create_subscription(Bool, c['capture_topic'], self.on_capture, 10)
        self.create_subscription(Imu, c['imu_topic'], self.on_imu, qos_profile_sensor_data)
        self.create_subscription(PwmOutputState, '/actuators/pwm_state', self.on_pwm, 10)
        self.create_subscription(CrsfChannels, '/rc/channels', self.on_escape_rc, qos_profile_sensor_data)
        for key, topic in (('visual', '/mission/visual_command'), ('goal', '/mission/navigation_command')):
            self.create_subscription(PropulsionCommand, topic,
                lambda msg, kind=key: self.on_candidate(kind, msg), 10)
        self.create_service(Trigger, '/mission/confirm_capture', self.confirm_capture)
        self.create_service(Trigger, '/mission/reset', self.reset)
        self.create_service(Trigger, '/mission/escape_boundary', self.escape_boundary)
        self.create_timer(1 / c['output_rate'], self.update)
        self.create_timer(5.0, self.health_summary)
        self.get_logger().info(f'Mission logs: {self.rec.directory}; confidence threshold={c["camera_confidence_threshold"]}')

    def health_summary(self):
        now = time.monotonic()
        imu_age = f'{now-self.imu_at:.3f}s' if self.imu_at else '未收到'
        self.get_logger().info(f'Mission heartbeat: stage={self.machine.stage}; '
            f'block={self.last_block or "none"}; IMU={imu_age}; '
            f'output=[{self.outputs[0]:.1f}, {self.outputs[1]:.1f}]%')

    def on_mode(self, msg):
        if not msg.data:
            self.held_vote.clear()
            self.stuck.cancel()
            self.escape.cancel('manual_takeover')
        self.auto = bool(msg.data)
        self.mode_at = time.monotonic()

    def on_imu(self, msg):
        if math.isfinite(msg.angular_velocity.z) and msg.angular_velocity_covariance[0] != -1:
            self.imu_at = time.monotonic()
            self.stuck_yaw_rate = float(msg.angular_velocity.z)

    def on_pwm(self, msg):
        self.pwm, self.pwm_at = msg, time.monotonic()
        self.rec.message('pwm', msg)

    def request_escape(self):
        now, c = time.monotonic(), self.c
        if (not self.auto or now-self.mode_at > c['mode_timeout'] or self.machine.rearm_required or
                self.machine.stage == 'complete' or not c['bidirectional_enabled'] or
                now-self.imu_at > c['imu_timeout'] or self.pwm is None or
                now-self.pwm_at > c['pwm_timeout'] or self.pwm.emergency_stop_active or self.pwm.watchdog_active):
            return False, '需要自动模式、双向电调、新鲜IMU/PWM，且已解除急停；故障/完成状态拒绝触发'
        if not self.escape.trigger(now):
            return False, '正在脱困，重复按键不会重置动作'
        self.rec.write('manual_escape_requested', stage=self.machine.stage, direction=c['escape_direction'],
                       rotation_deg=c['escape_rotation_deg'])
        return True, '开始主动脱困：旋转后前进转弯，再恢复任务'

    def escape_boundary(self, request, response):
        response.success, response.message = self.request_escape()
        return response

    def on_escape_rc(self, msg):
        channel = self.c['escape_rc_channel']
        if not channel:
            return
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        age = self.get_clock().now().nanoseconds/1e9-stamp
        if not msg.connected or not 0 <= age <= self.c['mode_timeout']:
            self.escape_rc_low_seen = False
            self.escape_rc_high = False
            self.escape_rc_frame = None
            return
        if msg.valid_frame_count == self.escape_rc_frame:
            return
        self.escape_rc_frame = msg.valid_frame_count
        high = msg.microseconds[channel-1] > 1500
        if not high:
            self.escape_rc_low_seen = True
        elif self.escape_rc_low_seen and not self.escape_rc_high:
            success, reason = self.request_escape()
            self.escape_rc_low_seen = False
            if not success:
                self.get_logger().warning(f'Escape key rejected: {reason}')
        self.escape_rc_high = high

    def on_navigation(self, msg):
        try:
            self.nav, self.nav_at = decode(msg.data), time.monotonic()
            if (self.nav.get('correction') or {}).get('reason') == 'visual_relocalization':
                key = ((self.nav.get('map') or {}).get('coord'), self.nav.get('frame'))
                if key != self.relocation_key:
                    self.relocation_key = key
                    self.machine.stream_changed()
                    self.rec.write('position_relocalized_wait_manual', frame=self.nav.get('frame'))
        except (ValueError, TypeError):
            return

    def on_candidate(self, kind, msg):
        if all(math.isfinite(float(v)) for v in (msg.channel_1_percent, msg.channel_2_percent)):
            self.candidates[kind] = (time.monotonic(), msg)
            self.rec.message(kind + '_candidate', msg)

    def on_goal(self, msg):
        # The goal node receives an internal enable signal, so checking its
        # mode alone would incorrectly allow goal changes during acquisition.
        try:
            request = decode(msg.data)
            if self.auto or time.monotonic() - self.mode_at > self.c['mode_timeout']:
                raise ValueError('请先切手动并保持遥控链路在线')
            self.goal_pub.publish(msg)
        except (ValueError, TypeError) as exc:
            try:
                request = decode(msg.data)
            except (ValueError, TypeError):
                request = {}
            self.reply_pub.publish(String(data=json.dumps(dict(ok=False,
                request_id=request.get('request_id'), error=str(exc)), ensure_ascii=False)))

    def on_target(self, msg):
        now = time.monotonic()
        previous_at = self.target_at
        self.target, self.target_at = msg, now
        good = (msg.target_valid and msg.imu_valid and
                msg.tracking_state == BallTargetState.TRACKING and
                math.isfinite(float(msg.confidence)) and
                msg.confidence >= self.c['camera_confidence_threshold'] and
                0 <= msg.detection_age_sec <= self.c['camera_detection_max_age_sec'])
        if good:
            if not self.camera_good_since or now-previous_at > self.c['command_timeout_sec']:
                self.camera_good_since = now
        else:
            self.camera_good_since = 0.0
        if self.camera_usable(now):
            self.camera_last_good = now
        # Camera-only capture must also be able to maintain possession.
        # A fresh close-ball observation clears pending loss just like drone held.
        if (self.machine.has_ball and self.c['camera_independent_capture_enabled'] and good and
                math.isfinite(float(msg.area_ratio)) and msg.area_ratio >= self.c['capture_area_ratio']):
            self.possession_guard.observe('held', now)
        self.rec.message('target', msg)

    def camera_usable(self, now):
        t = self.target
        return (t is not None and now - self.target_at <= self.c['command_timeout_sec'] and
                t.target_valid and t.imu_valid and
                math.isfinite(float(t.confidence)) and math.isfinite(float(t.bearing_rad)) and
                t.confidence >= self.c['camera_return_confidence'] and
                0 <= t.detection_age_sec <= self.c['camera_detection_max_age_sec'])

    def on_observation(self, msg):
        now = time.monotonic()
        self.rec.write('observation', raw=msg.data)
        try:
            r = decode(msg.data)
            o = r.get('observation')
            if not isinstance(o, dict):
                return
            identity = (r.get('session'), o.get('coord'))
            frame, received = o.get('frame'), r.get('received_monotonic')
            if not all(isinstance(v, str) and v for v in identity):
                return
            if type(frame) is not int or type(received) not in (int, float) or not 0 <= now - received <= .5:
                return
            if identity != self.identity:
                if self.identity is not None:
                    self.machine.stream_changed()
                    self.camera_good_since = self.camera_last_good = 0.0
                    self.rec.write('stream_changed', identity=identity)
                self.identity, self.frame, self.ball, self.ball_at = identity, -1, None, 0.0
                self.ball_velocity = [0.,0.]
                self.held_since = self.held_at = 0.0
                self.held_vote.clear()
                self.stuck.cancel()
                self.stuck_position = self.stuck_position_at = None
            if frame <= self.frame:
                return
            self.frame = frame
            boat_xy = (r.get('own_boat') or {}).get('xy_m')
            if r.get('valid') and point(boat_xy):
                self.stuck_position, self.stuck_position_at = list(boat_xy), received
            else:
                self.stuck_position = self.stuck_position_at = None
            self.drone_possession = (r.get('own_boat') or {}).get('possession', 'unknown') if r.get('valid') else 'unknown'
            self.possession_at = received
            if (self.auto and now-self.mode_at <= self.c['mode_timeout'] and
                    self.machine.stage in ('approach_ball', 'visual_acquire', 'wait_global') and r.get('valid')):
                self.held_vote.observe(received, self.drone_possession == 'held', self.c['drone_vote_window_sec'])
            else:
                self.held_vote.clear()
            if self.machine.has_ball:
                self.possession_guard.observe(self.drone_possession, received)
            ball = o.get('ball') or {}
            if o.get('field_valid') and ball.get('state') == 'detected' and point(ball.get('xy_m')):
                xy = ball['xy_m']
                elapsed = received-self.ball_at
                if self.ball is not None and .03 <= elapsed <= self.c['ball_position_timeout_sec']:
                    velocity = [(xy[i]-self.ball[i])/elapsed for i in range(2)]
                    speed = math.hypot(*velocity)
                    scale = min(1.,self.c['historical_ball_max_speed_m_s']/max(speed,1e-9))
                    self.ball_velocity = [.7*a+.3*b*scale for a,b in zip(self.ball_velocity,velocity)]
                else:
                    self.ball_velocity = [0.,0.]
                alpha = self.c['target_update_smoothing_gain']
                self.ball = [a + alpha * (b - a) for a, b in zip(self.ball, xy)] if self.ball and now-self.ball_at <= self.c['ball_position_timeout_sec'] else list(xy)
                self.ball_at = received
            held = (r.get('valid') and (r.get('own_boat') or {}).get('possession') == 'held' and
                    self.machine.stage == 'visual_acquire')
            if held:
                if not self.held_since or now-self.held_at > .5:
                    self.held_since = now
                self.held_at = now
            else:
                self.held_since = self.held_at = 0.0
        except (ValueError, TypeError, AttributeError):
            return

    def on_capture(self, msg):
        high = bool(msg.data)
        if high and not self.capture_high:
            self.capture()
        self.capture_high = high

    def capture(self, joint=False, drone_only=False):
        if not joint and not self.c['external_capture_override_enabled']:
            return False
        now = time.monotonic()
        active = self.auto and now-self.mode_at <= self.c['mode_timeout']
        old = self.machine.stage
        ok = self.machine.confirm_capture(active, drone_only=drone_only)
        if ok:
            self.capture_bridge_outputs = forward_floor(self.outputs, self.c['capture_min_forward_percent'])
            self.possession_guard.start(now)
            self.transition(old, now)
            self.rec.write('capture_confirmed')
            # The next update selects bridge/goal output under the SAME safety checks.
            # Never insert an unconditional neutral command at capture confirmation.
        return ok

    def confirm_capture(self, request, response):
        response.success = self.capture()
        response.message = '拿球已确认，转入球门导航' if response.success else '外部覆写未启用，或未处于自动视觉拿球阶段'
        return response

    def reset(self, request, response):
        if self.auto or time.monotonic()-self.mode_at > self.c['mode_timeout']:
            response.success, response.message = False, '请先切手动并保持遥控链路在线'
        else:
            self.machine = MissionState()
            self.possession_guard = PossessionGuard()
            self.held_vote.clear()
            self.outputs = [0., 0.]
            self.stuck.cancel()
            self.held_since = self.held_at = 0.0
            self.joint_since = 0.0
            self.capture_high = False
            self.rec.write('mission_reset')
            response.success, response.message = True, '任务已重置'
        return response

    def transition(self, previous, now):
        if previous != self.machine.stage:
            self.rec.write('transition', previous=previous, stage=self.machine.stage, has_ball=self.machine.has_ball)
            self.get_logger().info(f'Mission: {previous} -> {self.machine.stage}')
            self.stage_at = now
            self.start = None
            if not (previous in ('approach_ball', 'visual_acquire') and
                    self.machine.stage in ('approach_ball', 'visual_acquire', 'deliver_goal')):
                self.outputs = [0., 0.]
            self.phase.reset()
            self.held_since = self.held_at = 0.0
            self.joint_since = 0.0
            if previous == 'manual' and self.machine.stage not in ('manual', 'wait_manual_rearm'):
                self.run_id += 1

    def publish(self, values):
        msg = PropulsionCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.channel_1_percent, msg.channel_2_percent = map(float, values)
        msg.source = 'mission:' + self.machine.stage
        self.command_pub.publish(msg)
        self.rec.message('automatic_command', msg)

    def reacquire(self, pose, now, dt):
        """Aim at the fresh global ball even inside the former camera waiting radius."""
        c = self.c
        target = self.guidance_ball if self.guidance_ball is not None else self.ball
        if self.history_search and math.dist(pose[:2],target)<.4:
            heading = pose[2]+.4*math.sin(now-self.ball_at)
            target = [pose[0]+math.cos(heading),pose[1]+math.sin(heading)]
        if math.dist(pose[:2], target) < .1:
            target = [pose[0]+math.cos(pose[2]), pose[1]+math.sin(pose[2])]
        config = dict(c, goal_runthrough=True, cruise_speed_m_s=c['reacquire_speed_m_s'])
        if self.history_search:
            config['cruise_speed_m_s'] = c['historical_ball_search_speed_m_s']
        left, right, data = route_command(pose, pose[:2], target, self.nav['yaw_rate_rad_s'], config, self.phase)
        if not self.history_search and math.dist(pose[:2], self.ball) <= c['ball_approach_stop_radius_m']:
            # Apply the floor BEFORE slew. Applying it only afterwards can keep
            # both motors pinned at X and erase a low-speed turning request.
            left, right = forward_floor((left, right), c['capture_min_forward_percent'])
        delta = c['output_slew_rate_percent_s']*dt
        outputs = [p+max(-delta, min(delta, q-p)) for p, q in zip(self.outputs, (left, right))]
        data['global_reacquire_active'] = True
        return outputs, data

    def update(self):
        now = time.monotonic()
        c, n = self.c, self.nav
        dt = max(0., min(.2, now-self.tick_at)); self.tick_at = now
        mode_fresh = now-self.mode_at <= c['mode_timeout']
        automatic = self.auto and mode_fresh
        pose, visual_age = n.get('predicted_pose'), n.get('visual_age_sec')
        pose_valid = (isinstance(pose, list) and len(pose) == 4 and
                      all(type(v) in (int, float) and math.isfinite(v) for v in pose))
        nav_fresh = now-self.nav_at <= c['status_timeout_sec']
        rate = n.get('yaw_rate_rad_s')
        localization = (nav_fresh and pose_valid and type(rate) in (int, float) and math.isfinite(rate) and
                        type(visual_age) in (int, float) and
                        0 <= visual_age + now-self.nav_at <= c['max_prediction_sec'] and
                        self.identity is not None and (n.get('map') or {}).get('coord') == self.identity[1])
        ball_ready = self.ball is not None and now-self.ball_at <= c['ball_position_timeout_sec']
        self.history_search = (not ball_ready and self.ball is not None and
            c['historical_ball_search_enabled'] and not self.machine.has_ball and
            0 <= now-self.ball_at <= c['historical_ball_search_timeout_sec'])
        self.guidance_ball = self.ball
        if self.history_search:
            horizon = min(now-self.ball_at,c['historical_ball_prediction_sec'])
            self.guidance_ball = [x+v*horizon for x,v in zip(self.ball,self.ball_velocity)]
            ball_ready = True
        distance = math.dist(pose[:2], self.guidance_ball) if pose_valid and ball_ready else None
        camera = self.camera_usable(now)
        camera_ready = (camera and self.camera_good_since > 0 and
                        now-self.camera_good_since >= c['camera_confidence_hold_sec'])
        camera_lost = now-max(self.camera_last_good, self.stage_at) > c['camera_loss_return_sec']
        if not camera and localization and ball_ready:
            camera_lost = True  # use global guidance now, not after a neutral waiting interval
        old = self.machine.stage
        loss_reason = None
        if self.machine.has_ball and self.machine.stage != 'complete':
            loss_reason = self.possession_guard.loss_reason(
                now, c['ball_loss_confirm_sec'], max(c['ball_loss_confirm_sec'], c['possession_timeout_sec']))
            if loss_reason:
                self.machine.lose_ball()
                self.held_vote.clear()
                self.candidates.clear()
                self.target = None
                self.target_at = self.camera_good_since = self.camera_last_good = 0.0
                camera = camera_ready = False
                self.rec.write('ball_lost', reason=loss_reason, possession=self.drone_possession,
                               possession_age_sec=now-self.possession_at, previous=old)
                self.get_logger().warning(f'Ball lost: {loss_reason}; cancel goal delivery and reacquire')
        # A missing mode heartbeat stops output but is not an actual manual
        # takeover and must not clear the stream-change rearm latch.
        if mode_fresh:
            camera_only = c['continuous_camera_search_enabled'] and camera_ready and not (localization and ball_ready)
            self.machine.step(automatic, localization or camera_only, ball_ready or camera_only,
                              camera_only or (distance is not None and distance <= c['handover_radius_m']),
                              camera_ready, camera_lost, nav_fresh and n.get('state') == 'arrived' and
                              self.machine.stage == 'deliver_goal' and self.nav_at > self.stage_at)
        self.transition(old, now)
        vote = self.held_vote.evidence(now)
        if (not loss_reason and c['accept_drone_held'] and c['drone_vote_enabled'] and automatic and
                vote['fresh'] and vote['count'] >= c['drone_vote_min_frames'] and
                vote['duration_sec'] > c['drone_vote_duration_sec'] and vote['held_ratio'] >= c['drone_vote_ratio']):
            if self.capture(joint=True, drone_only=True):
                self.rec.write('drone_vote_capture_evidence', **vote)
                self.held_vote.clear()
        camera_evidence = (automatic and self.machine.stage == 'visual_acquire' and
                 camera and self.target.confidence >= c['camera_confidence_threshold'] and
                 math.isfinite(float(self.target.area_ratio)) and self.target.area_ratio >= c['capture_area_ratio'])
        drone_evidence = (c['accept_drone_held'] and self.held_since > 0 and now-self.held_at <= .5)
        joint = camera_evidence and (c['camera_independent_capture_enabled'] or drone_evidence)
        if joint:
            if not self.joint_since:
                self.joint_since = now
            if (now-self.joint_since >= c['capture_hold_sec'] and
                    (c['camera_independent_capture_enabled'] or now-self.held_since >= c['drone_held_confirm_sec'])):
                self.rec.write('camera_capture_evidence' if c['camera_independent_capture_enabled'] else 'joint_capture_evidence', held_duration_sec=now-self.held_since if self.held_since else None,
                               area_ratio=float(self.target.area_ratio), confidence=float(self.target.confidence))
                self.capture(joint=True)
        else:
            self.joint_since = 0.0
        stage = self.machine.stage
        block = None
        if loss_reason and not (localization and ball_ready) and not c['continuous_camera_search_enabled']: block = loss_reason
        elif not automatic: block = 'manual_or_mode_unavailable'
        elif self.machine.rearm_required: block = 'stream_changed_wait_manual'
        elif now-self.imu_at > c['imu_timeout']: block = 'imu_timeout'
        elif self.pwm is None or now-self.pwm_at > c['pwm_timeout']: block = 'pwm_feedback_timeout'
        elif self.pwm.emergency_stop_active or self.pwm.watchdog_active: block = 'actuator_stopped'
        elif nav_fresh and (stage == 'deliver_goal' or not c['continuous_camera_search_enabled']) and n.get('state') not in ('manual_or_mode_unavailable', 'navigating', 'arrived', 'goal_entry_verifying'):
            block = n.get('state') or 'navigation_unavailable'
        # Enable the selected visual controller independently of whether its
        # first command has arrived; otherwise command_timeout disables its
        # own producer and prevents a stable handover.
        visual_enabled = automatic and stage == 'visual_acquire' and camera and not block
        if not block and stage == 'wait_global':
            block = 'global_position_unavailable' if not localization else 'ball_position_unavailable'
        outputs, data = [0., 0.], {}
        if not block and stage == 'approach_ball' and localization and ball_ready:
            if self.start is None:
                self.start = pose[:2]
                self.rec.write('approach_plan', run_id=self.run_id, start_xy_m=self.start,
                               target_xy_m=self.ball, coord=self.identity[1])
            if distance > c['ball_approach_stop_radius_m'] and not self.history_search:
                config = dict(c, goal_runthrough=False, slowdown_distance_m=max(2., c['handover_radius_m']))
                if visual_age > c['prediction_slow_after_sec']:
                    config['cruise_speed_m_s'] = min(c['cruise_speed_m_s'], c['prediction_speed_cap_m_s'])
                left, right, data = route_command(pose, self.start, self.ball, n['yaw_rate_rad_s'], config, self.phase)
                delta = c['output_slew_rate_percent_s']*dt
                outputs = [p+max(-delta, min(delta, q-p)) for p, q in zip(self.outputs, [left, right])]
                if data.get('single_motor_turn'):
                    outputs = [0. if q == 0 else p for p, q in zip(outputs, [left, right])]
            else:
                outputs, data = self.reacquire(pose, now, dt)
        elif not block and stage in ('visual_acquire', 'deliver_goal'):
            key = 'visual' if stage == 'visual_acquire' else 'goal'
            sample = self.candidates.get(key)
            if stage == 'visual_acquire' and not camera:
                if localization and ball_ready:
                    outputs, data = self.reacquire(pose, now, dt)
                else:
                    block = 'camera_unreliable'
            elif sample is None or sample[0] < self.stage_at or now-sample[0] > c['command_timeout_sec']:
                if (stage == 'deliver_goal' and localization and
                        0 <= now-self.stage_at <= c['capture_handover_timeout_sec'] and
                        (sample is None or sample[0] < self.stage_at)):
                    outputs = list(self.capture_bridge_outputs)
                    data['capture_bridge_active'] = True
                else:
                    if stage == 'visual_acquire' and localization and ball_ready:
                        outputs, data = self.reacquire(pose, now, dt)
                    else:
                        block = key + '_command_timeout'
            else:
                msg = sample[1]
                # Enforce again at the final mission output, even if a stale
                # or external goal candidate requests reverse propulsion.
                lower = -100. if c['bidirectional_enabled'] and stage != 'deliver_goal' else 0.
                outputs = [max(lower, min(100., float(v))) for v in (msg.channel_1_percent, msg.channel_2_percent)]
                data['selected_command_source'] = key
        if stage == 'deliver_goal' and not localization:
            block, outputs = 'global_position_unavailable', [0., 0.]
        capture_near = (stage == 'visual_acquire' and camera and self.target is not None and
                        math.isfinite(float(self.target.area_ratio)) and
                        (self.target.area_ratio >= c['capture_creep_area_ratio'] or
                         (distance is not None and distance <= c['ball_approach_stop_radius_m'])))
        keep_forward = not block and (stage == 'deliver_goal' or capture_near or
            (loss_reason and localization and ball_ready) or
            (data.get('global_reacquire_active') and distance is not None and
             distance <= c['ball_approach_stop_radius_m']))
        # The visual controller already owns its alignment loop. Never compute
        # a second turn command over it or over normal global approach guidance.
        # An extra global pivot is only for near-ball visual-loss reacquisition.
        bearing = None
        if (not camera and data.get('global_reacquire_active') and localization and ball_ready and
                distance is not None and .1 < distance <= c['ball_approach_stop_radius_m']):
            error = math.atan2(self.guidance_ball[1]-pose[1], self.guidance_ball[0]-pose[0])-pose[2]
            bearing = -math.atan2(math.sin(error), math.cos(error))
        self.alignment = (not block and not self.machine.has_ball and c['bidirectional_enabled'] and
            c['alignment_pivot_enabled'] and bearing is not None and alignment_active(
                self.alignment, bearing, math.radians(c['alignment_enter_angle_deg']),
                math.radians(c['alignment_exit_angle_deg'])))
        if self.alignment:
            rate = -self.stuck_yaw_rate if c['alignment_imu_yaw_rate_reversed'] else self.stuck_yaw_rate
            desired = desired_yaw_rate(bearing-c['alignment_yaw_preview_sec']*rate, .035,
                c['alignment_bearing_kp'], c['alignment_maximum_yaw_rate'])
            requested = clamp(c['alignment_yaw_rate_kp']*(desired-rate),
                -c['alignment_pivot_percent'], c['alignment_pivot_percent'])
            turn = slew((self.outputs[0]-self.outputs[1])/2, requested, c['output_slew_rate_percent_s']*dt)
            turn = clamp(turn, -c['alignment_pivot_percent'], c['alignment_pivot_percent'])
            outputs = [turn,-turn]
            keep_forward = False
        data['alignment_pivot_active'] = bool(self.alignment)
        data['alignment_bearing_rad'] = bearing
        if self.history_search:
            keep_forward = False  # bounded low-speed search must not be raised to the capture floor
        if data.get('selected_command_source') == 'visual':
            # Preserve the standalone controller's complete output, including
            # its zero-common pivot and braking. Only close, already centered
            # capture confirmation may retain the existing forward bridge.
            keep_forward = bool(capture_near and min(outputs) >= 0 and
                self.target.area_ratio >= c['capture_area_ratio'] and
                abs(self.target.bearing_rad) <= math.radians(c['alignment_exit_angle_deg']))
        data['historical_ball_search_active'] = bool(self.history_search and localization and not block)
        data['historical_ball_predicted_xy_m'] = self.guidance_ball if self.history_search else None
        data['historical_ball_velocity_m_s'] = list(self.ball_velocity)
        if keep_forward:
            outputs = forward_floor(outputs, c['capture_min_forward_percent'])
        data['capture_forward_floor_active'] = bool(keep_forward)
        data['capture_min_forward_percent'] = c['capture_min_forward_percent']
        # No global position is required for operator-authorized camera search.
        # Recheck every physical safety gate rather than clearing arbitrary faults.
        search_safe = (automatic and not self.machine.rearm_required and not self.machine.has_ball and
            stage in ('wait_global','approach_ball','visual_acquire') and
            now-self.imu_at <= c['imu_timeout'] and self.pwm is not None and
            now-self.pwm_at <= c['pwm_timeout'] and not self.pwm.emergency_stop_active and not self.pwm.watchdog_active)
        searching = (c['continuous_camera_search_enabled'] and search_safe and
            (block in ('global_position_unavailable','ball_position_unavailable','camera_unreliable','visual_command_timeout')
             or (not block and max(abs(v) for v in outputs)<.01 and not self.alignment)))
        if searching:
            if self.search_started is None:
                self.search_started = now
            sign = 1. if int((now-self.search_started)/c['search_turn_period_sec']) % 2 == 0 else -1.
            outputs = [c['search_forward_percent']+sign*c['search_steering_percent'],
                       c['search_forward_percent']-sign*c['search_steering_percent']]
            block = None
            data['capture_forward_floor_active'] = False
        else:
            self.search_started = None
        data['continuous_camera_search_active'] = bool(searching)
        escape_before = self.escape.phase
        escape_safe = (automatic and not self.machine.rearm_required and stage != 'complete' and
            c['bidirectional_enabled'] and now-self.imu_at <= c['imu_timeout'] and self.pwm is not None and
            now-self.pwm_at <= c['pwm_timeout'] and not self.pwm.emergency_stop_active and not self.pwm.watchdog_active)
        rate = -self.stuck_yaw_rate if c['imu_yaw_rate_reversed'] else self.stuck_yaw_rate
        escape_output = self.escape.step(now, rate, escape_safe, c)
        recovery = self.escape.phase != 'idle'
        if escape_output is not None:
            outputs = escape_output
            if recovery:
                block = None  # explicit operator motion needs IMU/RC/PWM, not visual acquisition
            data['capture_forward_floor_active'] = False
        if self.escape.phase != escape_before:
            self.rec.write('manual_escape_phase', previous=escape_before, phase=self.escape.phase,
                           reason=self.escape.reason, turned_deg=math.degrees(self.escape.turned))
        data['stuck_recovery_active'] = False
        data['escape_active'] = recovery
        data['escape_phase'] = self.escape.phase
        data['escape_turned_deg'] = math.degrees(self.escape.turned)
        data['escape_end_reason'] = self.escape.reason
        if self.escape.reason == 'rotation_timeout' and not recovery:
            block, outputs = 'escape_rotation_timeout_wait_operator', [0., 0.]
        data['stuck_position_age_sec'] = now-self.stuck_position_at if self.stuck_position_at is not None else None
        # Heartbeat enable is separate from RC mode; only goal delivery activates
        # the goal controller. Its commands never reach the arbiter directly.
        self.enable_pub.publish(Bool(data=automatic and stage == 'deliver_goal' and not self.machine.rearm_required))
        self.visual_enable_pub.publish(Bool(data=visual_enabled))
        if block != self.last_block:
            self.last_block = block
            if automatic and block:
                self.get_logger().warning(f'Mission output blocked: {block}; stage={stage}')
        self.outputs = outputs
        self.publish(outputs)
        status = dict(stage=stage, automatic=automatic, has_ball=self.machine.has_ball,
            run_id=self.run_id, blocked_reason=block, ball_xy_m=self.ball,
            reverse_propulsion_prohibited=stage == 'deliver_goal' and not recovery,
            imu_age_sec=now-self.imu_at if self.imu_at else None,
            mode_age_sec=now-self.mode_at if self.mode_at else None,
            navigation_status_age_sec=now-self.nav_at if self.nav_at else None,
            ball_age_sec=now-self.ball_at if self.ball_at else None, ball_distance_m=distance,
            coord=self.identity[1] if self.identity else None,
            camera_confidence=float(self.target.confidence) if self.target and math.isfinite(float(self.target.confidence)) else None,
            camera_confidence_threshold=c['camera_confidence_threshold'],
            capture_confirmation=('camera_or_drone_vote' if c['accept_drone_held'] and c['drone_vote_enabled'] else 'camera_only') if c['camera_independent_capture_enabled'] else (('joint_or_drone_vote' if c['drone_vote_enabled'] else 'drone_held_and_camera_area') if c['accept_drone_held'] else 'external_only'),
            drone_vote_count=vote['count'], drone_vote_held_ratio=vote['held_ratio'],
            drone_vote_duration_sec=vote['duration_sec'], drone_vote_fresh=vote['fresh'],
            drone_held_fresh=self.drone_possession == 'held' and now-self.possession_at <= .5,
            drone_possession=self.drone_possession,
            possession_age_sec=now-self.possession_at if self.possession_at else None,
            ball_loss_reason=loss_reason,
            capture_area_ratio_threshold=c['capture_area_ratio'],
            camera_area_ratio=float(self.target.area_ratio) if self.target and math.isfinite(float(self.target.area_ratio)) else None,
            capture_evidence_duration_sec=now-self.joint_since if self.joint_since else 0.,
            goal_side=n.get('goal_side'), predicted_pose=pose if pose_valid else None,
            target_xy_m=self.guidance_ball if stage in ('approach_ball', 'visual_acquire') else n.get('target_xy_m') if stage in ('deliver_goal', 'complete') else None,
            output_percent=outputs, **data)
        self.status_pub.publish(String(data=json.dumps(status, allow_nan=False)))
        self.rec.write('tick', **status)


def main(args=None):
    faulthandler.enable()
    faulthandler.register(signal.SIGUSR1, all_threads=True)
    rclpy.init(args=args)
    node = MissionManager()
    node.create_timer(1.0, lambda: faulthandler.dump_traceback_later(10.0))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        faulthandler.cancel_dump_traceback_later()
        if rclpy.ok():
            node.publish([0., 0.])
            node.enable_pub.publish(Bool(data=False))
            node.visual_enable_pub.publish(Bool(data=False))
        node.rec.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
