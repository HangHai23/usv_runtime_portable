"""Continuous navigation, mode arbitration input, delayed correction, raw logs."""
import json
import math
import signal
import time
import faulthandler
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from rcl_interfaces.msg import ParameterEvent
from std_msgs.msg import Bool, String
from usv_interfaces.msg import PropulsionCommand, PwmOutputState, CrsfChannels
from .control import goal_center, route_command, TurnPhase
from .estimation import DelayedEstimator, MapMemory, VisualConsensus, point
from .recording import Recorder

DEFAULTS = dict(
    goal_alignment_distance_m=0.2, goal_alignment_kp=0.8,
    goal_alignment_max_yaw_rate_rad_s=0.6,
    bidirectional_enabled=False,
    command_forward_only=False,
    goal_entry_depth_m=0.5, goal_post_margin_m=0.4,
    single_motor_turn_enabled=False,
    obstacle_clearance_m=0.8, obstacle_speed_cap_m_s=0.35, obstacle_preview_lifetime_sec=60.0,
    route_mode='goal', route_seed=42, map_smoothing_gain=0.1, map_update_tolerance_m=0.5,
    prediction_slow_after_sec=1.0, prediction_speed_cap_m_s=0.25,
    route_file='', route_start_tolerance_m=0.8, route_max_error_m=1.5,
    route_boundary_margin_m=0.5, route_max_prediction_sec=2.0,
    route_curve_speed_factor=0.7, route_lateral_acceleration_m_s2=0.12,
    observation_topic='/vision/global_observation', imu_topic='/imu/data_raw',
    command_topic='/navigation/test_command', status_topic='/navigation/status',
    goal_side='left', armed=False, require_mode=True, measurement_delay_sec=0.2,
    max_prediction_sec=10.0, imu_timeout=0.3, pwm_timeout=0.5, mode_timeout=0.3,
    output_rate=20.0, imu_yaw_rate_reversed=False, imu_bias_rad_s=0.0,
    position_correction_gain=0.65, heading_correction_gain=0.5,
    maximum_position_innovation_m=4.0, maximum_visual_speed_m_s=3.0,
    speed_gain_m_s_per_us=0.00415, turn_gain_rad_s_per_us=0.00335,
    speed_time_constant_sec=2.7, yaw_time_constant_sec=1.1,
    left_start_us=1190.0, right_start_us=1100.0, command_span_us=810.0,
    motor_turn_sign=1.0, cruise_speed_m_s=0.45, maximum_command_percent=100.0,
    maximum_differential_us=810.0, maximum_yaw_rate_rad_s=0.35,
    heading_kp=1.2, yaw_feedback_gain=0.6, speed_feedback_gain=0.25,
    lookahead_m=1.5, slowdown_distance_m=2.0, arrival_radius_m=0.6,
    output_slew_rate_percent_s=20.0,
    yaw_brake_preview_sec=0.5, turn_enter_angle_rad=0.8,
    turn_exit_angle_rad=0.25, turn_exit_yaw_rate_rad_s=0.15, turn_speed_cap_m_s=0.20,
    uncertainty_initial_m=0.2, uncertainty_growth_m_s=0.12,
    map_cache_file='~/Downloads/usv_runtime_state/map_cache.json',
    log_directory='~/Downloads/usv_navigation_logs',
)


class GoalNavigationNode(Node):
    def __init__(self):
        super().__init__('goal_navigation')
        for k, v in DEFAULTS.items():
            self.declare_parameter(k, v)
        self.c = {k: self.get_parameter(k).value for k in DEFAULTS}
        c = self.c
        if not 0<c['goal_entry_depth_m']<=2 or not 0<c['goal_post_margin_m']<2:
            raise ValueError('invalid goal entry depth or post margin')
        self.active_gate = None
        self.goal_stop_latched = False
        for key in ('obstacle_clearance_m','obstacle_speed_cap_m_s','obstacle_preview_lifetime_sec'):
            if not math.isfinite(c[key]) or c[key]<=0:
                raise ValueError(f'{key} must be positive and finite')
        self.obstacle_plan = None
        self.pending_obstacle_plan = None
        self.obstacle_plan_id = None
        if c['route_mode'] != 'goal' or c['route_file']:
            raise ValueError('Only direct goal entry is supported; remove route_file and use route_mode=goal')
        if not 0 < c['map_smoothing_gain'] <= 1 or c['map_update_tolerance_m'] <= 0:
            raise ValueError('invalid map smoothing parameters')
        if c['prediction_slow_after_sec'] <= 0 or c['prediction_speed_cap_m_s'] <= 0:
            raise ValueError('invalid prediction speed parameters')
        self.route = None
        for k in ('route_start_tolerance_m','route_max_error_m','route_max_prediction_sec',
                  'route_curve_speed_factor','route_lateral_acceleration_m_s2'):
            if c[k] <= 0: raise ValueError(f'{k} must be positive')
        if not 0 < c['route_curve_speed_factor'] <= 1 or c['route_boundary_margin_m'] < 0:
            raise ValueError('Invalid route safety parameters')
        for k, default in DEFAULTS.items():
            if type(default) is float and not math.isfinite(c[k]):
                raise ValueError(f'{k} must be finite')
        for k in ('max_prediction_sec', 'imu_timeout', 'pwm_timeout', 'mode_timeout', 'output_rate',
                  'speed_gain_m_s_per_us', 'turn_gain_rad_s_per_us', 'speed_time_constant_sec',
                  'yaw_time_constant_sec', 'command_span_us', 'lookahead_m', 'slowdown_distance_m',
                  'arrival_radius_m', 'maximum_visual_speed_m_s', 'maximum_position_innovation_m',
                  'output_slew_rate_percent_s', 'maximum_yaw_rate_rad_s', 'maximum_differential_us'):
            if c[k] <= 0:
                raise ValueError(f'{k} must be positive')
        if c['goal_side'] not in ('left', 'right') or c['motor_turn_sign'] not in (-1., 1.):
            raise ValueError('invalid goal side or motor turn sign')
        if not 0 <= c['measurement_delay_sec'] <= 2 or not 0 < c['maximum_command_percent'] <= 100:
            raise ValueError('invalid delay or output cap')
        for k in ('position_correction_gain', 'heading_correction_gain'):
            if not 0 < c[k] <= 1:
                raise ValueError(f'{k} must be in (0,1]')
        if not 0 <= c['yaw_brake_preview_sec'] <= 3:
            raise ValueError('yaw_brake_preview_sec must be in 0..3')
        if not 0 < c['turn_exit_angle_rad'] < c['turn_enter_angle_rad'] < math.pi:
            raise ValueError('require 0 < turn_exit_angle_rad < turn_enter_angle_rad < pi')
        if c['turn_exit_yaw_rate_rad_s'] <= 0:
            raise ValueError('turn_exit_yaw_rate_rad_s must be positive')
        if not 0 <= c['turn_speed_cap_m_s'] <= c['cruise_speed_m_s']:
            raise ValueError('turn_speed_cap_m_s must be between zero and cruise speed')
        self.rec = Recorder(c['log_directory'], c)
        if self.route:
            (self.rec.directory/'route.json').write_text(json.dumps(self.route.data,indent=2))
        self.map = MapMemory(c['map_smoothing_gain'], c['map_update_tolerance_m'])
        self.last_map = 0.0
        self.approach = None
        self.route_joined = False
        self.cache_path = Path(c['map_cache_file']).expanduser()
        try:
            self.cached = json.loads(self.cache_path.read_text())
        except (OSError, ValueError):
            self.cached = None
        self.est = DelayedEstimator(c['speed_time_constant_sec'])
        self.visual_consensus = VisualConsensus()
        self.last_imu = self.last_pwm = self.last_mode = self.last_visual = 0.0
        self.rate = self.raw_rate = 0.0
        self.auto = False
        self.stream_rearm_required = False
        self.pwm = None
        self.frame = -1
        self.previous_visual = None
        self.session = None
        self.fault = None
        self.start = None
        self.turn_phase = TurnPhase()
        self.arrived = False
        self.run_id = 0
        self.run_started = None
        self.running = False
        self.outputs = [0., 0.]
        self.last_tick = time.monotonic()
        self.counts = dict(accepted=0, rejected=0, duplicates=0)
        self.last_correction = {}
        self.command_pub = self.create_publisher(PropulsionCommand, c['command_topic'], 10)
        self.status_pub = self.create_publisher(String, c['status_topic'], 10)
        self.plan_reply_pub = self.create_publisher(String, '/navigation/goal_reply', 10)
        self.create_subscription(String, '/navigation/goal_request', self.on_goal_request, 10)
        self.create_subscription(String, c['observation_topic'], self.on_observation, 100)
        self.create_subscription(Imu, c['imu_topic'], self.on_imu, qos_profile_sensor_data)
        self.create_subscription(PwmOutputState, '/actuators/pwm_state', self.on_pwm, 100)
        self.create_subscription(Bool, '/control/automatic_mode', self.on_mode, 10)
        self.create_subscription(String, '/vision/udp_diagnostics', lambda m: self.rec.write('udp', raw=m.data), 200)
        self.create_subscription(PropulsionCommand, '/propulsion/command', lambda m: self.rec.message('arbitrated_command', m), 100)
        self.create_subscription(CrsfChannels, '/rc/channels', lambda m: self.rec.message('rc', m), qos_profile_sensor_data)
        self.create_subscription(ParameterEvent, '/parameter_events', lambda m: self.rec.message('parameter_event', m), 100)
        self.create_timer(1 / c['output_rate'], self.update)
        self.get_logger().info(f'Navigation v2 logs: {self.rec.directory}')

    def end_run(self, reason):
        if self.running:
            self.rec.write('run_end', run_id=self.run_id, reason=reason)
            self.rec.file.flush()
            summary = dict(run_id=self.run_id, reason=reason, start_xy_m=self.start,
                elapsed_sec=time.monotonic()-self.run_started, final_predicted_state=self.est.state,
                goal_side=self.c['goal_side'], map=self.map.snapshot(), counts=self.counts.copy())
            (self.rec.directory / f'run_{self.run_id:04d}.json').write_text(json.dumps(summary, indent=2))
            self.running = False

    def on_goal_request(self, msg):
        request_id=None
        try:
            if len(msg.data)>1000:raise ValueError('请求过大')
            r=json.loads(msg.data);request_id=r.get('request_id')
            if not isinstance(request_id,str) or len(request_id)>80:raise ValueError('无效请求')
            if self.auto or time.monotonic()-self.last_mode>self.c['mode_timeout']:
                raise ValueError('请先切手动并保持遥控链路在线')
            if r.get('goal_side') not in ('left','right'):raise ValueError('球门选择无效')
            self.c['goal_side']=r['goal_side'];self.active_gate=None;self.start=None;self.arrived=False
            self.rec.write('goal_selected',goal_side=self.c['goal_side'])
            reply=dict(ok=True,request_id=request_id,goal_side=self.c['goal_side'])
        except (ValueError,TypeError,AttributeError) as e:
            reply=dict(ok=False,request_id=request_id,error=str(e))
        self.plan_reply_pub.publish(String(data=json.dumps(reply)))


    def on_mode(self, msg):
        self.last_mode = time.monotonic()
        if not msg.data:
            self.stream_rearm_required = False
            if self.fault in ('goal_boundary_stop','goal_overshoot_stop'):
                self.rec.write('goal_fault_cleared_by_manual',reason=self.fault)
                self.fault = None
            self.goal_stop_latched = False
        if bool(msg.data) != self.auto:
            self.active_gate = None
            self.approach = None
            self.route_joined = False
            if self.route: self.route.reset()
            self.turn_phase.reset()
            self.rec.write('mode', automatic=bool(msg.data))
            self.outputs = [0., 0.]
            self.start = None
            self.arrived = False
            if not msg.data:
                self.end_run('manual_takeover')
            self.auto = bool(msg.data)

    def on_pwm(self, msg):
        self.rec.message('pwm', msg)
        self.advance(time.monotonic())
        self.pwm = msg
        self.last_pwm = time.monotonic()

    def effective_pwm(self, now):
        if self.pwm is None or now - self.last_pwm > self.c['pwm_timeout']:
            return [0., 0.]
        if self.pwm.emergency_stop_active or self.pwm.watchdog_active or self.pwm.dry_run:
            return [0., 0.]
        values = [float(self.pwm.pulse_width_us[0]) - self.c['left_start_us'],
                  float(self.pwm.pulse_width_us[1]) - self.c['right_start_us']]
        return values if self.c['bidirectional_enabled'] else [max(0., v) for v in values]

    def advance(self, now):
        target = sum(self.effective_pwm(now)) / 2 * self.c['speed_gain_m_s_per_us']
        rate = self.rate if now - self.last_imu <= self.c['imu_timeout'] else 0.0
        self.est.advance(now, rate, target)

    def on_imu(self, msg):
        raw = float(msg.angular_velocity.z)
        values = [raw, msg.angular_velocity.x, msg.angular_velocity.y,
                  msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
                  msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        if not all(math.isfinite(v) for v in values) or msg.angular_velocity_covariance[0] == -1:
            self.rec.write('invalid_imu')
            return
        self.rec.message('imu', msg)
        now = time.monotonic()
        self.advance(now)
        self.raw_rate = raw
        self.rate = (raw - self.c['imu_bias_rad_s']) * (-1 if self.c['imu_yaw_rate_reversed'] else 1)
        self.last_imu = now

    def on_observation(self, msg):
        now = time.monotonic()
        self.rec.write('observation', raw=msg.data)
        try:
            report = json.loads(msg.data)
            if report.get('reset_required'):
                self.fault = 'coordinate_or_session_reset'
                return
            obs = report.get('observation')
            if not isinstance(obs, dict):
                return
            session = report.get('session')
            if ((self.session is not None and session != self.session) or
                    (self.map.coord and obs['coord'] != self.map.coord)):
                self.reset_stream(session, obs['coord'])
            if self.map.coord is None and isinstance(self.cached, dict) and self.cached.get('coord') == obs['coord']:
                self.map.ingest(dict(coord=obs['coord'], field_valid=True,
                    size_m=self.cached['size_m'], goals=[dict(side=s, ends_m=e) for s, e in self.cached['goals'].items()]))
            changed = self.map.ingest(obs)
            if obs.get('field_valid') is True and self.map.size:
                self.last_map = now
            if changed:
                self.cache_path.parent.mkdir(parents=True, exist_ok=True)
                tmp = self.cache_path.with_suffix('.tmp')
                tmp.write_text(json.dumps(self.map.snapshot(), indent=2))
                tmp.replace(self.cache_path)
                self.rec.write('map', map=self.map.snapshot())
            if not report.get('valid') or self.map.size is None:
                return
            frame = obs['frame']
            if type(frame) is not int or frame <= self.frame:
                self.counts['duplicates'] += 1
                return
            xy = point(report['own_boat']['xy_m'])
            heading = float(report['own_boat']['heading_rad'])
            ms = obs['processed_unix_ms']
            if type(ms) is not int or not math.isfinite(heading):
                raise ValueError('invalid timestamp/heading')
            # Same-machine monotonic reception, not subtraction of unsynchronized wall clocks.
            received = float(report.get('received_monotonic', now))
            if not math.isfinite(received) or received > now or now - received > 2:
                raise ValueError('invalid local receipt timestamp')
            stamp = received - self.c['measurement_delay_sec']
            consistent=self.visual_consensus.observe(frame,ms,xy,heading,received,self.c['maximum_visual_speed_m_s'])
            # Recover only after independent, distinct, locally fresh visual
            # frames agree. A single jump can never reset the estimator.
            recover=(consistent and now-received<=.5 and self.est.state is not None and
                     now-self.last_visual>1.0)
            speed = None
            if self.previous_visual and not recover:
                px, py, pm = self.previous_visual
                dt = (ms - pm) / 1000
                if dt <= 0:
                    raise ValueError('nonmonotonic visual timestamp')
                if dt <= 3:
                    vx, vy = (xy[0]-px)/dt, (xy[1]-py)/dt
                    if math.hypot(vx, vy) > self.c['maximum_visual_speed_m_s']:
                        raise ValueError('visual velocity outlier')
                    speed = max(0., vx * math.cos(heading) + vy * math.sin(heading))
            self.advance(now)
            if recover:
                old=list(self.est.state)
                self.est.initialize([*xy,heading],stamp,now,self.rate,0.)
                correction=dict(accepted=True,reason='visual_relocalization',previous_prediction=old)
                self.publish()
                self.outputs=[0.,0.]
                self.end_run('visual_relocalization')
                self.start=None;self.active_gate=None;self.arrived=False;self.goal_stop_latched=False
                self.turn_phase.reset()
                self.stream_rearm_required=True
                self.rec.write('visual_relocalization',old_prediction=old,visual_pose=[*xy,heading])
            elif self.est.state is None:
                self.est.initialize([*xy, heading], stamp, now, self.rate, 0.)
                correction = dict(accepted=True, reason='initialize')
            else:
                correction = self.est.correct([*xy, heading], stamp,
                    self.c['position_correction_gain'], self.c['heading_correction_gain'], speed,
                    self.c['maximum_position_innovation_m'])
            self.last_correction = correction
            self.rec.write('visual', frame=frame, received_monotonic=received,
                measurement_monotonic=stamp, measurement_ms=ms, xy_m=xy, heading=heading,
                speed_m_s=speed, correction=correction)
            if not correction['accepted']:
                self.counts['rejected'] += 1
                return
            self.frame, self.session = frame, session
            self.previous_visual = (*xy, ms)
            self.last_visual = received
            self.counts['accepted'] += 1
        except (ValueError, TypeError, KeyError, AttributeError, IndexError, OverflowError) as e:
            self.counts['rejected'] += 1
            self.rec.write('rejected', reason=str(e))
        except OSError as e:
            self.fault = 'map_io_error'
            self.get_logger().error(str(e))

    def publish(self, left=0., right=0.):
        msg = PropulsionCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.channel_1_percent, msg.channel_2_percent = float(left), float(right)
        msg.source = f"goal_navigation:{self.c['goal_side']}"
        self.command_pub.publish(msg)

    def reset_stream(self, session, coord):
        """Recover observation reception, never silently resume an armed mission."""
        self.publish()
        self.active_gate = None
        self.goal_stop_latched = False
        self.end_run('sender_stream_changed')
        self.stream_rearm_required = True
        self.outputs = [0., 0.]
        self.start = None
        self.arrived = False
        self.approach = None
        self.route_joined = False
        self.turn_phase.reset()
        self.est = DelayedEstimator(self.c['speed_time_constant_sec'])
        self.visual_consensus = VisualConsensus()
        self.previous_visual = None
        self.frame = -1
        self.last_visual = 0.0
        self.session = session
        self.pending_obstacle_plan = None
        if self.obstacle_plan is not None:
            self.obstacle_plan = None
            self.obstacle_plan_id = None
            self.route = None
        if self.map.coord != coord:
            self.map = MapMemory(self.c['map_smoothing_gain'], self.c['map_update_tolerance_m'])
            self.last_map = 0.0
            if not self.c['route_file']:
                self.route = None
        if self.route:
            self.route.reset()
        self.rec.write('stream_recovered', session=session, coord=coord, manual_rearm_required=True)

    def update(self):
        from .direct_goal import update_goal
        update_goal(self)




def main(args=None):
    faulthandler.enable()
    faulthandler.register(signal.SIGUSR1, all_threads=True)
    rclpy.init(args=args)
    node = GoalNavigationNode()
    node.create_timer(1.0, lambda: faulthandler.dump_traceback_later(10.0))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        faulthandler.cancel_dump_traceback_later()
        # launch may forward SIGINT after the process group already received it.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.end_run('shutdown')
        node.rec.close()
        if rclpy.ok():
            node.publish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
