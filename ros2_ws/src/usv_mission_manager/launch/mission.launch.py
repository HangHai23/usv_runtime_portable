"""Load live configurations and keep automatic command publishers exclusive."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def live_config(package, filename):
    share = Path(get_package_share_directory(package))
    for parent in share.parents:
        source = parent / 'src' / package / 'config' / filename
        if source.is_file():
            return source
    return share / 'config' / filename


def settings(package, filename, node):
    path = live_config(package, filename)
    return path, yaml.safe_load(path.read_text())[node]['ros__parameters']


def include(package, filename, **args):
    path = Path(get_package_share_directory(package)) / 'launch' / filename
    # Includes otherwise share launch configuration scope: a child's `port`
    # can overwrite the IMU serial port with the dashboard TCP port.
    return GroupAction(scoped=True, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(path)), launch_arguments=args.items())])


def generate_launch_description():
    nav_path, nav = settings('usv_goal_navigation', 'goal_navigation.yaml', 'goal_navigation')
    mission_path, mission = settings('usv_mission_manager', 'mission.yaml', 'mission_manager')
    target_path, target = settings('usv_ball_follow', 'target_estimator.yaml', 'target_state_estimator')
    follow_path, follow = settings('usv_ball_follow', 'ball_follow_controller.yaml', 'ball_follow_controller')
    # Read the current navigation model on EVERY launch; no copied stale gains.
    for key in ('speed_gain_m_s_per_us', 'turn_gain_rad_s_per_us', 'command_span_us', 'bidirectional_enabled'):
        follow[key] = nav[key]
    follow['navigation_model_enabled'] = True
    follow['output_topic'] = '/mission/visual_command'
    follow['enable_topic'] = '/mission/visual_enabled'
    if mission['capture_area_ratio'] >= follow['stop_area_ratio']:
        raise ValueError('capture_area_ratio必须小于追球stop_area_ratio，避免到达停动力距离后仍无法确认拿球')
    goal = dict(nav, command_topic='/mission/navigation_command', armed=True, require_mode=True)
    # This controller is enabled only after capture: retain signed PWM sensing
    # for the bidirectional hardware but disallow reversing while carrying ball.
    goal['command_forward_only'] = True
    goal['goal_side'] = LaunchConfiguration('goal_side')
    manager = dict(nav, **mission)
    # One set of alignment thresholds for camera and global reacquisition.
    for key in ('alignment_pivot_enabled', 'alignment_enter_angle_deg',
                'alignment_exit_angle_deg', 'alignment_pivot_percent'):
        manager[key] = follow[key]
    for key, source in (('alignment_bearing_kp','bearing_kp'),
                        ('alignment_yaw_rate_kp','yaw_rate_kp'),
                        ('alignment_yaw_preview_sec','yaw_brake_preview_sec'),
                        ('alignment_maximum_yaw_rate','maximum_yaw_rate_rad_s'),
                        ('alignment_imu_yaw_rate_reversed','imu_yaw_rate_reversed')):
        manager[key] = follow[source]
    return LaunchDescription([
        DeclareLaunchArgument('goal_side', default_value=nav['goal_side']),
        DeclareLaunchArgument('web_port', default_value='8080'),
        DeclareLaunchArgument('imu_port', default_value=str(settings('usv_imu_driver', 'imu_driver.yaml', 'imu_driver')[1]['port'])),
        DeclareLaunchArgument('dry_run', default_value='true'),
        LogInfo(msg=f'Live mission config: {mission_path}; shared navigation model: {nav_path}'),
        LogInfo(msg=f'Live camera target config: {target_path}; follow config: {follow_path}'),
        include('usv_imu_driver', 'imu.launch.py', port=LaunchConfiguration('imu_port')),
        include('usv_udp_position', 'udp_position.launch.py'),
        include('usv_camera_driver', 'camera.launch.py'),
        include('usv_yolo_detector', 'yolo_detector.launch.py'),
        Node(package='usv_ball_follow', executable='target_state_estimator',
             name='target_state_estimator', output='screen', parameters=[target]),
        Node(package='usv_ball_follow', executable='ball_follow_controller',
             name='ball_follow_controller', output='screen', parameters=[follow]),
        Node(package='usv_goal_navigation', executable='goal_navigation_node',
             name='goal_navigation', output='screen', parameters=[goal],
             remappings=[('/control/automatic_mode', '/mission/navigation_enabled'),
                         ('/navigation/goal_request', '/mission/navigation_goal_request')]),
        Node(package='usv_mission_manager', executable='mission_manager',
             name='mission_manager', output='screen', parameters=[manager]),
        include('usv_rc_teleop', 'control.launch.py',
                start_rc_nodes='true', force_automatic_without_rc='false',
                emergency_stop_enabled='true', dry_run=LaunchConfiguration('dry_run')),
        include('usv_web_monitor', 'web_monitor.launch.py', port=LaunchConfiguration('web_port')),
        Node(package='usv_ball_follow', executable='telemetry_recorder',
             name='ball_follow_telemetry_recorder', output='screen'),
    ])
