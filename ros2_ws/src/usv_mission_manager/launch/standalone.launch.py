"""Exclusive formal tracking/navigation startup, safely defaults to no PWM writes."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include(package, filename, **args):
    file = Path(get_package_share_directory(package))/'launch'/filename
    return GroupAction(scoped=True, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(file)), launch_arguments=args.items())])


def setup(context):
    mode = LaunchConfiguration('mode').perform(context)
    if mode not in ('tracking', 'navigation'):
        raise ValueError('mode must be tracking or navigation')
    actions = [include('usv_imu_driver', 'imu.launch.py'),
               include('usv_rc_teleop', 'control.launch.py',
                       dry_run=LaunchConfiguration('dry_run'), start_rc_nodes='true',
                       force_automatic_without_rc='false', emergency_stop_enabled='true'),
               include('usv_web_monitor', 'web_monitor.launch.py', port=LaunchConfiguration('web_port'))]
    if mode == 'tracking':
        actions += [include('usv_camera_driver', 'camera.launch.py'),
                    include('usv_yolo_detector', 'yolo_detector.launch.py'),
                    include('usv_ball_follow', 'ball_follow.launch.py'),
                    Node(package='usv_ball_follow', executable='telemetry_recorder', output='screen')]
    else:
        actions += [include('usv_udp_position', 'udp_position.launch.py'),
                    include('usv_goal_navigation', 'goal_navigation.launch.py',
                            goal_side=LaunchConfiguration('goal_side'), armed='true', require_mode='true',
                            command_topic='/propulsion/automatic_command')]
    return actions


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('mode', default_value='tracking'),
        DeclareLaunchArgument('goal_side', default_value='right'),
        DeclareLaunchArgument('dry_run', default_value='true'),
        DeclareLaunchArgument('web_port', default_value='8080'), OpaqueFunction(function=setup)])
