from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def generate_launch_description():
    share = Path(get_package_share_directory("usv_goal_navigation"))
    config = share / "config" / "goal_navigation.yaml"
    for parent in share.parents:
        candidate = parent / "src/usv_goal_navigation/config/goal_navigation.yaml"
        if candidate.is_file():
            config = candidate
            break
    settings = yaml.safe_load(config.read_text())["goal_navigation"]["ros__parameters"]
    return LaunchDescription(
        [
            LogInfo(msg=f"Live goal navigation config: {config}"),
            DeclareLaunchArgument('route_file', default_value=str(settings.get('route_file',''))),
            DeclareLaunchArgument("goal_side", default_value=str(settings['goal_side'])),
            DeclareLaunchArgument("armed", default_value=str(settings['armed']).lower()),
            DeclareLaunchArgument("require_mode", default_value=str(settings['require_mode']).lower()),
            DeclareLaunchArgument(
                "command_topic", default_value=str(settings['command_topic'])
            ),
            DeclareLaunchArgument("measurement_delay_sec", default_value=str(settings['measurement_delay_sec'])),
            Node(
                package="usv_goal_navigation",
                executable="goal_navigation_node",
                name="goal_navigation",
                output="screen",
                parameters=[
                    str(config),
                    {
                        'route_file': ParameterValue(LaunchConfiguration('route_file'), value_type=str),
                        "goal_side": LaunchConfiguration("goal_side"),
                        "require_mode": ParameterValue(LaunchConfiguration("require_mode"), value_type=bool),
                        "armed": ParameterValue(
                            LaunchConfiguration("armed"), value_type=bool
                        ),
                        "command_topic": LaunchConfiguration("command_topic"),
                        "measurement_delay_sec": ParameterValue(
                            LaunchConfiguration("measurement_delay_sec"),
                            value_type=float,
                        ),
                    },
                ],
            ),
        ]
    )
