from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def config_path(package_share: Path, package_name: str, filename: str) -> str:
    for ancestor in package_share.parents:
        candidate = ancestor / "src" / package_name / "config" / filename
        if candidate.is_file():
            return str(candidate)
    return str(package_share / "config" / filename)


def generate_launch_description():
    package_share = Path(get_package_share_directory("usv_pwm_actuator"))
    default_config = config_path(
        package_share, "usv_pwm_actuator", "pwm_actuator.yaml"
    )
    return LaunchDescription(
        [
            LogInfo(msg=f"Live PWM config: {default_config}"),
            DeclareLaunchArgument("backend", default_value="i2c"),
            DeclareLaunchArgument("port", default_value="/dev/ttyTHS2"),
            DeclareLaunchArgument("i2c_bus", default_value="7"),
            DeclareLaunchArgument("i2c_address", default_value="45"),
            DeclareLaunchArgument("dry_run", default_value="true"),
            Node(
                package="usv_pwm_actuator",
                executable="pwm_actuator_node",
                name="pwm_actuator",
                output="screen",
                parameters=[
                    default_config,
                    {
                        "backend": LaunchConfiguration("backend"),
                        "port": LaunchConfiguration("port"),
                        "i2c_bus": LaunchConfiguration("i2c_bus"),
                        "i2c_address": LaunchConfiguration("i2c_address"),
                        "dry_run": LaunchConfiguration("dry_run"),
                    },
                ],
            ),
        ]
    )
