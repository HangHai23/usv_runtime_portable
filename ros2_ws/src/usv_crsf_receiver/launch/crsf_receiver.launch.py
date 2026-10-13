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
    package_share = Path(get_package_share_directory("usv_crsf_receiver"))
    default_config = config_path(
        package_share, "usv_crsf_receiver", "crsf_receiver.yaml"
    )
    return LaunchDescription(
        [
            LogInfo(msg=f"Live CRSF config: {default_config}"),
            DeclareLaunchArgument("port", default_value="/dev/ttyTHS1"),
            Node(
                package="usv_crsf_receiver",
                executable="crsf_receiver_node",
                name="crsf_receiver",
                output="screen",
                parameters=[default_config, {"port": LaunchConfiguration("port")}],
            ),
        ]
    )
