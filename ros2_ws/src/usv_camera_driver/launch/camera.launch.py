from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def config_path(package_share: Path) -> str:
    """Prefer editable source YAML, falling back to the installed copy."""
    for ancestor in package_share.parents:
        candidate = ancestor / "src" / "usv_camera_driver" / "config" / "camera.yaml"
        if candidate.is_file():
            return str(candidate)
    return str(package_share / "config" / "camera.yaml")


def generate_launch_description():
    share = Path(get_package_share_directory("usv_camera_driver"))
    config = config_path(share)
    return LaunchDescription(
        [
            LogInfo(msg=f"Live camera config: {config}"),
            DeclareLaunchArgument("device", default_value="auto"),
            Node(
                package="usv_camera_driver",
                executable="camera_node",
                name="camera_node",
                output="screen",
                parameters=[config, {"device": LaunchConfiguration("device")}],
            ),
        ]
    )
