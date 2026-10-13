from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def config_path(package_share: Path) -> str:
    """Prefer editable source YAML, falling back to the installed copy."""
    for ancestor in package_share.parents:
        candidate = ancestor / "src" / "usv_yolo_detector" / "config" / "yolo_detector.yaml"
        if candidate.is_file():
            return str(candidate)
    return str(package_share / "config" / "yolo_detector.yaml")


def generate_launch_description():
    share = Path(get_package_share_directory("usv_yolo_detector"))
    config = config_path(share)
    return LaunchDescription(
        [
            LogInfo(msg=f"Live YOLO config: {config}"),
            Node(
                package="usv_yolo_detector",
                executable="yolo_detector_node",
                name="yolo_detector",
                output="screen",
                parameters=[config],
            ),
        ]
    )
