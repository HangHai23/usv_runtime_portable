from pathlib import Path
import yaml

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
    udp_share = Path(get_package_share_directory('usv_udp_position'))
    udp_config = udp_share / 'config' / 'udp_position.yaml'
    for ancestor in udp_share.parents:
        candidate = ancestor / 'src' / 'usv_udp_position' / 'config' / 'udp_position.yaml'
        if candidate.is_file():
            udp_config = candidate
            break
    ball_type = yaml.safe_load(udp_config.read_text())['udp_position_receiver']['ros__parameters'].get('ball_type', 'red')
    if ball_type not in ('red', 'multicolor'):
        raise ValueError('ball_type must be red or multicolor')
    detector = yaml.safe_load(Path(config).read_text())['yolo_detector']['ros__parameters']
    engine = detector[ball_type + '_engine_path']
    return LaunchDescription(
        [
            LogInfo(msg=f"Live YOLO config: {config}"),
            LogInfo(msg=f"Ball type: {ball_type}; engine: {engine}; selection config: {udp_config}"),
            Node(
                package="usv_yolo_detector",
                executable="yolo_detector_node",
                name="yolo_detector",
                output="screen",
                parameters=[config, {'engine_path': engine}],
            ),
        ]
    )
