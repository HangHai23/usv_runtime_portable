from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def config_path(package_share: Path, filename: str) -> str:
    """Prefer editable source YAML, falling back to the installed copy."""
    for ancestor in package_share.parents:
        candidate = ancestor / "src" / "usv_ball_follow" / "config" / filename
        if candidate.is_file():
            return str(candidate)
    return str(package_share / "config" / filename)


def generate_launch_description():
    share = Path(get_package_share_directory("usv_ball_follow"))
    estimator_config = config_path(share, "target_estimator.yaml")
    controller_config = config_path(share, "ball_follow_controller.yaml")
    return LaunchDescription(
        [
            LogInfo(msg=f"Live target-estimator config: {estimator_config}"),
            LogInfo(msg=f"Live ball-follow config: {controller_config}"),
            Node(
                package="usv_ball_follow",
                executable="target_state_estimator",
                name="target_state_estimator",
                output="screen",
                parameters=[estimator_config],
            ),
            Node(
                package="usv_ball_follow",
                executable="ball_follow_controller",
                name="ball_follow_controller",
                output="screen",
                parameters=[controller_config],
            ),
        ]
    )
