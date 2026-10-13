from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('usv_udp_position'))
    config = share / 'config' / 'udp_position.yaml'
    for parent in share.parents:
        candidate = parent / 'src' / 'usv_udp_position' / 'config' / 'udp_position.yaml'
        if candidate.is_file():
            config = candidate
            break
    return LaunchDescription([
        LogInfo(msg=f'Live UDP config: {config}'),
        Node(package='usv_udp_position', executable='udp_position_node',
             name='udp_position_receiver', output='screen', parameters=[str(config)]),
    ])
