from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("bind_address", default_value="0.0.0.0"),
            DeclareLaunchArgument("port", default_value="8080"),
            Node(
                package="usv_web_monitor",
                executable="web_monitor",
                name="usv_web_monitor",
                output="screen",
                parameters=[
                    {
                        "bind_address": LaunchConfiguration("bind_address"),
                        "port": ParameterValue(
                            LaunchConfiguration("port"), value_type=int
                        ),
                    }
                ],
            ),
        ]
    )
