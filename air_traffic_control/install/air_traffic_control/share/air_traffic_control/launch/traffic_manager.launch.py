from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_count", default_value="10"),
            DeclareLaunchArgument("safety_distance_m", default_value="0.35"),
            DeclareLaunchArgument("clear_distance_m", default_value="0.55"),
            Node(
                package="air_traffic_control",
                executable="traffic_manager_node",
                name="centralized_traffic_manager",
                output="screen",
                parameters=[
                    {
                        "robot_count": LaunchConfiguration("robot_count"),
                        "safety_distance_m": LaunchConfiguration("safety_distance_m"),
                        "clear_distance_m": LaunchConfiguration("clear_distance_m"),
                    }
                ],
            ),
        ]
    )
