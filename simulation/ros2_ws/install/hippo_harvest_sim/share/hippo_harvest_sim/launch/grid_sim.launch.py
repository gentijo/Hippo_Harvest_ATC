from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = PathJoinSubstitution([FindPackageShare("hippo_harvest_sim"), "rviz", "hippo_harvest_grid.rviz"])

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            Node(
                package="swri_transform_util",
                executable="initialize_origin.py",
                name="initialize_origin",
                output="screen",
                parameters=[
                    {
                        "local_xy_frame": "/map",
                        "local_xy_origin": "swri",
                        "local_xy_origins": [29.45196669, -98.61370577, 233.719, 0.0],
                    }
                ],
            ),
            Node(
                package="hippo_harvest_sim",
                executable="grid_map_publisher",
                name="grid_map_publisher",
                output="screen",
            ),
            Node(
                package="hippo_harvest_sim",
                executable="simple_robot_node",
                name="simple_robot_node",
                output="screen",
            ),
            Node(
                package="hippo_harvest_sim",
                executable="synthetic_command_integrated_localization_node",
                name="synthetic_command_integrated_localization_node",
                output="screen",
            ),
            Node(
                package="hippo_harvest_sim",
                executable="point_to_point_nav_node",
                name="point_to_point_nav_node",
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
                condition=IfCondition(use_rviz),
            ),
        ]
    )
