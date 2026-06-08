from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("hippo_harvest_sim")
    map_yaml = PathJoinSubstitution([pkg_share, "maps", "grid_nav2_map.yaml"])
    params_file = PathJoinSubstitution([pkg_share, "config", "nav2_grid_params.yaml"])
    rviz_config = PathJoinSubstitution([pkg_share, "rviz", "hippo_harvest_nav2.rviz"])

    use_rviz = LaunchConfiguration("use_rviz")
    autostart = LaunchConfiguration("autostart")

    lifecycle_nodes = [
        "map_server",
        "planner_server",
        "controller_server",
        "behavior_server",
        "bt_navigator",
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("map", default_value=map_yaml),
            DeclareLaunchArgument("params_file", default_value=params_file),
            DeclareLaunchArgument("rviz_config", default_value=rviz_config),
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
                executable="initial_pose_publisher",
                name="initial_pose_publisher",
                output="screen",
            ),
            Node(
                package="hippo_harvest_sim",
                executable="waypoint_marker_publisher",
                name="waypoint_marker_publisher",
                output="screen",
            ),
            Node(
                package="hippo_harvest_sim",
                executable="nav2_work_area_goal",
                name="nav2_work_area_goal",
                output="screen",
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {"yaml_filename": LaunchConfiguration("map")},
                ],
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[LaunchConfiguration("params_file")],
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=[LaunchConfiguration("params_file")],
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=[LaunchConfiguration("params_file")],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=[LaunchConfiguration("params_file")],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[
                    {"use_sim_time": True},
                    {"autostart": autostart},
                    {"node_names": lifecycle_nodes},
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", LaunchConfiguration("rviz_config")],
                output="screen",
                condition=IfCondition(use_rviz),
            ),
        ]
    )
