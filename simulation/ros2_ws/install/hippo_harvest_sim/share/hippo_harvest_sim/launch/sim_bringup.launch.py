from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("hippo_harvest_sim")
    world = PathJoinSubstitution([pkg_share, "worlds", "hippo_harvest.world.sdf"])
    nav2_params = PathJoinSubstitution([pkg_share, "config", "nav2_params.yaml"])
    map_yaml = PathJoinSubstitution([pkg_share, "maps", "hippo_harvest_map.yaml"])

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("world", default_value=world),
            Node(
                package="ros_gz_sim",
                executable="gz_sim",
                arguments=["-r", LaunchConfiguration("world")],
                output="screen",
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                    {"yaml_filename": map_yaml},
                ],
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[nav2_params, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=[nav2_params, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=[nav2_params, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
            ),
            Node(
                package="hippo_harvest_sim",
                executable="random_work_area_goal",
                name="random_work_area_goal",
                output="screen",
                parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            ),
        ]
    )
