from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


DEFAULT_ROBOT_COUNT = 10


def _robot_navigation_group(robot_index: int, robot_count: int, params_file):
    robot_name = f"robot{robot_index}"
    base_frame_id = f"{robot_name}/base_link"
    common_remappings = [
        ("map", "/map"),
        (f"/{robot_name}/map", "/map"),
    ]
    lifecycle_nodes = [
        "planner_server",
        "controller_server",
        "behavior_server",
        "bt_navigator",
    ]
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=robot_name,
            param_rewrites={
                "bt_navigator.ros__parameters.robot_base_frame": base_frame_id,
                "bt_navigator.ros__parameters.odom_topic": "odom",
                "behavior_server.ros__parameters.robot_base_frame": base_frame_id,
                "controller_server.ros__parameters.odom_topic": "odom",
                "global_costmap.global_costmap.ros__parameters.robot_base_frame": base_frame_id,
                "global_costmap.global_costmap.ros__parameters.static_layer.map_topic": "/map",
                "local_costmap.local_costmap.ros__parameters.robot_base_frame": base_frame_id,
                "local_costmap.local_costmap.ros__parameters.static_layer.map_topic": "/map",
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    return GroupAction(
        [
            PushRosNamespace(robot_name),
            Node(
                package="hippo_harvest_sim",
                executable="simple_robot_node",
                name="simple_robot_node",
                output="screen",
                parameters=[{"robot_name": robot_name}],
            ),
            Node(
                package="hippo_harvest_sim",
                executable="synthetic_command_integrated_localization_node",
                name="synthetic_command_integrated_localization_node",
                output="screen",
                parameters=[
                    {
                        "base_frame_id": base_frame_id,
                        "robot_index": robot_index,
                        "robot_name": robot_name,
                    }
                ],
            ),
            Node(
                package="hippo_harvest_sim",
                executable="initial_pose_publisher",
                name="initial_pose_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_count": robot_count,
                        "robot_index": robot_index,
                    }
                ],
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[configured_params],
                remappings=common_remappings,
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=[configured_params],
                remappings=common_remappings,
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=[configured_params],
                remappings=common_remappings,
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=[configured_params],
                remappings=common_remappings,
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "autostart": True,
                        "node_names": lifecycle_nodes,
                    }
                ],
            ),
        ]
    )


def generate_launch_description():
    pkg_share = FindPackageShare("hippo_harvest_sim")
    map_yaml = PathJoinSubstitution([pkg_share, "maps", "grid_nav2_map.yaml"])
    nav2_params_file = PathJoinSubstitution([pkg_share, "config", "nav2_grid_params.yaml"])
    nav2_params_file_no_traffic = PathJoinSubstitution([pkg_share, "config", "nav2_grid_params_no_traffic.yaml"])
    rviz_config = PathJoinSubstitution([pkg_share, "rviz", "hippo_harvest_nav2.rviz"])

    use_rviz = LaunchConfiguration("use_rviz")
    autostart = LaunchConfiguration("autostart")
    robot_count = LaunchConfiguration("robot_count")
    start_stagger_sec = LaunchConfiguration("start_stagger_sec")
    use_traffic_map = LaunchConfiguration("use_traffic_map")

    def launch_setup(context, *args, **kwargs):
        count = int(robot_count.perform(context))
        traffic_map_enabled = use_traffic_map.perform(context).lower() in ("1", "true", "yes", "on")
        chosen_params_file = nav2_params_file if traffic_map_enabled else nav2_params_file_no_traffic
        actions = [
            Node(
                package="hippo_harvest_sim",
                executable="waypoint_marker_publisher",
                name="waypoint_marker_publisher",
                output="screen",
                parameters=[{"robot_count": count}],
            ),
            Node(
                package="hippo_harvest_sim",
                executable="multi_robot_orchestrator",
                name="multi_robot_orchestrator",
                output="screen",
                parameters=[
                    {
                        "robot_count": count,
                        "start_stagger_sec": float(start_stagger_sec.perform(context)),
                    }
                ],
            ),
            Node(
                package="hippo_harvest_sim",
                executable="traffic_map_publisher",
                name="traffic_map_publisher",
                output="screen",
                parameters=[{"robot_count": count}],
            ),
        ]

        for robot_index in range(1, count + 1):
            actions.append(
                _robot_navigation_group(
                    robot_index,
                    count,
                    chosen_params_file,
                )
            )

        return actions

    launch_actions = [
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("robot_count", default_value=str(DEFAULT_ROBOT_COUNT)),
        DeclareLaunchArgument("start_stagger_sec", default_value="1.0"),
        DeclareLaunchArgument("use_traffic_map", default_value="false"),
        DeclareLaunchArgument("map", default_value=map_yaml),
        DeclareLaunchArgument("rviz_config", default_value=rviz_config),
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[
                nav2_params_file,
                {"yaml_filename": LaunchConfiguration("map")},
            ],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_map",
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
                    "autostart": autostart,
                    "node_names": ["map_server"],
                }
            ],
        ),
        OpaqueFunction(function=launch_setup),
    ]

    launch_actions.append(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            output="screen",
            condition=IfCondition(use_rviz),
        )
    )

    return LaunchDescription(launch_actions)
