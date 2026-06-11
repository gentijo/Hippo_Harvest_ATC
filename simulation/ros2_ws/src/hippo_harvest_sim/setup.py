from setuptools import find_packages, setup


package_name = "hippo_harvest_sim"


setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/grid_sim.launch.py", "launch/sim_bringup.launch.py", "launch/nav2_rviz.launch.py"]),
        ("share/" + package_name + "/maps", ["maps/empty_grid_map.yaml", "maps/empty_grid_map.pgm", "maps/grid_nav2_map.yaml", "maps/grid_nav2_map.pgm", "maps/hippo_harvest_map.yaml", "maps/hippo_harvest_map.pgm", "maps/goals.csv"]),
        (
            "share/" + package_name + "/config",
            ["config/nav2_params.yaml", "config/nav2_grid_params.yaml", "config/nav2_grid_params_atc.yaml"],
        ),
        ("share/" + package_name + "/rviz", ["rviz/hippo_harvest_grid.rviz", "rviz/hippo_harvest_nav2.rviz"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Hippo Harvest",
    maintainer_email="user@example.com",
    description="Simple grid simulation with custom nav and synthetic localization.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "grid_map_publisher = hippo_harvest_sim.grid_map_publisher:main",
            "simple_robot_node = hippo_harvest_sim.simple_robot_node:main",
            "synthetic_command_integrated_localization_node = hippo_harvest_sim.synthetic_command_integrated_localization_node:main",
            "point_to_point_nav_node = hippo_harvest_sim.point_to_point_nav_node:main",
            "initial_pose_publisher = hippo_harvest_sim.initial_pose_publisher:main",
            "waypoint_marker_publisher = hippo_harvest_sim.waypoint_marker_publisher:main",
            "nav2_work_area_goal = hippo_harvest_sim.nav2_work_area_goal:main",
            "multi_robot_orchestrator = hippo_harvest_sim.multi_robot_orchestrator:main",
            "traffic_map_publisher = hippo_harvest_sim.traffic_map_publisher:main",
        ],
    },
)
