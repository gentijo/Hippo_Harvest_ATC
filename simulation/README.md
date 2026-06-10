# Hippo Harvest Simple Grid Simulation

This simulation is a single-robot, empty-grid ROS 2 Jazzy simulation.

## Environment

- World size: `2.5 m x 2.5 m`
- Grid resolution: `0.025 m` (`25 mm`)
- Grid size: `100 x 100` cells
- Occupancy: empty grid
- Waypoint `A`: fixed at the southwest corner cell
- Waypoint `B`: randomly chosen from well inside the map

## Robot

- Single robot
- Footprint: `0.30 m x 0.30 m`
- Drive: differential drive with front caster
- Position reference: the grid cell containing the robot center point
- Demonstration traversal speed: `0.10 m/s`
- Command interface: `geometry_msgs/msg/Twist`
- Localization update rate remains `10 Hz`

## Node Split

- `grid_map_publisher`: publishes an empty occupancy grid
- `simple_robot_node`: listens to `/cmd_vel`, clamps commands, and republishes `/executed_cmd_vel`
- `synthetic_command_integrated_localization_node`: integrates `/executed_cmd_vel` into pose, odometry, TF, and center-cell location
- `point_to_point_nav_node`: starts at `A`, selects `B`, drives from `A` to `B`, then stops

## Recommended Visualization

Use RViz2.

Build once:

```bash
cd /opt/code/HippoHarvest/simulation/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select hippo_harvest_sim
```

Run the sim with RViz2:

```bash
/opt/code/HippoHarvest/simulation/scripts/run_with_rviz.sh
```

Manual run:

```bash
cd /opt/code/HippoHarvest/simulation/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch hippo_harvest_sim grid_sim.launch.py use_rviz:=true
```

## RViz Config

- Ready-made RViz config: `/opt/code/HippoHarvest/simulation/rviz/hippo_harvest_grid.rviz`
- RViz launcher script: `/opt/code/HippoHarvest/simulation/scripts/run_with_rviz.sh`

## Notes

- Localization is synthetic command-integrated localization, not external sensor-based localization.
- The simulation logs stream to the terminal and are also saved to `/opt/code/HippoHarvest/simulation/rviz/simulation.log`.
