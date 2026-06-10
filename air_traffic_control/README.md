# Air Traffic Control RQT Workspace

This is a separate ROS 2 workspace for the first read-only pass of the air
traffic controller prototype.

The plugin follows the observability portion of the project requirements and
ADRs:

- collect each robot's current pose
- collect active Nav2 goal status
- collect optional current-goal pose topics when available
- calculate each robot's nearest neighbor
- calculate nearest-neighbor distance
- flag whether that distance is increasing, decreasing, stable, or unknown
- display the data in an RQT table

This pass does not pause robots, inject keep-out zones, or modify Nav2 goals.

The `traffic_manager_node` is the next pass: it applies the centralized local
conflict policy from ADR 0003 by pausing yielders through `/<robot>/atc/pause`
and publishing protected-zone markers on `/atc/protected_zones`.

## Build

From this workspace:

```bash
cd /opt/code/air_traffic_control
colcon build
source install/setup.bash
```

When running with the simulator, also source the simulator workspace in the
same shell.

## Run

Traffic manager:

```bash
source /opt/ros/jazzy/setup.bash
source /opt/code/air_traffic_control/install/setup.bash
ros2 launch air_traffic_control traffic_manager.launch.py robot_count:=10
```

RQT monitor:

```bash
rqt
```

Then open:

```text
Plugins -> Hippo Harvest -> Air Traffic Control Monitor
```

If RQT places it under the generic Python plugin group instead, search for
`Air Traffic Control Monitor`.

## Default Topic Assumptions

The panel defaults to `robot1` through `robot20`.

For each robot it subscribes to:

- `/<robot>/synthetic_pose`
- `/<robot>/navigate_to_pose/_action/status`
- `/<robot>/atc/current_goal`
- `/<robot>/nav/current_goal`
- `/<robot>/goal_pose`
- `/<robot>/atc/pause`

The current simulator publishes pose topics, Nav2 status topics, and decoded
ATC goal name/pose topics from the multi-robot orchestrator.
