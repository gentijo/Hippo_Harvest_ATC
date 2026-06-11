# Hippo Harvest ATC

Hippo Harvest ATC is a ROS 2 prototype for a centralized air-traffic-control style coordinator for a fleet of robots. The repository contains two cooperating pieces:

- a simulation workspace that produces robot motion, goals, and synthetic localization
- an air-traffic-control workspace that observes the fleet, detects local conflict, and publishes pause / protection decisions

The project is designed to show the reasoning behind the coordination strategy, not to provide a full production fleet manager.

## System Overview

The system runs as a Dockerized ROS 2 environment with a shared code volume mounted into the container. At runtime:

1. The simulator publishes robot pose, goal, and navigation state.
2. The ATC node watches those topics, computes pairwise separation, and selects a deterministic robot to yield when the fleet starts closing on a conflict.
3. The ATC RQT plugin provides the operator-facing view of the same state by showing pose freshness, nearest neighbors, distance trends, and collision clusters.
4. Both workspaces emit telemetry so the run can be inspected live in Grafana or reviewed later from logs and diagnostic snapshots.

## Simulation

The simulation lives under [`simulation/`](./simulation). It is a small, purpose-built ROS 2 Jazzy environment that demonstrates the fleet behavior in a controlled map.

### What It Does

- Publishes a simple map and robot world model
- Moves robots through goal sequences in a repeatable way
- Provides synthetic localization instead of external sensor fusion
- Publishes the topics that the ATC workspace consumes
- Supports RViz visualization so the behavior is easy to inspect

### Main Simulation Components

- `grid_map_publisher`: publishes the empty occupancy grid
- `simple_robot_node`: clamps commanded velocity and republishes the executed command
- `synthetic_command_integrated_localization_node`: integrates motion into pose, odometry, and TF
- `multi_robot_orchestrator`: coordinates robot goals and run context
- `point_to_point_nav_node` and `nav2_work_area_goal`: drive robot goal behavior in the nav2-based flow

### Visualization

The recommended visualization is RViz2. The launch scripts under [`simulation/scripts/`](./simulation/scripts) build the workspace if needed, then open the Nav2/RViz view that shows the map and robot motion.

## Air Traffic Control

The ATC workspace lives under [`air_traffic_control/`](./air_traffic_control). It contains the centralized coordinator plus the RQT panel used to observe the fleet.

### Traffic Manager

[`traffic_manager_node.py`](./air_traffic_control/src/air_traffic_control/air_traffic_control/traffic_manager_node.py) is the core coordination node. It:

- subscribes to robot pose and goal topics
- measures nearest-neighbor distance and whether the gap is closing
- selects a deterministic yielder when two robots converge
- publishes pause state to `/<robot>/atc/pause`
- publishes protected-zone markers on `/atc/protected_zones`
- breaks yield cycles and deadlocks so the fleet can unwind
- optionally publishes an occupancy-grid view of fleet protection zones

This code follows ADR 0003, which uses centralized fleet state and perfect localization instead of lidar-first conflict detection.

### RQT Monitor

[`air_traffic_control_plugin.py`](./air_traffic_control/src/air_traffic_control/air_traffic_control/air_traffic_control_plugin.py) is the human-facing dashboard. It:

- subscribes to each robot pose topic
- reads Nav2 action status and ATC goal topics
- computes nearest neighbors and distance trend
- highlights stale robot feeds
- groups collision events into connected clusters
- shows the ATC event stream from the traffic manager

The plugin is intentionally read-only. It does not pause robots or change Nav2 state. It just makes the fleet story visible.

## Telemetry And Measurement

Telemetry is built into both the sim and ATC code paths.

### ATC Telemetry

The ATC telemetry helpers in [`air_traffic_control/telemetry.py`](./air_traffic_control/src/air_traffic_control/air_traffic_control/telemetry.py) create:

- OpenTelemetry spans for important events
- OpenTelemetry logs for the same events
- shared run context topics so multiple nodes can correlate the same run

The ATC traffic manager emits decisions and diagnostics as:

- spans and logs through OTLP
- ROS topic events on `/atc/events`
- a JSONL diagnostic stream in `/tmp/air_traffic_control/atc_diagnostics.jsonl`

The traffic manager also publishes run-time state such as paused robots, active yields, deadlock release state, and protected-zone snapshots.

### Simulation Telemetry

The simulation telemetry helpers in [`simulation/ros2_ws/src/hippo_harvest_sim/hippo_harvest_sim/telemetry.py`](./simulation/ros2_ws/src/hippo_harvest_sim/hippo_harvest_sim/telemetry.py) use the same pattern:

- they export spans and logs when an OTLP collector is available
- they propagate a shared run ID and robot trace context
- they degrade cleanly to local console logging if telemetry is unavailable

The launch scripts write simulator output to files such as:

- [`simulation/rviz/nav2_rviz.log`](./simulation/rviz/nav2_rviz.log)
- [`simulation/rviz/simulation.log`](./simulation/rviz/simulation.log)

### Why This Matters

This instrumentation made it possible to measure the prototype, not just watch it:

- we could see when a conflict was detected
- we could confirm which robot yielded and why
- we could measure pause duration, yield cycles, and stale feeds
- we could correlate simulator behavior with ATC decisions in the same run

## MCP And Debugging Workflow

We used the MCP-backed observability workflow with the Grafana stack to debug and measure the code while iterating.

In practice, that meant:

- running the Grafana/OpenTelemetry stack on the `observe` network
- connecting the IDE or code window to the Grafana MCP server
- inspecting traces in Tempo and logs in Loki while the simulator and ATC were running
- using the live telemetry output to verify that code changes produced the expected runtime behavior

That workflow was especially useful for answering questions like:

- did a code change reduce noisy yield decisions?
- did the traffic manager emit the expected pause and resume events?
- did the simulator and ATC agree on the same run context?
- did a conflict show up in the logs before it appeared in the RViz / RQT views?

## How To Run

See [`docs/HowToRun.md`](./docs/HowToRun.md) for the full setup and launch sequence.

## Repository Layout

- [`simulation/`](./simulation) - the robot simulation workspace and launch scripts
- [`air_traffic_control/`](./air_traffic_control) - the ATC workspace, traffic manager, and RQT plugin
- [`docs/`](./docs) - project write-up, ADRs, and run instructions

## Design Notes

- The prototype assumes perfect localization and a known map, as described in the ADRs and project brief.
- The ATC logic is centralized and deterministic so it is easy to explain and demo.
- The RQT panel is observability only. The coordination decisions live in the traffic manager.
- Telemetry is optional, but when it is enabled it gives us a repeatable way to measure behavior across runs.
