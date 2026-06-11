# ADR 0001: Use RViz2 as the Primary Simulation Visualization Tool

- Status: Accepted
- Date: 2026-06-09

## Context

The Hippo Harvest simulation environment supports more than one way to visualize robot state and navigation behavior:

- `RViz2` for occupancy grids, TF, robot markers, paths, and Nav2 outputs
- `Gazebo` for higher-fidelity world simulation
- `Mapviz` for map-centric visualization workflows

The current development loop for the lightweight simulation is centered on a synthetic, code-driven grid environment rather than physics-based world simulation. In this mode, the team primarily needs to inspect:

- the occupancy grid
- robot poses and TF frames
- waypoint markers
- planned paths
- Nav2 behavior and debugging overlays

The project brief frames the problem as an air traffic control system for roughly 20 robots operating in close proximity with little or no supervision. The required solution should help prevent collisions, avoid deadlocks, and demonstrate the treatment of important trade-offs through a prototype rather than a full production system.

The brief also gives several simplifying assumptions:

- perfect localization
- a known shared map
- optional supplemental map data structures
- constant connectivity between robots and centralized services
- no need to model humans or unrelated physical infrastructure

Given those assumptions, the main engineering goal in this phase is fast iteration while building and debugging planning, coordination, localization, orchestration, and visualization behavior for multiple robots.

## Decision

We will use `RViz2` as the primary visualization and operator-facing tool for the lightweight simulation environment.

`Gazebo` will remain available for higher-fidelity world simulation when physics, collisions, or richer environment behavior must be evaluated.

`Mapviz` may remain available as an optional companion tool, but it is not the primary interface for the simulation workflow.

## Rationale

`RViz2` was selected because it is the best fit for the current simulation architecture and development priorities.

### Why RViz2

- It directly supports the ROS 2 data products already produced by the simulation, including `/map`, TF, markers, robot paths, and Nav2 plans.
- It provides the shortest feedback loop for debugging navigation and localization behavior.
- It is lightweight compared with running a full physics simulator.
- It is already familiar to ROS 2 users and aligns with standard Nav2 debugging practices.
- The project already includes an RViz configuration and launch path, making it the lowest-friction default.
- It matches the prototype-oriented nature of the project brief, where the emphasis is on demonstrating system thinking and trade-offs rather than building a full high-fidelity simulator.

### Why not Gazebo as the primary tool

- Gazebo introduces significantly more runtime and configuration overhead.
- The lightweight simulation does not depend on physics to model its core behaviors.
- Most day-to-day debugging in this project is about maps, routes, transforms, and planner/controller behavior rather than contact dynamics or sensor realism.
- Using Gazebo as the default would slow iteration for common development tasks.
- The problem statement already grants perfect localization and a known map, which reduces the immediate value of leading with a physics-heavy simulation environment.

### Why not Mapviz as the primary tool

- Mapviz is less well aligned with the project's primary debugging needs around TF, Nav2 state, occupancy grids, and interactive ROS robot visualization.
- RViz2 provides a more standard and better-integrated experience for robot-centric inspection in ROS 2.
- The simulation stack already publishes the kinds of topics RViz2 is designed to visualize with minimal extra setup.
- Mapviz is less effective as the main environment for presenting prototype behavior around close-proximity robot coordination and deadlock debugging.

## Consequences

### Positive

- Faster local iteration for simulation and navigation work
- Lower compute overhead for routine development
- Better visibility into TF, paths, markers, and map state
- Simpler onboarding for ROS 2 and Nav2 debugging

### Negative

- RViz2 does not provide physics simulation.
- RViz2 is not a substitute for validating contact behavior, realistic motion dynamics, or richer world interactions.
- Teams needing geospatial or custom map presentation features may still prefer Mapviz for specific tasks.

## Alternatives Considered

### Gazebo-first workflow

Rejected as the default because it optimizes for simulation fidelity over iteration speed, while the current lightweight environment is intentionally synthetic and grid-driven.

### Mapviz-first workflow

Rejected as the default because it is a weaker fit for robot-centric ROS 2 debugging and Nav2 visualization than RViz2.

### Multi-tool equal support

Rejected as the primary recommendation because it increases documentation, support, and operator complexity without improving the main development loop.

## Follow-up

- Keep `RViz2` as the recommended default in simulation documentation and launch scripts.
- Continue to preserve `Gazebo` as the path for higher-fidelity validation.
- Treat `Mapviz` as optional rather than the primary supported operator view.
