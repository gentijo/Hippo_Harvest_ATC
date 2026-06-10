# How To Run

This document describes how to start the Hippo Harvest simulation environment using the provided Docker image and then launch the ROS 2 simulation modules inside the running container.

This guide assumes:

- Docker is installed on the host machine
- the repository is available on the host
- ROS 2 environment sourcing will be added to the container startup profile
- the user does not need to manually run `source /opt/ros/jazzy/setup.bash` or `source install/setup.bash`
- an X Windows server is available on the host if visual tools such as RViz2 or Mapviz will be used

## Overview

The project uses a Docker-based development environment defined by [Dockerfile.HippoHarvest](/opt/code/HippoHarvest/Dockerfile.HippoHarvest). Once the container is running, the ROS 2 simulation can be launched from the workspace in `simulation/ros2_ws`.

Any visual component in this project depends on a host X Windows server. The ROS 2 GUI applications run inside the container, but their windows must be displayed by the host operating system through X11-compatible display forwarding.

The primary simulation package is:

- `hippo_harvest_sim`

The main launch modes currently available are:

- lightweight grid simulation with RViz2
- Nav2-based simulation with RViz2
- lightweight grid simulation with Mapviz

## Build the Docker image

On Linux hosts, the preferred path is to use the provided launcher script instead of building and running the container manually. That script handles X11 access, image build, Docker network setup, and container launch parameters in one step.

If you want the exact build details, review [startHippoContainer](/opt/code/HippoHarvest/startHippoContainer), which builds the Docker image from [Dockerfile.HippoHarvest](/opt/code/HippoHarvest/Dockerfile.HippoHarvest).

## Start the container

### Recommended Linux host workflow

For Linux hosts, use the provided launcher script:

```bash
/opt/code/HippoHarvest/startHippoContainer
```

This script performs the following tasks:

- updates X11 access using `xhost`
- creates the Docker network `hippo-rosnet` if it does not already exist
- builds the Docker image `hippo-ros` from `Dockerfile.HippoHarvest`
- launches the container with the required X11 and device-mapping parameters

The script currently launches the container with settings equivalent to:

- `--net=hippo-rosnet`
- `--name hippoharvest`
- `--hostname hippoharvest`
- `--privileged`
- `--env DISPLAY=unix$DISPLAY`
- `-v /tmp/.X11-unix:/tmp/.X11-unix`
- `-v /dev:/dev`
- `-v <repo>:/opt/code`

This is the easiest way to run the project on a Linux host when GUI tools such as RViz2 or Mapviz are needed.

If you are interested in the exact container build and launch parameters, review the launcher script directly:

- [startHippoContainer](/opt/code/HippoHarvest/startHippoContainer)

After the container starts, move into the workspace area as needed:

```bash
cd /opt/code/HippoHarvest
```

## Host X Windows support

This project has an external dependency on a host X Windows server whenever visual tools are used.

This applies to:

- `RViz2`
- `Mapviz`
- any future ROS 2 GUI tool launched from the container

The container provides the application runtime, but the host must provide the display server that actually opens the window.

### Linux hosts

Linux is usually the easiest environment for this project because X11 support is commonly available already or can be enabled with standard Docker display forwarding techniques.

This repository already includes a launcher script, [startHippoContainer](/opt/code/HippoHarvest/startHippoContainer), that handles the Linux-specific X11 setup path.

In practice, Linux setups often involve:

- passing the `DISPLAY` environment variable into the container
- mounting `/tmp/.X11-unix`
- allowing the container to connect to the host X server

In this project, the launcher script does that setup automatically and also uses `xhost` to relax X server access control for the container session.

### macOS hosts

macOS can be supported if `XQuartz` is installed and configured on the host.

In that case:

- `XQuartz` acts as the host X server
- the container connects to the host display service
- extra host-side configuration is usually required before GUI applications will render correctly

This path is workable, but generally needs more setup than Linux.

### Windows hosts

Windows hosts can also be supported, but usually require a separate X Windows server such as:

- `VcXsrv`
- `Xming`

In this setup:

- the X server runs on the Windows host
- GUI applications inside the container connect to that display service
- networking and access control between the host and container may require troubleshooting

Setting up connectivity between a Windows host and the container can be tricky.

Common trouble areas include:

- firewall configuration
- Docker networking
- `DISPLAY` environment configuration
- access permissions on the X server

## Workspace layout

The ROS 2 workspace used by the simulation is:

- `/opt/code/HippoHarvest/simulation/ros2_ws`

The helper scripts live in:

- `/opt/code/HippoHarvest/simulation/scripts`

The project documentation lives in:

- `/opt/code/HippoHarvest/simulation/docs`

## Build the simulation package

If the workspace has not yet been built, or if package files have changed, build the simulation package from the ROS 2 workspace:

```bash
cd /opt/code/HippoHarvest/simulation/ros2_ws
colcon build --packages-select hippo_harvest_sim
```

In many cases you do not need to do this manually because the helper launch scripts check whether the package needs to be rebuilt and will run `colcon build` automatically when required.

## Recommended launch paths

### 1. Lightweight grid simulation with RViz2

This is the simplest and recommended visualization path for the prototype simulation:

```bash
/opt/code/HippoHarvest/simulation/scripts/run_with_rviz.sh
```

What this does:

- ensures `hippo_harvest_sim` is built
- launches `grid_sim.launch.py`
- opens the simulation in RViz2
- writes logs to `simulation/rviz/simulation.log`

### 2. Nav2 simulation with RViz2

To run the Nav2-oriented simulation flow with RViz2:

```bash
/opt/code/HippoHarvest/simulation/scripts/run_nav2_rviz.sh
```

What this does:

- ensures `hippo_harvest_sim` is built
- launches `nav2_rviz.launch.py`
- opens RViz2
- writes logs to `simulation/rviz/nav2_rviz.log`

### 3. Lightweight grid simulation with Mapviz

If Mapviz is needed for comparison or alternate visualization:

```bash
/opt/code/HippoHarvest/simulation/scripts/run_with_mapviz.sh
```

What this does:

- installs the Mapviz configuration into a local home directory
- launches the grid simulation in the background
- starts Mapviz
- writes logs to the `simulation/mapviz` directory

## Manual ROS 2 launch commands

If you prefer to run launch files directly instead of using the helper scripts, use the commands below.

From the workspace:

```bash
cd /opt/code/HippoHarvest/simulation/ros2_ws
```

### Grid simulation with RViz2

```bash
ros2 launch hippo_harvest_sim grid_sim.launch.py use_rviz:=true
```

### Nav2 simulation with RViz2

```bash
ros2 launch hippo_harvest_sim nav2_rviz.launch.py use_rviz:=true
```

### Gazebo-oriented bringup

```bash
ros2 launch hippo_harvest_sim sim_bringup.launch.py
```

## Running subsequent modules

Once the core simulation is running, additional ROS 2 modules can be launched inside the same container using normal `ros2 run` or `ros2 launch` commands.

This is the intended workflow for adding modules such as:

- air traffic control
- coordination logic
- experimental monitoring nodes
- prototype navigation helpers

Because the ROS environment will be sourced automatically by the shell profile, these modules can be started directly from new terminals or shell sessions inside the container without additional sourcing steps.

## Logs and outputs

Common log locations include:

- `/opt/code/HippoHarvest/simulation/rviz/simulation.log`
- `/opt/code/HippoHarvest/simulation/rviz/nav2_rviz.log`
- `/opt/code/HippoHarvest/simulation/mapviz/simulation.log`
- `/opt/code/HippoHarvest/simulation/mapviz/mapviz.log`

These logs are useful for debugging launch issues, node startup behavior, and runtime failures.

## Typical workflow

1. On Linux hosts, run `/opt/code/HippoHarvest/startHippoContainer`.
2. If you need the exact container build and run details, review `startHippoContainer`.
3. Enter the repository or ROS workspace directory.
4. Launch the simulation using one of the helper scripts.
5. Start any additional ROS 2 modules needed for the experiment.
6. Review RViz2, Mapviz, and log outputs as needed.

## Notes

- The helper scripts are the easiest way to run the project because they handle build checks and log capture.
- On Linux, `startHippoContainer` is the preferred entry point because it handles `xhost`, image build, network setup, and X11-related Docker parameters.
- The Docker image currently uses `osrf/ros:jazzy-desktop-full` as its base, which is suitable for GUI-based ROS 2 tools such as RViz2.
- Visual tools in this project require host X Windows support.
- Linux is typically the simplest host environment for GUI support.
- macOS can be supported with `XQuartz`.
- Windows can be supported with `VcXsrv` or `Xming`, but host-to-container GUI connectivity may require extra setup and debugging.
