# How to Run Hippo Harvest

This guide shows how to start the Docker environment, launch the Air Traffic Control panel, and run the simulator.

## Prerequisites

- Docker installed and running
- A Linux desktop session if you want GUI apps such as `rqt` and `rviz2`
- The repository cloned to `/opt/code`

If your host blocks X11 access, allow local Docker GUI access before starting the container:

```bash
xhost +local:root
```

## Start the container

From `/opt/code`:

```bash
docker network create ros-net
docker network create observe
docker compose up -d
docker exec -it hippoharvest bash
```

The `docker network create` commands are safe to run more than once.

## Air Traffic Control panel

Inside the container:

```bash
cd /opt/code/air_traffic_control
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
rqt --force-discover &
ros2 launch air_traffic_control traffic_manager.launch.py robot_count:=10
```

In `rqt`, open:

```text
Plugins -> Hippo Harvest -> Air Traffic Control Monitor
```

If it shows up under the generic Python plugin group instead, search for `Air Traffic Control Monitor`.

## Simulator

Open a second terminal in the same container and run:

```bash
cd /opt/code
./simulation/scripts/run_nav2_rviz.sh
```

This script will build the simulator package if needed, then launch Nav2 and RViz2.

## What You Should See

- `rqt` shows the Air Traffic Control table and live action log
- `rviz2` shows the robots moving through the map
- Each robot starts at home, visits two random waypoints, and then returns home

## Telemetry and Grafana

Telemetry is optional. When it is enabled, the simulator and ATC nodes export OpenTelemetry data to the endpoint in `OTEL_EXPORTER_OTLP_ENDPOINT` and write simulator logs to `simulation/rviz/nav2_rviz.log`.

If you have a Grafana/OpenTelemetry stack running on the `observe` network, you can open:

```text
http://localhost:3000
```

There you can inspect traces in Tempo and logs in Loki.

## Notes

- The simulator keeps running until you stop the terminal or interrupt the launch process.
- The ATC panel is read-only in this first pass; it observes robot state and displays nearest-neighbor conflict information.
