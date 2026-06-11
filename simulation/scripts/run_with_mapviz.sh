#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="/opt/code/HippoHarvest/simulation"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/telemetry_logging.sh"
WS_DIR="$ROOT_DIR/ros2_ws"
MAPVIZ_HOME="$ROOT_DIR/mapviz/home"
MAPVIZ_CONFIG_SRC="$ROOT_DIR/mapviz/hippo_harvest_grid.mvc"
MAPVIZ_CONFIG_DST="$MAPVIZ_HOME/.mapviz_config"
MAPVIZ_LOG="$ROOT_DIR/mapviz/mapviz.log"
SIM_LOG="$ROOT_DIR/mapviz/simulation.log"

mkdir -p "$MAPVIZ_HOME"
cp "$MAPVIZ_CONFIG_SRC" "$MAPVIZ_CONFIG_DST"
echo "Installed Mapviz config to $MAPVIZ_CONFIG_DST"

source /opt/ros/jazzy/setup.bash
source "$WS_DIR/install/setup.bash"

cd "$WS_DIR"
run_with_sim_logging "$SIM_LOG" \
  stdbuf -oL -eL ros2 launch hippo_harvest_sim grid_sim.launch.py &
SIM_PID=$!
echo "Started simulation in background with PID $SIM_PID"
echo "Simulation log: $SIM_LOG"

cleanup() {
  if [ -n "${MAPVIZ_PID:-}" ] && kill -0 "$MAPVIZ_PID" >/dev/null 2>&1; then
    kill "$MAPVIZ_PID" >/dev/null 2>&1 || true
  fi
  if kill -0 "$SIM_PID" >/dev/null 2>&1; then
    kill "$SIM_PID" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

for _ in $(seq 1 20); do
  if ros2 topic list 2>/dev/null | grep -q ^/robot_marker && ros2 topic list 2>/dev/null | grep -q ^/waypoint_markers; then
    break
  fi
  sleep 0.5
done

HOME="$MAPVIZ_HOME" ros2 run mapviz mapviz >"$MAPVIZ_LOG" 2>&1 &
MAPVIZ_PID=$!
echo "Started Mapviz in background with PID $MAPVIZ_PID"
echo "Mapviz log: $MAPVIZ_LOG"

wait "$SIM_PID"
