#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="/opt/code/HippoHarvest/simulation"
WS_DIR="$ROOT_DIR/ros2_ws"
PKG_NAME="hippo_harvest_sim"
PKG_ROOT="$WS_DIR/src/$PKG_NAME"
SIM_LOG="$ROOT_DIR/rviz/nav2_gazebo.log"
mkdir -p "$ROOT_DIR/rviz"

ensure_built() {
  local install_setup="$WS_DIR/install/setup.bash"
  local install_launch="$WS_DIR/install/$PKG_NAME/share/$PKG_NAME/launch/sim_bringup.launch.py"
  local package_changed=""
  if [[ -f "$install_setup" ]]; then
    package_changed=$(find "$PKG_ROOT" -type f -newer "$install_setup" -print -quit)
  fi

  if [[ ! -f "$install_setup" || ! -f "$install_launch" || -n "$package_changed" ]]; then
    echo "Building $PKG_NAME so installed launch files are up to date..."
    cd "$WS_DIR"
    source /opt/ros/jazzy/setup.bash
    colcon build --packages-select "$PKG_NAME"
  fi
}

source /opt/ros/jazzy/setup.bash
ensure_built
source "$WS_DIR/install/setup.bash"

cd "$WS_DIR"
exec stdbuf -oL -eL ros2 launch $PKG_NAME sim_bringup.launch.py 2>&1 | tee "$SIM_LOG"
