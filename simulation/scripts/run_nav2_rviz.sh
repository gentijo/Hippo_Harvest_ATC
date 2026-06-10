#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
source "$SCRIPT_DIR/telemetry_logging.sh"
WS_DIR="$ROOT_DIR/ros2_ws"
PKG_NAME="hippo_harvest_sim"
PKG_ROOT="$WS_DIR/src/$PKG_NAME"
SIM_LOG="$ROOT_DIR/rviz/nav2_rviz.log"
ATC_WS_DIR="$(cd "$ROOT_DIR/.." && pwd)/air_traffic_control"
ATC_SETUP="$ATC_WS_DIR/install/setup.bash"
ROBOT_COUNT="${ROBOT_COUNT:-10}"
START_ATC="${START_ATC:-0}"
SIM_INSTALL_SETUP="$WS_DIR/install/setup.bash"
mkdir -p "$ROOT_DIR/rviz"

ensure_built() {
  local install_setup="$WS_DIR/install/setup.bash"
  local install_launch="$WS_DIR/install/$PKG_NAME/share/$PKG_NAME/launch/nav2_rviz.launch.py"
  local package_changed=""
  if [[ -f "$install_setup" ]]; then
    package_changed=$(find "$PKG_ROOT" -type f -newer "$install_setup" -print -quit)
  fi

  if [[ ! -f "$install_setup" || ! -f "$install_launch" || -n "$package_changed" ]]; then
    echo "Building $PKG_NAME so installed launch files are up to date..."
    cd "$WS_DIR"
    source /opt/ros/jazzy/setup.bash
    if ! colcon build --packages-select "$PKG_NAME"; then
      echo "Normal simulator install is not writable; building a temporary simulator overlay in /tmp..." >&2
      colcon --log-base /tmp/hippo_harvest_colcon_logs \
        build \
        --packages-select "$PKG_NAME" \
        --build-base /tmp/hippo_harvest_build \
        --install-base /tmp/hippo_harvest_install
      SIM_INSTALL_SETUP="/tmp/hippo_harvest_install/setup.bash"
    fi
  fi
}

source /opt/ros/jazzy/setup.bash
ensure_built
source "$SIM_INSTALL_SETUP"
if [[ -f "$ATC_SETUP" ]]; then
  source "$ATC_SETUP"
fi

ATC_PID=""
cleanup() {
  if [[ -n "$ATC_PID" ]] && kill -0 "$ATC_PID" 2>/dev/null; then
    kill "$ATC_PID"
    wait "$ATC_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if [[ "$START_ATC" == "1" ]] && ros2 pkg prefix air_traffic_control >/dev/null 2>&1; then
  ros2 launch air_traffic_control traffic_manager.launch.py robot_count:="$ROBOT_COUNT" &
  ATC_PID="$!"
elif [[ "$START_ATC" == "1" ]]; then
  echo "START_ATC=1 requested but air_traffic_control is not available; running simulator without ATC guard." >&2
fi

cd "$WS_DIR"
run_with_sim_logging "$SIM_LOG" \
  stdbuf -oL -eL ros2 launch "$PKG_NAME" nav2_rviz.launch.py \
  use_rviz:=true \
  robot_count:="$ROBOT_COUNT"
