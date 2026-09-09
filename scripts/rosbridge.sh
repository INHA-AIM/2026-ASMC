#!/usr/bin/env bash
# Start roscore + rosbridge_server for MORAI (ROS WebSocket, default 9090).
# This is NOT the competition UDP bridge. UDP: ./scripts/udp_bridge.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
# shellcheck source=asmc_env.sh
source "$SCRIPT_DIR/asmc_env.sh"

cd "$ASMC_WS_ROOT"
source /opt/ros/noetic/setup.bash
source devel/setup.bash 2>/dev/null || true

[ -f "$MORAI_BRIDGE_ENV" ] && source "$MORAI_BRIDGE_ENV"

BRIDGE_PORT="${MORAI_BRIDGE_PORT:-9090}"
BRIDGE_BIND="${MORAI_BRIDGE_BIND:-0.0.0.0}"
SIMULATOR_PROFILE="${ASMC_SIMULATOR_PROFILE:-dev_26r1_h3_ros_experimental}"

case "$SIMULATOR_PROFILE" in
  competition_*)
    echo "[rosbridge] ERROR: competition simulator transport must be UDP." >&2
    echo "  Use: ./scripts/udp_bridge.sh --simulator competition" >&2
    exit 2
    ;;
esac

export ROS_MASTER_URI="http://127.0.0.1:11311"
export ROS_IP="${ROS_IP:-127.0.0.1}"

_ensure_morai_msgs() {
  _morai_msgs_is_beta() {
    rosmsg show morai_msgs/EgoVehicleStatus 2>/dev/null \
      | grep -q 'float32 wheel_angle'
  }

  if rospack find morai_msgs >/dev/null 2>&1 && _morai_msgs_is_beta; then
    return 0
  fi
  echo "[rosbridge] morai_msgs missing or not beta_drive — building morai_msgs..."
  source /opt/ros/noetic/setup.bash
  catkin_make --pkg morai_msgs
  source devel/setup.bash
  if ! rospack find morai_msgs >/dev/null 2>&1; then
    echo "[rosbridge] ERROR: morai_msgs build failed. Run: ./scripts/build_ws.sh"
    exit 1
  fi
  if ! _morai_msgs_is_beta; then
    echo "[rosbridge] ERROR: EgoVehicleStatus missing wheel_angle (expected beta_drive)."
    echo "  Check: rosmsg show morai_msgs/EgoVehicleStatus"
    exit 1
  fi
  echo "[rosbridge] morai_msgs ready (beta_drive)."
}

_ensure_rosbridge() {
  if rospack find rosbridge_server >/dev/null 2>&1; then
    return 0
  fi
  echo "[rosbridge] rosbridge_server not found — installing..."
  apt-get update -qq
  DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    ros-noetic-rosbridge-server \
    ros-noetic-rosbridge-suite \
    >/dev/null
  source /opt/ros/noetic/setup.bash
  if ! rospack find rosbridge_server >/dev/null 2>&1; then
    echo "[rosbridge] ERROR: rosbridge install failed."
    exit 1
  fi
  echo "[rosbridge] rosbridge installed."
}

echo "=========================================="
echo "  rosbridge (WebSocket) — NOT UDP bridge"
echo "  For UDP: ./scripts/udp_bridge.sh"
echo "=========================================="
echo "  ASMC_WS_ROOT     : $ASMC_WS_ROOT"
echo "  ROS_MASTER_URI   : $ROS_MASTER_URI"
echo "  Listening        : ${BRIDGE_BIND}:${BRIDGE_PORT}"
echo "  Simulator        : ${SIMULATOR_PROFILE}"
echo ""
if [ -f /.dockerenv ]; then
  echo "  Windows MORAI: Bridge IP = 127.0.0.1  PORT = ${BRIDGE_PORT}"
  echo ""
fi
echo "  MORAI UI: Ego + Simulator Network -> same IP/PORT, ROS, Connected"
echo "  WARNING: ASMC uses beta_drive messages; direct 26.R1 Ego/Control schemas differ."
echo "           This is an experimental development path, never the competition path."
echo "  Verify: ./scripts/diagnose_morai_rosbridge.sh"
echo "=========================================="
echo ""

_ensure_rosbridge
_ensure_morai_msgs

if ! rostopic list >/dev/null 2>&1; then
  echo "[rosbridge] Starting roscore..."
  roscore &
  ROSCORE_PID=$!
  for _ in $(seq 1 15); do
    rostopic list >/dev/null 2>&1 && break
    sleep 1
  done
  if ! rostopic list >/dev/null 2>&1; then
    echo "[rosbridge] ERROR: roscore failed."
    kill "$ROSCORE_PID" 2>/dev/null || true
    exit 1
  fi
  echo "[rosbridge] roscore ready."
fi

echo "Starting rosbridge on port ${BRIDGE_PORT}..."
exec roslaunch rosbridge_server rosbridge_websocket.launch \
  port:="${BRIDGE_PORT}" \
  address:="${BRIDGE_BIND}"
