#!/usr/bin/env bash
# Start MORAI UDP ↔ ROS bridges (NOT rosbridge / WebSocket).
# Packages: morai_udp_bridge + morai_sensor_bridge
# For WebSocket rosbridge instead: ./scripts/rosbridge.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
# shellcheck source=asmc_env.sh
source "$SCRIPT_DIR/asmc_env.sh"

cd "$ASMC_WS_ROOT"
source /opt/ros/noetic/setup.bash
source devel/setup.bash 2>/dev/null || true

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
export ROS_IP="${ROS_IP:-127.0.0.1}"

if ! rospack find morai_udp_bridge >/dev/null 2>&1 \
  || ! rospack find morai_sensor_bridge >/dev/null 2>&1; then
  echo "[udp_bridge] packages not found — build first:"
  echo "  ./scripts/build_ws.sh"
  echo "  # or: catkin_make --pkg morai_udp_bridge morai_sensor_bridge"
  exit 1
fi

if ! rostopic list >/dev/null 2>&1; then
  echo "[udp_bridge] Starting roscore..."
  roscore &
  ROSCORE_PID=$!
  for _ in $(seq 1 15); do
    rostopic list >/dev/null 2>&1 && break
    sleep 1
  done
  if ! rostopic list >/dev/null 2>&1; then
    echo "[udp_bridge] ERROR: roscore failed."
    kill "$ROSCORE_PID" 2>/dev/null || true
    exit 1
  fi
  echo "[udp_bridge] roscore ready."
fi

echo "=========================================="
echo "  UDP Bridge (morai_udp + morai_sensor)"
echo "  NOT rosbridge — binary UDP ↔ ROS topics"
echo "=========================================="
echo "  ASMC_WS_ROOT   : $ASMC_WS_ROOT"
echo "  ROS_MASTER_URI : $ROS_MASTER_URI"
echo "  Launch         : integration_launch/morai_udp_bridges.launch"
CMD_IP="${MORAI_CMD_IP:-127.0.0.1}"
# If caller already passed cmd_ip:=..., do not override.
has_cmd_ip=0
for a in "$@"; do
  case "$a" in cmd_ip:=*) has_cmd_ip=1 ;; esac
done
EXTRA=()
if [ "$has_cmd_ip" -eq 0 ]; then
  EXTRA+=( "cmd_ip:=${CMD_IP}" )
fi
echo "  cmd_ip         : ${CMD_IP} (Ctrl → Windows MORAI Host PORT)"
echo "  Verify         : rostopic hz /Ego_topic"
echo "                  rostopic hz /image_jpeg/compressed"
echo "                  rostopic hz /sem_front/image_jpeg/compressed  # if enable_semantic_camera:=true"
echo "=========================================="
echo ""

exec roslaunch integration_launch morai_udp_bridges.launch "${EXTRA[@]}" "$@"
