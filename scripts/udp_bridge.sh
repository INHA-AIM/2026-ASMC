#!/usr/bin/env bash
# Start MORAI UDP ↔ ROS bridges (NOT rosbridge / WebSocket).
# Packages: morai_udp_bridge + morai_sensor_bridge
#
# Simulator selection labels diagnostics/rosbag provenance. Both profiles use
# the same canonical ROS topics; packet compatibility is verified per release.
# The bridge always opens the union of ASMC_sensor_set_v2 and GT sensor ports.
# Optional GT ports simply remain idle when v2 is loaded in MORAI.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
# shellcheck source=asmc_env.sh
source "$SCRIPT_DIR/asmc_env.sh"

usage() {
  cat <<'EOF'
Usage: ./scripts/udp_bridge.sh [--simulator 26r1|competition] [roslaunch overrides]

Simulator profiles:
  26r1        MORAI 26.R1.H3 development simulator (default)
  competition MORAI 25.S4.MolitComp03 competition simulator

The bridge always listens for the v2 sensors and the optional GT Semantic
cameras/Instance LiDAR. Loading v2 leaves the optional ports idle; loading GT
activates them automatically. The script does not change MORAI UI settings.

Examples:
  ./scripts/udp_bridge.sh --simulator 26r1
  ./scripts/udp_bridge.sh --simulator competition
  ./scripts/udp_bridge.sh --simulator 26r1 timestamp_mode:=source
EOF
}

SIMULATOR="${ASMC_SIMULATOR:-26r1}"
LAUNCH_ARGS=()
while [ "$#" -gt 0 ]; do
  case "$1" in
    --simulator)
      if [ "$#" -lt 2 ]; then
        echo "[udp_bridge] ERROR: --simulator requires a value." >&2
        exit 2
      fi
      SIMULATOR="$2"
      shift 2
      ;;
    --simulator=*)
      SIMULATOR="${1#*=}"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    simulator_profile:=*)
      echo "[udp_bridge] ERROR: use --simulator instead of simulator_profile." >&2
      exit 2
      ;;
    *)
      LAUNCH_ARGS+=("$1")
      shift
      ;;
  esac
done

case "${SIMULATOR,,}" in
  26r1|26.r1|26.r1.h3|dev|dev_26r1_h3)
    SIMULATOR_PROFILE="dev_26r1_h3"
    SIMULATOR_LABEL="26.R1.H3 development"
    ;;
  competition|25s4|25.s4|25.s4.molitcomp03|competition_25s4_molitcomp03)
    SIMULATOR_PROFILE="competition_25s4_molitcomp03"
    SIMULATOR_LABEL="25.S4.MolitComp03 competition"
    ;;
  *)
    echo "[udp_bridge] ERROR: unknown simulator '$SIMULATOR'." >&2
    usage >&2
    exit 2
    ;;
esac

# Parse and shift our CLI arguments before sourcing catkin setup files. Catkin
# setup forwards the caller's positional parameters to _setup_util.py.
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
echo "  Simulator      : ${SIMULATOR_LABEL} (${SIMULATOR_PROFILE})"
CMD_IP="${MORAI_CMD_IP:-127.0.0.1}"
# If caller already passed cmd_ip:=..., do not override.
has_cmd_ip=0
has_semantic=0
has_instance=0
for a in "${LAUNCH_ARGS[@]}"; do
  case "$a" in
    cmd_ip:=*) has_cmd_ip=1 ;;
    enable_semantic_camera:=*) has_semantic=1 ;;
    enable_instance_lidar:=*) has_instance=1 ;;
  esac
done
EXTRA=()
if [ "$has_cmd_ip" -eq 0 ]; then
  EXTRA+=( "cmd_ip:=${CMD_IP}" )
fi
EXTRA+=( "simulator_profile:=${SIMULATOR_PROFILE}" )
if [ "$has_semantic" -eq 0 ]; then
  EXTRA+=( "enable_semantic_camera:=true" )
fi
if [ "$has_instance" -eq 0 ]; then
  EXTRA+=( "enable_instance_lidar:=true" )
fi
echo "  cmd_ip         : ${CMD_IP} (Ctrl → Windows MORAI Host PORT)"
echo "  Sensor ports   : v2 + optional GT union (automatic)"
echo "  Timestamp      : receive default; GT pairing: timestamp_mode:=source"
echo "  Verify         : rostopic hz /Ego_topic"
echo "                  rostopic hz /image_jpeg/compressed"
echo "                  rostopic hz /sem_front/image_jpeg/compressed  # GT only"
echo "                  rostopic hz /velodyne_points_instance         # GT only"
echo "=========================================="
echo ""

exec roslaunch integration_launch morai_udp_bridges.launch \
  "${EXTRA[@]}" "${LAUNCH_ARGS[@]}"
