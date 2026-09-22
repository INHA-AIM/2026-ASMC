#!/usr/bin/env bash
# Start MORAI UDP ↔ ROS bridges (NOT rosbridge / WebSocket).
# Packages: morai_udp_bridge + morai_sensor_bridge
#
# Both supported MORAI releases run the same morai_udp protocol and canonical
# ROS topics. Release selection belongs to the JSON imported in the MORAI UI,
# not to this runtime command.
# The bridge opens the union of ASMC_sensor_set_v2 and GT sensor ports; optional
# GT ports simply remain idle when a regular sensor set is loaded in MORAI.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
# shellcheck source=asmc_env.sh
source "$SCRIPT_DIR/asmc_env.sh"

usage() {
  cat <<'EOF'
Usage: ./scripts/udp_bridge.sh [--transport local|lan2lan] [roslaunch overrides]

Transport profiles:
  local   MORAI and Autonav bridge on one workstation via 127.0.0.1 (default)
  lan2lan MORAI PC 192.168.0.1 and Autonav PC 192.168.0.10

The bridge always listens for the v2 sensors and the optional GT Semantic
cameras/Instance LiDAR. Loading v2 leaves the optional ports idle; loading GT
activates them automatically. The script does not change MORAI UI settings.

Examples:
  ./scripts/udp_bridge.sh
  ./scripts/udp_bridge.sh --transport lan2lan
  ./scripts/udp_bridge.sh timestamp_mode:=source
EOF
}

RUNTIME_PROFILE="morai_udp"
TRANSPORT="${ASMC_TRANSPORT:-local}"
LAUNCH_ARGS=()
while [ "$#" -gt 0 ]; do
  case "$1" in
    --transport)
      if [ "$#" -lt 2 ]; then
        echo "[udp_bridge] ERROR: --transport requires a value." >&2
        exit 2
      fi
      TRANSPORT="$2"
      shift 2
      ;;
    --transport=*)
      TRANSPORT="${1#*=}"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --simulator|--simulator=*|simulator_profile:=*)
      echo "[udp_bridge] ERROR: --simulator was removed because both MORAI releases use one runtime." >&2
      echo "  Import the release-matching JSON in MORAI; select only --transport here." >&2
      exit 2
      ;;
    runtime_profile:=*)
      echo "[udp_bridge] ERROR: runtime_profile is fixed to morai_udp." >&2
      exit 2
      ;;
    transport_profile:=*)
      echo "[udp_bridge] ERROR: use --transport instead of transport_profile." >&2
      exit 2
      ;;
    *)
      LAUNCH_ARGS+=("$1")
      shift
      ;;
  esac
done

case "${TRANSPORT,,}" in
  local)
    TRANSPORT_PROFILE="local"
    TRANSPORT_CMD_IP="127.0.0.1"
    ;;
  lan2lan)
    TRANSPORT_PROFILE="lan2lan"
    TRANSPORT_CMD_IP="192.168.0.1"
    ;;
  *)
    echo "[udp_bridge] ERROR: unknown transport '$TRANSPORT'." >&2
    usage >&2
    exit 2
    ;;
esac
NETWORK_PRESET_26R1="config/morai/network/generated/ASMC_network_set_26r1_${TRANSPORT_PROFILE}.json"
NETWORK_PRESET_COMP="config/morai/network/generated/ASMC_network_set_molitcomp03_${TRANSPORT_PROFILE}.json"

# Parse and shift our CLI arguments before sourcing catkin setup files. Catkin
# setup forwards the caller's positional parameters to _setup_util.py.
cd "$ASMC_WS_ROOT"
for preset in "$NETWORK_PRESET_26R1" "$NETWORK_PRESET_COMP"; do
  if [ ! -f "$ASMC_WS_ROOT/$preset" ]; then
    echo "[udp_bridge] ERROR: generated network preset is missing: $preset" >&2
    echo "  python3 tools/bridge/render_morai_network_profiles.py --write" >&2
    exit 1
  fi
done
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
echo "  Runtime        : ${RUNTIME_PROFILE} (26r1 and competition shared)"
echo "  Transport      : ${TRANSPORT_PROFILE}"
echo "  MORAI preset   : import ONE file matching the installed release"
echo "                   26r1       -> ${NETWORK_PRESET_26R1}"
echo "                   competition -> ${NETWORK_PRESET_COMP}"
CMD_IP="${MORAI_CMD_IP:-$TRANSPORT_CMD_IP}"
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
EXTRA+=( "runtime_profile:=${RUNTIME_PROFILE}" )
EXTRA+=( "transport_profile:=${TRANSPORT_PROFILE}" )
if [ "$has_semantic" -eq 0 ]; then
  EXTRA+=( "enable_semantic_camera:=true" )
fi
if [ "$has_instance" -eq 0 ]; then
  EXTRA+=( "enable_instance_lidar:=true" )
fi
echo "  cmd_ip         : ${CMD_IP} (Ctrl → Windows MORAI Host PORT)"
echo "  Ctrl default   : 9093/9094, mode 2"
echo "  Ego status     : 9088/9099 (both MORAI releases normalized)"
echo "  Competition    : 7603/7502 -> /Competition_topic"
echo "  Sensor ports   : v2 + optional GT union (automatic)"
echo "  Timestamp      : receive default; GT pairing: timestamp_mode:=source"
echo "  Verify         : rostopic hz /Ego_topic"
echo "                  rostopic hz /Competition_topic"
echo "                  rostopic hz /image_jpeg/compressed"
echo "                  rostopic hz /sem_front/image_jpeg/compressed  # GT only"
echo "                  rostopic hz /velodyne_points_instance         # GT only"
echo "=========================================="
echo ""

exec roslaunch integration_launch morai_udp_bridges.launch \
  "${EXTRA[@]}" "${LAUNCH_ARGS[@]}"
