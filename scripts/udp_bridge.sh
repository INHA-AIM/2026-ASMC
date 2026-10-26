#!/usr/bin/env bash
# Start MORAI UDP ↔ ROS bridges (NOT rosbridge / WebSocket).
# Package: asmc_bridge (network UDP + sensor UDP + vehicle state)
#
# Both supported MORAI releases run the same morai_udp protocol and canonical
# ROS topics. Release selection belongs to the JSON imported in the MORAI UI,
# not to this runtime command.
# The default competition stream profile opens only the deployment allowlist.
# Object/Semantic/Instance inputs require the explicit local --gt option.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
# shellcheck source=asmc_env.sh
source "$SCRIPT_DIR/asmc_env.sh"

usage() {
  cat <<'EOF'
Usage: ./scripts/udp_bridge.sh [--transport local|lan2lan] [--gt] [--require-competition-status] [roslaunch overrides]

Transport profiles:
  local   MORAI and Autonav bridge on one workstation via 127.0.0.1 (default)
  lan2lan MORAI PC 192.168.0.1 and Autonav PC 192.168.0.10

Inputs:
  default  CompetitionInfoPublisher/Collision + RGB/LiDAR/GPS/IMU
           local additionally retains legacy Detail diagnostics
  --gt     adds Object/Semantic/Instance LiDAR; local transport only

The script does not change MORAI UI settings. Import the generated sensor JSON
matching the selected transport. v3 configures Intensity LiDAR at 15 Hz; GT
configures Intensity and Instance LiDAR at 15 Hz. Actual scan arrival can be lower.

For the verified MolitComp03 UI, CompetitionInfoPublisher uses 9088 -> 9099
and the ROS compatibility name /Ego_topic. --require-competition-status checks
that input. Legacy 9798 is local Detail diagnostics, not a field acceptance gate.

Examples:
  ./scripts/udp_bridge.sh
  ./scripts/udp_bridge.sh --gt
  ./scripts/udp_bridge.sh --transport lan2lan
  ./scripts/udp_bridge.sh --transport lan2lan --require-competition-status
  ./scripts/udp_bridge.sh timestamp_mode:=source
EOF
}

RUNTIME_PROFILE="morai_udp"
TRANSPORT="${ASMC_TRANSPORT:-local}"
GT=false
REQUIRE_COMPETITION_STATUS=false
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
    --gt)
      GT=true
      shift
      ;;
    --streams|--streams=*)
      echo "[udp_bridge] ERROR: --streams was replaced by --gt; omit --gt for normal inputs." >&2
      exit 2
      ;;
    --require-competition-status)
      REQUIRE_COMPETITION_STATUS=true
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
    stream_profile:=*)
      echo "[udp_bridge] ERROR: use --gt instead of stream_profile; omit it for normal inputs." >&2
      exit 2
      ;;
    --*)
      echo "[udp_bridge] ERROR: unknown option '$1'; see --help." >&2
      exit 2
      ;;
    *)
      LAUNCH_ARGS+=("$1")
      shift
      ;;
  esac
done

if [ -n "${ASMC_STREAMS:-}" ]; then
  echo "[udp_bridge] ERROR: ASMC_STREAMS was removed; unset it and use --gt when needed." >&2
  exit 2
fi

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
# Keep the node-level profiles stable; the public CLI only exposes GT on/off.
STREAM_PROFILE="competition"
if [ "$GT" = true ]; then
  STREAM_PROFILE="development"
  if [ "$TRANSPORT_PROFILE" != "local" ]; then
    echo "[udp_bridge] ERROR: --gt is local-only; use --transport local or omit --gt." >&2
    exit 2
  fi
fi
if [ "$REQUIRE_COMPETITION_STATUS" = true ] && [ "$GT" = true ]; then
  echo "[udp_bridge] ERROR: strict Competition acceptance cannot be combined with --gt." >&2
  exit 2
fi
PRESET_TRANSPORT="Local"
if [ "$TRANSPORT_PROFILE" = "lan2lan" ]; then PRESET_TRANSPORT="LAN2LAN"; fi
NETWORK_PRESET_26R1="config/morai/network/261027_ASMC_${PRESET_TRANSPORT}_Network_26.R1.H3.json"
NETWORK_PRESET_COMP="config/morai/network/261027_ASMC_${PRESET_TRANSPORT}_Network_25.S4.MolitComp03.json"
SENSOR_PRESET_V3="config/morai/sensors/261027_ASMC_${PRESET_TRANSPORT}_Sensor_25.S4.MolitComp03.json"
SENSOR_PRESET_R1="config/morai/sensors/261027_ASMC_${PRESET_TRANSPORT}_Sensor_26.R1.H3.json"
SENSOR_PRESET_GT="config/morai/sensors/261027_ASMC_Local_SensorGT_25.S4.MolitComp03.json"
SENSOR_PRESET_GT_R1="config/morai/sensors/261027_ASMC_Local_SensorGT_26.R1.H3.json"

# Parse and shift our CLI arguments before sourcing catkin setup files. Catkin
# setup forwards the caller's positional parameters to _setup_util.py.
cd "$ASMC_WS_ROOT"
PRESETS=( "$NETWORK_PRESET_26R1" "$NETWORK_PRESET_COMP" "$SENSOR_PRESET_V3" "$SENSOR_PRESET_R1" )
if [ "$GT" = true ]; then PRESETS+=( "$SENSOR_PRESET_GT" "$SENSOR_PRESET_GT_R1" ); fi
for preset in "${PRESETS[@]}"; do
  if [ ! -f "$ASMC_WS_ROOT/$preset" ]; then
    echo "[udp_bridge] ERROR: generated MORAI preset is missing: $preset" >&2
    echo "  python3 tools/bridge/render_morai_network_profiles.py --write" >&2
    echo "  python3 tools/bridge/render_morai_sensor_profiles.py --write"
    echo "  python3 tools/bridge/render_morai_release_presets.py --write" >&2
    exit 1
  fi
done
source /opt/ros/noetic/setup.bash
source devel/setup.bash 2>/dev/null || true

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
export ROS_IP="${ROS_IP:-127.0.0.1}"

if ! rospack find asmc_bridge >/dev/null 2>&1; then
  echo "[udp_bridge] asmc_bridge package not found — build first:"
  echo "  ./scripts/build_ws.sh"
  echo "  # or: catkin_make --pkg asmc_bridge"
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
echo "  Launch         : asmc_bridge/bridge.launch"
echo "  Runtime        : ${RUNTIME_PROFILE} (26r1 and competition shared)"
echo "  Transport      : ${TRANSPORT_PROFILE}"
echo "  GT inputs      : ${GT} (internal stream profile: ${STREAM_PROFILE})"
echo "  MORAI preset   : import ONE file matching the installed release"
echo "                   26r1       -> ${NETWORK_PRESET_26R1}"
echo "                   competition -> ${NETWORK_PRESET_COMP}"
if [ "$GT" = true ]; then
  echo "  Sensor preset  : MolitComp03 GT -> ${SENSOR_PRESET_GT}"
  echo "                   26r1 GT -> ${SENSOR_PRESET_GT_R1}"
  echo "                   Intensity + Instance LiDAR configured 15 Hz; reload in MORAI UI"
else
  echo "  Sensor preset  : MolitComp03 v3 -> ${SENSOR_PRESET_V3}"
  echo "                   26r1 v3 -> ${SENSOR_PRESET_R1}"
  echo "                   configured 15 Hz; actual complete scans may arrive slower"
fi
# Receive sockets bind the same ports for both transports. Only the outgoing
# Ctrl destination changes, so derive it from the explicit transport instead
# of a container-wide environment value that could silently keep LAN2LAN on
# 127.0.0.1.
CMD_IP="$TRANSPORT_CMD_IP"
# If caller already passed cmd_ip:=..., use that explicit override.
has_cmd_ip=0
for a in "${LAUNCH_ARGS[@]}"; do
  case "$a" in
    cmd_ip:=*)
      has_cmd_ip=1
      CMD_IP="${a#*=}"
      ;;
  esac
done
EXTRA=()
if [ "$has_cmd_ip" -eq 0 ]; then
  EXTRA+=( "cmd_ip:=${CMD_IP}" )
fi
EXTRA+=( "runtime_profile:=${RUNTIME_PROFILE}" )
EXTRA+=( "transport_profile:=${TRANSPORT_PROFILE}" )
EXTRA+=( "stream_profile:=${STREAM_PROFILE}" )
EXTRA+=( "competition_status_required:=${REQUIRE_COMPETITION_STATUS}" )
echo "  cmd_ip         : ${CMD_IP} (Ctrl → Windows MORAI Host PORT)"
echo "  Ctrl default   : 9093/9094, UDP CtrlMode=2 (Auto)"
echo "  MORAI JSON     : egoCtrlMode=3 (Auto); separate from UDP CtrlMode"
echo "  Status 9099    : verified MolitComp03 CompetitionInfoPublisher -> /Ego_topic"
echo "  Detail 9798    : local diagnostic only; disabled on LAN"
echo "  9099 gate      : required=${REQUIRE_COMPETITION_STATUS}; LAN input is always enabled"
echo "  TrafficLight GT: 7603/7502 (type 258; not consumed)"
echo "  Vehicle state  : GPS/IMU -> /vehicle/state"
echo "  Sensor ports   : competition allowlist 9291..9303"
if [ "$STREAM_PROFILE" = "development" ]; then
  echo "                   + Object/Semantic/Instance development inputs"
fi
echo "  Timestamp      : receive default; GT pairing: timestamp_mode:=source"
echo "  Verify         : rostopic hz /Ego_topic"
if [ "$TRANSPORT_PROFILE" = "local" ]; then
  echo "                  rostopic hz /Competition_topic # legacy local diagnostic, not Competition acceptance"
fi
echo "                  rostopic hz /image_jpeg/compressed"
echo "                  rostopic hz /vehicle/state"
if [ "$STREAM_PROFILE" = "development" ]; then
  echo "                  rostopic hz /sem_front/image_jpeg/compressed"
  echo "                  rostopic hz /velodyne_points_instance"
else
  echo "                  python3 scripts/verify_morai_topics.py --profile competition --transport ${TRANSPORT_PROFILE}"
  echo "                  verified 9099 gate: add --require-competition-status"
fi
echo "=========================================="
echo ""

exec roslaunch asmc_bridge bridge.launch \
  "${EXTRA[@]}" "${LAUNCH_ARGS[@]}"
