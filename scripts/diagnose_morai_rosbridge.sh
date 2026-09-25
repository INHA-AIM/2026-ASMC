#!/usr/bin/env bash
# MORAI <-> rosbridge (WebSocket) diagnostic — NOT the UDP bridge.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
# shellcheck source=asmc_env.sh
source "$SCRIPT_DIR/asmc_env.sh"

cd "$ASMC_WS_ROOT"
source /opt/ros/noetic/setup.bash 2>/dev/null || true
source devel/setup.bash 2>/dev/null || true
[ -f "$MORAI_BRIDGE_ENV" ] && source "$MORAI_BRIDGE_ENV"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"

_port_listening() {
  local port="${1:-9090}"
  if command -v ss >/dev/null 2>&1; then
    ss -tln 2>/dev/null | grep -qE ":${port}([^0-9]|$)" && return 0
  fi
  if command -v netstat >/dev/null 2>&1; then
    netstat -tln 2>/dev/null | grep -qE ":${port}([^0-9]|$)" && return 0
  fi
  python3 -c "
import socket
s = socket.socket()
s.settimeout(1.0)
s.connect(('127.0.0.1', int(${port})))
s.close()
" 2>/dev/null && return 0
  pgrep -f "rosbridge_websocket" >/dev/null 2>&1
}

echo "=============================================="
echo " MORAI rosbridge Diagnostic (WebSocket — NOT UDP)"
echo "=============================================="
echo "ASMC_WS_ROOT       = $ASMC_WS_ROOT"
echo "ROS_MASTER_URI     = $ROS_MASTER_URI"
echo "MORAI Bridge IP    = ${MORAI_BRIDGE_IP:-127.0.0.1}  (set this in MORAI UI)"
echo "MORAI Bridge PORT  = ${MORAI_BRIDGE_PORT:-9090}"
echo ""

if ! rostopic list >/dev/null 2>&1; then
  echo "[FAIL] roscore not running -> start: ./scripts/rosbridge.sh"
  exit 1
fi
echo "[OK] roscore reachable"

if _port_listening "${MORAI_BRIDGE_PORT:-9090}"; then
  echo "[OK] port ${MORAI_BRIDGE_PORT:-9090} listening (rosbridge)"
else
  echo "[FAIL] port ${MORAI_BRIDGE_PORT:-9090} not listening"
  echo "  Run ./scripts/rosbridge.sh in another terminal and keep it open."
  exit 1
fi

echo ""
echo "--- rosbridge clients ---"
clients_out=$(timeout 3 rostopic echo /connected_clients -n 1 2>/dev/null || true)
echo "$clients_out"
if echo "$clients_out" | grep -q 'clients: \[\]'; then
  echo "[FAIL] No MORAI WebSocket client connected."
  exit 1
fi
echo "[OK] At least one rosbridge client connected"

echo ""
echo "--- MORAI topics ---"
for t in /Ego_topic /Object_topic; do
  if rostopic list 2>/dev/null | grep -F "$t" >/dev/null; then
    echo "  [OK] $t"
  else
    echo "  [--] $t"
  fi
done

if rostopic list 2>/dev/null | grep -F "/Ego_topic" >/dev/null; then
  echo ""
  echo "Ready: ./tools/research/lbc_teacher_bev/scripts/collect_lbc_bev.sh"
  exit 0
fi
echo ""
echo "[WARN] /Ego_topic missing — Play + Ego Network Connected?"
exit 1
