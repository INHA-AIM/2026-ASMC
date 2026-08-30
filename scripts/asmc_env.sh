#!/usr/bin/env bash
# ASMC workspace root (host clone path or /root/ws in Docker).
# Source from other scripts: source "$(dirname "$0")/asmc_env.sh"
if [ -n "${ASMC:-}" ] && [ -d "${ASMC}/src" ]; then
  export ASMC_WS_ROOT="$(cd "${ASMC}" && pwd)"
elif [ -d /root/ws/src ]; then
  export ASMC_WS_ROOT=/root/ws
else
  export ASMC_WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
fi

# Rosbridge (WebSocket) env — NOT the UDP bridge.
export MORAI_ROSBRIDGE_ENV="${ASMC_WS_ROOT}/config/morai_rosbridge.env"
# Backward-compatible alias (old name = rosbridge, not UDP).
export MORAI_BRIDGE_ENV="${MORAI_ROSBRIDGE_ENV}"
