#!/usr/bin/env bash
# DEPRECATED name — this is the ROS/WebSocket rosbridge, not the UDP bridge.
# Use: ./scripts/rosbridge.sh
# UDP: ./scripts/udp_bridge.sh [--transport local|lan2lan]
echo "[deprecated] scripts/bridge.sh → scripts/rosbridge.sh (rosbridge WebSocket, NOT UDP)" >&2
exec "$(cd "$(dirname "$(readlink -f "$0")")" && pwd)/rosbridge.sh" "$@"
