#!/usr/bin/env bash
# ROS 없이 Frenet Planner와 MPC Controller를 빌드하고 함께 실행한다.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
"$REPO_ROOT/scripts/build_planning_control.sh"

PLANNER_BIN="$REPO_ROOT/src/frenet_planner/build/frenet_planner_node"
MPC_BIN="$REPO_ROOT/src/mpc_controller/build/mpc_node"

"$PLANNER_BIN" &
PLANNER_PID=$!

cleanup() {
  kill -TERM "$PLANNER_PID" 2>/dev/null || true
  wait "$PLANNER_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

sleep 1
"$MPC_BIN"
