#!/usr/bin/env bash
# Verify an already-running perception graph from the ROS Noetic container.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
CONTAINER="${ASMC_ROS_CONTAINER:-asmc-ros-noetic}"

if [[ -f /.dockerenv ]]; then
  echo "ERROR: run scripts/verify_perception.sh from the WSL/Linux host." >&2
  exit 2
fi

running="$(docker inspect "$CONTAINER" --format '{{.State.Running}}' 2>/dev/null || true)"
if [[ "$running" != "true" ]]; then
  echo "ERROR: ${CONTAINER} is not running; start it with scripts/docker_ros_up.sh up." >&2
  exit 2
fi

tty_args=()
if [[ -t 0 && -t 1 ]]; then
  tty_args=(-it)
fi

echo "[INFO] checking the already-running perception graph via ${CONTAINER}"
exec docker exec "${tty_args[@]}" "$CONTAINER" bash -lc \
  'verifier_args=("$@"); set --; cd /root/ws && source /opt/ros/noetic/setup.bash && source devel/setup.bash && exec python3 scripts/verify_perception_runtime.py "${verifier_args[@]}"' \
  asmc-perception-verifier "$@"
