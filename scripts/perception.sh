#!/usr/bin/env bash
# Launch the canonical perception runtime in its CUDA/TwinLite container.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
CONTAINER="${ASMC_PERCEPTION_CONTAINER:-asmc-perception-train}"

if [[ -f /.dockerenv ]]; then
  echo "ERROR: run scripts/perception.sh from the WSL/Linux host, not inside a container." >&2
  echo "The camera runtime needs the ${CONTAINER} CUDA/TwinLite environment." >&2
  exit 2
fi

running="$(docker inspect "$CONTAINER" --format '{{.State.Running}}' 2>/dev/null || true)"
if [[ "$running" != "true" ]]; then
  echo "[INFO] starting ${CONTAINER}"
  "$SCRIPT_DIR/docker_perception_up.sh" up
fi

tty_args=()
if [[ -t 0 && -t 1 ]]; then
  tty_args=(-it)
fi

echo "[INFO] perception runtime container: ${CONTAINER}"
echo "[INFO] launch: asmc_perception/perception.launch $*"

exec docker exec "${tty_args[@]}" "$CONTAINER" bash -lc \
  'cd /root/ws && source /opt/ros/noetic/setup.bash && source devel/setup.bash && exec roslaunch asmc_perception perception.launch "$@"' \
  asmc-perception-launch "$@"
