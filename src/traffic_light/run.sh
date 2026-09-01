#!/usr/bin/env bash
set -euo pipefail

DEPLOY_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUN_ARGS=("$@")

if [[ -f /opt/ros/noetic/setup.bash ]]; then
  # A sourced script sees this script's positional parameters. Hide our CLI
  # options so catkin's setup parser does not consume --image-topic, etc.
  set --
  source /opt/ros/noetic/setup.bash
fi

exec python3 "${DEPLOY_DIR}/live_infer.py" \
  --ckpt "${DEPLOY_DIR}/best.pt" "${RUN_ARGS[@]}"
