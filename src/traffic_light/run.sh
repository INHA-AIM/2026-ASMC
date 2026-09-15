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

TRAFFIC_LIGHT_PYTHON="${ASMC_TRAFFIC_LIGHT_PYTHON:-/usr/bin/python3}"

if [[ ! -x "$TRAFFIC_LIGHT_PYTHON" ]]; then
  echo "ERROR: 신호등 추론 Python 실행 파일이 없습니다: $TRAFFIC_LIGHT_PYTHON" >&2
  exit 1
fi

exec "$TRAFFIC_LIGHT_PYTHON" "${DEPLOY_DIR}/live_infer.py" \
  --ckpt "${DEPLOY_DIR}/best.pt" "${RUN_ARGS[@]}"
