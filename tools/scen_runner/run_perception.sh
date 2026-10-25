#!/usr/bin/env bash
set -eo pipefail
SCEN_STACK_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCEN_STACK_WS="$(cd "$SCEN_STACK_DIR/../.." && pwd)"
source /opt/ros/noetic/setup.bash
source "$SCEN_STACK_WS/devel/setup.bash"
if [[ -f "$SCEN_STACK_WS/src/lidar+sensorfusion/devel/setup.bash" ]]; then
  source "$SCEN_STACK_WS/src/lidar+sensorfusion/devel/setup.bash" --extend
fi
export PYTHONPATH="$SCEN_STACK_DIR${PYTHONPATH:+:$PYTHONPATH}"
exec /usr/bin/python3 -m scen_runner.orchestration.stack perception "$@"
