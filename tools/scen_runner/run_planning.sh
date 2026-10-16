#!/usr/bin/env bash
set -eo pipefail
SCEN_STACK_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCEN_STACK_WS="$(cd "$SCEN_STACK_DIR/../.." && pwd)"
source /opt/ros/noetic/setup.bash
source "$SCEN_STACK_WS/devel/setup.bash"
export PYTHONPATH="$SCEN_STACK_DIR${PYTHONPATH:+:$PYTHONPATH}"
# The legacy BBOX message lives in the separately built perception workspace.
SCEN_STACK_MESSAGES="$SCEN_STACK_WS/src/lidar+sensorfusion/devel/lib/python3/dist-packages"
if [ -d "$SCEN_STACK_MESSAGES" ]; then
  export PYTHONPATH="$PYTHONPATH:$SCEN_STACK_MESSAGES"
fi
exec /usr/bin/python3 -m scen_runner.orchestration.stack planning "$@"
