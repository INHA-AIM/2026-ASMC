#!/usr/bin/env bash
set -eo pipefail
SCEN_STACK_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCEN_STACK_WS="$(cd "$SCEN_STACK_DIR/../.." && pwd)"
source /opt/ros/noetic/setup.bash
source "$SCEN_STACK_WS/devel/setup.bash"
export PYTHONPATH="$SCEN_STACK_DIR${PYTHONPATH:+:$PYTHONPATH}"
exec python3 -m scen_runner.orchestration.stack bridge "$@"
