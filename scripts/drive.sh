#!/usr/bin/env bash
# Container-owned master, bridge, perception, autonomy and web consoles.
set -euo pipefail
workspace="$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)"
if [[ ! -r /opt/ros/noetic/setup.bash || ! -r "$workspace/devel/setup.bash" ]]; then
  echo 'Run inside the built ROS container for this checkout.' >&2
  exit 2
fi
source /opt/ros/noetic/setup.bash
source "$workspace/devel/setup.bash"
export PYTHONPATH="$workspace/tools/scen_runner:$workspace/tools/runtime:${PYTHONPATH:-}"
exec python3 "$workspace/tools/runtime/standalone.py" "$@"
