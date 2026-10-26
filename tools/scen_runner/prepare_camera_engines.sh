#!/usr/bin/env bash
# Run manually after stopping MORAI/perception; never starts vehicle control.
set -eo pipefail
SCEN_ENGINE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source /opt/ros/noetic/setup.bash
source "$SCEN_ENGINE_ROOT/devel/setup.bash"
cd "$SCEN_ENGINE_ROOT"
SCEN_ENGINE_DEPS="$SCEN_ENGINE_ROOT/tools/scen_runner/.runtime/engine_build_python"
export PYTHONPATH="$SCEN_ENGINE_DEPS${PYTHONPATH:+:$PYTHONPATH}"
export YOLO_AUTOINSTALL=false
if [[ "${1:-}" != "--check" ]]; then
  # Keep build-only dependencies local; do not upgrade the ROS/PyTorch NumPy.
  if ! /usr/bin/python3 -c 'import onnxruntime, coloredlogs, flatbuffers, humanfriendly' 2>/dev/null; then
    /usr/bin/python3 -m pip install --target "$SCEN_ENGINE_DEPS" --no-deps \
      onnxruntime coloredlogs flatbuffers humanfriendly
  fi
fi
exec /usr/bin/python3 tools/perception/prepare_local_camera_engines.py "$@"
