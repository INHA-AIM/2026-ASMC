#!/usr/bin/env bash
# bake → pose noise inject → drivable eval → bev eval → report
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
echo "[perception_eval] stub run_all — see docs/archive/perception/agentic-loop.md"
python3 tools/perception_eval/run_bake.py "$@"
python3 tools/perception_eval/inject_pose_noise.py "$@"
python3 tools/perception_eval/run_eval_drivable.py "$@"
python3 tools/perception_eval/run_eval_bev.py "$@"
python3 tools/perception_eval/report.py "$@"
echo "[perception_eval] stub done (no real metrics yet)"
