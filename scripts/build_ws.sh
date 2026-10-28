#!/usr/bin/env bash
# ROS1 catkin workspace 빌드. Behavior/Frenet/MPC도 현재 이 workspace에 포함된다.
set -euo pipefail
cd "$(dirname "$(readlink -f "$0")")/.."

if [[ ! -d src/MORAI-ROS_morai_msgs && ! -L src/MORAI-ROS_morai_msgs ]]; then
  echo "[build_ws] ERROR: src/MORAI-ROS_morai_msgs 가 없습니다."
  echo "  docs/onboarding/README.md §2 참고 (submodule 초기화)"
  exit 1
fi

# bash -u: ROS_MASTER_URI may be unset when sourcing noetic setup
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
source /opt/ros/noetic/setup.bash
exec python3 tools/build/build_workspace.py "$@"
