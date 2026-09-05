#!/usr/bin/env bash
# ROS 모듈 catkin 빌드. UDP-only Planning/Control은 별도 CMake 빌드한다.
set -euo pipefail
cd "$(dirname "$(readlink -f "$0")")/.."

if [[ ! -d src/MORAI-ROS_morai_msgs && ! -L src/MORAI-ROS_morai_msgs ]]; then
  echo "[build_ws] ERROR: src/MORAI-ROS_morai_msgs 가 없습니다."
  echo "  docs/getting-started.md §3 참고 (submodule 또는 symlink)"
  exit 1
fi

# bash -u: ROS_MASTER_URI may be unset when sourcing noetic setup
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
source /opt/ros/noetic/setup.bash
catkin_make "$@"
echo "[build_ws] done. source devel/setup.bash"
echo "[build_ws] planning/control: ./scripts/build_planning_control.sh"
