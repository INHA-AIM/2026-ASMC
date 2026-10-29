#!/usr/bin/env bash
# Run from the host. /root/ws/bags is the repository's bind-mounted bags folder.
set -euo pipefail

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  echo "Usage: $0"
  echo "Records roundabout planning, objects, trajectory and control in bags/."
  echo "Stop with Ctrl-C and wait for rosbag to close the file."
  exit 0
fi
if (( $# )); then
  echo "Unexpected arguments. Use --help for usage." >&2
  exit 2
fi

docker_args=(exec -i)
if [[ -t 0 && -t 1 ]]; then
  docker_args+=(-t)
fi
exec docker "${docker_args[@]}" asmc-ros-noetic bash -lc '
  set -eo pipefail
  source /opt/ros/noetic/setup.bash
  source /root/ws/devel/setup.bash
  mkdir -p /root/ws/bags
  bag_path="/root/ws/bags/roundabout_$(TZ=Asia/Seoul date +%Y%m%d_%H%M%S_%N).bag"
  echo "Recording to: $bag_path"
  exec rosbag record --buffsize=512 -O "$bag_path" \
    /planning/vehicle/state \
    /planning/objects \
    /behavior/command_v2 \
    /planning/frenet_feedback_v2 \
    /planning/feedback \
    /planning/trajectory \
    /ctrl_cmd/normal \
    /ctrl_cmd \
    /control/binding_status \
    /planning/mode
'
