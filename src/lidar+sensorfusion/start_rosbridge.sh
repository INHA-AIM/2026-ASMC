#!/usr/bin/env bash
set -eo pipefail

workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/noetic/setup.bash
source "$workspace/devel/setup.bash"

actual_package="$(rospack find morai_msgs)"
expected_package="$workspace/src/morai_msgs"

if [[ "$actual_package" != "$expected_package" ]]; then
  echo "ERROR: wrong morai_msgs is active: $actual_package" >&2
  echo "Expected: $expected_package" >&2
  exit 1
fi

if ! rosmsg show morai_msgs/EgoVehicleStatus | grep -q 'float32 wheel_angle'; then
  echo "ERROR: EgoVehicleStatus does not contain wheel_angle." >&2
  exit 1
fi

echo "OK: using $actual_package"
echo "OK: EgoVehicleStatus contains wheel_angle"
echo "Starting rosbridge websocket on TCP 9090..."
exec roslaunch rosbridge_server rosbridge_websocket.launch
