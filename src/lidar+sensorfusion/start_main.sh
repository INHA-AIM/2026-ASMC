#!/usr/bin/env bash
set -eo pipefail

workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
start_velodyne_driver="${START_VELODYNE_DRIVER:-false}"

# Keep the lock descriptor open for the lifetime of roslaunch. This prevents a
# second start_main.sh from replacing ROS nodes and racing for the camera/LiDAR
# UDP ports.
exec 9>"$workspace/.start_main.lock"
if ! flock -n 9; then
    echo "ERROR: DAECHANGMO is already running (start_main.sh lock is held)." >&2
    echo "Stop the existing launch with Ctrl+C before starting it again." >&2
    exit 1
fi

# Also catch a launch started manually, or a partial launch left alive after
# /Main exited. roslaunch must be the only owner of these three receive ports.
if [[ "$start_velodyne_driver" == "true" ]] &&
   ss -H -lun "sport = :9299" 2>/dev/null | grep -q .; then
    echo "ERROR: UDP 9299 is already in use." >&2
    echo "Use morai_sensor_bridge with START_VELODYNE_DRIVER=false," >&2
    echo "or stop the other UDP receiver before enabling the local driver." >&2
    exit 1
fi

source /opt/ros/noetic/setup.bash
source "$workspace/devel/setup.bash"

cd "$workspace"
echo "Starting lidar+sensorfusion (local Velodyne driver: $start_velodyne_driver)..."
exec roslaunch Main main.launch \
    start_velodyne_driver:="$start_velodyne_driver" \
    lidar_port:=9299 lidar_cut_angle:=0.0 "$@"
