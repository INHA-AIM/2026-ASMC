#!/usr/bin/env bash
set -eo pipefail

workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
expected_package="$workspace/src/morai_msgs"
log_file="$(mktemp /tmp/daechangmo-lidar-check.XXXXXX.log)"
launch_pid=""

cleanup() {
  if [[ -n "$launch_pid" ]] && kill -0 "$launch_pid" 2>/dev/null; then
    kill -INT "$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
  fi
  rm -f "$log_file"
}
trap cleanup EXIT INT TERM

source /opt/ros/noetic/setup.bash
source "$workspace/devel/setup.bash"

check_vlp16_params() {
  local namespace="$1"
  local model
  local cut_angle

  model="$(rosparam get "/${namespace}_driver/model" 2>/dev/null || true)"
  cut_angle="$(rosparam get "/${namespace}_driver/cut_angle" 2>/dev/null || true)"

  if [[ "$model" != "VLP16" ]]; then
    echo "FAIL: running Velodyne model is '${model:-unknown}', expected VLP16" >&2
    echo "Stop the old Main launch with Ctrl+C, then run ./start_main.sh again." >&2
    return 1
  fi
  if ! python3 - "$cut_angle" <<'PY'
import math
import sys

try:
    value = float(sys.argv[1])
except (IndexError, ValueError):
    raise SystemExit(1)
raise SystemExit(0 if math.isclose(value, 0.0, abs_tol=1e-9) else 1)
PY
  then
    echo "FAIL: VLP-16 cut_angle is '${cut_angle:-unknown}', expected 0.0" >&2
    return 1
  fi
  echo "PASS: Velodyne uses VLP16 with azimuth-cut scan splitting"
}

check_vlp16_messages() {
  python3 - <<'PY'
import sys

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from velodyne_msgs.msg import VelodyneScan

rospy.init_node("daechangmo_vlp16_verify", anonymous=True, disable_signals=True)
try:
    scan = rospy.wait_for_message("/velodyne_packets", VelodyneScan, timeout=8.0)
    cloud = rospy.wait_for_message("/velodyne_points", PointCloud2, timeout=8.0)
except rospy.ROSException:
    print("WARN: no VLP-16 data received (check MORAI Connect/Destination IP/port 9299)")
    raise SystemExit(2)

if not scan.packets:
    print("FAIL: /velodyne_packets contains no packets", file=sys.stderr)
    raise SystemExit(1)

product_id = scan.packets[0].data[1205]
if product_id != 0x22:
    print(f"FAIL: MORAI packet product ID is 0x{product_id:02x}, expected VLP-16 (0x22)",
          file=sys.stderr)
    raise SystemExit(1)

rings = {int(point[0]) for point in pc2.read_points(
    cloud, field_names=("ring",), skip_nans=True)}
if not rings or min(rings) < 0 or max(rings) > 15:
    print(f"FAIL: converted cloud has invalid VLP-16 rings: {sorted(rings)}", file=sys.stderr)
    raise SystemExit(1)

print(f"PASS: MORAI product ID 0x22 converted to VLP-16 rings {min(rings)}..{max(rings)}")
PY
}

actual_package="$(rospack find morai_msgs)"
if [[ "$actual_package" != "$expected_package" ]]; then
  echo "FAIL: wrong morai_msgs: $actual_package" >&2
  exit 1
fi
echo "PASS: correct morai_msgs overlay"

python3 - <<'PY'
from morai_msgs.msg import EgoVehicleStatus
from rosbridge_library.internal.message_conversion import populate_instance

message = EgoVehicleStatus()
populate_instance({"wheel_angle": 1.25}, message)
assert message.wheel_angle == 1.25
print("PASS: rosbridge accepts EgoVehicleStatus.wheel_angle")
PY

running_nodes="$(rosnode list 2>/dev/null || true)"

# The 2026-ASMC integration uses one repository bridge for camera, LiDAR, GPS,
# and IMU. In this mode there is intentionally no local Velodyne nodelet and no
# /velodyne_packets topic; validate the converted ROS topics instead.
if grep -qx '/morai_sensor_bridge' <<<"$running_nodes"; then
  python3 - <<'PY'
import rospy
from sensor_msgs.msg import CompressedImage, PointCloud2

rospy.init_node("asmc_sensor_fusion_verify", anonymous=True, disable_signals=True)
try:
    cloud = rospy.wait_for_message("/velodyne_points", PointCloud2, timeout=8.0)
    image = rospy.wait_for_message("/image_jpeg/compressed", CompressedImage, timeout=8.0)
except rospy.ROSException as exc:
    print(f"FAIL: ASMC sensor bridge is running but sensor data is missing: {exc}")
    raise SystemExit(1)

if cloud.width * cloud.height == 0:
    print("FAIL: /velodyne_points is empty")
    raise SystemExit(1)
if not image.data:
    print("FAIL: /image_jpeg/compressed is empty")
    raise SystemExit(1)
print("PASS: morai_sensor_bridge publishes non-empty camera and LiDAR data")
PY
  exit 0
fi

# If the real stack is already running, inspect it in place. Starting another
# manager with the same ROS node names would unregister the live Velodyne nodes.
if grep -qx '/velodyne_nodelet_manager' <<<"$running_nodes"; then
  check_vlp16_params "velodyne_nodelet_manager"
  if ! ss -lun | grep -qE '(^|:)9299[[:space:]]'; then
    echo "FAIL: Velodyne manager is running but UDP 9299 is not listening" >&2
    exit 1
  fi
  echo "PASS: running Velodyne driver is listening on UDP 9299"
  check_vlp16_messages || {
    status=$?
    [[ "$status" -eq 2 ]] || exit "$status"
  }
  exit 0
fi

if grep -qx '/Main' <<<"$running_nodes"; then
  echo "FAIL: Main is running, but its Velodyne manager is missing." >&2
  echo "Stop start_main.sh with Ctrl+C, then run ./start_main.sh again." >&2
  exit 1
fi

# No real stack is running. Use a unique manager name so this standalone check
# cannot collide with a concurrently starting Main launch.
roslaunch velodyne_pointcloud VLP16_points.launch \
  manager:=daechangmo_verify_velodyne port:=9299 cut_angle:=0.0 >"$log_file" 2>&1 &
launch_pid=$!

for _ in $(seq 1 40); do
  if ss -lun | grep -qE '(^|:)9299[[:space:]]'; then
    echo "PASS: Velodyne driver is listening on UDP 9299"
    check_vlp16_params "daechangmo_verify_velodyne"
    check_vlp16_messages || {
      status=$?
      [[ "$status" -eq 2 ]] || exit "$status"
    }
    exit 0
  fi
  if ! kill -0 "$launch_pid" 2>/dev/null; then
    break
  fi
  sleep 0.25
done

echo "FAIL: Velodyne driver did not bind UDP 9299" >&2
cat "$log_file" >&2
exit 1
