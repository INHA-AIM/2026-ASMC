#!/bin/bash
# MORAI 없이 rosbag으로 인지 파이프라인 E2E 스모크 테스트
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/noetic/setup.bash
source "$workspace/devel/setup.bash"
LOG=/tmp/e2e
mkdir -p "$LOG"
pkill -f rosmaster 2>/dev/null; pkill -f YOLODetection 2>/dev/null; sleep 1
roscore > "$LOG/roscore.log" 2>&1 &
sleep 3
# CameraMode is already ROS mode for the 2026-ASMC sensor bridge.
rosrun Main main /image_jpeg/compressed:=/cam_front > "$LOG/main.log" 2>&1 &
MAIN_PID=$!
rosrun Main YOLODetection.py /camera/image/compressed:=/cam_front > "$LOG/yolo.log" 2>&1 &
sleep 10
rosbag play -q /home/autonav/geom_static.bag /lidar3D:=/velodyne_points /Ego_topic:=/Competition_topic > "$LOG/bag.log" 2>&1 &
sleep 5
echo '=== nodes ==='; rosnode list
echo '=== topics ==='; rostopic list
if kill -0 "$MAIN_PID" 2>/dev/null; then echo MAIN_RUNNING; else echo MAIN_DEAD; fi
echo '=== hz /lidar_tracking_markers ==='; timeout 6 rostopic hz /lidar_tracking_markers 2>&1 | tail -2
echo '=== hz /ctrl_cmd_0 ==='; timeout 4 rostopic hz /ctrl_cmd_0 2>&1 | tail -2
echo '=== YOLO bbox sample ==='; timeout 15 rostopic echo -n1 /YOLO/BBox 2>&1 | head -15
echo '=== main.log ==='; tail -15 "$LOG/main.log"
echo '=== yolo.log ==='; tail -8 "$LOG/yolo.log"
kill "$MAIN_PID" 2>/dev/null
pkill -f YOLODetection 2>/dev/null; pkill -f 'rosbag' 2>/dev/null; pkill -f rosmaster 2>/dev/null; pkill -f rosout 2>/dev/null
echo TEST_DONE
