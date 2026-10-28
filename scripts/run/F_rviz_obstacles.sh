#!/usr/bin/env bash
# F: 장애물 박스 RViz (재밍 판단 장애물 + LiDAR OBB + 카메라 BBOX 이미지). 보기 전용, A~E 뒤에 실행
source "$(dirname "$(readlink -f "$0")")/_common.sh"
run_in_container "F_rviz_obstacles" \
  "rosrun rviz rviz -d /root/ws/src/jamming_lane_boundary_adapter/rviz/obstacle_boxes.rviz __name:=obstacle_boxes_rviz"
