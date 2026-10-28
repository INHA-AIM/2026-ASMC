#!/usr/bin/env bash
# D: LiDAR (asmc_lidar). 재밍 구간에서도 계속 처리해 재밍 회피에 LiDAR 물체·벽을 준다.
source "$(dirname "$(readlink -f "$0")")/_common.sh"
# RViz 끄기: RVIZ=false ./scripts/run/D_lidar.sh
run_in_container "D_lidar" "roslaunch asmc_lidar lidar.launch rviz:=${RVIZ:-true}"
