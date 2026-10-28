#!/usr/bin/env bash
# E: 판단·제어 (재밍 판정 + 플래너 + 컨트롤러). 차가 움직이므로 맨 마지막에 실행
source "$(dirname "$(readlink -f "$0")")/_common.sh"
# RViz 끄기: RVIZ=false ./scripts/run/E_autonomy.sh
run_in_container "E_autonomy" "roslaunch asmc_planner autonomy_no_bridges_v2.launch rviz:=${RVIZ:-true}"
