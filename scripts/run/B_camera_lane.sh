#!/usr/bin/env bash
# B: 차선 카메라 (RTX50: cu128 PyTorch 런타임)
source "$(dirname "$(readlink -f "$0")")/_common.sh"
run_in_container "B_camera_lane" 'export ASMC_LANE_PYTHON=/opt/yolo-venv/bin/python &&
  roslaunch asmc_camera _runtime.launch mode:=active models:=lane \
    lane_runtime_config:=/root/ws/src/asmc_camera/config/camera/lane_runtime_pytorch.yaml'
