#!/usr/bin/env bash
# C: 객체 카메라 (COCO7 three-view, PyTorch RTX50)
source "$(dirname "$(readlink -f "$0")")/_common.sh"
run_in_container "C_camera_objects" 'CFG=$(rospack find asmc_camera)/config/camera &&
  roslaunch asmc_camera camera_objects.launch \
    node_name:=camera_objects managed:=true initial_mode:=2 \
    config:=$CFG/coco7_three_view_pytorch_rtx50.yaml \
    component_config:=$CFG/objects_runtime_pytorch.yaml \
    conf_thres:=0.7'
