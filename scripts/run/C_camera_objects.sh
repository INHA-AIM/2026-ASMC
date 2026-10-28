#!/usr/bin/env bash
# C: 객체 카메라 (collect v5 E3 mission15, TensorRT three-view)
# Packaged engine: RTX4060. Other GPUs require an E3 engine built for that runtime;
# see docs/operations/camera-deployment.md. No automatic engine build here.
source "$(dirname "$(readlink -f "$0")")/_common.sh"
run_in_container "C_camera_objects" 'roslaunch asmc_camera camera.launch models:=objects detector:=e3'
