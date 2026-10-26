#!/usr/bin/env bash
# Run from the repository root after the single workspace build.
source .github/ci/ros-env.sh
port=$(python3 -c 'import socket; s=socket.socket(); s.bind(("127.0.0.1",0)); print(s.getsockname()[1]); s.close()')
export ROS_MASTER_URI=http://127.0.0.1:$port
export ROS_HOSTNAME=localhost
roscore -p "$port" > "$ASMC_CI_OUTPUT/camera-control-roscore.log" 2>&1 &
core_pid=$!
trap 'kill "$core_pid" 2>/dev/null || true; wait "$core_pid" 2>/dev/null || true' EXIT
python3 tools/perception/check_inference_control_ros.py
python3 tools/perception/check_camera_coco_ros.py --profile baseline
python3 tools/perception/check_camera_coco_ros.py --profile all
python3 tools/perception/check_camera_seg_ros.py
python3 tools/perception/check_camera_seg_ros.py --batched
roslaunch --nodes asmc_camera camera_objects_coco_experiment.launch classes:=all
roslaunch --nodes asmc_camera camera_objects_coco_experiment.launch classes:=baseline debug:=false
roslaunch --nodes asmc_camera camera_objects_seg_experiment.launch model:=yolo26 views:=front debug:=false
roslaunch --nodes asmc_camera camera_objects_seg_experiment.launch model:=yoloe views:=all debug:=false
roslaunch --nodes asmc_camera camera_objects_seg_experiment.launch model:=yolo26 classes:=mission debug:=false
roslaunch --nodes asmc_camera camera_objects_seg_experiment.launch model:=yoloe classes:=mission views:=all debug:=false
roslaunch --nodes asmc_camera camera_objects_yoloe_l_experiment.launch debug:=false
roslaunch --nodes asmc_camera camera_objects_yoloe_l_experiment.launch prompt_profile:=base27 batch_mode:=batch3 debug:=false
roslaunch --nodes asmc_camera camera_objects_three_view_experiment.launch debug:=false
roslaunch --nodes asmc_camera camera_objects_three_view_experiment.launch debug:=true
roslaunch --nodes asmc_camera camera_yoloe_experiment.launch model:=m front_width:=640 prompt_profile:=obstacle17
roslaunch --nodes asmc_camera camera_yoloe_experiment.launch model:=l front_width:=1280 start_camera_lane:=true start_traffic_light:=true
python3 tools/perception/check_region_example_ros.py
python3 tools/perception/check_object_visualization_ros.py
python3 tools/perception/check_object_visualization_ros.py --hood-mask
python3 tools/perception/check_object_visualization_ros.py --hood-mask --debug-raw
python3 tools/perception/check_object_visualization_ros.py --hood-mask --view left
python3 tools/perception/check_object_visualization_ros.py --hood-mask --view right
python3 tools/perception/check_object_mosaic_ros.py --output "$ASMC_CI_OUTPUT/object-mosaic-ros-check.json"
python3 tools/perception/check_object_view_ros.py
python3 tools/perception/check_hood_mask_ros.py
python3 tools/perception/check_ego_hood_filter_ros.py
python3 tools/perception/check_object_visualization_ros.py --ego-hood-filter
