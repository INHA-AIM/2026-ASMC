cmake_minimum_required(VERSION 3.0.2)
project(asmc_camera)

find_package(catkin REQUIRED COMPONENTS
  asmc_msgs
  message_generation
  rospy
  sensor_msgs
  std_msgs
)

catkin_python_setup()

add_message_files(FILES InferenceStatus.msg)
add_service_files(FILES SetInferenceMode.srv)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS
    asmc_msgs
    message_runtime
    rospy
    sensor_msgs
    std_msgs
)

catkin_install_python(PROGRAMS
  scripts/camera_region_example_node.py
  scripts/object_visualizer_node.py
  scripts/object_detection_node.py
  scripts/camera_bev_fusion_node.py
  scripts/camera_bev_node.py
  scripts/camera_bev_visualizer_node.py
  scripts/lane_vector_visualizer_node.py
  scripts/lane_vectorizer_node.py
  scripts/semantic_inference_node.py
  scripts/semantic_visualizer_node.py
  scripts/traffic_light_node.py
  scripts/traffic_light_closed_loop_acceptance.py
  scripts/traffic_light_vehicle_acceptance.py
  scripts/traffic_light_visualizer_node.py
  tools/bev/audit_shadow_lane_map_alignment.py
  tools/bev/audit_shadow_lane_results.py
  tools/bev/build_shadow_lane_gt.py
  tools/bev/calibrate_shadow_lane_gt.py
  tools/bev/run_shadow_lane_bag_batch.py
  tools/bev/shadow_lane_family_offline_node.py
  tools/bev/shadow_map_rgb_bev_node.py
  tools/bev/summarize_invalid_map_distance.py
  tools/bev/summarize_invalid_support_review.py
  tools/bev/summarize_shadow_lane_family.py
  tools/bev/summarize_shadow_planning_usability.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_facade_contract.py)
  catkin_add_nosetests(test/test_traffic_light_contract.py)
  catkin_add_nosetests(test/test_traffic_light_visualization.py)
  catkin_add_nosetests(test/camera)
  catkin_add_nosetests(test/bev)
  catkin_add_nosetests(test/detection)
  catkin_add_nosetests(test/runtime)
endif()

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
