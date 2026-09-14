cmake_minimum_required(VERSION 3.0.2)
project(drivable_bev)

find_package(catkin REQUIRED COMPONENTS
  asmc_msgs
  camera_semantic_perception
  diagnostic_msgs
  geometry_msgs
  message_filters
  morai_msgs
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    asmc_msgs
    camera_semantic_perception
    diagnostic_msgs
    geometry_msgs
    message_filters
    morai_msgs
    rospy
    sensor_msgs
    std_msgs
    visualization_msgs
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_grid.py)
  catkin_add_nosetests(test/test_calibration.py)
  catkin_add_nosetests(test/test_projector.py)
  catkin_add_nosetests(test/test_performance_monitor.py)
  catkin_add_nosetests(test/test_visualization.py)
  catkin_add_nosetests(test/test_fusion.py)
  catkin_add_nosetests(test/test_offline_evaluation.py)
  catkin_add_nosetests(test/test_model_grid_projection.py)
  catkin_add_nosetests(test/test_lane_evidence.py)
  catkin_add_nosetests(test/test_lane_tracking.py)
  catkin_add_nosetests(test/test_spline_fitting.py)
  catkin_add_nosetests(test/test_boundary_candidates.py)
  catkin_add_nosetests(test/test_boundary_output.py)
  catkin_add_nosetests(test/test_bag_batch.py)
  catkin_add_nosetests(test/test_shadow_result_audit.py)
  catkin_add_nosetests(test/test_shadow_map_gt.py)
  catkin_add_nosetests(test/test_pipeline_audit.py)
  catkin_add_nosetests(test/test_rgb_bev.py)
  catkin_add_nosetests(test/test_map_lane_calibration.py)
  catkin_add_nosetests(test/test_shared_lane_family.py)
  catkin_add_nosetests(test/test_shared_runtime_audit.py)
endif()

catkin_install_python(PROGRAMS
  scripts/camera_bev_node.py
  scripts/camera_bev_fusion_node.py
  scripts/camera_bev_visualizer_node.py
  scripts/lane_vectorizer_node.py
  scripts/lane_vector_visualizer_node.py
  scripts/run_shadow_lane_bag_batch.py
  scripts/audit_shadow_lane_results.py
  scripts/build_shadow_lane_gt.py
  scripts/audit_shadow_lane_map_alignment.py
  scripts/shadow_map_rgb_bev_node.py
  scripts/calibrate_shadow_lane_gt.py
  scripts/shadow_lane_family_offline_node.py
  scripts/summarize_shadow_lane_family.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
