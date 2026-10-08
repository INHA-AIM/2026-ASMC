cmake_minimum_required(VERSION 3.0.2)
project(Main)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  geometry_msgs
  morai_msgs
  sensor_msgs
  pcl_conversions
  Msg
  asmc_lidar
  asmc_msgs
  diagnostic_msgs
  tf
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)

find_package(OpenCV REQUIRED)
find_package(OpenSSL REQUIRED)

find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS asmc_lidar asmc_msgs diagnostic_msgs roscpp roslib std_msgs geometry_msgs tf tf2 tf2_ros tf2_geometry_msgs visualization_msgs morai_msgs Msg pcl_conversions
)

include_directories(
  include
  src
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# 인지(라이다/카메라/센서퓨전) 전용 라이브러리.
# Planning/Control/Visualizer 소스는 이 리포에 없으므로 제외.
add_library(myHeader
  src/Geometry/FrameBoundary.cpp
  src/Geometry/FrameBoundaryPublisher.cpp
  src/Global/Global.cpp
  src/Parser/Parser.cpp
  src/Parser/GpsSanityFilter.cpp
  src/Parser/SensorPoseHistory.cpp
  src/Camera/Camera.cpp
  src/Camera/TrafficLight.cpp
  src/Camera/StopLine.cpp
  src/Camera/LaneDetection.cpp
  src/DeadReckoning/DeadReckoning.cpp
  src/Lidar/LidarProcess/LidarProcess.cpp
  src/Lidar/UDPReceive/LiDARReceiver.cpp
  src/Lidar/Clustering/Clustering.cpp
  src/Lidar/FrameFilter/FrameFilter.cpp
  src/Lidar/RoiVoxel/RoiVoxel.cpp
  src/Lidar/Visualization/Visualization.cpp
  src/Lidar/Tracking/Tracking.cpp
  src/Lidar/Tracking/MotionClassifier.cpp
  src/Lidar/Kalman/Kalman.cpp
  src/Lidar/ImuRansac/ImuFusion.cpp
  src/Lidar/ImuRansac/Ransac.cpp
  src/Lidar/HungarianAlg/HungarianAlg.cpp
  src/Lidar/IMMFilter/IMMFilter.cpp
  src/SensorFusion/SensorFusion.cpp
  src/SensorFusion/FusionCamera.cpp
  src/SensorFusion/FusionLiDAR.cpp
  src/PlannerInterface/DrivableFootprint.cpp
  src/PlannerInterface/PlannerObjectAdapter.cpp
  src/PlannerInterface/DrivableBoundaryFilter.cpp
  src/PlannerInterface/PlanningRelevanceFilter.cpp
  src/PlannerInterface/KnownStaticObjectFilter.cpp
  src/PlannerInterface/StaticStructureExclusionFilter.cpp
  src/PlannerInterface/PlannerObjectSender.cpp
  src/PlannerInterface/PlannerObjectRosPublisher.cpp
)
add_dependencies(myHeader ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(myHeader ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} yaml-cpp Geographic OpenSSL::Crypto)

add_executable(main src/main.cpp)
add_dependencies(main myHeader ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(main myHeader
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  yaml-cpp
  Geographic
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(rosbag REQUIRED)
  add_executable(lidar_continuous_replay test/lidar_continuous_replay.cpp)
  target_include_directories(lidar_continuous_replay PRIVATE ${rosbag_INCLUDE_DIRS})
  add_dependencies(lidar_continuous_replay ${catkin_EXPORTED_TARGETS})
  target_link_libraries(lidar_continuous_replay myHeader ${rosbag_LIBRARIES})
  add_rostest_gtest(test_frame_boundary_ros test/frame_boundary.test
    test/test_frame_boundary_ros.cpp test/test_runtime_globals.cpp)
  if(TARGET test_frame_boundary_ros)
    target_link_libraries(test_frame_boundary_ros myHeader ${catkin_LIBRARIES})
  endif()
  catkin_add_nosetests(test/test_camera_objects_adapter.py)
  catkin_add_nosetests(test/test_fusion_continuous_replay.py)
  catkin_install_python(PROGRAMS test/test_camera_objects_adapter_ros.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  add_rostest(test/camera_objects_adapter.test)
  catkin_add_gtest(test_ground_evidence test/test_ground_evidence.cpp)
  catkin_add_gtest(test_adaptive_clustering
    test/test_adaptive_clustering.cpp
    test/test_runtime_globals.cpp
  )
  if(TARGET test_adaptive_clustering)
    target_link_libraries(test_adaptive_clustering
      myHeader ${catkin_LIBRARIES} ${PCL_LIBRARIES})
  endif()
  catkin_add_gtest(test_tracking_association
    test/test_tracking_association.cpp
    test/test_runtime_globals.cpp
  )
  if(TARGET test_tracking_association)
    target_link_libraries(test_tracking_association
      myHeader ${catkin_LIBRARIES} ${PCL_LIBRARIES})
  endif()
  catkin_add_gtest(test_motion_classifier
    test/test_motion_classifier.cpp
    test/test_runtime_globals.cpp
  )
  if(TARGET test_motion_classifier)
    target_link_libraries(test_motion_classifier myHeader ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_fusion_class_policy
    test/test_fusion_class_policy.cpp
  )
  if(TARGET test_fusion_class_policy)
    target_link_libraries(test_fusion_class_policy ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_fusion_time_matcher
    test/test_fusion_time_matcher.cpp
  )
  if(TARGET test_fusion_time_matcher)
    target_link_libraries(test_fusion_time_matcher ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_fusion_replay test/test_fusion_replay.cpp)
  if(TARGET test_fusion_replay)
    target_link_libraries(test_fusion_replay ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_fusion_state_propagation test/test_fusion_state_propagation.cpp)
  if(TARGET test_fusion_state_propagation)
    add_dependencies(test_fusion_state_propagation ${catkin_EXPORTED_TARGETS})
    target_link_libraries(test_fusion_state_propagation ${catkin_LIBRARIES})
  endif()
  add_executable(fusion_continuous_trace_replay test/fusion_continuous_trace_replay.cpp)
  add_dependencies(fusion_continuous_trace_replay ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(fusion_continuous_trace_replay yaml-cpp ${catkin_LIBRARIES})
  add_executable(fusion_class_trace_replay test/fusion_class_trace_replay.cpp)
  add_dependencies(fusion_class_trace_replay ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(fusion_class_trace_replay yaml-cpp ${catkin_LIBRARIES})
  catkin_add_gtest(test_drivable_footprint
    test/test_drivable_footprint.cpp
    src/PlannerInterface/DrivableFootprint.cpp
  )
  if(TARGET test_drivable_footprint)
    target_link_libraries(test_drivable_footprint ${OpenCV_LIBRARIES})
  endif()
  catkin_add_gtest(test_drivable_boundary_filter
    test/test_drivable_boundary_filter.cpp
    src/PlannerInterface/DrivableBoundaryFilter.cpp
    src/PlannerInterface/DrivableFootprint.cpp
  )
  if(TARGET test_drivable_boundary_filter)
    target_link_libraries(test_drivable_boundary_filter
      ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} yaml-cpp)
  endif()
  catkin_add_gtest(test_planning_relevance_filter
    test/test_planning_relevance_filter.cpp
    src/PlannerInterface/PlanningRelevanceFilter.cpp
  )
  if(TARGET test_planning_relevance_filter)
    target_link_libraries(test_planning_relevance_filter
      ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_known_static_object_filter
    test/test_known_static_object_filter.cpp
    src/PlannerInterface/KnownStaticObjectFilter.cpp
  )
  if(TARGET test_known_static_object_filter)
    target_link_libraries(test_known_static_object_filter
      ${catkin_LIBRARIES} yaml-cpp)
  endif()
  catkin_add_gtest(test_static_structure_exclusion_filter
    test/test_static_structure_exclusion_filter.cpp
    src/PlannerInterface/StaticStructureExclusionFilter.cpp
  )
  if(TARGET test_static_structure_exclusion_filter)
    target_link_libraries(test_static_structure_exclusion_filter
      ${catkin_LIBRARIES} yaml-cpp)
  endif()
  catkin_add_gtest(test_gps_sanity_filter
    test/test_gps_sanity_filter.cpp
    src/Parser/GpsSanityFilter.cpp
  )
  if(TARGET test_gps_sanity_filter)
    target_link_libraries(test_gps_sanity_filter ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_sensor_pose_history
    test/test_sensor_pose_history.cpp
    test/test_runtime_globals.cpp
  )
  if(TARGET test_sensor_pose_history)
    target_link_libraries(test_sensor_pose_history myHeader ${catkin_LIBRARIES})
  endif()
endif()

catkin_install_python(PROGRAMS
  scripts/camera_objects_yolo_bbox_adapter.py
  src/Camera/YOLO/YOLODetection.py
  scripts/build_known_static_object_catalog.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# YOLO 노드는 CUDA 12.8 PyTorch가 설치된 Python 3.9 전용 환경에서 실행해야
# 한다. catkin_install_python이 만드는 relay 스크립트는 빌드 시점의 시스템
# Python으로 shebang이 고정되므로, 인터프리터를 명시하는 shell 래퍼를 같은
# libexec 디렉터리에 함께 둔다.
#   devel space  : 아래 file(COPY)가 devel/lib/Main 에 복사한다.
#   install space: install(PROGRAMS)가 shebang을 고치지 않고 그대로 설치한다.
# 두 경우 모두 래퍼와 YOLODetection.py가 같은 디렉터리에 놓이므로, 래퍼는
# 자기 절대 경로를 기준으로 노드 스크립트를 찾을 수 있다.
file(COPY scripts/yolo_detection_node
  DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}
  FILE_PERMISSIONS
    OWNER_READ OWNER_WRITE OWNER_EXECUTE
    GROUP_READ GROUP_EXECUTE
    WORLD_READ WORLD_EXECUTE
)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/yolo_detection_node
)

install(PROGRAMS scripts/yolo_detection_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
