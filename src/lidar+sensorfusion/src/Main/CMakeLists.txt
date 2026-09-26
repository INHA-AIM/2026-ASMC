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
  Msg
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  diagnostic_msgs
)

find_package(OpenCV REQUIRED)

find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp roslib std_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs visualization_msgs diagnostic_msgs morai_msgs Msg
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
  src/Global/Global.cpp
  src/Parser/Parser.cpp
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
  src/Lidar/TunnelFilter/TunnelFilterCore.cpp
  src/Lidar/TunnelFilter/TunnelFilterRos.cpp
  src/SensorFusion/SensorFusion.cpp
  src/SensorFusion/FusionCamera.cpp
  src/SensorFusion/FusionLiDAR.cpp
  src/PlannerInterface/PlannerObjectAdapter.cpp
  src/PlannerInterface/PlannerObjectSender.cpp
  src/PlannerInterface/PlannerObjectRosPublisher.cpp
)
add_dependencies(myHeader ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(myHeader ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} yaml-cpp Geographic)

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
  catkin_add_gtest(test_motion_classifier
    test/test_motion_classifier.cpp
    src/Lidar/Tracking/MotionClassifier.cpp
  )
  if(TARGET test_motion_classifier)
    target_link_libraries(test_motion_classifier ${catkin_LIBRARIES})
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
  # 터널 실험 필터의 순수 로직(ROS 비의존) 단위 테스트.
  catkin_add_gtest(test_tunnel_filter_core
    test/test_tunnel_filter_core.cpp
    src/Lidar/TunnelFilter/TunnelFilterCore.cpp
  )
  if(TARGET test_tunnel_filter_core)
    target_link_libraries(test_tunnel_filter_core ${PCL_LIBRARIES})
  endif()
endif()

# 터널 실험 필터 결정론적 오프라인 평가기 (원본 bag 읽기 전용).
# 실시간 Main과 같은 라이브러리 함수를 bag 순서대로 호출한다.
find_package(rosbag QUIET)
if(rosbag_FOUND)
  add_executable(tunnel_filter_offline_eval tools/tunnel_filter_offline_eval.cpp)
  add_dependencies(tunnel_filter_offline_eval myHeader
    ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_include_directories(tunnel_filter_offline_eval PRIVATE
    ${rosbag_INCLUDE_DIRS})
  target_link_libraries(tunnel_filter_offline_eval myHeader
    ${catkin_LIBRARIES}
    ${rosbag_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
    yaml-cpp
    Geographic
  )
endif()

catkin_install_python(PROGRAMS
  src/Camera/YOLO/YOLODetection.py
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
