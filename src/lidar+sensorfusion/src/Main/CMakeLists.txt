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
)

find_package(OpenCV REQUIRED)

find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp roslib std_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs morai_msgs Msg
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
  src/Lidar/Kalman/Kalman.cpp
  src/Lidar/ImuRansac/ImuFusion.cpp
  src/Lidar/ImuRansac/Ransac.cpp
  src/Lidar/HungarianAlg/HungarianAlg.cpp
  src/Lidar/IMMFilter/IMMFilter.cpp
  src/SensorFusion/SensorFusion.cpp
  src/SensorFusion/FusionCamera.cpp
  src/SensorFusion/FusionLiDAR.cpp
  src/PlannerInterface/PlannerObjectAdapter.cpp
  src/PlannerInterface/PlannerObjectSender.cpp
)
add_dependencies(myHeader ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(myHeader ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} Geographic)

add_executable(main src/main.cpp)
add_dependencies(main myHeader ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(main myHeader
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  yaml-cpp
  Geographic
)

catkin_install_python(PROGRAMS
  src/Camera/YOLO/YOLODetection.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
