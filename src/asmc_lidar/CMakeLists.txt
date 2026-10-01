cmake_minimum_required(VERSION 3.16)
project(asmc_lidar LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs visualization_msgs diagnostic_msgs pcl_conversions pcl_ros)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation search)
find_package(yaml-cpp REQUIRED)

catkin_package(INCLUDE_DIRS include LIBRARIES asmc_lidar_core
  CATKIN_DEPENDS roscpp sensor_msgs visualization_msgs diagnostic_msgs pcl_conversions pcl_ros)

add_library(asmc_lidar_core
  src/core.cpp src/lshape.cpp src/general.cpp
  vendor/jaeho/Lidar/TunnelFilter/TunnelFilterCore.cpp
  vendor/patchwork/src/patchworkpp.cpp vendor/patchwork/src/plane_fit.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../tools/lidar/frame_time/core.cpp)
target_include_directories(asmc_lidar_core PUBLIC include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../tools/lidar
  vendor/jaeho vendor/patchwork/include ${PCL_INCLUDE_DIRS})
target_link_libraries(asmc_lidar_core PUBLIC Eigen3::Eigen ${PCL_LIBRARIES} yaml-cpp)
target_compile_options(asmc_lidar_core PRIVATE -Wall -Wextra)

add_executable(asmc_lidar_review_node src/ros_node.cpp)
target_include_directories(asmc_lidar_review_node PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(asmc_lidar_review_node PRIVATE asmc_lidar_core ${catkin_LIBRARIES})
add_dependencies(asmc_lidar_review_node ${catkin_EXPORTED_TARGETS})

add_executable(asmc_lidar_offline src/offline.cpp)
target_link_libraries(asmc_lidar_offline PRIVATE asmc_lidar_core)

if(CATKIN_ENABLE_TESTING)
  add_executable(asmc_lidar_core_test test/core_test.cpp)
  target_link_libraries(asmc_lidar_core_test PRIVATE asmc_lidar_core)
  add_test(NAME asmc_lidar_core_test COMMAND asmc_lidar_core_test)
endif()

install(TARGETS asmc_lidar_core asmc_lidar_review_node asmc_lidar_offline
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
install(DIRECTORY config launch rviz DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
