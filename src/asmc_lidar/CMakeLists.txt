cmake_minimum_required(VERSION 3.16)
project(asmc_lidar LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Handoff foundation only. Jaeho adds detection/tracking/ROS targets after porting
# the original pipeline. Do not pull research backends into the default build.
find_package(catkin REQUIRED COMPONENTS asmc_msgs)
find_package(Eigen3 REQUIRED)
catkin_package(INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES asmc_lidar_frame_time CATKIN_DEPENDS asmc_msgs)

add_library(asmc_lidar_frame_time src/frame_time_core.cpp src/projection.cpp src/bounded_pose.cpp)
target_include_directories(asmc_lidar_frame_time PUBLIC include)
target_link_libraries(asmc_lidar_frame_time PUBLIC Eigen3::Eigen)
target_compile_options(asmc_lidar_frame_time PRIVATE -Wall -Wextra)

if(CATKIN_ENABLE_TESTING)
  add_executable(asmc_lidar_geometry_boundary_test test/geometry_boundary_test.cpp)
  target_link_libraries(asmc_lidar_geometry_boundary_test PRIVATE asmc_lidar_frame_time)
  add_test(NAME asmc_lidar_geometry_boundary_test COMMAND asmc_lidar_geometry_boundary_test)
  add_executable(asmc_lidar_frame_time_core_test test/frame_time_core_test.cpp)
  target_link_libraries(asmc_lidar_frame_time_core_test PRIVATE asmc_lidar_frame_time)
  add_test(NAME asmc_lidar_frame_time_core_test COMMAND asmc_lidar_frame_time_core_test)
  add_executable(asmc_lidar_imu_selection_test test/imu_selection_test.cpp)
  target_include_directories(asmc_lidar_imu_selection_test PRIVATE include)
  target_compile_options(asmc_lidar_imu_selection_test PRIVATE -UNDEBUG)
  add_test(NAME asmc_lidar_imu_selection_test COMMAND asmc_lidar_imu_selection_test)
endif()

install(TARGETS asmc_lidar_frame_time
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
