cmake_minimum_required(VERSION 3.16)
project(asmc_ground_support_parity LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
find_package(PCL REQUIRED COMPONENTS common filters search segmentation)
find_package(Eigen3 REQUIRED)
add_executable(pcl_parity pcl_parity.cpp)
target_include_directories(pcl_parity PRIVATE
  /root/ws/src/asmc_lidar/include
  /root/ws/src/asmc_lidar/vendor/jaeho
  /root/ws/src/asmc_lidar/vendor/patchwork/include
  /root/ws/tools/lidar
  ${PCL_INCLUDE_DIRS})
target_link_libraries(pcl_parity PRIVATE
  /root/ws/devel/lib/libasmc_lidar_core.so Eigen3::Eigen ${PCL_LIBRARIES})
