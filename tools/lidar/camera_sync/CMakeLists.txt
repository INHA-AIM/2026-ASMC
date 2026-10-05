cmake_minimum_required(VERSION 3.16)
project(asmc_lidar_camera_geometry LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(LEGACY_MAIN_SOURCE "" CACHE PATH "Immutable Son 6435af1 Main package")
find_package(Python3 REQUIRED COMPONENTS Interpreter)
execute_process(COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/prepare_reference.py
  --source ${LEGACY_MAIN_SOURCE} --output ${CMAKE_CURRENT_BINARY_DIR}
  RESULT_VARIABLE reference_status)
if(NOT reference_status EQUAL 0)
  message(FATAL_ERROR "Pinned reference extraction failed")
endif()
find_package(catkin REQUIRED COMPONENTS roscpp pcl_conversions morai_msgs Msg tf tf2_ros)
find_package(PCL REQUIRED COMPONENTS common filters segmentation search)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
add_executable(tunnel_geometry_eval geometry.cpp
  ${LEGACY_MAIN_SOURCE}/src/Lidar/TunnelFilter/TunnelFilterCore.cpp
  ${LEGACY_MAIN_SOURCE}/src/Lidar/RoiVoxel/RoiVoxel.cpp)
target_include_directories(tunnel_geometry_eval PRIVATE ${CMAKE_CURRENT_BINARY_DIR}
  ${LEGACY_MAIN_SOURCE}/src ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(tunnel_geometry_eval PRIVATE yaml-cpp ${PCL_LIBRARIES}
  ${catkin_LIBRARIES} ${OpenCV_LIBS})
