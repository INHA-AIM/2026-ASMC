cmake_minimum_required(VERSION 3.16)
project(asmc_lidar_l1b4 LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(LEGACY_MAIN_SOURCE "" CACHE PATH "Read-only pinned Son Main source")
find_package(Python3 REQUIRED COMPONENTS Interpreter)
execute_process(COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/prepare.py
  --source ${LEGACY_MAIN_SOURCE} --output ${CMAKE_CURRENT_BINARY_DIR}
  RESULT_VARIABLE status)
if(NOT status EQUAL 0)
  message(FATAL_ERROR "Pinned source verification failed")
endif()
find_package(catkin REQUIRED COMPONENTS roscpp pcl_conversions morai_msgs Msg tf tf2_ros)
find_package(PCL REQUIRED COMPONENTS common filters segmentation search)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
add_library(l1b4_policy policy.cpp
  ${CMAKE_CURRENT_BINARY_DIR}/pinned/Lidar/TunnelFilter/TunnelFilterCore.cpp)
target_include_directories(l1b4_policy PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/pinned
  ${CMAKE_CURRENT_SOURCE_DIR} ${LEGACY_MAIN_SOURCE}/src ${PCL_INCLUDE_DIRS})
target_link_libraries(l1b4_policy PUBLIC ${PCL_LIBRARIES})
add_executable(tunnel_policy_eval evaluate.cpp ${LEGACY_MAIN_SOURCE}/src/Lidar/RoiVoxel/RoiVoxel.cpp)
target_include_directories(tunnel_policy_eval PRIVATE ${CMAKE_CURRENT_BINARY_DIR}
  ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(tunnel_policy_eval PRIVATE l1b4_policy yaml-cpp ${catkin_LIBRARIES} ${OpenCV_LIBS})
enable_testing()
add_executable(test_geometry_policy test_policy.cpp)
target_link_libraries(test_geometry_policy PRIVATE l1b4_policy)
add_test(NAME geometry_policy COMMAND test_geometry_policy)
