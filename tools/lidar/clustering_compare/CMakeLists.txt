cmake_minimum_required(VERSION 3.16)
project(asmc_lidar_l1b5a LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(LEGACY_MAIN_SOURCE "" CACHE PATH "Read-only pinned Son Main source")
set(TRAVEL_CORE_SOURCE "" CACHE PATH "Read-only fixed TRAVEL core/travel directory")
find_package(Python3 REQUIRED COMPONENTS Interpreter)
execute_process(COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/prepare.py
  --source ${LEGACY_MAIN_SOURCE} --travel ${TRAVEL_CORE_SOURCE} --output ${CMAKE_CURRENT_BINARY_DIR}
  RESULT_VARIABLE status)
if(NOT status EQUAL 0)
  message(FATAL_ERROR "Pinned source verification/generation failed")
endif()
find_package(catkin REQUIRED COMPONENTS roscpp pcl_conversions morai_msgs Msg tf tf2_ros)
find_package(PCL REQUIRED COMPONENTS common filters segmentation search)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
add_library(l1b5_frozen_filter ../tunnel_geometry/policy.cpp
  ${CMAKE_CURRENT_BINARY_DIR}/pinned/Lidar/TunnelFilter/TunnelFilterCore.cpp)
target_include_directories(l1b5_frozen_filter PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/pinned
  ${CMAKE_CURRENT_SOURCE_DIR}/../tunnel_geometry ${LEGACY_MAIN_SOURCE}/src ${PCL_INCLUDE_DIRS})
target_link_libraries(l1b5_frozen_filter PUBLIC ${PCL_LIBRARIES})
add_executable(clustering_eval ${CMAKE_CURRENT_BINARY_DIR}/cluster_evaluate.cpp
  ${LEGACY_MAIN_SOURCE}/src/Lidar/RoiVoxel/RoiVoxel.cpp)
target_include_directories(clustering_eval PRIVATE ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR}
  ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(clustering_eval PRIVATE l1b5_frozen_filter yaml-cpp ${catkin_LIBRARIES} ${OpenCV_LIBS})
enable_testing()
add_executable(test_clustering test_clustering.cpp)
target_include_directories(test_clustering PRIVATE ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR}
  ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(test_clustering PRIVATE l1b5_frozen_filter yaml-cpp ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_test(NAME indexed_clustering COMMAND test_clustering)
