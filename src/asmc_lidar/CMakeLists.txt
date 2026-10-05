cmake_minimum_required(VERSION 3.16)
project(asmc_lidar LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS asmc_bridge asmc_msgs morai_msgs roscpp sensor_msgs visualization_msgs diagnostic_msgs pcl_conversions pcl_ros)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation search)
find_package(yaml-cpp REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(OpenSSL REQUIRED)

catkin_package(INCLUDE_DIRS include LIBRARIES asmc_lidar_core
  CATKIN_DEPENDS asmc_bridge asmc_msgs morai_msgs roscpp sensor_msgs visualization_msgs diagnostic_msgs pcl_conversions pcl_ros)

add_library(asmc_lidar_core
  src/core.cpp src/lshape.cpp src/general.cpp
  vendor/jaeho/Lidar/TunnelFilter/TunnelFilterCore.cpp
  vendor/patchwork/src/patchworkpp.cpp vendor/patchwork/src/plane_fit.cpp
  src/frame_time_core.cpp)
target_include_directories(asmc_lidar_core PUBLIC include
  vendor/jaeho vendor/patchwork/include ${PCL_INCLUDE_DIRS})
target_link_libraries(asmc_lidar_core PUBLIC Eigen3::Eigen ${PCL_LIBRARIES} yaml-cpp)
target_compile_options(asmc_lidar_core PRIVATE -Wall -Wextra)

# SHA-checked H1 variant in a private namespace; old runtime symbols unchanged.
set(LIVE_PP "${CMAKE_CURRENT_BINARY_DIR}/ground_live_patchwork")
execute_process(COMMAND ${Python3_EXECUTABLE}
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/generate_patchwork.py
  ${CMAKE_CURRENT_SOURCE_DIR}/../.. ${LIVE_PP} RESULT_VARIABLE LIVE_PP_RESULT)
if(NOT LIVE_PP_RESULT EQUAL 0)
  message(FATAL_ERROR "Pinned H1 source generation failed")
endif()
find_package(PCL REQUIRED COMPONENTS common filters segmentation search visualization)
set(LIVE_LINEFIT "${CMAKE_CURRENT_BINARY_DIR}/ground_live_linefit.cc")
execute_process(COMMAND ${Python3_EXECUTABLE}
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/generate_linefit.py
  ${CMAKE_CURRENT_SOURCE_DIR}/vendor/linefit/src/ground_segmentation.cc
  ${LIVE_LINEFIT} RESULT_VARIABLE LIVE_LINEFIT_RESULT)
if(NOT LIVE_LINEFIT_RESULT EQUAL 0)
  message(FATAL_ERROR "Pinned linefit adapter generation failed")
endif()
execute_process(COMMAND ${Python3_EXECUTABLE}
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/generate_diagnostics.py
  ${CMAKE_CURRENT_SOURCE_DIR}/../.. ${LIVE_PP} ${LIVE_LINEFIT}
  RESULT_VARIABLE DIAGNOSTIC_RESULT)
if(NOT DIAGNOSTIC_RESULT EQUAL 0)
  message(FATAL_ERROR "Review diagnostic generation failed")
endif()
add_library(asmc_ground_live_backend
  src/ground_live.cpp src/ground_config.cpp src/road_anchor.cpp src/unified.cpp src/tracking.cpp ${LIVE_PP}/src/patchworkpp.cpp ${LIVE_PP}/src/plane_fit.cpp
  ${LIVE_LINEFIT} ${CMAKE_CURRENT_BINARY_DIR}/ground_live_linefit_segment.cc
  vendor/linefit/src/bin.cc vendor/linefit/src/viewer.cc)
target_compile_definitions(asmc_ground_live_backend PRIVATE patchwork=patchwork_live)
target_include_directories(asmc_ground_live_backend BEFORE PRIVATE
  ${LIVE_PP}/include ${CMAKE_CURRENT_BINARY_DIR}/ground_live_linefit_include vendor/linefit/include)
target_link_libraries(asmc_ground_live_backend PUBLIC asmc_lidar_core ${PCL_LIBRARIES} pthread OpenSSL::Crypto)
target_compile_options(asmc_ground_live_backend PRIVATE -O2)
add_executable(asmc_lidar_ground_live_node src/ground_live_node.cpp)
target_include_directories(asmc_lidar_ground_live_node PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(asmc_lidar_ground_live_node PRIVATE asmc_ground_live_backend ${catkin_LIBRARIES})
add_dependencies(asmc_lidar_ground_live_node ${catkin_EXPORTED_TARGETS})
add_executable(asmc_lidar_ground_live_offline src/ground_live_offline.cpp)
target_link_libraries(asmc_lidar_ground_live_offline PRIVATE asmc_ground_live_backend)
add_executable(asmc_lidar_unified_offline src/unified_offline.cpp)
target_link_libraries(asmc_lidar_unified_offline PRIVATE asmc_ground_live_backend)

add_executable(asmc_lidar_review_node src/ros_node.cpp)
target_include_directories(asmc_lidar_review_node PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(asmc_lidar_review_node PRIVATE asmc_lidar_core ${catkin_LIBRARIES})
add_dependencies(asmc_lidar_review_node ${catkin_EXPORTED_TARGETS})

catkin_install_python(PROGRAMS scripts/ground_compare_view_node.py scripts/legacy_yolo_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(asmc_lidar_offline src/offline.cpp)
target_link_libraries(asmc_lidar_offline PRIVATE asmc_lidar_core)
add_executable(asmc_lidar_ground_compare src/ground_compare.cpp)
target_link_libraries(asmc_lidar_ground_compare PRIVATE asmc_lidar_core)
target_compile_options(asmc_lidar_ground_compare PRIVATE -Wall -Wextra)

if(CATKIN_ENABLE_TESTING)
  add_executable(asmc_lidar_tracking_test test/tracking_test.cpp)
  target_link_libraries(asmc_lidar_tracking_test PRIVATE asmc_ground_live_backend)
  target_compile_options(asmc_lidar_tracking_test PRIVATE -UNDEBUG)
  add_test(NAME asmc_lidar_tracking_test COMMAND asmc_lidar_tracking_test)
  add_executable(asmc_lidar_unified_test test/unified_test.cpp)
  target_link_libraries(asmc_lidar_unified_test PRIVATE asmc_ground_live_backend)
  target_compile_options(asmc_lidar_unified_test PRIVATE -UNDEBUG)
  add_test(NAME asmc_lidar_unified_test COMMAND asmc_lidar_unified_test)
  add_executable(asmc_lidar_frame_time_core_test test/frame_time_core_test.cpp)
  target_link_libraries(asmc_lidar_frame_time_core_test PRIVATE asmc_lidar_core)
  target_compile_options(asmc_lidar_frame_time_core_test PRIVATE -UNDEBUG)
  add_test(NAME asmc_lidar_frame_time_core_test COMMAND asmc_lidar_frame_time_core_test)
  add_executable(asmc_lidar_ground_candidate_test test/ground_candidate_test.cpp)
  target_link_libraries(asmc_lidar_ground_candidate_test PRIVATE asmc_ground_live_backend)
  target_compile_options(asmc_lidar_ground_candidate_test PRIVATE -UNDEBUG)
  add_test(NAME asmc_lidar_ground_candidate_test COMMAND asmc_lidar_ground_candidate_test)
  add_executable(asmc_lidar_ground_sequence_test test/ground_sequence_test.cpp)
  target_link_libraries(asmc_lidar_ground_sequence_test PRIVATE asmc_ground_live_backend)
  add_executable(asmc_lidar_linefit_coverage_test test/linefit_coverage_test.cpp)
  target_include_directories(asmc_lidar_linefit_coverage_test BEFORE PRIVATE
    ${CMAKE_CURRENT_BINARY_DIR}/ground_live_linefit_include vendor/linefit/include)
  target_link_libraries(asmc_lidar_linefit_coverage_test PRIVATE asmc_ground_live_backend)
  target_compile_options(asmc_lidar_linefit_coverage_test PRIVATE -UNDEBUG)
  add_test(NAME asmc_lidar_linefit_coverage_test COMMAND asmc_lidar_linefit_coverage_test)
  add_executable(asmc_lidar_ground_live_test test/ground_live_test.cpp)
  target_link_libraries(asmc_lidar_ground_live_test PRIVATE asmc_ground_live_backend)
  target_compile_options(asmc_lidar_ground_live_test PRIVATE -UNDEBUG)
  add_test(NAME asmc_lidar_ground_live_test COMMAND asmc_lidar_ground_live_test
    ${CMAKE_CURRENT_SOURCE_DIR}/config/autonav_v3.yaml)
  add_executable(asmc_lidar_core_test test/core_test.cpp)
  target_link_libraries(asmc_lidar_core_test PRIVATE asmc_lidar_core)
  target_compile_options(asmc_lidar_core_test PRIVATE -UNDEBUG)
  add_test(NAME asmc_lidar_core_test COMMAND asmc_lidar_core_test)
  add_executable(asmc_lidar_frame_contract_test test/frame_contract_test.cpp)
  target_link_libraries(asmc_lidar_frame_contract_test PRIVATE asmc_lidar_core)
  target_compile_options(asmc_lidar_frame_contract_test PRIVATE -UNDEBUG)
  add_test(NAME asmc_lidar_frame_contract_test COMMAND asmc_lidar_frame_contract_test)
  add_executable(asmc_lidar_imu_selection_test test/imu_selection_test.cpp)
  target_link_libraries(asmc_lidar_imu_selection_test PRIVATE asmc_lidar_core)
  target_compile_options(asmc_lidar_imu_selection_test PRIVATE -UNDEBUG)
  add_test(NAME asmc_lidar_imu_selection_test COMMAND asmc_lidar_imu_selection_test)
endif()

install(TARGETS asmc_lidar_core asmc_lidar_review_node asmc_lidar_offline asmc_lidar_ground_compare
  asmc_ground_live_backend asmc_lidar_ground_live_node
  asmc_lidar_ground_live_offline
  asmc_lidar_unified_offline
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
install(DIRECTORY config launch rviz DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
