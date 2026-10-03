cmake_minimum_required(VERSION 3.16)
project(asmc_lidar LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs visualization_msgs diagnostic_msgs pcl_conversions pcl_ros)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation search)
find_package(yaml-cpp REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

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

# SHA-checked H1 variant in a private namespace; old runtime symbols unchanged.
set(LIVE_PP "${CMAKE_CURRENT_BINARY_DIR}/ground_live_patchwork")
execute_process(COMMAND ${Python3_EXECUTABLE}
  ${CMAKE_CURRENT_SOURCE_DIR}/../../tools/lidar/height_review/generate.py
  ${CMAKE_CURRENT_SOURCE_DIR}/../.. ${LIVE_PP} RESULT_VARIABLE LIVE_PP_RESULT)
if(NOT LIVE_PP_RESULT EQUAL 0)
  message(FATAL_ERROR "Pinned H1 source generation failed")
endif()
find_package(PCL REQUIRED COMPONENTS common filters segmentation search visualization)
set(LIVE_LINEFIT "${CMAKE_CURRENT_BINARY_DIR}/ground_live_linefit.cc")
execute_process(COMMAND ${Python3_EXECUTABLE}
  ${CMAKE_CURRENT_SOURCE_DIR}/../../tools/lidar/ground_live_review/generate_linefit.py
  ${CMAKE_CURRENT_SOURCE_DIR}/vendor/linefit/src/ground_segmentation.cc
  ${LIVE_LINEFIT} RESULT_VARIABLE LIVE_LINEFIT_RESULT)
if(NOT LIVE_LINEFIT_RESULT EQUAL 0)
  message(FATAL_ERROR "Pinned linefit adapter generation failed")
endif()
add_library(asmc_ground_live_backend
  src/ground_live.cpp ${LIVE_PP}/src/patchworkpp.cpp ${LIVE_PP}/src/plane_fit.cpp
  ${LIVE_LINEFIT} vendor/linefit/src/segment.cc
  vendor/linefit/src/bin.cc vendor/linefit/src/viewer.cc)
target_compile_definitions(asmc_ground_live_backend PRIVATE patchwork=patchwork_live)
target_include_directories(asmc_ground_live_backend BEFORE PRIVATE
  ${LIVE_PP}/include ${CMAKE_CURRENT_BINARY_DIR}/ground_live_linefit_include vendor/linefit/include)
target_link_libraries(asmc_ground_live_backend PUBLIC asmc_lidar_core ${PCL_LIBRARIES} pthread)
target_compile_options(asmc_ground_live_backend PRIVATE -O2)
add_executable(asmc_lidar_ground_live_node src/ground_live_node.cpp)
target_include_directories(asmc_lidar_ground_live_node PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(asmc_lidar_ground_live_node PRIVATE asmc_ground_live_backend ${catkin_LIBRARIES})
add_dependencies(asmc_lidar_ground_live_node ${catkin_EXPORTED_TARGETS})
add_executable(asmc_lidar_ground_live_offline src/ground_live_offline.cpp)
target_link_libraries(asmc_lidar_ground_live_offline PRIVATE asmc_ground_live_backend)

add_executable(asmc_lidar_review_node src/ros_node.cpp)
target_include_directories(asmc_lidar_review_node PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(asmc_lidar_review_node PRIVATE asmc_lidar_core ${catkin_LIBRARIES})
add_dependencies(asmc_lidar_review_node ${catkin_EXPORTED_TARGETS})

catkin_install_python(PROGRAMS scripts/ground_compare_view_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(asmc_lidar_offline src/offline.cpp)
target_link_libraries(asmc_lidar_offline PRIVATE asmc_lidar_core)
add_executable(asmc_lidar_ground_compare src/ground_compare.cpp)
target_link_libraries(asmc_lidar_ground_compare PRIVATE asmc_lidar_core)
target_compile_options(asmc_lidar_ground_compare PRIVATE -Wall -Wextra)

if(CATKIN_ENABLE_TESTING)
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
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
install(DIRECTORY config launch rviz DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
