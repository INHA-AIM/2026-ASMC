cmake_minimum_required(VERSION 3.16)
project(asmc_lidar LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Camera receiver is reusable handoff infrastructure. Detection/tracking/front
# fusion are ported below as an opt-in shadow pipeline; 3-view fusion and the
# operational cutover remain Jaeho migration work. No camera inference or nested
# Main dependencies.
find_package(catkin REQUIRED COMPONENTS asmc_msgs roscpp diagnostic_msgs)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(Threads REQUIRED)
# catkin's system-dependency exporter uses these case-sensitive variable names.
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(yaml-cpp_LIBRARIES yaml-cpp)
set(OpenSSL_INCLUDE_DIRS ${OPENSSL_INCLUDE_DIR})
set(OpenSSL_LIBRARIES ${OPENSSL_LIBRARIES})
catkin_package(INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES asmc_lidar_frame_time asmc_lidar_camera_core asmc_lidar_camera_ros
  CATKIN_DEPENDS asmc_msgs roscpp diagnostic_msgs
  DEPENDS Eigen3 yaml-cpp OpenSSL)

add_library(asmc_lidar_frame_time src/frame_time_core.cpp src/projection.cpp src/bounded_pose.cpp)
target_include_directories(asmc_lidar_frame_time PUBLIC include)
target_link_libraries(asmc_lidar_frame_time PUBLIC Eigen3::Eigen)
target_compile_options(asmc_lidar_frame_time PRIVATE -Wall -Wextra)

add_library(asmc_lidar_camera_core src/camera_observations.cpp src/camera_calibration.cpp)
target_include_directories(asmc_lidar_camera_core PUBLIC include)
target_link_libraries(asmc_lidar_camera_core PUBLIC asmc_lidar_frame_time yaml-cpp OpenSSL::Crypto Threads::Threads)
target_compile_options(asmc_lidar_camera_core PRIVATE -Wall -Wextra)
add_library(asmc_lidar_camera_ros src/camera_receiver.cpp)
target_include_directories(asmc_lidar_camera_ros PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(asmc_lidar_camera_ros PUBLIC asmc_lidar_camera_core ${catkin_LIBRARIES})
add_dependencies(asmc_lidar_camera_ros ${catkin_EXPORTED_TARGETS})
add_executable(camera_observation_probe examples/camera_observation_probe.cpp)
target_link_libraries(camera_observation_probe asmc_lidar_camera_ros)
add_dependencies(camera_observation_probe ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(camera_receiver_ros_test test/camera_receiver_ros.test test/camera_receiver_ros_test.cpp)
  if(TARGET camera_receiver_ros_test)
    target_link_libraries(camera_receiver_ros_test asmc_lidar_camera_ros)
    add_dependencies(camera_receiver_ros_test ${catkin_EXPORTED_TARGETS})
  endif()
  foreach(test camera_buffer_test camera_calibration_test)
    add_executable(asmc_lidar_${test} test/${test}.cpp)
    target_link_libraries(asmc_lidar_${test} asmc_lidar_camera_core)
    target_compile_options(asmc_lidar_${test} PRIVATE -UNDEBUG)
    add_test(NAME asmc_lidar_${test} COMMAND asmc_lidar_${test})
  endforeach()
  add_test(NAME asmc_lidar_camera_calibration_hash_test
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/camera_calibration_hash_test.py $<TARGET_FILE:asmc_lidar_camera_calibration_test>)
  add_test(NAME asmc_lidar_camera_contract_test
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/camera_objects_contract_test.py)
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

install(TARGETS asmc_lidar_frame_time asmc_lidar_camera_core asmc_lidar_camera_ros camera_observation_probe
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# ---------------------------------------------------------------------------
# Shadow LiDAR pipeline: port of Main's common-geometry LiDAR path
# (ROI -> voxel -> roll/pitch leveling -> RANSAC -> clustering -> OBB ->
# tracking -> front camera fusion -> typed local/map), a ROS shadow node and a
# bag replay that both call the same runtime.
#
# Package-private on purpose: these libraries are STATIC and are NOT listed in
# catkin_package(LIBRARIES ...) above, so Main (which links ${catkin_LIBRARIES}
# of asmc_lidar) never links them. The catkin_package() exports above are
# unchanged. When PCL, pcl_conversions, GeographicLib or morai_msgs are not
# available the shadow targets are skipped with a warning; the targets above
# build exactly as before. CI configures with ASMC_LIDAR_SHADOW_REQUIRED=ON so
# that a missing dependency (or missing rostest data) fails instead of skipping.
# ---------------------------------------------------------------------------
option(ASMC_LIDAR_SHADOW_PIPELINE "Build the asmc_lidar shadow pipeline, node and replay" ON)
option(ASMC_LIDAR_SHADOW_REQUIRED "Fail the configure step instead of skipping shadow targets or tests" OFF)
if(ASMC_LIDAR_SHADOW_REQUIRED AND NOT ASMC_LIDAR_SHADOW_PIPELINE)
  message(FATAL_ERROR "ASMC_LIDAR_SHADOW_REQUIRED=ON needs ASMC_LIDAR_SHADOW_PIPELINE=ON")
endif()
set(ASMC_LIDAR_SHADOW_ACTIVE OFF)
if(ASMC_LIDAR_SHADOW_PIPELINE)
  find_package(PCL 1.10 QUIET COMPONENTS common filters kdtree search sample_consensus segmentation)
  foreach(shadow_dep pcl_conversions sensor_msgs std_msgs geometry_msgs morai_msgs rosbag tf2_ros)
    find_package(${shadow_dep} QUIET)
  endforeach()
  find_path(ASMC_LIDAR_GEOGRAPHICLIB_INCLUDE_DIR GeographicLib/UTMUPS.hpp)
  find_library(ASMC_LIDAR_GEOGRAPHICLIB_LIBRARY NAMES Geographic)
  set(shadow_missing "")
  foreach(shadow_flag PCL_FOUND pcl_conversions_FOUND sensor_msgs_FOUND std_msgs_FOUND
      geometry_msgs_FOUND morai_msgs_FOUND rosbag_FOUND tf2_ros_FOUND
      ASMC_LIDAR_GEOGRAPHICLIB_INCLUDE_DIR ASMC_LIDAR_GEOGRAPHICLIB_LIBRARY)
    if(NOT ${shadow_flag})
      list(APPEND shadow_missing ${shadow_flag})
    endif()
  endforeach()
  if(shadow_missing AND ASMC_LIDAR_SHADOW_REQUIRED)
    message(FATAL_ERROR "asmc_lidar shadow pipeline is required but cannot be built; missing: ${shadow_missing}")
  elseif(shadow_missing)
    message(WARNING "asmc_lidar shadow pipeline not built; missing: ${shadow_missing}")
  else()
    set(ASMC_LIDAR_SHADOW_ACTIVE ON)
  endif()
endif()

if(ASMC_LIDAR_SHADOW_ACTIVE)
  set(shadow_msg_include_dirs ${sensor_msgs_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS} ${morai_msgs_INCLUDE_DIRS} ${pcl_conversions_INCLUDE_DIRS}
    ${rosbag_INCLUDE_DIRS} ${tf2_ros_INCLUDE_DIRS})
  set(shadow_msg_targets ${catkin_EXPORTED_TARGETS} ${sensor_msgs_EXPORTED_TARGETS}
    ${std_msgs_EXPORTED_TARGETS} ${geometry_msgs_EXPORTED_TARGETS} ${morai_msgs_EXPORTED_TARGETS})

  add_library(asmc_lidar_pipeline STATIC
    src/preprocessing/roi_voxel.cpp
    src/preprocessing/leveling.cpp
    src/ground/ground_filter.cpp
    src/clustering/clustering.cpp
    src/obb/obb.cpp
    src/tracking/scan_frame.cpp
    src/tracking/kalman.cpp
    src/tracking/hungarian.cpp
    src/tracking/detections.cpp
    src/tracking/tracker.cpp
    src/pose/pose_history.cpp
    src/pose/gps_ingest.cpp
    src/pipeline/pipeline.cpp
    src/experimental/tunnel_filter_stage.cpp
    src/experimental/tunnel_filter/Lidar/TunnelFilter/TunnelFilterCore.cpp
    src/fusion/front_camera_fusion.cpp
    src/fusion/buffer_camera_source.cpp)
  set_target_properties(asmc_lidar_pipeline PROPERTIES POSITION_INDEPENDENT_CODE ON)
  target_include_directories(asmc_lidar_pipeline
    PUBLIC include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${ASMC_LIDAR_GEOGRAPHICLIB_INCLUDE_DIR}
    PRIVATE src/experimental/tunnel_filter)
  target_link_libraries(asmc_lidar_pipeline PUBLIC asmc_lidar_frame_time asmc_lidar_camera_core
    ${PCL_LIBRARIES} ${ASMC_LIDAR_GEOGRAPHICLIB_LIBRARY} ${catkin_LIBRARIES})
  target_compile_options(asmc_lidar_pipeline PRIVATE -Wall -Wextra)
  # Preserved PR #102 core: byte-identical copy, compiled without new warnings.
  set_source_files_properties(src/experimental/tunnel_filter/Lidar/TunnelFilter/TunnelFilterCore.cpp
    PROPERTIES COMPILE_OPTIONS "-w")

  add_library(asmc_lidar_shadow_runtime STATIC
    src/ros/runtime_config.cpp
    src/ros/typed_output.cpp
    src/ros/shadow_runtime.cpp
    src/ros/ros_inputs.cpp)
  set_target_properties(asmc_lidar_shadow_runtime PROPERTIES POSITION_INDEPENDENT_CODE ON)
  target_include_directories(asmc_lidar_shadow_runtime PUBLIC include ${catkin_INCLUDE_DIRS} ${shadow_msg_include_dirs})
  target_link_libraries(asmc_lidar_shadow_runtime PUBLIC asmc_lidar_pipeline yaml-cpp OpenSSL::Crypto ${catkin_LIBRARIES})
  target_compile_options(asmc_lidar_shadow_runtime PRIVATE -Wall -Wextra)
  add_dependencies(asmc_lidar_shadow_runtime ${shadow_msg_targets})

  add_executable(asmc_lidar_shadow_node src/ros/shadow_node.cpp)
  target_link_libraries(asmc_lidar_shadow_node asmc_lidar_shadow_runtime asmc_lidar_camera_ros
    ${tf2_ros_LIBRARIES} ${catkin_LIBRARIES})
  target_compile_options(asmc_lidar_shadow_node PRIVATE -Wall -Wextra)
  add_dependencies(asmc_lidar_shadow_node ${shadow_msg_targets})

  add_executable(asmc_lidar_shadow_replay tools/shadow_replay.cpp)
  target_link_libraries(asmc_lidar_shadow_replay asmc_lidar_shadow_runtime ${rosbag_LIBRARIES} ${catkin_LIBRARIES})
  target_compile_options(asmc_lidar_shadow_replay PRIVATE -Wall -Wextra)
  add_dependencies(asmc_lidar_shadow_replay ${shadow_msg_targets})

  install(TARGETS asmc_lidar_shadow_node asmc_lidar_shadow_replay
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

  if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    # gtest binaries are built with the package (not only by `make tests`) so
    # a plain ctest run, as in CI, never meets an unbuilt registered test.
    macro(asmc_lidar_shadow_gtest name)
      catkin_add_gtest(${name} ${ARGN})
      if(TARGET ${name})
        target_link_libraries(${name} asmc_lidar_shadow_runtime)
        target_include_directories(${name} PRIVATE src src/experimental/tunnel_filter)
        set_target_properties(${name} PROPERTIES EXCLUDE_FROM_ALL FALSE)
        add_dependencies(${name} ${shadow_msg_targets})
      endif()
    endmacro()
    asmc_lidar_shadow_gtest(asmc_lidar_test_tunnel_filter_core test/experimental/test_tunnel_filter_core.cpp)
    asmc_lidar_shadow_gtest(asmc_lidar_test_preprocessing test/pipeline/test_preprocessing.cpp)
    asmc_lidar_shadow_gtest(asmc_lidar_test_ground test/pipeline/test_ground.cpp)
    asmc_lidar_shadow_gtest(asmc_lidar_test_clustering test/pipeline/test_clustering.cpp)
    asmc_lidar_shadow_gtest(asmc_lidar_test_obb test/pipeline/test_obb.cpp)
    asmc_lidar_shadow_gtest(asmc_lidar_test_scan_frame test/pipeline/test_scan_frame.cpp)
    asmc_lidar_shadow_gtest(asmc_lidar_test_tracker test/pipeline/test_tracker.cpp)
    asmc_lidar_shadow_gtest(asmc_lidar_test_pose_history test/pipeline/test_pose_history.cpp)
    asmc_lidar_shadow_gtest(asmc_lidar_test_gps_ingest test/pipeline/test_gps_ingest.cpp)
    asmc_lidar_shadow_gtest(asmc_lidar_test_fusion_policy test/pipeline/test_fusion_policy.cpp)
    asmc_lidar_shadow_gtest(asmc_lidar_test_front_camera_fusion test/pipeline/test_front_camera_fusion.cpp)
    asmc_lidar_shadow_gtest(asmc_lidar_test_tunnel_filter_stage test/pipeline/test_tunnel_filter_stage.cpp)
    asmc_lidar_shadow_gtest(asmc_lidar_test_pipeline test/pipeline/test_pipeline.cpp)
    asmc_lidar_shadow_gtest(asmc_lidar_test_typed_output test/shadow/test_typed_output.cpp)
    asmc_lidar_shadow_gtest(asmc_lidar_test_runtime test/shadow/test_runtime.cpp)
    # The node rostest needs the camera calibration data of the sibling asmc_camera
    # source package (data only; no build or package.xml dependency).
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../asmc_camera/config/cameras_v3.yaml)
      add_rostest(test/ros/shadow_node.test DEPENDENCIES asmc_lidar_shadow_node)
    elseif(ASMC_LIDAR_SHADOW_REQUIRED)
      message(FATAL_ERROR "asmc_lidar shadow rostest is required but ../asmc_camera/config/cameras_v3.yaml was not found")
    else()
      message(STATUS "asmc_lidar shadow rostest skipped: ../asmc_camera/config/cameras_v3.yaml not found")
    endif()
  endif()
endif()

# Isolation contract (pure Python, no PCL needed): runs whenever tests are enabled.
if(CATKIN_ENABLE_TESTING)
  add_test(NAME asmc_lidar_shadow_contract_test
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/shadow_contract_test.py)
endif()
