cmake_minimum_required(VERSION 3.16)
project(asmc_lidar LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Camera receiver is reusable handoff infrastructure. Detection/tracking/front
# fusion form the canonical pipeline below. Jaeho owns subsequent tracking,
# velocity and 3-view fusion improvements. No camera inference or nested
# Main dependencies.
find_package(catkin REQUIRED COMPONENTS asmc_geometry asmc_msgs roscpp diagnostic_msgs)
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
  CATKIN_DEPENDS asmc_geometry asmc_msgs roscpp diagnostic_msgs
  DEPENDS Eigen3 yaml-cpp OpenSSL)

add_library(asmc_lidar_frame_time src/core/frame_time_core.cpp src/geometry/projection.cpp src/geometry/bounded_pose.cpp)
target_include_directories(asmc_lidar_frame_time PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(asmc_lidar_frame_time PUBLIC Eigen3::Eigen)
target_compile_options(asmc_lidar_frame_time PRIVATE -Wall -Wextra)

add_library(asmc_lidar_camera_core src/fusion/camera_observations.cpp src/geometry/camera_calibration.cpp)
target_include_directories(asmc_lidar_camera_core PUBLIC include)
target_link_libraries(asmc_lidar_camera_core PUBLIC asmc_lidar_frame_time yaml-cpp OpenSSL::Crypto Threads::Threads)
target_compile_options(asmc_lidar_camera_core PRIVATE -Wall -Wextra)
add_library(asmc_lidar_camera_ros src/ros/camera_receiver.cpp)
target_include_directories(asmc_lidar_camera_ros PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(asmc_lidar_camera_ros PUBLIC asmc_lidar_camera_core ${catkin_LIBRARIES})
add_dependencies(asmc_lidar_camera_ros ${catkin_EXPORTED_TARGETS})
add_executable(camera_observation_probe examples/camera_observation_probe.cpp)
target_link_libraries(camera_observation_probe asmc_lidar_camera_ros)
add_dependencies(camera_observation_probe ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
  foreach(confirmation_test observation_confirmation_standalone adaptive_confirmation_standalone)
    add_executable(asmc_lidar_${confirmation_test} test/experimental/${confirmation_test}.cpp)
    target_include_directories(asmc_lidar_${confirmation_test} PRIVATE include)
    target_compile_options(asmc_lidar_${confirmation_test} PRIVATE -Wall -Wextra -Werror -UNDEBUG)
    add_test(NAME asmc_lidar_${confirmation_test} COMMAND asmc_lidar_${confirmation_test})
  endforeach()
  find_package(rostest REQUIRED)
  add_rostest_gtest(camera_receiver_ros_test test/camera_receiver_ros.test test/camera_receiver_ros_test.cpp)
  if(TARGET camera_receiver_ros_test)
    set_target_properties(camera_receiver_ros_test PROPERTIES EXCLUDE_FROM_ALL FALSE)
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
catkin_install_python(PROGRAMS scripts/lidar_markers.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY rviz DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# ---------------------------------------------------------------------------
# Canonical LiDAR pipeline: preserved common-geometry LiDAR path
# (ROI -> voxel -> roll/pitch leveling -> RANSAC -> clustering -> OBB ->
# tracking -> front camera fusion -> typed local/map), a ROS migration node and a
# bag replay that both call the same runtime.
#
# Processing libraries remain package-private; only coordinate/time and camera
# receiver libraries are exported. Normal builds require all pipeline inputs.
# A receiver-only development build explicitly sets both PIPELINE and
# PIPELINE_REQUIRED to OFF. CI requires the full pipeline and its test data.
# ---------------------------------------------------------------------------
option(ASMC_LIDAR_PIPELINE "Build the canonical pipeline, node and replay" ON)
option(ASMC_LIDAR_PIPELINE_REQUIRED "Fail the configure step instead of skipping pipeline targets or tests" ON)
if(ASMC_LIDAR_PIPELINE_REQUIRED AND NOT ASMC_LIDAR_PIPELINE)
  message(FATAL_ERROR "ASMC_LIDAR_PIPELINE_REQUIRED=ON needs ASMC_LIDAR_PIPELINE=ON")
endif()
set(ASMC_LIDAR_PIPELINE_ACTIVE OFF)
if(ASMC_LIDAR_PIPELINE)
  find_package(PCL 1.10 QUIET COMPONENTS common filters kdtree search sample_consensus segmentation)
  foreach(pipeline_dep pcl_conversions sensor_msgs std_msgs geometry_msgs morai_msgs rosbag tf2_ros std_srvs)
    find_package(${pipeline_dep} QUIET)
  endforeach()
  find_path(ASMC_LIDAR_GEOGRAPHICLIB_INCLUDE_DIR GeographicLib/UTMUPS.hpp)
  find_library(ASMC_LIDAR_GEOGRAPHICLIB_LIBRARY NAMES Geographic)
  set(pipeline_missing "")
  foreach(pipeline_flag PCL_FOUND pcl_conversions_FOUND sensor_msgs_FOUND std_msgs_FOUND
      geometry_msgs_FOUND morai_msgs_FOUND rosbag_FOUND tf2_ros_FOUND std_srvs_FOUND
      ASMC_LIDAR_GEOGRAPHICLIB_INCLUDE_DIR ASMC_LIDAR_GEOGRAPHICLIB_LIBRARY)
    if(NOT ${pipeline_flag})
      list(APPEND pipeline_missing ${pipeline_flag})
    endif()
  endforeach()
  if(pipeline_missing AND ASMC_LIDAR_PIPELINE_REQUIRED)
    message(FATAL_ERROR "asmc_lidar canonical pipeline is required but cannot be built; missing: ${pipeline_missing}")
  elseif(pipeline_missing)
    message(WARNING "asmc_lidar migration pipeline not built; missing: ${pipeline_missing}")
  else()
    set(ASMC_LIDAR_PIPELINE_ACTIVE ON)
  endif()
endif()

if(ASMC_LIDAR_PIPELINE_ACTIVE)
  find_package(OpenCV REQUIRED COMPONENTS core imgproc)
  find_package(ZLIB REQUIRED)
  # Private decision-only library; never exported to Main and never owns ROS state.
  add_library(asmc_lidar_hd_map_audit STATIC src/filtering/hd_map_audit.cpp src/filtering/delivery_preview.cpp)
  target_include_directories(asmc_lidar_hd_map_audit PUBLIC include ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(asmc_lidar_hd_map_audit PUBLIC yaml-cpp OpenSSL::Crypto ZLIB::ZLIB ${OpenCV_LIBRARIES})
  target_compile_options(asmc_lidar_hd_map_audit PRIVATE -Wall -Wextra)
  if(CATKIN_ENABLE_TESTING)
    add_executable(asmc_lidar_hd_map_audit_probe test/hd_map_audit_probe.cpp)
    target_link_libraries(asmc_lidar_hd_map_audit_probe asmc_lidar_hd_map_audit)
    add_test(NAME asmc_lidar_hd_map_audit_equivalence
      COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/hd_map_audit_equivalence.py
        --probe $<TARGET_FILE:asmc_lidar_hd_map_audit_probe>)
  endif()
  set(pipeline_msg_include_dirs ${sensor_msgs_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS} ${morai_msgs_INCLUDE_DIRS} ${pcl_conversions_INCLUDE_DIRS}
    ${rosbag_INCLUDE_DIRS} ${tf2_ros_INCLUDE_DIRS})
  set(pipeline_msg_targets ${catkin_EXPORTED_TARGETS} ${sensor_msgs_EXPORTED_TARGETS}
    ${std_msgs_EXPORTED_TARGETS} ${geometry_msgs_EXPORTED_TARGETS} ${morai_msgs_EXPORTED_TARGETS})

  add_library(asmc_lidar_pipeline STATIC
    src/preprocessing/roi_voxel.cpp
    src/preprocessing/leveling.cpp
    src/preprocessing/ground_filter.cpp
    src/detection/clustering.cpp
    src/detection/obb.cpp
    src/detection/euclidean_components.cpp
    src/geometry/scan_frame.cpp
    src/tracking/kalman.cpp
    src/tracking/motion_classifier.cpp
    src/tracking/hungarian.cpp
    src/detection/detections.cpp
    src/tracking/tracker.cpp
    src/geometry/pose_history.cpp
    src/geometry/gps_ingest.cpp
    src/pipeline/pipeline.cpp
    src/experimental/tunnel_filter_stage.cpp
    src/experimental/tunnel_route.cpp
    src/experimental/tunnel_filter/Lidar/TunnelFilter/TunnelFilterCore.cpp
    src/fusion/multi_camera_fusion.cpp
    src/fusion/front_camera_fusion.cpp
    src/fusion/buffer_camera_source.cpp)
  set_target_properties(asmc_lidar_pipeline PROPERTIES POSITION_INDEPENDENT_CODE ON)
  target_include_directories(asmc_lidar_pipeline
    PUBLIC include ${PCL_INCLUDE_DIRS} ${ASMC_LIDAR_GEOGRAPHICLIB_INCLUDE_DIR}
    PRIVATE src/experimental/tunnel_filter)
  target_link_libraries(asmc_lidar_pipeline PUBLIC asmc_lidar_frame_time asmc_lidar_camera_core
    ${PCL_LIBRARIES} ${ASMC_LIDAR_GEOGRAPHICLIB_LIBRARY})
  target_compile_options(asmc_lidar_pipeline PRIVATE -Wall -Wextra)
  # PR #102 algorithms preserved; M1 extends protected semantic labels only.
  set_source_files_properties(src/experimental/tunnel_filter/Lidar/TunnelFilter/TunnelFilterCore.cpp
    PROPERTIES COMPILE_OPTIONS "-w")

  add_library(asmc_lidar_runtime STATIC
    src/filtering/observation_quality.cpp
    src/ros/runtime_config.cpp
    src/ros/typed_output.cpp
    src/ros/runtime.cpp
    src/ros/ros_inputs.cpp)
  set_target_properties(asmc_lidar_runtime PROPERTIES POSITION_INDEPENDENT_CODE ON)
  target_include_directories(asmc_lidar_runtime PUBLIC include ${catkin_INCLUDE_DIRS} ${pipeline_msg_include_dirs})
  target_link_libraries(asmc_lidar_runtime PUBLIC asmc_lidar_pipeline asmc_lidar_hd_map_audit yaml-cpp OpenSSL::Crypto ${catkin_LIBRARIES})
  target_compile_options(asmc_lidar_runtime PRIVATE -Wall -Wextra)
  add_dependencies(asmc_lidar_runtime ${pipeline_msg_targets})

  add_executable(lidar_pipeline_node src/ros/lidar_pipeline_node.cpp src/ros/debug_publisher.cpp)
  # A new YAML cannot update an old executable. Stamp package C++ content in
  # the node; edits/additions/removals reconfigure and relink it automatically.
  file(GLOB_RECURSE lidar_identity_sources CONFIGURE_DEPENDS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
    "include/*.hpp" "include/*.h" "src/*.cpp" "src/*.hpp" "src/*.h")
  list(APPEND lidar_identity_sources CMakeLists.txt package.xml cmake/build_info.hpp.in)
  list(SORT lidar_identity_sources)
  set(lidar_identity "")
  foreach(path IN LISTS lidar_identity_sources)
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/${path}")
    file(SHA256 "${CMAKE_CURRENT_SOURCE_DIR}/${path}" file_sha)
    string(APPEND lidar_identity "${path}\n${file_sha}\n")
  endforeach()
  string(SHA256 lidar_source_sha256 "${lidar_identity}")
  configure_file(cmake/build_info.hpp.in generated/asmc_lidar/build_info.hpp @ONLY)
  target_include_directories(lidar_pipeline_node PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/generated")
  target_link_libraries(lidar_pipeline_node asmc_lidar_runtime asmc_lidar_camera_ros
    ${tf2_ros_LIBRARIES} ${catkin_LIBRARIES})
  target_compile_options(lidar_pipeline_node PRIVATE -Wall -Wextra)
  add_dependencies(lidar_pipeline_node ${pipeline_msg_targets})

  add_executable(lidar_replay tools/lidar_replay.cpp)
  target_link_libraries(lidar_replay asmc_lidar_runtime ${rosbag_LIBRARIES} ${catkin_LIBRARIES})
  target_compile_options(lidar_replay PRIVATE -Wall -Wextra)
  add_dependencies(lidar_replay ${pipeline_msg_targets})

  install(PROGRAMS tools/analyze_velocity_bag.py tools/measure_velocity_response.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  install(FILES tools/tracking_samples.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

  install(TARGETS lidar_pipeline_node lidar_replay
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
  # Read-only web monitor (python, no build): node script, pure-python core and web assets.
  catkin_install_python(PROGRAMS scripts/asmc_lidar_monitor_node.py scripts/asmc_lidar_monitor_core.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  install(DIRECTORY web DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

  if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_test(NAME asmc_lidar_runtime_build_identity
      COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/runtime_build_identity_test.py
        $<TARGET_FILE:lidar_pipeline_node> ${CMAKE_CURRENT_SOURCE_DIR})
    # gtest binaries are built with the package (not only by `make tests`) so
    # a plain ctest run, as in CI, never meets an unbuilt registered test.
    macro(lidar_pipeline_gtest name)
      catkin_add_gtest(${name} ${ARGN})
      if(TARGET ${name})
        target_link_libraries(${name} asmc_lidar_runtime)
        target_include_directories(${name} PRIVATE src src/experimental/tunnel_filter)
        set_target_properties(${name} PROPERTIES EXCLUDE_FROM_ALL FALSE)
        add_dependencies(${name} ${pipeline_msg_targets})
      endif()
    endmacro()
    lidar_pipeline_gtest(asmc_lidar_test_tunnel_filter_core test/experimental/test_tunnel_filter_core.cpp)
    lidar_pipeline_gtest(asmc_lidar_test_preprocessing test/pipeline/test_preprocessing.cpp)
    lidar_pipeline_gtest(asmc_lidar_test_ground test/pipeline/test_ground.cpp)
    lidar_pipeline_gtest(asmc_lidar_test_clustering test/pipeline/test_clustering.cpp)
    lidar_pipeline_gtest(asmc_lidar_test_obb test/pipeline/test_obb.cpp)
    lidar_pipeline_gtest(asmc_lidar_test_euclidean_components test/pipeline/test_euclidean_components.cpp)
    lidar_pipeline_gtest(asmc_lidar_test_scan_frame test/pipeline/test_scan_frame.cpp)
    lidar_pipeline_gtest(asmc_lidar_test_tracker test/pipeline/test_tracker.cpp)
    lidar_pipeline_gtest(asmc_lidar_test_pose_history test/pipeline/test_pose_history.cpp)
    lidar_pipeline_gtest(asmc_lidar_test_gps_ingest test/pipeline/test_gps_ingest.cpp)
    lidar_pipeline_gtest(asmc_lidar_test_fusion_policy test/pipeline/test_fusion_policy.cpp)
    lidar_pipeline_gtest(asmc_lidar_test_front_camera_fusion test/pipeline/test_front_camera_fusion.cpp)
    lidar_pipeline_gtest(asmc_lidar_test_tunnel_filter_stage test/pipeline/test_tunnel_filter_stage.cpp)
    lidar_pipeline_gtest(asmc_lidar_test_tunnel_route test/pipeline/test_tunnel_route.cpp)
    lidar_pipeline_gtest(asmc_lidar_test_tunnel_runtime test/runtime/test_tunnel_runtime.cpp)
    lidar_pipeline_gtest(asmc_lidar_test_source_time test/pipeline/test_source_time.cpp)
    lidar_pipeline_gtest(asmc_lidar_test_pipeline test/pipeline/test_pipeline.cpp)
    lidar_pipeline_gtest(asmc_lidar_test_motion_classifier test/pipeline/test_motion_classifier.cpp)
    lidar_pipeline_gtest(asmc_lidar_test_observation_quality test/pipeline/test_observation_quality.cpp)
    lidar_pipeline_gtest(asmc_lidar_test_stage_observer test/pipeline/test_stage_observer.cpp)
    lidar_pipeline_gtest(asmc_lidar_test_typed_output test/runtime/test_typed_output.cpp)
    lidar_pipeline_gtest(asmc_lidar_test_runtime test/runtime/test_runtime.cpp)
    lidar_pipeline_gtest(asmc_lidar_test_delivery_preview test/runtime/test_delivery_preview.cpp)
    # The node rostest needs the camera calibration data of the sibling asmc_camera
    # source package (data only; no build or package.xml dependency).
    # Web monitor rostest: shadow node (debug stages on) + monitor node + synthetic inputs, no camera data needed.
    add_rostest(test/ros/monitor_node.test DEPENDENCIES lidar_pipeline_node)
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../asmc_camera/config/cameras_v3.yaml)
      add_rostest(test/ros/lidar_pipeline_node.test DEPENDENCIES lidar_pipeline_node)
    elseif(ASMC_LIDAR_PIPELINE_REQUIRED)
      message(FATAL_ERROR "asmc_lidar pipeline rostest is required but ../asmc_camera/config/cameras_v3.yaml was not found")
    else()
      message(STATUS "asmc_lidar pipeline rostest skipped: ../asmc_camera/config/cameras_v3.yaml not found")
    endif()
  endif()
endif()

# Isolation contract (pure Python, no PCL needed): runs whenever tests are enabled.
if(CATKIN_ENABLE_TESTING)
  add_test(NAME asmc_lidar_fusion_audit_test
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/../../tools/perception/test_camera_lidar_fusion_audit.py)
  add_test(NAME asmc_lidar_pipeline_contract_test
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/pipeline_contract_test.py)
  add_test(NAME asmc_lidar_monitor_core_test
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/monitor/test_monitor_core.py)
  add_test(NAME asmc_lidar_object_markers_test
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/object_markers_test.py)
endif()
