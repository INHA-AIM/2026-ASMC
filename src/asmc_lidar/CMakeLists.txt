cmake_minimum_required(VERSION 3.16)
project(asmc_lidar LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Camera receiver is reusable handoff infrastructure; detection/fusion/tracking
# remain Jaeho migration work. No camera inference or nested Main dependencies.
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
