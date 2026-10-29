include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../asmc_geometry/include)
cmake_minimum_required(VERSION 3.0.2)
project(lane_boundary_adapter)

add_compile_options(-std=c++17 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  asmc_msgs
  diagnostic_msgs
  geometry_msgs
  morai_msgs
  roscpp
  std_msgs
  visualization_msgs
  sensor_msgs
)

find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenSSL REQUIRED)

add_message_files(FILES CameraBBox.msg CameraBBoxFrame.msg CameraBBoxLaneResult.msg)
generate_messages(DEPENDENCIES std_msgs sensor_msgs geometry_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lane_boundary_adapter
  CATKIN_DEPENDS
    message_runtime
    asmc_msgs
    diagnostic_msgs
    geometry_msgs
    morai_msgs
    roscpp
    std_msgs
    visualization_msgs
    sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(lane_boundary_adapter
  src/boundary_family_adapter.cpp
  src/lane_boundary_adapter.cpp
  src/ground_projection.cpp
)
add_dependencies(lane_boundary_adapter ${catkin_EXPORTED_TARGETS})
target_link_libraries(lane_boundary_adapter ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(lane_boundary_monitor_node
  src/lane_boundary_monitor_node.cpp
)
add_dependencies(lane_boundary_monitor_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(lane_boundary_monitor_node
  lane_boundary_adapter
  ${catkin_LIBRARIES}
)

add_executable(boundary_family_monitor_node
  src/boundary_family_monitor_node.cpp
)
add_dependencies(boundary_family_monitor_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(boundary_family_monitor_node
  lane_boundary_adapter
  ${catkin_LIBRARIES}
)

add_executable(camera_bbox_lane_node src/camera_bbox_lane_node.cpp)
add_dependencies(camera_bbox_lane_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(camera_bbox_lane_node lane_boundary_adapter ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} yaml-cpp OpenSSL::Crypto)
add_executable(camera_bbox_lane_change_node src/camera_bbox_lane_change_node.cpp)
add_dependencies(camera_bbox_lane_change_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(camera_bbox_lane_change_node lane_boundary_adapter ${catkin_LIBRARIES})
catkin_install_python(PROGRAMS scripts/camera_bbox_crop.py scripts/camera_bbox_frame_bridge.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(decision_guard_test test/decision_guard_test.cpp)
  catkin_add_gtest(line_plan_resume_test test/line_plan_resume_test.cpp)
  if(TARGET line_plan_resume_test)
    add_dependencies(line_plan_resume_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(line_plan_resume_test lane_boundary_adapter ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(dynamic_obstacle_risk_test test/dynamic_obstacle_risk_test.cpp)
  catkin_add_gtest(obstacle_memory_test test/obstacle_memory_test.cpp)
  add_executable(local_support_probe EXCLUDE_FROM_ALL test/local_support_probe.cpp)
  add_dependencies(local_support_probe ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(local_support_probe lane_boundary_adapter ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} yaml-cpp OpenSSL::Crypto)
  # Historical S1/S2/S3 ROS fixtures use the archived wire/runtime, not PR127.
  catkin_add_gtest(contact_clearance_test test/contact_clearance_test.cpp)
  catkin_add_gtest(local_obstacle_support_test test/local_obstacle_support_test.cpp)
  if(TARGET local_obstacle_support_test)
    add_dependencies(local_obstacle_support_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(local_obstacle_support_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
  endif()

  catkin_add_nosetests(test/test_rviz_launch_contract.py)
  find_package(rostest REQUIRED)
  add_rostest(test/camera_frame_bridge.test)
  add_rostest(test/camera_bbox_lane_change.test DEPENDENCIES camera_bbox_lane_change_node camera_bbox_lane_node)
  add_rostest(test/camera_bbox_inactive_lifecycle.test DEPENDENCIES camera_bbox_lane_change_node camera_bbox_lane_node)
  add_rostest(test/camera_bbox_clipped.test DEPENDENCIES camera_bbox_lane_node)
  add_rostest(test/camera_bbox_ego_mask.test DEPENDENCIES camera_bbox_lane_node)
  add_rostest(test/camera_bbox_lane.test DEPENDENCIES camera_bbox_lane_node)
  add_rostest(test/camera_bbox_imu_projection.test DEPENDENCIES camera_bbox_lane_node)

  catkin_add_gtest(ground_projection_test test/ground_projection_test.cpp)
  if(TARGET ground_projection_test)
    add_dependencies(ground_projection_test ${catkin_EXPORTED_TARGETS})
    target_link_libraries(ground_projection_test lane_boundary_adapter ${catkin_LIBRARIES})
  endif()

  catkin_add_gtest(lane_boundary_adapter_test
    test/lane_boundary_adapter_test.cpp
  )
  if(TARGET lane_boundary_adapter_test)
    add_dependencies(lane_boundary_adapter_test ${catkin_EXPORTED_TARGETS})
    target_link_libraries(lane_boundary_adapter_test
      lane_boundary_adapter
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(boundary_family_adapter_test
    test/boundary_family_adapter_test.cpp
  )
  if(TARGET boundary_family_adapter_test)
    add_dependencies(boundary_family_adapter_test ${catkin_EXPORTED_TARGETS})
    target_link_libraries(boundary_family_adapter_test
      lane_boundary_adapter
      ${catkin_LIBRARIES}
    )
  endif()
endif()

install(TARGETS
    lane_boundary_adapter
    boundary_family_monitor_node
    lane_boundary_monitor_node
    camera_bbox_lane_node
    camera_bbox_lane_change_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

install(DIRECTORY config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
