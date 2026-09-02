cmake_minimum_required(VERSION 3.0.2)
project(frenet_planner)

add_compile_options(-std=c++17 -O2)

find_package(catkin REQUIRED COMPONENTS
  asmc_msgs
  roscpp
)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS src
  LIBRARIES frenet_planner_core
  CATKIN_DEPENDS asmc_msgs roscpp
)

include_directories(
  src
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

# Geometry primitives are kept separate so the existing foundation test can
# exercise them without pulling in the complete planner.
add_library(frenet_geometry
  src/frenet_planner/geometry/polygon_utils.cpp
)
target_include_directories(frenet_geometry PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Transport-independent planner implementation shared by the ROS and legacy
# UDP runtimes. Networking and ROS callback code must not be added here.
add_library(frenet_planner_core
  src/frenet_planner/frenet_planner.cpp
  src/frenet_planner/obstacle/legacy_object_adapter.cpp
  src/frenet_planner/global/data_logger.cpp
  src/frenet_planner/visualization/planner_debug_writer.cpp
  src/frenet_planner/frenet/ref_line.cpp
  src/frenet_planner/frenet/path_generator.cpp
  src/frenet_planner/frenet/cost.cpp
  src/frenet_planner/frenet/collision_checker.cpp
  src/frenet_planner/frenet/leader_selector.cpp
  src/frenet_planner/frenet/avoidance_selector.cpp
  src/frenet_planner/frenet/merge_selector.cpp
  src/frenet_planner/math/polynomial.cpp
  src/frenet_planner/math/frenet_converter.cpp
)
add_dependencies(frenet_planner_core
  ${catkin_EXPORTED_TARGETS}
)
target_compile_definitions(frenet_planner_core PRIVATE
  FRENET_PACKAGE_SRC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/src"
)
target_link_libraries(frenet_planner_core
  frenet_geometry
  yaml-cpp
)

# Existing verified UDP runtime. It remains buildable as a regression baseline
# while the ROS runtime is introduced, but the two runtimes must not be launched
# simultaneously.
add_executable(frenet_planner_udp_legacy
  src/main.cpp
  src/udp_network/udp_manager.cpp
  src/udp_network/gps_transform.cpp
  src/udp_network/sender/sender.cpp
  src/udp_network/sender/planned_path_sender.cpp
  src/udp_network/receiver/receiver.cpp
  src/udp_network/receiver/ego_info_receiver.cpp
  src/udp_network/receiver/object_info_receiver.cpp
  src/udp_network/receiver/gps_receiver.cpp
  src/udp_network/receiver/imu_receiver.cpp
)
target_compile_definitions(frenet_planner_udp_legacy PRIVATE
  FRENET_PACKAGE_SRC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/src"
)
target_link_libraries(frenet_planner_udp_legacy
  frenet_planner_core
  Threads::Threads
)

# Minimal ROS lifecycle target. Subscribers, converters, the 20 Hz timer, and
# publishers are intentionally added in the next approved implementation step.
add_executable(frenet_planner_node
  src/ros/frenet_planner_node.cpp
)
add_dependencies(frenet_planner_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(frenet_planner_node
  frenet_planner_core
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  add_executable(geometry_foundation_test
    tests/geometry_foundation_test.cpp
  )
  target_link_libraries(geometry_foundation_test
    frenet_geometry
  )
  add_test(NAME geometry_foundation_test COMMAND geometry_foundation_test)

  add_executable(legacy_object_adapter_test
    tests/legacy_object_adapter_test.cpp
  )
  target_link_libraries(legacy_object_adapter_test
    frenet_planner_core
  )
  add_test(NAME legacy_object_adapter_test COMMAND legacy_object_adapter_test)

  add_executable(polygon_collision_test
    tests/polygon_collision_test.cpp
  )
  target_link_libraries(polygon_collision_test
    frenet_planner_core
  )
  add_test(NAME polygon_collision_test COMMAND polygon_collision_test)
endif()

install(TARGETS
    frenet_geometry
    frenet_planner_core
    frenet_planner_udp_legacy
    frenet_planner_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY src/frenet_planner/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

install(PROGRAMS tools/planner_visualizer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
    src/frenet_planner/config
    src/udp_network
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.yaml"
)
