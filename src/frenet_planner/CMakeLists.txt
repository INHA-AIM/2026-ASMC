cmake_minimum_required(VERSION 3.0.2)
project(frenet_planner)

add_compile_options(-std=c++17 -O2)

find_package(catkin REQUIRED COMPONENTS
  asmc_msgs
  morai_msgs
  roscpp
  sensor_msgs
)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS src
  LIBRARIES frenet_planner_core
  CATKIN_DEPENDS asmc_msgs morai_msgs roscpp sensor_msgs
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

# ROS runtime with input validation, command-based Plan() invocation,
# fail-safe handling, and trajectory/feedback publication.
add_library(frenet_planner_message_converter
  src/ros/message_converter.cpp
)
add_dependencies(frenet_planner_message_converter
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(frenet_planner_message_converter
  frenet_geometry
  ${catkin_LIBRARIES}
)

# v2 인터페이스 어댑터. D3-1은 enum 매핑만 담고, D3-2에서 validation과
# fingerprint가 여기에 추가된다. FSM 계약 경계이므로 planner core가 아니라
# ROS 계층에 둔다 (docs/fsm-frenet-interface-v2.md 12.8절).
add_library(frenet_behavior_command_adapter
  src/ros/behavior_command_adapter.cpp
)
add_dependencies(frenet_behavior_command_adapter
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(frenet_behavior_command_adapter
  ${catkin_LIBRARIES}
)

add_executable(frenet_planner_node
  src/ros/main.cpp
  src/ros/frenet_planner_node.cpp
)
add_dependencies(frenet_planner_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(frenet_planner_node
  frenet_planner_message_converter
  frenet_planner_core
  ${catkin_LIBRARIES}
)

# Canonical vehicle-state boundary. GPS/IMU is the competition source and
# EgoVehicleStatus is development-only. No runtime fallback connects them.
add_library(frenet_vehicle_state_adapter
  src/ros/state/vehicle_state_adapter.cpp
  src/udp_network/gps_transform.cpp
)
add_dependencies(frenet_vehicle_state_adapter ${catkin_EXPORTED_TARGETS})
target_link_libraries(frenet_vehicle_state_adapter ${catkin_LIBRARIES})

add_executable(gps_imu_vehicle_state_node
  src/ros/state/gps_imu_vehicle_state_node.cpp)
add_dependencies(gps_imu_vehicle_state_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(gps_imu_vehicle_state_node
  frenet_vehicle_state_adapter ${catkin_LIBRARIES})

add_executable(ego_vehicle_state_dev_node
  src/ros/state/ego_vehicle_state_dev_node.cpp)
add_dependencies(ego_vehicle_state_dev_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(ego_vehicle_state_dev_node
  frenet_vehicle_state_adapter ${catkin_LIBRARIES})

add_executable(empty_objects_test_publisher
  src/ros/state/empty_objects_test_publisher.cpp)
add_dependencies(empty_objects_test_publisher ${catkin_EXPORTED_TARGETS})
target_link_libraries(empty_objects_test_publisher ${catkin_LIBRARIES})

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

  add_executable(message_converter_test
    tests/message_converter_test.cpp
  )
  add_dependencies(message_converter_test
    ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(message_converter_test
    frenet_planner_message_converter
    ${catkin_LIBRARIES}
  )
  add_test(NAME message_converter_test COMMAND message_converter_test)

  add_executable(planner_command_defaults_test
    tests/planner_command_defaults_test.cpp
  )
  target_include_directories(planner_command_defaults_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )
  add_test(NAME planner_command_defaults_test
           COMMAND planner_command_defaults_test)

  add_executable(command_keep_plan_test
    tests/command_keep_plan_test.cpp
  )
  target_compile_definitions(command_keep_plan_test PRIVATE
    FRENET_PACKAGE_SRC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/src"
    FRENET_TEST_WAYPOINT_FILE="${CMAKE_CURRENT_SOURCE_DIR}/../../R_KR_PG_KATRI/2026_molit_comp_global_path.txt"
  )
  target_link_libraries(command_keep_plan_test
    frenet_planner_core
  )
  add_test(NAME command_keep_plan_test COMMAND command_keep_plan_test)

  add_executable(command_validation_test
    tests/command_validation_test.cpp
  )
  target_compile_definitions(command_validation_test PRIVATE
    FRENET_PACKAGE_SRC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/src"
    FRENET_TEST_WAYPOINT_FILE="${CMAKE_CURRENT_SOURCE_DIR}/../../R_KR_PG_KATRI/2026_molit_comp_global_path.txt"
  )
  target_link_libraries(command_validation_test
    frenet_planner_core
  )
  add_test(NAME command_validation_test COMMAND command_validation_test)

  add_executable(command_stop_plan_test
    tests/command_stop_plan_test.cpp
  )
  target_compile_definitions(command_stop_plan_test PRIVATE
    FRENET_PACKAGE_SRC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/src"
    FRENET_TEST_WAYPOINT_FILE="${CMAKE_CURRENT_SOURCE_DIR}/../../R_KR_PG_KATRI/2026_molit_comp_global_path.txt"
  )
  target_link_libraries(command_stop_plan_test
    frenet_planner_core
  )
  add_test(NAME command_stop_plan_test COMMAND command_stop_plan_test)

  # D3-1: BehaviorCommand <-> 내부 BehaviorState enum 매핑 회귀.
  # static_cast가 실제로 틀린다는 것까지 negative case로 고정한다.
  add_executable(v2_enum_mapping_test
    tests/v2_enum_mapping_test.cpp
  )
  add_dependencies(v2_enum_mapping_test
    ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(v2_enum_mapping_test
    frenet_behavior_command_adapter
    ${catkin_LIBRARIES}
  )
  add_test(NAME v2_enum_mapping_test COMMAND v2_enum_mapping_test)

  # D1-3: legacy(간이 FSM) 행동 결정의 회귀 커버리지. D5/D6/D8에서 모드별로
  # legacy 조각을 제거할 때 검증된 회전교차로/고속도로 주행이 깨지지 않았는지
  # 확인하는 안전망이다 (docs/d1-legacy-behavior-boundary.md).
  add_executable(legacy_behavior_regression_test
    tests/legacy_behavior_regression_test.cpp
  )
  target_compile_definitions(legacy_behavior_regression_test PRIVATE
    FRENET_PACKAGE_SRC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/src"
    FRENET_TEST_WAYPOINT_FILE="${CMAKE_CURRENT_SOURCE_DIR}/../../R_KR_PG_KATRI/2026_molit_comp_global_path.txt"
  )
  target_link_libraries(legacy_behavior_regression_test
    frenet_planner_core
  )
  add_test(NAME legacy_behavior_regression_test
           COMMAND legacy_behavior_regression_test)

  add_executable(vehicle_state_adapter_test tests/vehicle_state_adapter_test.cpp)
  add_dependencies(vehicle_state_adapter_test ${catkin_EXPORTED_TARGETS})
  target_link_libraries(vehicle_state_adapter_test
    frenet_vehicle_state_adapter ${catkin_LIBRARIES})
  add_test(NAME vehicle_state_adapter_test COMMAND vehicle_state_adapter_test)
endif()

install(TARGETS
    frenet_geometry
    frenet_planner_core
    frenet_planner_message_converter
    frenet_behavior_command_adapter
    frenet_vehicle_state_adapter
    frenet_planner_udp_legacy
    frenet_planner_node
    gps_imu_vehicle_state_node
    ego_vehicle_state_dev_node
    empty_objects_test_publisher
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

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY
    src/frenet_planner/config/
    src/udp_network/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/legacy
  FILES_MATCHING PATTERN "*.yaml"
)
