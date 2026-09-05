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
  src/frenet_planner/frenet/curve_speed.cpp
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

# v2 transaction 관리. adapter가 stateless인 것과 달리 이쪽은 명령들 사이의
# 관계(transaction / terminal latch / preemption)를 갖는 stateful 계층이다
# (docs/fsm-frenet-interface-v2.md 12.8절).
add_library(frenet_command_execution_manager
  src/ros/command_execution_manager.cpp
)
add_dependencies(frenet_command_execution_manager
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(frenet_command_execution_manager
  frenet_behavior_command_adapter
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
  frenet_command_execution_manager
  frenet_behavior_command_adapter
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

# D4-4: v2 ROS 인터페이스 통합 회귀. 실행 중인 roscore와 frenet_planner_node가
# 필요하므로 add_test()에 등록하지 않는다(catkin_make run_tests가 자동으로
# 돌릴 수 없다). 수동 실행:
#
#   rosrun frenet_planner v2_ros_regression_test
#
# D10에서 params.yaml의 배포 절대경로 문제가 정리되면 rostest로 편입한다.
add_executable(v2_ros_regression_test
  tests/v2_ros_regression_test.cpp
)
add_dependencies(v2_ros_regression_test
  ${catkin_EXPORTED_TARGETS}
)
target_compile_definitions(v2_ros_regression_test PRIVATE
  FRENET_TEST_WAYPOINT_FILE="${CMAKE_CURRENT_SOURCE_DIR}/../../R_KR_PG_KATRI/2026_molit_comp_global_path.txt"
)
target_link_libraries(v2_ros_regression_test
  ${catkin_LIBRARIES}
)

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

  # D5-1: 외부 FOLLOW의 authoritative 대상 확보 회귀. resolver를 legacy
  # nearest selector로 바꾸면 2건이 실패한다.
  add_executable(command_follow_plan_test
    tests/command_follow_plan_test.cpp
  )
  target_compile_definitions(command_follow_plan_test PRIVATE
    FRENET_PACKAGE_SRC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/src"
    FRENET_TEST_WAYPOINT_FILE="${CMAKE_CURRENT_SOURCE_DIR}/../../R_KR_PG_KATRI/2026_molit_comp_global_path.txt"
  )
  target_link_libraries(command_follow_plan_test
    frenet_planner_core
  )
  add_test(NAME command_follow_plan_test COMMAND command_follow_plan_test)

  # D6-1: 외부 AVOID의 authoritative 대상 확보 회귀. resolver를 legacy
  # FindStaticAvoidanceTarget()으로 바꾸면 nearest-object 검사가 실패한다.
  add_executable(command_avoid_target_test
    tests/command_avoid_target_test.cpp
  )
  target_compile_definitions(command_avoid_target_test PRIVATE
    FRENET_PACKAGE_SRC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/src"
    FRENET_TEST_WAYPOINT_FILE="${CMAKE_CURRENT_SOURCE_DIR}/../../R_KR_PG_KATRI/2026_molit_comp_global_path.txt"
  )
  target_link_libraries(command_avoid_target_test
    frenet_planner_core
    frenet_command_execution_manager
    ${catkin_LIBRARIES}
  )
  add_test(NAME command_avoid_target_test COMMAND command_avoid_target_test)

  # D4-3: 외부 EMERGENCY_STOP의 직접 감속 궤적 회귀. EMERGENCY 전용 분기를
  # 제거하면(후보 파이프라인으로 흘려보내면) 10건이 실패한다.
  add_executable(command_emergency_plan_test
    tests/command_emergency_plan_test.cpp
  )
  target_compile_definitions(command_emergency_plan_test PRIVATE
    FRENET_PACKAGE_SRC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/src"
    FRENET_TEST_WAYPOINT_FILE="${CMAKE_CURRENT_SOURCE_DIR}/../../R_KR_PG_KATRI/2026_molit_comp_global_path.txt"
  )
  target_link_libraries(command_emergency_plan_test
    frenet_planner_core
  )
  add_test(NAME command_emergency_plan_test
           COMMAND command_emergency_plan_test)

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

  # D3-2: BehaviorCommand validation + transaction fingerprint 회귀.
  # PREPARE -> EXECUTE가 같은 transaction으로 인정되는지, 형식 오류가 STALE로
  # 가려지지 않는지를 중점적으로 검증한다.
  add_executable(behavior_command_adapter_test
    tests/behavior_command_adapter_test.cpp
  )
  add_dependencies(behavior_command_adapter_test
    ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(behavior_command_adapter_test
    frenet_behavior_command_adapter
    ${catkin_LIBRARIES}
  )
  add_test(NAME behavior_command_adapter_test
           COMMAND behavior_command_adapter_test)

  # D3-3: transaction / terminal latch / preemption 회귀.
  add_executable(command_execution_manager_test
    tests/command_execution_manager_test.cpp
  )
  add_dependencies(command_execution_manager_test
    ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(command_execution_manager_test
    frenet_command_execution_manager
    ${catkin_LIBRARIES}
  )
  add_test(NAME command_execution_manager_test
           COMMAND command_execution_manager_test)

  # fix/yuntae-frenet-speed-continuity: longitudinal candidate validity 회귀.
  # negative s_d(reverse) candidate와 excessive jerk candidate가 accel-only
  # 필터를 통과해 valid로 남는지(수정 전) / 걸러지는지(수정 후) 확인한다.
  add_executable(longitudinal_kinematics_test
    tests/longitudinal_kinematics_test.cpp
  )
  target_link_libraries(longitudinal_kinematics_test
    frenet_planner_core
  )
  add_test(NAME longitudinal_kinematics_test COMMAND longitudinal_kinematics_test)

  # fix/yuntae-frenet-speed-continuity Phase 1: distance-aware curve speed
  # envelope 회귀. 실제 LookaheadTargetSpeed/VelocityFromCurvature
  # (frenet/curve_speed.hpp)를 손으로 구성한 RefLine으로 직접 호출한다.
  add_executable(curve_speed_envelope_test
    tests/curve_speed_envelope_test.cpp
  )
  target_link_libraries(curve_speed_envelope_test
    frenet_planner_core
  )
  add_test(NAME curve_speed_envelope_test COMMAND curve_speed_envelope_test)

  # fix/yuntae-frenet-speed-continuity Phase 2: target speed recovery rate
  # limit 회귀. 실제 ApplyTargetSpeedRateLimit(frenet/curve_speed.hpp)를
  # 직접 호출한다.
  add_executable(target_speed_rate_limit_test
    tests/target_speed_rate_limit_test.cpp
  )
  target_link_libraries(target_speed_rate_limit_test
    frenet_planner_core
  )
  add_test(NAME target_speed_rate_limit_test COMMAND target_speed_rate_limit_test)

  # fix/yuntae-frenet-speed-continuity Phase 5: low-speed KEEP candidate
  # starvation handoff 회귀.
  add_executable(low_speed_keep_handoff_test
    tests/low_speed_keep_handoff_test.cpp
  )
  target_compile_definitions(low_speed_keep_handoff_test PRIVATE
    FRENET_PACKAGE_SRC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/src"
    FRENET_TEST_WAYPOINT_FILE="${CMAKE_CURRENT_SOURCE_DIR}/../../R_KR_PG_KATRI/2026_molit_comp_global_path.txt"
  )
  target_link_libraries(low_speed_keep_handoff_test
    frenet_planner_core
  )
  add_test(NAME low_speed_keep_handoff_test COMMAND low_speed_keep_handoff_test)

  # fix/yuntae-frenet-speed-continuity Phase A: KEEP replanning
  # braking-memory 회귀.
  add_executable(braking_memory_test
    tests/braking_memory_test.cpp
  )
  target_compile_definitions(braking_memory_test PRIVATE
    FRENET_PACKAGE_SRC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/src"
    FRENET_TEST_WAYPOINT_FILE="${CMAKE_CURRENT_SOURCE_DIR}/../../R_KR_PG_KATRI/2026_molit_comp_global_path.txt"
  )
  target_link_libraries(braking_memory_test
    frenet_planner_core
  )
  add_test(NAME braking_memory_test COMMAND braking_memory_test)

  # fix/yuntae-frenet-speed-continuity Phase B: moderate-speed KEEP
  # curvature starvation 회귀.
  add_executable(curvature_continuity_test
    tests/curvature_continuity_test.cpp
  )
  target_compile_definitions(curvature_continuity_test PRIVATE
    FRENET_PACKAGE_SRC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/src"
    FRENET_TEST_WAYPOINT_FILE="${CMAKE_CURRENT_SOURCE_DIR}/../../R_KR_PG_KATRI/2026_molit_comp_global_path.txt"
  )
  target_link_libraries(curvature_continuity_test
    frenet_planner_core
  )
  add_test(NAME curvature_continuity_test COMMAND curvature_continuity_test)

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
    frenet_command_execution_manager
    frenet_vehicle_state_adapter
    frenet_planner_udp_legacy
    frenet_planner_node
    gps_imu_vehicle_state_node
    ego_vehicle_state_dev_node
    empty_objects_test_publisher
    v2_ros_regression_test
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
