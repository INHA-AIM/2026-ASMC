cmake_minimum_required(VERSION 3.0.2)
project(asmc_planner LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  planning_inputs asmc_msgs morai_msgs roscpp std_msgs std_srvs nav_msgs diagnostic_msgs geometry_msgs
  sensor_msgs visualization_msgs)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED)
catkin_package(INCLUDE_DIRS src
  LIBRARIES asmc_signal_policy_core asmc_planner_lane_reference_core asmc_planner_frenet_core asmc_planner_behavior_core
    asmc_planner_execution_core asmc_planner_behavior_module asmc_planner_ros_adapters
  CATKIN_DEPENDS planning_inputs asmc_msgs morai_msgs roscpp std_msgs std_srvs nav_msgs diagnostic_msgs
    geometry_msgs sensor_msgs visualization_msgs)

# Shared reference preparation/execution. Production uses the existing highway
# launch gate; the separate empty-road trial remains available.
add_library(asmc_planner_lane_reference_core
  src/Execution/HighwayReferenceExecution.cpp
  src/Execution/ReferenceHandover.cpp
  src/Execution/LaneReferenceExecution.cpp)
target_include_directories(asmc_planner_lane_reference_core PUBLIC src)
target_include_directories(asmc_planner_lane_reference_core SYSTEM PRIVATE ${Boost_INCLUDE_DIRS})
target_compile_options(asmc_planner_lane_reference_core PRIVATE -O2 -Wall -Wextra)
set_target_properties(asmc_planner_lane_reference_core PROPERTIES CXX_EXTENSIONS OFF)

# Reference.cpp stays owned by the existing Frenet target, with its old flags.
add_executable(highway_merge_shadow tools/frenet/highway_merge_shadow.cpp)
target_link_libraries(highway_merge_shadow asmc_planner_lane_reference_core)
install(TARGETS highway_merge_shadow RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(highway_merge_preflight tools/highway_merge_preflight.cpp)
target_link_libraries(highway_merge_preflight asmc_planner_lane_reference_core)
install(TARGETS highway_merge_preflight RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(lane_reference_audit tools/lane_reference_audit.cpp)
target_include_directories(lane_reference_audit PRIVATE src)
target_include_directories(lane_reference_audit SYSTEM PRIVATE ${Boost_INCLUDE_DIRS})
target_compile_options(lane_reference_audit PRIVATE -O2)
target_link_libraries(lane_reference_audit asmc_planner_lane_reference_core)
install(TARGETS asmc_planner_lane_reference_core lane_reference_audit
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY src/Map/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/Map
  FILES_MATCHING PATTERN "*.hpp")

add_library(asmc_planner_execution_core
  src/Execution/Acceptance/CommandExecutionManager.cpp
  src/Execution/Acceptance/CommandFingerprint.cpp
  src/Execution/RequestTransaction.cpp)
target_include_directories(asmc_planner_execution_core PUBLIC src ${catkin_INCLUDE_DIRS})
add_dependencies(asmc_planner_execution_core ${catkin_EXPORTED_TARGETS})
target_link_libraries(asmc_planner_execution_core ${catkin_LIBRARIES})
set_target_properties(asmc_planner_execution_core PROPERTIES CXX_EXTENSIONS OFF)
# The accepted transaction previously compiled with -O2; request sequencing
# retains the Behavior package's original unoptimized compilation setting.
set_source_files_properties(src/Execution/Acceptance/CommandExecutionManager.cpp
  src/Execution/Acceptance/CommandFingerprint.cpp PROPERTIES COMPILE_OPTIONS "-O2")

add_library(asmc_signal_policy_core src/Signal/SignalPolicy.cpp)
target_include_directories(asmc_signal_policy_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src ${asmc_msgs_INCLUDE_DIRS})
target_compile_options(asmc_signal_policy_core PRIVATE -Wall -Wextra)
set_target_properties(asmc_signal_policy_core PROPERTIES CXX_EXTENSIONS OFF)

add_library(asmc_planner_behavior_core
  src/Behavior/FSM/DecisionStability.cpp
  src/Execution/ManeuverLifecycle.cpp
  src/Execution/BehaviorFsmCompatibility.cpp)
target_include_directories(asmc_planner_behavior_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src)
set_target_properties(asmc_planner_behavior_core PROPERTIES CXX_EXTENSIONS OFF)
target_compile_options(asmc_planner_behavior_core PRIVATE -Wall -Wextra)
target_link_libraries(asmc_planner_behavior_core asmc_signal_policy_core)

add_library(asmc_planner_frenet_core
  src/Frenet/Maneuver/Merge/HighwayTransitionPath.cpp
  src/Map/HdLaneMap.cpp
  src/Map/HighwayMergeRoute.cpp
  src/Frenet/Maneuver/Merge/HighwayExecution.cpp
  src/Frenet/Config/ConfigLoader.cpp
  src/Frenet/Config/ConfigValidator.cpp
  src/Frenet/Evaluation/CollisionChecker.cpp
  src/Frenet/Evaluation/StoppingSafety.cpp
  src/Frenet/Evaluation/Cost.cpp
  src/Frenet/Evaluation/KinematicFilter.cpp
  src/Frenet/Generator/BoundaryConditionPolicy.cpp
  src/Frenet/Generator/CartesianRetimer.cpp
  src/Frenet/Generator/SpatialStop.cpp
  src/Frenet/Generator/CandidateCombiner.cpp
  src/Frenet/Generator/LateralGenerator.cpp
  src/Frenet/Generator/LongitudinalGenerator.cpp
  src/Frenet/Generator/SamplingUtils.cpp
  src/Frenet/Geometry/PolygonUtils.cpp
  src/Frenet/Maneuver/Avoid/Avoid.cpp
  src/Frenet/Maneuver/Avoid/AvoidLifecycle.cpp
  src/Frenet/Maneuver/Avoid/AvoidTargetResolver.cpp
  src/Frenet/Maneuver/Avoid/AvoidCompletion.cpp
  src/Frenet/Maneuver/Avoid/MapAvoidReconnection.cpp
  src/Frenet/Maneuver/Avoid/MapAvoid.cpp
  src/Frenet/Maneuver/Follow.cpp
  src/Frenet/Maneuver/Keep.cpp
  src/Frenet/Maneuver/LaneChange/LaneChange.cpp
  src/Frenet/Maneuver/LaneChange/LaneChangeLifecycle.cpp
  src/Frenet/Maneuver/ManeuverUtils.cpp
  src/Frenet/Maneuver/Merge/MergeGapEvaluator.cpp
  src/Frenet/Maneuver/Merge/RoundaboutExecution.cpp
  src/Frenet/Maneuver/Merge/RoundaboutMerge.cpp
  src/Frenet/Maneuver/Stop.cpp
  src/Frenet/Map/AvoidRoadMap.cpp
  src/Frenet/Math/FrenetConverter.cpp
  src/Frenet/Math/Polynomial.cpp
  src/Frenet/Pipeline/TrajectoryPipeline.cpp
  # Geometry preparation/projection is also consumed by highway prediction.
  # Keep it below reference execution to avoid a reverse library dependency.
  src/Frenet/Reference/LaneReference.cpp
  src/Frenet/Reference/Reference.cpp
  src/Frenet/Speed/CurveSpeed.cpp
  src/Frenet/Speed/SpeedProfiler.cpp
  src/Frenet/Speed/TargetSpeedResolver.cpp
  src/Frenet/frenet_planner.cpp
  src/Frenet/visualization/planner_debug_writer.cpp)
target_include_directories(asmc_planner_frenet_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src)
# Preserve the compilation settings of the original frenet_planner targets.
set_target_properties(asmc_planner_frenet_core PROPERTIES CXX_EXTENSIONS OFF)
target_compile_options(asmc_planner_frenet_core PRIVATE -O2)
target_compile_definitions(asmc_planner_frenet_core PRIVATE
  FRENET_PACKAGE_SRC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/src")
target_link_libraries(asmc_planner_frenet_core yaml-cpp asmc_signal_policy_core)
target_include_directories(asmc_planner_frenet_core SYSTEM PRIVATE ${Boost_INCLUDE_DIRS})
target_link_libraries(asmc_planner_lane_reference_core PUBLIC asmc_planner_frenet_core)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  # Fail at the library boundary, even when an executable also happens to
  # link another library that could hide an undeclared reverse dependency.
  foreach(core asmc_planner_frenet_core asmc_planner_lane_reference_core)
    set_property(TARGET ${core} APPEND_STRING PROPERTY LINK_FLAGS " -Wl,--no-undefined")
  endforeach()
endif()

# Phase 1 executable remains inert while pure Frenet calculations are linked
# through the separate library above.
add_executable(asmc_planner_scaffold
  src/main.cpp
  src/Planning/Planner.cpp)
target_include_directories(asmc_planner_scaffold PRIVATE src)

install(TARGETS asmc_planner_scaffold
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS asmc_signal_policy_core asmc_planner_frenet_core asmc_planner_behavior_core asmc_planner_execution_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY src/Frenet/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/Frenet
  FILES_MATCHING PATTERN "*.hpp")
install(DIRECTORY src/Global/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/Global
  FILES_MATCHING PATTERN "*.hpp")
install(DIRECTORY src/Behavior src/Execution src/Planning src/Signal
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp")

add_library(asmc_planner_behavior_module
  src/Behavior/Decision/behavior_decision.cpp
  src/Behavior/Decision/behavior_scorer.cpp
  src/Behavior/Decision/transition_condition.cpp
  src/Behavior/Feature/DynamicObjectPolicy.cpp
  src/Behavior/Feature/feature_builder.cpp
  src/Behavior/Feature/object_projector.cpp
  src/Behavior/Map/mgeo_loader.cpp
  src/Behavior/Map/route_manager.cpp
  src/Behavior/Rule/hard_rule_filter.cpp
  src/Output/Behavior/data_logger.cpp
  src/Output/Behavior/utils.cpp
  src/Execution/IntentEncoder.cpp
  src/Parser/Traffic/mock_traffic_light_provider.cpp)
target_include_directories(asmc_planner_behavior_module PUBLIC src ${catkin_INCLUDE_DIRS})
add_dependencies(asmc_planner_behavior_module ${catkin_EXPORTED_TARGETS})
target_compile_options(asmc_planner_behavior_module PRIVATE -Wall -Wextra)
target_link_libraries(asmc_planner_behavior_module
  asmc_planner_behavior_core asmc_planner_execution_core asmc_planner_lane_reference_core asmc_planner_frenet_core yaml-cpp ${catkin_LIBRARIES})

add_library(asmc_planner_ros_adapters
  src/Signal/SignalShadow.cpp
  src/Output/BehaviorMessages.cpp
  src/Output/PlanningFeedback.cpp
  src/Output/PlanningOutput.cpp
  src/Parser/BehaviorCommandAdapter.cpp
  src/Parser/HdMapLaneInput.cpp
  src/Parser/BehaviorInput.cpp
  src/Parser/DetectedObjects.cpp
  src/Parser/MoraiObjects.cpp
  src/Parser/PlanningInput.cpp
  src/Parser/PlanningInputCallbacks.cpp
  src/Parser/RuntimeConfig.cpp
  src/Parser/Traffic/ros_traffic_light_provider.cpp
  src/Planning/LegacyCycleCompatibility.cpp
  src/Planning/PlanningPort.cpp
  src/Planning/HighwayReferencePort.cpp
  src/Planning/ProductionRuntime.cpp
  src/Planning/GpsModeSource.cpp
  src/Planning/TargetProjection.cpp
  src/Visualization/Behavior/behavior_visualization.cpp
  src/Visualization/PlanningMarkers.cpp)
target_include_directories(asmc_planner_ros_adapters PUBLIC src ${catkin_INCLUDE_DIRS})
add_dependencies(asmc_planner_ros_adapters ${catkin_EXPORTED_TARGETS})
target_link_libraries(asmc_planner_ros_adapters
  asmc_planner_lane_reference_core asmc_planner_frenet_core asmc_planner_behavior_module
  asmc_planner_execution_core yaml-cpp ${catkin_LIBRARIES})
set_source_files_properties(
  src/Output/PlanningFeedback.cpp
  src/Output/PlanningOutput.cpp
  src/Parser/BehaviorCommandAdapter.cpp
  src/Parser/PlanningInput.cpp
  src/Parser/PlanningInputCallbacks.cpp
  src/Parser/RuntimeConfig.cpp
  src/Planning/LegacyCycleCompatibility.cpp
  src/Planning/PlanningPort.cpp
  src/Planning/TargetProjection.cpp
  src/Visualization/PlanningMarkers.cpp PROPERTIES COMPILE_OPTIONS "-O2")
add_executable(asmc_planner_node src/Planning/production_main.cpp)
add_executable(reference_handover_trial tools/reference_handover_trial.cpp)
add_dependencies(reference_handover_trial ${catkin_EXPORTED_TARGETS})
target_link_libraries(reference_handover_trial asmc_planner_ros_adapters)
install(TARGETS reference_handover_trial RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
add_dependencies(asmc_planner_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(asmc_planner_node asmc_planner_ros_adapters)
add_executable(asmc_control_selector_node
  src/Planning/control_selector_main.cpp src/Planning/ControlSelector.cpp)
target_include_directories(asmc_control_selector_node PRIVATE src ${catkin_INCLUDE_DIRS})
add_dependencies(asmc_control_selector_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(asmc_control_selector_node ${catkin_LIBRARIES})

add_executable(asmc_competition_speed_state_node src/competition_speed_state_node.cpp)
target_include_directories(asmc_competition_speed_state_node PRIVATE src ${catkin_INCLUDE_DIRS})
add_dependencies(asmc_competition_speed_state_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(asmc_competition_speed_state_node ${catkin_LIBRARIES})
install(TARGETS asmc_competition_speed_state_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS asmc_planner_behavior_module asmc_planner_ros_adapters asmc_planner_node
    asmc_control_selector_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY src/Parser src/Output src/Visualization
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp")
install(DIRECTORY config launch web DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  foreach(clock_test object_prediction_clock_test stop_generation_options_test)
    add_executable(${clock_test} tests/frenet/${clock_test}.cpp)
    target_link_libraries(${clock_test} asmc_planner_frenet_core)
    add_test(NAME ${clock_test} COMMAND ${clock_test})
  endforeach()

  foreach(regression highway_prediction_reuse_test reference_projection_window_test highway_cruise_policy_test follow_gap_recovery_test initial_state_recovery_test highway_reference_sampling_test highway_lane_change_execution_test highway_lane_motion_test highway_readiness_test highway_lane_change_recovery_test)
    add_executable(${regression} tests/frenet/${regression}.cpp)
    target_compile_definitions(${regression} PRIVATE
      FRENET_TEST_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/frenet/params.yaml")
    target_link_libraries(${regression} asmc_planner_lane_reference_core yaml-cpp)
    add_test(NAME ${regression} COMMAND ${regression})
  endforeach()
  add_executable(hd_lane_map_product_test tests/frenet/hd_lane_map_product_test.cpp)
  target_include_directories(hd_lane_map_product_test SYSTEM PRIVATE ${Boost_INCLUDE_DIRS})
  target_link_libraries(hd_lane_map_product_test asmc_planner_lane_reference_core yaml-cpp)
  target_compile_definitions(hd_lane_map_product_test PRIVATE
    HD_LANE_MAP_PRODUCT="${CMAKE_CURRENT_SOURCE_DIR}/../asmc_hd_map/config/kcity2025_compact_drivable_map.json"
    FRENET_TEST_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/frenet/params.yaml")
  add_test(NAME hd_lane_map_product_test COMMAND hd_lane_map_product_test)
  add_executable(lane_reference_execution_test tests/frenet/lane_reference_execution_test.cpp)
  target_link_libraries(lane_reference_execution_test asmc_planner_lane_reference_core yaml-cpp)
  add_executable(prepared_static_inputs_test tests/frenet/prepared_static_inputs_test.cpp)
  target_compile_definitions(prepared_static_inputs_test PRIVATE
    FRENET_TEST_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/frenet/params.yaml"
    FRENET_TEST_PACKAGE="${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(prepared_static_inputs_test asmc_planner_frenet_core yaml-cpp)
  add_test(NAME prepared_static_inputs_test COMMAND prepared_static_inputs_test)
  add_executable(highway_reference_execution_test tests/frenet/highway_reference_execution_test.cpp)
  target_compile_definitions(highway_reference_execution_test PRIVATE
    FRENET_TEST_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/frenet/params.yaml")
  target_link_libraries(highway_reference_execution_test asmc_planner_lane_reference_core yaml-cpp)
  add_test(NAME highway_reference_execution_test COMMAND highway_reference_execution_test)
  add_executable(highway_source_approach_test tests/frenet/highway_source_approach_test.cpp)
  target_compile_definitions(highway_source_approach_test PRIVATE
    FRENET_TEST_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/frenet/params.yaml")
  target_link_libraries(highway_source_approach_test asmc_planner_lane_reference_core yaml-cpp)
  add_test(NAME highway_source_approach_test COMMAND highway_source_approach_test)
  add_executable(highway_transition_recovery_test tests/frenet/highway_transition_recovery_test.cpp)
  target_compile_definitions(highway_transition_recovery_test PRIVATE
    FRENET_TEST_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/frenet/params.yaml")
  target_link_libraries(highway_transition_recovery_test asmc_planner_lane_reference_core yaml-cpp)
  add_test(NAME highway_transition_recovery_test COMMAND highway_transition_recovery_test)
  add_executable(highway_lane_change_policy_test tests/behavior/highway_lane_change_policy_test.cpp)
  target_compile_definitions(highway_lane_change_policy_test PRIVATE BEHAVIOR_PLANNER_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  add_dependencies(highway_lane_change_policy_test ${catkin_EXPORTED_TARGETS})
  target_link_libraries(highway_lane_change_policy_test asmc_planner_behavior_module asmc_planner_lane_reference_core)
  add_test(NAME highway_lane_change_policy_test COMMAND highway_lane_change_policy_test)
  add_executable(highway_reference_feature_test tests/behavior/highway_reference_feature_test.cpp)
  target_compile_definitions(highway_reference_feature_test PRIVATE
    BEHAVIOR_PLANNER_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(highway_reference_feature_test asmc_planner_behavior_module asmc_planner_lane_reference_core)
  add_test(NAME highway_reference_feature_test COMMAND highway_reference_feature_test)
  target_compile_definitions(lane_reference_execution_test PRIVATE
    FRENET_TEST_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/frenet/params.yaml")
  add_test(NAME lane_reference_execution_test COMMAND lane_reference_execution_test)
  add_executable(lane_reference_test tests/frenet/lane_reference_test.cpp)
  target_include_directories(lane_reference_test PRIVATE src)
  target_link_libraries(lane_reference_test asmc_planner_lane_reference_core)
  add_test(NAME lane_reference_test COMMAND lane_reference_test)
  add_executable(hd_map_lane_input_test tests/frenet/hd_map_lane_input_test.cpp src/Parser/HdMapLaneInput.cpp)
  target_include_directories(hd_map_lane_input_test PRIVATE src ${catkin_INCLUDE_DIRS})
  add_dependencies(hd_map_lane_input_test ${catkin_EXPORTED_TARGETS})
  target_link_libraries(hd_map_lane_input_test asmc_planner_lane_reference_core ${catkin_LIBRARIES})
  add_test(NAME hd_map_lane_input_test COMMAND hd_map_lane_input_test)
  catkin_add_gtest(asmc_mode_detector_test tests/mode_detector_test.cpp)
  if(TARGET asmc_mode_detector_test)
    target_include_directories(asmc_mode_detector_test PRIVATE src ${catkin_INCLUDE_DIRS})
    target_compile_definitions(asmc_mode_detector_test PRIVATE
      ASMC_REPO_ROOT="${CMAKE_CURRENT_SOURCE_DIR}/../..")
    add_dependencies(asmc_mode_detector_test ${catkin_EXPORTED_TARGETS})
    target_link_libraries(asmc_mode_detector_test ${catkin_LIBRARIES} gtest_main)
  endif()
  find_package(rostest REQUIRED)
  add_rostest(tests/control_selector.test DEPENDENCIES asmc_control_selector_node)
  add_executable(gps_mode_source_test_node
    tests/gps_mode_source_main.cpp src/Planning/GpsModeSource.cpp)
  target_include_directories(gps_mode_source_test_node PRIVATE src ${catkin_INCLUDE_DIRS})
  add_dependencies(gps_mode_source_test_node ${catkin_EXPORTED_TARGETS})
  target_link_libraries(gps_mode_source_test_node ${catkin_LIBRARIES})
  add_rostest(tests/gps_mode_source.test DEPENDENCIES gps_mode_source_test_node)

  add_executable(highway_runtime_test tests/behavior/highway_runtime_test.cpp)
  target_include_directories(highway_runtime_test PRIVATE src ${catkin_INCLUDE_DIRS} ${GTEST_INCLUDE_DIRS})
  add_dependencies(highway_runtime_test ${catkin_EXPORTED_TARGETS})
  target_compile_definitions(highway_runtime_test PRIVATE
    FRENET_TEST_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/frenet/params.yaml")
  target_link_libraries(highway_runtime_test asmc_planner_ros_adapters ${catkin_LIBRARIES} gtest)
  add_rostest(tests/highway_runtime.test DEPENDENCIES highway_runtime_test)
  # Explicit offline gate; preserve the existing registered regression set.
  add_executable(asmc_internal_flow_equivalence tests/internal_flow_equivalence.cpp)
  add_dependencies(asmc_internal_flow_equivalence ${catkin_EXPORTED_TARGETS})
  target_link_libraries(asmc_internal_flow_equivalence
    asmc_planner_execution_core asmc_planner_ros_adapters ${catkin_LIBRARIES})
  add_test(NAME signal_shadow_isolation COMMAND asmc_internal_flow_equivalence)
endif()

# Regression assets transferred without changing assertions.
include_directories(src ${catkin_INCLUDE_DIRS})
add_executable(v2_ros_regression_test
  tests/frenet/v2_ros_regression_test.cpp
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
  tools/empty_objects_test_publisher.cpp)
add_dependencies(empty_objects_test_publisher ${catkin_EXPORTED_TARGETS})
target_link_libraries(empty_objects_test_publisher ${catkin_LIBRARIES})

# LANE_CHANGE FSM wire contract 재현 tool. Behavior Planner의 scorer/policy는
# LANE_CHANGE를 자동으로 고르지 않으므로, 이 tool로 PREPARE/EXECUTE/CANCEL
# handshake를 손으로 재현해 검증한다.
add_executable(lane_change_test_publisher
  tools/lane_change_test_publisher.cpp)
add_dependencies(lane_change_test_publisher ${catkin_EXPORTED_TARGETS})
target_link_libraries(lane_change_test_publisher ${catkin_LIBRARIES})


if(CATKIN_ENABLE_TESTING)

  add_executable(roundabout_transaction_test tests/behavior/roundabout_transaction_test.cpp




    )
  add_dependencies(roundabout_transaction_test ${catkin_EXPORTED_TARGETS})
  target_link_libraries(roundabout_transaction_test asmc_planner_ros_adapters ${catkin_LIBRARIES} yaml-cpp)
  add_test(NAME roundabout_transaction_test COMMAND roundabout_transaction_test)

  catkin_add_gtest(fsm_command_binding_test
    tests/behavior/fsm_command_binding_test.cpp
    )
  if(TARGET fsm_command_binding_test)
    target_link_libraries(fsm_command_binding_test asmc_planner_behavior_core gtest_main)
    target_include_directories(fsm_command_binding_test PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src)
  endif()

  catkin_add_gtest(detected_msg_converter_test
    tests/behavior/detected_msg_converter_test.cpp
    )
  if(TARGET detected_msg_converter_test)
    add_dependencies(detected_msg_converter_test
      ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_include_directories(detected_msg_converter_test PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src ${catkin_INCLUDE_DIRS})
    target_link_libraries(detected_msg_converter_test asmc_planner_ros_adapters ${catkin_LIBRARIES})
  endif()

  catkin_add_gtest(behavior_logic_test
    tests/behavior/behavior_logic_test.cpp





    )
  if(TARGET behavior_logic_test)
    add_dependencies(behavior_logic_test
      ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_include_directories(behavior_logic_test PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src ${catkin_INCLUDE_DIRS})
    target_link_libraries(behavior_logic_test asmc_planner_ros_adapters ${catkin_LIBRARIES})
  endif()

  catkin_add_gtest(route_signal_movement_test
    tests/behavior/route_signal_movement_test.cpp


    )
  if(TARGET route_signal_movement_test)
    add_dependencies(route_signal_movement_test
      ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_include_directories(route_signal_movement_test PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src ${catkin_INCLUDE_DIRS})
    target_compile_definitions(route_signal_movement_test PRIVATE
      BEHAVIOR_PLANNER_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
    target_link_libraries(route_signal_movement_test
      asmc_planner_ros_adapters ${catkin_LIBRARIES} yaml-cpp)
  endif()

  catkin_add_gtest(feature_builder_avoid_test
    tests/behavior/feature_builder_avoid_test.cpp




    )
  if(TARGET feature_builder_avoid_test)
    add_dependencies(feature_builder_avoid_test
      ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_include_directories(feature_builder_avoid_test PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src ${catkin_INCLUDE_DIRS})
    target_compile_definitions(feature_builder_avoid_test PRIVATE
      BEHAVIOR_PLANNER_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
    target_link_libraries(feature_builder_avoid_test
      asmc_planner_ros_adapters ${catkin_LIBRARIES} yaml-cpp)
  endif()

  file(GLOB FRENET_REGRESSION_SOURCES tests/frenet/*.cpp tools/frenet/*.cpp)
  set_source_files_properties(${FRENET_REGRESSION_SOURCES}
    PROPERTIES COMPILE_OPTIONS "-O2")

  add_executable(avoid_speed_memory_test tests/frenet/avoid_speed_memory_test.cpp)
  target_include_directories(avoid_speed_memory_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src)
  add_test(NAME avoid_speed_memory_test COMMAND avoid_speed_memory_test)

  add_executable(geometry_foundation_test
    tests/frenet/geometry_foundation_test.cpp
  )
  target_link_libraries(geometry_foundation_test
    asmc_planner_frenet_core
  )
  add_test(NAME geometry_foundation_test COMMAND geometry_foundation_test)

  add_executable(polygon_collision_test
    tests/frenet/polygon_collision_test.cpp
  )
  target_link_libraries(polygon_collision_test
    asmc_planner_frenet_core
  )
  add_test(NAME polygon_collision_test COMMAND polygon_collision_test)

  add_executable(following_rear_collision_test tests/frenet/following_rear_collision_test.cpp)
  target_compile_definitions(following_rear_collision_test PRIVATE
    FRENET_TEST_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/frenet/params.yaml")
  target_link_libraries(following_rear_collision_test asmc_planner_frenet_core)
  add_test(NAME following_rear_collision_test COMMAND following_rear_collision_test)
  add_executable(stopping_continuation_test tests/frenet/stopping_continuation_test.cpp)
  target_compile_definitions(stopping_continuation_test PRIVATE
    FRENET_TEST_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/frenet/params.yaml")
  target_link_libraries(stopping_continuation_test asmc_planner_frenet_core)
  add_test(NAME stopping_continuation_test COMMAND stopping_continuation_test)

  add_executable(message_converter_test
    tests/frenet/message_converter_test.cpp
  )
  add_dependencies(message_converter_test
    ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(message_converter_test
    asmc_planner_ros_adapters
    ${catkin_LIBRARIES}
  )
  add_test(NAME message_converter_test COMMAND message_converter_test)

  add_executable(planner_command_defaults_test
    tests/frenet/planner_command_defaults_test.cpp
  )
  target_include_directories(planner_command_defaults_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )
  add_test(NAME planner_command_defaults_test
           COMMAND planner_command_defaults_test)

  add_executable(command_keep_plan_test
    tests/frenet/command_keep_plan_test.cpp
  )
  target_compile_definitions(command_keep_plan_test PRIVATE
    FRENET_PACKAGE_SRC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/src"
    FRENET_TEST_WAYPOINT_FILE="${CMAKE_CURRENT_SOURCE_DIR}/../../R_KR_PG_KATRI/2026_molit_comp_global_path.txt"
  )
  target_link_libraries(command_keep_plan_test
    asmc_planner_frenet_core
  )
  add_test(NAME command_keep_plan_test COMMAND command_keep_plan_test)

  add_executable(config_loader_validation_test
    tests/frenet/config_loader_validation_test.cpp
  )
  target_compile_definitions(config_loader_validation_test PRIVATE
    FRENET_PACKAGE_SRC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/src"
    FRENET_TEST_WAYPOINT_FILE="${CMAKE_CURRENT_SOURCE_DIR}/../../R_KR_PG_KATRI/2026_molit_comp_global_path.txt"
  )
  target_link_libraries(config_loader_validation_test
    asmc_planner_frenet_core
  )
  add_test(NAME config_loader_validation_test COMMAND config_loader_validation_test)

  add_executable(command_validation_test
    tests/frenet/command_validation_test.cpp
  )
  target_compile_definitions(command_validation_test PRIVATE
    FRENET_PACKAGE_SRC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/src"
    FRENET_TEST_WAYPOINT_FILE="${CMAKE_CURRENT_SOURCE_DIR}/../../R_KR_PG_KATRI/2026_molit_comp_global_path.txt"
  )
  target_link_libraries(command_validation_test
    asmc_planner_frenet_core
  )
  add_test(NAME command_validation_test COMMAND command_validation_test)

  add_executable(command_stop_plan_test
    tests/frenet/command_stop_plan_test.cpp
  )
  target_compile_definitions(command_stop_plan_test PRIVATE
    FRENET_PACKAGE_SRC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/src"
    FRENET_TEST_WAYPOINT_FILE="${CMAKE_CURRENT_SOURCE_DIR}/../../R_KR_PG_KATRI/2026_molit_comp_global_path.txt"
  )
  target_link_libraries(command_stop_plan_test
    asmc_planner_frenet_core
  )
  add_test(NAME command_stop_plan_test COMMAND command_stop_plan_test)

  # D5-1: 외부 FOLLOW의 authoritative 대상 확보 회귀. resolver를 legacy
  # nearest selector로 바꾸면 2건이 실패한다.
  add_executable(command_follow_plan_test
    tests/frenet/command_follow_plan_test.cpp
  )
  target_compile_definitions(command_follow_plan_test PRIVATE
    FRENET_PACKAGE_SRC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/src"
    FRENET_TEST_WAYPOINT_FILE="${CMAKE_CURRENT_SOURCE_DIR}/../../R_KR_PG_KATRI/2026_molit_comp_global_path.txt"
  )
  target_link_libraries(command_follow_plan_test
    asmc_planner_frenet_core
  )
  add_test(NAME command_follow_plan_test COMMAND command_follow_plan_test)

  # D6-1: 외부 AVOID의 authoritative 대상 확보 회귀. resolver를
  # nearest-object selection으로 되돌리면 이 검사가 실패한다.
  add_executable(command_avoid_target_test
    tests/frenet/command_avoid_target_test.cpp
  )
  target_compile_definitions(command_avoid_target_test PRIVATE
    FRENET_PACKAGE_SRC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/src"
    FRENET_TEST_WAYPOINT_FILE="${CMAKE_CURRENT_SOURCE_DIR}/../../R_KR_PG_KATRI/2026_molit_comp_global_path.txt"
  )
  target_link_libraries(command_avoid_target_test
    asmc_planner_frenet_core
    asmc_planner_execution_core asmc_planner_ros_adapters
    ${catkin_LIBRARIES}
  )
  add_test(NAME command_avoid_target_test COMMAND command_avoid_target_test)

  # 외부 LANE_CHANGE의 PREPARE -> READY -> EXECUTE -> COMMITTED -> EXECUTING
  # 회귀. AVOID의 transaction/lifecycle 패턴 재사용을 검증한다.
  add_executable(command_lane_change_plan_test
    tests/frenet/command_lane_change_plan_test.cpp
  )
  target_compile_definitions(command_lane_change_plan_test PRIVATE
    FRENET_PACKAGE_SRC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/src"
    FRENET_TEST_WAYPOINT_FILE="${CMAKE_CURRENT_SOURCE_DIR}/../../R_KR_PG_KATRI/2026_molit_comp_global_path.txt"
  )
  target_link_libraries(command_lane_change_plan_test
    asmc_planner_frenet_core
    asmc_planner_execution_core asmc_planner_ros_adapters
    ${catkin_LIBRARIES}
  )
  add_test(NAME command_lane_change_plan_test COMMAND command_lane_change_plan_test)

  # D4-3: 외부 EMERGENCY_STOP의 직접 감속 궤적 회귀. EMERGENCY 전용 분기를
  # 제거하면(후보 파이프라인으로 흘려보내면) 10건이 실패한다.
  add_executable(command_emergency_plan_test
    tests/frenet/command_emergency_plan_test.cpp
  )
  target_compile_definitions(command_emergency_plan_test PRIVATE
    FRENET_PACKAGE_SRC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/src"
    FRENET_TEST_WAYPOINT_FILE="${CMAKE_CURRENT_SOURCE_DIR}/../../R_KR_PG_KATRI/2026_molit_comp_global_path.txt"
  )
  target_link_libraries(command_emergency_plan_test
    asmc_planner_frenet_core
  )
  add_test(NAME command_emergency_plan_test
           COMMAND command_emergency_plan_test)

  # D3-1: BehaviorCommand <-> 내부 BehaviorState enum 매핑 회귀.
  # static_cast가 실제로 틀린다는 것까지 negative case로 고정한다.
  add_executable(v2_enum_mapping_test
    tests/frenet/v2_enum_mapping_test.cpp
  )
  add_dependencies(v2_enum_mapping_test
    ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(v2_enum_mapping_test
    asmc_planner_ros_adapters
    ${catkin_LIBRARIES}
  )
  add_test(NAME v2_enum_mapping_test COMMAND v2_enum_mapping_test)

  # D3-2: BehaviorCommand validation + transaction fingerprint 회귀.
  # PREPARE -> EXECUTE가 같은 transaction으로 인정되는지, 형식 오류가 STALE로
  # 가려지지 않는지를 중점적으로 검증한다.
  add_executable(behavior_command_adapter_test
    tests/frenet/behavior_command_adapter_test.cpp
  )
  add_dependencies(behavior_command_adapter_test
    ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(behavior_command_adapter_test
    asmc_planner_ros_adapters
    ${catkin_LIBRARIES}
  )
  add_test(NAME behavior_command_adapter_test
           COMMAND behavior_command_adapter_test)

  # D3-3: transaction / terminal latch / preemption 회귀.
  add_executable(command_execution_manager_test
    tests/frenet/command_execution_manager_test.cpp
  )
  add_dependencies(command_execution_manager_test
    ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(command_execution_manager_test
    asmc_planner_execution_core asmc_planner_ros_adapters
    ${catkin_LIBRARIES}
  )
  add_test(NAME command_execution_manager_test
           COMMAND command_execution_manager_test)

  # Offline shadow tool consumes externally supplied, decoded bag frames.
  add_executable(roundabout_merge_shadow tools/frenet/roundabout_merge_shadow.cpp)
  target_link_libraries(roundabout_merge_shadow asmc_planner_frenet_core yaml-cpp)

  add_executable(merge_motion_recovery_test tests/frenet/merge_motion_recovery_test.cpp)
  target_include_directories(merge_motion_recovery_test PRIVATE src)
  target_link_libraries(merge_motion_recovery_test asmc_planner_frenet_core)
  add_test(NAME merge_motion_recovery_test COMMAND merge_motion_recovery_test)
  add_executable(observed_motion_test tests/frenet/observed_motion_test.cpp)
  target_include_directories(observed_motion_test PRIVATE src)
  target_link_libraries(observed_motion_test asmc_planner_frenet_core)
  add_test(NAME observed_motion_test COMMAND observed_motion_test)

  add_executable(object_observation_alignment_test tests/frenet/object_observation_alignment_test.cpp)
  target_link_libraries(object_observation_alignment_test asmc_planner_frenet_core yaml-cpp)
  target_compile_definitions(object_observation_alignment_test PRIVATE
    ASMC_ALIGNMENT_SOURCE="${CMAKE_CURRENT_SOURCE_DIR}")
  add_test(NAME object_observation_alignment_test COMMAND object_observation_alignment_test)

  add_executable(low_speed_keep_collision_test tests/frenet/low_speed_keep_collision_test.cpp)
  target_link_libraries(low_speed_keep_collision_test asmc_planner_frenet_core yaml-cpp)
  target_compile_definitions(low_speed_keep_collision_test PRIVATE
    ASMC_LOW_KEEP_SOURCE="${CMAKE_CURRENT_SOURCE_DIR}")
  add_test(NAME low_speed_keep_collision_test COMMAND low_speed_keep_collision_test)

  add_executable(keep_future_acceleration_test tests/frenet/keep_future_acceleration_test.cpp)
  target_link_libraries(keep_future_acceleration_test asmc_planner_frenet_core yaml-cpp)
  target_compile_definitions(keep_future_acceleration_test PRIVATE
    ASMC_FUTURE_SOURCE="${CMAKE_CURRENT_SOURCE_DIR}")
  add_test(NAME keep_future_acceleration_test COMMAND keep_future_acceleration_test)

  add_executable(keep_departure_road_test tests/frenet/keep_departure_road_test.cpp)
  target_link_libraries(keep_departure_road_test asmc_planner_frenet_core yaml-cpp)
  target_compile_definitions(keep_departure_road_test PRIVATE
    ASMC_DEPARTURE_ROAD_SOURCE="${CMAKE_CURRENT_SOURCE_DIR}")
  add_test(NAME keep_departure_road_test COMMAND keep_departure_road_test)

  foreach(merge_test roundabout_candidate_test roundabout_execution_test roundabout_prediction_test roundabout_departure_bounds_test hd_map_bundle_test)
    add_executable(${merge_test} tests/frenet/${merge_test}.cpp)
    target_link_libraries(${merge_test} asmc_planner_frenet_core yaml-cpp)
    target_compile_definitions(${merge_test} PRIVATE
      FRENET_PACKAGE_SRC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/src"
      FRENET_TEST_WAYPOINT_FILE="${CMAKE_CURRENT_SOURCE_DIR}/../../R_KR_PG_KATRI/2026_molit_comp_global_path.txt")
    add_test(NAME ${merge_test} COMMAND ${merge_test})
  endforeach()

  add_executable(roundabout_execution_shadow tools/frenet/roundabout_execution_shadow.cpp)
  target_link_libraries(roundabout_execution_shadow asmc_planner_frenet_core yaml-cpp)

  # MERGE contract only: does not enable the runtime capability gate.
  add_executable(merge_contract_test tests/frenet/merge_contract_test.cpp)
  add_dependencies(merge_contract_test ${catkin_EXPORTED_TARGETS})
  target_link_libraries(merge_contract_test
    asmc_planner_execution_core asmc_planner_ros_adapters
    ${catkin_LIBRARIES}
  )
  add_test(NAME merge_contract_test COMMAND merge_contract_test)

  # fix/yuntae-frenet-speed-continuity: longitudinal candidate validity 회귀.
  # negative s_d(reverse) candidate와 excessive jerk candidate가 accel-only
  # 필터를 통과해 valid로 남는지(수정 전) / 걸러지는지(수정 후) 확인한다.
  add_executable(longitudinal_kinematics_test
    tests/frenet/longitudinal_kinematics_test.cpp
  )
  target_link_libraries(longitudinal_kinematics_test
    asmc_planner_frenet_core
  )
  add_test(NAME longitudinal_kinematics_test COMMAND longitudinal_kinematics_test)

  # fix/yuntae-frenet-speed-continuity Phase 1: distance-aware curve speed
  # envelope 회귀. 실제 LookaheadTargetSpeed/VelocityFromCurvature
  # (Speed/CurveSpeed.hpp)를 손으로 구성한 RefLine으로 직접 호출한다.
  add_executable(curve_speed_envelope_test
    tests/frenet/curve_speed_envelope_test.cpp
  )
  target_link_libraries(curve_speed_envelope_test
    asmc_planner_frenet_core
  )
  add_test(NAME curve_speed_envelope_test COMMAND curve_speed_envelope_test)

  # Current curve ceiling must produce braking before the curve, including
  # repeated 20Hz replanning and the KEEP continuity candidate path.
  add_executable(spatial_keep_speed_test tests/frenet/spatial_keep_speed_test.cpp)
  target_link_libraries(spatial_keep_speed_test asmc_planner_frenet_core)
  add_test(NAME spatial_keep_speed_test COMMAND spatial_keep_speed_test)

  # Legacy target name retained for build compatibility. The resolver now
  # owns only stateless raw-cap composition; temporal limits are covered by
  # speed_profiler_test below.
  add_executable(target_speed_rate_limit_test
    tests/frenet/target_speed_rate_limit_test.cpp
  )
  target_link_libraries(target_speed_rate_limit_test
    asmc_planner_frenet_core
  )
  add_test(NAME target_speed_rate_limit_test COMMAND target_speed_rate_limit_test)

  # General Speed Profiler: raw cap -> jerk/acceleration-limited normal KEEP
  # applied target. This owns the temporal invariant formerly split between
  # TargetSpeedResolver and candidate-aware regeneration.
  add_executable(speed_profiler_test
    tests/frenet/speed_profiler_test.cpp
  )
  target_link_libraries(speed_profiler_test
    asmc_planner_frenet_core
  )
  add_test(NAME speed_profiler_test COMMAND speed_profiler_test)

  # fix/yuntae-frenet-speed-continuity Phase 5: low-speed KEEP candidate
  # starvation handoff 회귀.
  add_executable(low_speed_keep_handoff_test
    tests/frenet/low_speed_keep_handoff_test.cpp
  )
  target_compile_definitions(low_speed_keep_handoff_test PRIVATE
    FRENET_PACKAGE_SRC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/src"
    FRENET_TEST_WAYPOINT_FILE="${CMAKE_CURRENT_SOURCE_DIR}/../../R_KR_PG_KATRI/2026_molit_comp_global_path.txt"
  )
  target_link_libraries(low_speed_keep_handoff_test
    asmc_planner_frenet_core
  )
  add_test(NAME low_speed_keep_handoff_test COMMAND low_speed_keep_handoff_test)

  # fix/yuntae-frenet-speed-continuity Phase A: KEEP replanning
  # braking-memory 회귀.
  add_executable(braking_memory_test
    tests/frenet/braking_memory_test.cpp
  )
  target_compile_definitions(braking_memory_test PRIVATE
    FRENET_PACKAGE_SRC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/src"
    FRENET_TEST_WAYPOINT_FILE="${CMAKE_CURRENT_SOURCE_DIR}/../../R_KR_PG_KATRI/2026_molit_comp_global_path.txt"
  )
  target_link_libraries(braking_memory_test
    asmc_planner_frenet_core
  )
  add_test(NAME braking_memory_test COMMAND braking_memory_test)

  # fix/yuntae-frenet-speed-continuity Phase B: moderate-speed KEEP
  # curvature starvation 회귀.
  add_executable(curvature_continuity_test
    tests/frenet/curvature_continuity_test.cpp
  )
  target_compile_definitions(curvature_continuity_test PRIVATE
    FRENET_PACKAGE_SRC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/src"
    FRENET_TEST_WAYPOINT_FILE="${CMAKE_CURRENT_SOURCE_DIR}/../../R_KR_PG_KATRI/2026_molit_comp_global_path.txt"
  )
  target_link_libraries(curvature_continuity_test
    asmc_planner_frenet_core
  )
  add_test(NAME curvature_continuity_test COMMAND curvature_continuity_test)

  # fix/yuntae-planner Commit A: KEEP centerline candidate 안정화 회귀.
  add_executable(keep_candidate_target_test
    tests/frenet/keep_candidate_target_test.cpp
  )
  target_link_libraries(keep_candidate_target_test
    asmc_planner_frenet_core
  )
  add_test(NAME keep_candidate_target_test COMMAND keep_candidate_target_test)

  add_executable(keep_cruise_scope_test tests/frenet/keep_cruise_scope_test.cpp)
  target_link_libraries(keep_cruise_scope_test asmc_planner_frenet_core)
  add_test(NAME keep_cruise_scope_test COMMAND keep_cruise_scope_test)

  # Recorded AVOID approach failure and low-speed SHIFT continuity regression.
  add_executable(avoid_tracking_handoff_test tests/frenet/avoid_tracking_handoff_test.cpp)
  target_compile_definitions(avoid_tracking_handoff_test PRIVATE
    FRENET_PACKAGE_SRC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/src"
    FRENET_TEST_WAYPOINT_FILE="${CMAKE_CURRENT_SOURCE_DIR}/../../R_KR_PG_KATRI/2026_molit_comp_global_path.txt"
  )
  target_link_libraries(avoid_tracking_handoff_test asmc_planner_frenet_core)
  add_test(NAME avoid_tracking_handoff_test COMMAND avoid_tracking_handoff_test)

  add_executable(avoid_boundary_recovery_test tests/frenet/avoid_boundary_recovery_test.cpp)
  target_link_libraries(avoid_boundary_recovery_test asmc_planner_frenet_core)
  add_test(NAME avoid_boundary_recovery_test COMMAND avoid_boundary_recovery_test)

  add_executable(avoid_road_constraints_test tests/frenet/avoid_road_constraints_test.cpp)
  target_compile_definitions(avoid_road_constraints_test PRIVATE
    FRENET_PACKAGE_SRC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/src"
    FRENET_TEST_WAYPOINT_FILE="${CMAKE_CURRENT_SOURCE_DIR}/../../R_KR_PG_KATRI/2026_molit_comp_global_path.txt")
  target_link_libraries(avoid_road_constraints_test asmc_planner_frenet_core)
  add_test(NAME avoid_road_constraints_test COMMAND avoid_road_constraints_test)

  # R5-F: TrajectoryRequest -> GenerateRequestedCandidates 경계 계약 회귀.
  add_executable(request_native_generator_test
    tests/frenet/request_native_generator_test.cpp
  )
  target_link_libraries(request_native_generator_test
    asmc_planner_frenet_core
  )
  add_test(NAME request_native_generator_test COMMAND request_native_generator_test)

  # fix/yuntae-planner Commit F: 폐루프 reference topology(전처리/조회) 연속성 회귀.
  add_executable(closed_loop_reference_test
    tests/frenet/closed_loop_reference_test.cpp
  )
  target_compile_definitions(closed_loop_reference_test PRIVATE
    FRENET_PACKAGE_SRC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/src"
    FRENET_TEST_WAYPOINT_FILE="${CMAKE_CURRENT_SOURCE_DIR}/../../R_KR_PG_KATRI/2026_molit_comp_global_path.txt"
  )
  target_link_libraries(closed_loop_reference_test
    asmc_planner_frenet_core
  )
  add_test(NAME closed_loop_reference_test COMMAND closed_loop_reference_test)

  add_executable(highway_execution_test tests/frenet/highway_execution_test.cpp)
  target_link_libraries(highway_execution_test asmc_planner_frenet_core)
  target_compile_definitions(highway_execution_test PRIVATE
    FRENET_TEST_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/frenet/params.yaml")
  add_test(NAME highway_execution_test COMMAND highway_execution_test)
  add_executable(highway_speed_policy_test tests/behavior/highway_speed_policy_test.cpp)
  target_link_libraries(highway_speed_policy_test asmc_planner_behavior_module)
  add_dependencies(highway_speed_policy_test ${catkin_EXPORTED_TARGETS})
  add_test(NAME highway_speed_policy_test COMMAND highway_speed_policy_test)
  add_test(NAME highway_native_preflight_test COMMAND ${CMAKE_COMMAND} -E env
    ASMC_HIGHWAY_PREFLIGHT=$<TARGET_FILE:highway_merge_preflight>
    ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/../../tools/planning/test_highway_merge_native.py)
  add_test(NAME highway_asset_gate_test COMMAND ${PYTHON_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/../../tools/planning/test_check_highway_merge.py)
  add_test(NAME planner_hd_map_bundle_test COMMAND ${PYTHON_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/../../tools/planning/test_sync_hd_map.py)

  # MERGE gap/TTC 순수 계산 회귀. legacy_behavior_regression_test 삭제로
  # 없어진 MERGE 안전망을 대체한다 - behavior auto-selection이 아니라
  # FindRoundaboutGap/FindHighwayMergeGap 자체의 gap/TTC 판단만 검증한다.
  add_executable(merge_gap_test
    tests/frenet/merge_gap_test.cpp
  )
  target_link_libraries(merge_gap_test
    asmc_planner_frenet_core
  )
  add_test(NAME merge_gap_test COMMAND merge_gap_test)


endif()

catkin_install_python(PROGRAMS
  scripts/behavior_dashboard_server.py
  scripts/open_dashboard.py
  scripts/pedal_calibration_node.py
  scripts/steering_calibration_node.py
  tools/frenet/planner_visualizer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
# An imported module must not be a catkin executable relay (the relay exports
# no module globals on import). Keep the same adjacent-module layout in both spaces.
configure_file(scripts/dashboard_observation.py
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/dashboard_observation.py COPYONLY)
install(FILES scripts/dashboard_observation.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS v2_ros_regression_test empty_objects_test_publisher lane_change_test_publisher
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(asmc_behavior_acceptance_node tests/acceptance/behavior_main.cpp)
add_dependencies(asmc_behavior_acceptance_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(asmc_behavior_acceptance_node asmc_planner_ros_adapters)
install(TARGETS asmc_behavior_acceptance_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  add_executable(planner_lifecycle_equivalence
    tests/equivalence/phase5_execution_equivalence.cpp
    tests/equivalence/phase5_legacy_manager.cpp)
  add_executable(planner_request_equivalence
    tests/equivalence/phase5_resume_request_equivalence.cpp
    tests/equivalence/phase5_legacy_request.cpp)
  add_executable(planner_maneuver_equivalence tests/equivalence/phase6_maneuver_equivalence.cpp)
  add_executable(planner_fsm_equivalence
    tests/equivalence/planner_fsm_equivalence.cpp
    tests/equivalence/planner_baseline_fsm.cpp)
  foreach(gate planner_lifecycle_equivalence planner_request_equivalence
      planner_maneuver_equivalence planner_fsm_equivalence)
    add_dependencies(${gate} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(${gate} asmc_planner_ros_adapters)
  endforeach()
endif()


if(CATKIN_ENABLE_TESTING)
  foreach(regression avoid_completion_test avoid_reconnection_test avoid_return_distance_test avoid_return_progress_test avoid_side_search_test spatial_stop_speed_test stop_road_braking_test vehicle_curvature_test)
    add_executable(${regression} tests/frenet/${regression}.cpp)
    target_link_libraries(${regression} asmc_planner_frenet_core yaml-cpp)
    target_compile_definitions(${regression} PRIVATE
      FRENET_PACKAGE_SRC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/src"
      FRENET_TEST_WAYPOINT_FILE="${CMAKE_CURRENT_SOURCE_DIR}/../../R_KR_PG_KATRI/2026_molit_comp_global_path.txt")
    add_test(NAME ${regression} COMMAND ${regression})
  endforeach()
endif()

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(dynamic_object_policy_test tests/behavior/dynamic_object_policy_test.cpp)
  if(TARGET dynamic_object_policy_test)
    target_compile_definitions(dynamic_object_policy_test PRIVATE ASMC_POLICY_SOURCE="${CMAKE_CURRENT_SOURCE_DIR}")
    target_link_libraries(dynamic_object_policy_test asmc_planner_ros_adapters ${catkin_LIBRARIES} gtest_main)
  endif()
endif()

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(competition_speed_alignment_test tests/behavior/competition_speed_alignment_test.cpp)
  if(TARGET competition_speed_alignment_test)
    target_include_directories(competition_speed_alignment_test PRIVATE src)
    target_link_libraries(competition_speed_alignment_test gtest_main)
  endif()
endif()

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(signal_policy_test tests/behavior/signal_policy_test.cpp)
  catkin_add_gtest(original_frame_test tests/behavior/original_frame_test.cpp)
  if(TARGET original_frame_test)
    add_dependencies(original_frame_test ${catkin_EXPORTED_TARGETS})
    target_include_directories(original_frame_test PRIVATE src ${catkin_INCLUDE_DIRS})
    target_link_libraries(original_frame_test ${catkin_LIBRARIES} gtest_main)
  endif()
  if(TARGET signal_policy_test)
    target_link_libraries(signal_policy_test asmc_planner_behavior_core asmc_planner_frenet_core gtest_main)
  endif()
  add_rostest_gtest(signal_journal_test tests/signal_journal.test tests/behavior/signal_journal_test.cpp)
  if(TARGET signal_journal_test)
    add_dependencies(signal_journal_test ${catkin_EXPORTED_TARGETS})
    target_link_libraries(signal_journal_test asmc_planner_ros_adapters ${catkin_LIBRARIES})
  endif()
endif()

# Pure HTTP prebind helper must accompany the catkin-installed ROS script.
configure_file(scripts/dashboard_startup.py
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/dashboard_startup.py COPYONLY)
install(FILES scripts/dashboard_startup.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
# LOCAL-1 is opt-in; reuse the existing pure geometry/speed implementation.
add_library(asmc_local_planner_core src/local/planner.cpp src/local/speed_profile.cpp)
target_include_directories(asmc_local_planner_core PUBLIC src)
target_compile_options(asmc_local_planner_core PRIVATE -O2 -Wall -Wextra)
target_link_libraries(asmc_local_planner_core asmc_planner_frenet_core)
add_executable(asmc_local_planner_node src/local/node.cpp)
add_dependencies(asmc_local_planner_node ${catkin_EXPORTED_TARGETS})
target_include_directories(asmc_local_planner_node PRIVATE src ${catkin_INCLUDE_DIRS})
target_compile_options(asmc_local_planner_node PRIVATE -O2)
target_link_libraries(asmc_local_planner_node asmc_local_planner_core ${catkin_LIBRARIES} pthread)
install(TARGETS asmc_local_planner_core asmc_local_planner_node
 ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
 add_executable(local_planner_test tests/local_planner_test.cpp)
 target_include_directories(local_planner_test PRIVATE src)
 target_link_libraries(local_planner_test asmc_local_planner_core pthread)
 add_test(NAME local_planner_regression COMMAND local_planner_test)
endif()

# Observer-only exact-stamp raster cache shared by dashboard source/install runs.
configure_file(scripts/local_debug_cache.py
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/local_debug_cache.py COPYONLY)
install(FILES scripts/local_debug_cache.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
