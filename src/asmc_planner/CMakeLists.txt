cmake_minimum_required(VERSION 3.0.2)
project(asmc_planner LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  planning_inputs asmc_msgs morai_msgs roscpp std_msgs geometry_msgs
  sensor_msgs visualization_msgs)
find_package(yaml-cpp REQUIRED)
find_package(gencpp REQUIRED)
# The external debug datatype stays behavior_planner/UrbanFeatureDebug.
# Generate its unchanged schema privately without depending on that package.
set(COMPAT_DEBUG_INCLUDE ${CMAKE_CURRENT_BINARY_DIR}/compat_include)
set(COMPAT_DEBUG_HEADER ${COMPAT_DEBUG_INCLUDE}/behavior_planner/UrbanFeatureDebug.h)
file(MAKE_DIRECTORY ${COMPAT_DEBUG_INCLUDE}/behavior_planner)
add_custom_command(OUTPUT ${COMPAT_DEBUG_HEADER}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENCPP_BIN}
    ${CMAKE_CURRENT_SOURCE_DIR}/msg/compat/UrbanFeatureDebug.msg
    -Istd_msgs:${std_msgs_DIR}/../msg
    -Ibehavior_planner:${CMAKE_CURRENT_SOURCE_DIR}/msg/compat
    -p behavior_planner -o ${COMPAT_DEBUG_INCLUDE}/behavior_planner
    -e ${GENCPP_TEMPLATE_DIR}
  DEPENDS msg/compat/UrbanFeatureDebug.msg
    ${std_msgs_DIR}/../msg/Header.msg ${GENCPP_BIN})
add_custom_target(asmc_planner_compat_debug_header DEPENDS ${COMPAT_DEBUG_HEADER})
catkin_package(INCLUDE_DIRS src
  LIBRARIES asmc_planner_frenet_core asmc_planner_behavior_core
    asmc_planner_execution_core asmc_planner_behavior_module asmc_planner_ros_adapters
  CATKIN_DEPENDS planning_inputs asmc_msgs morai_msgs roscpp std_msgs
    geometry_msgs sensor_msgs visualization_msgs)

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

add_library(asmc_planner_behavior_core
  src/Behavior/FSM/DecisionStability.cpp
  src/Execution/ManeuverLifecycle.cpp
  src/Execution/BehaviorFsmCompatibility.cpp)
target_include_directories(asmc_planner_behavior_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src)
set_target_properties(asmc_planner_behavior_core PROPERTIES CXX_EXTENSIONS OFF)
target_compile_options(asmc_planner_behavior_core PRIVATE -Wall -Wextra)

add_library(asmc_planner_frenet_core
  src/Frenet/Config/ConfigLoader.cpp
  src/Frenet/Config/ConfigValidator.cpp
  src/Frenet/Evaluation/CollisionChecker.cpp
  src/Frenet/Evaluation/Cost.cpp
  src/Frenet/Evaluation/KinematicFilter.cpp
  src/Frenet/Generator/BoundaryConditionPolicy.cpp
  src/Frenet/Generator/CandidateCombiner.cpp
  src/Frenet/Generator/LateralGenerator.cpp
  src/Frenet/Generator/LongitudinalGenerator.cpp
  src/Frenet/Generator/SamplingUtils.cpp
  src/Frenet/Geometry/PolygonUtils.cpp
  src/Frenet/Maneuver/Avoid/Avoid.cpp
  src/Frenet/Maneuver/Avoid/AvoidLifecycle.cpp
  src/Frenet/Maneuver/Avoid/AvoidTargetResolver.cpp
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
target_link_libraries(asmc_planner_frenet_core yaml-cpp)

# Phase 1 executable remains inert while pure Frenet calculations are linked
# through the separate library above.
add_executable(asmc_planner_scaffold
  src/main.cpp
  src/Planning/Planner.cpp)
target_include_directories(asmc_planner_scaffold PRIVATE src)

install(TARGETS asmc_planner_scaffold
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS asmc_planner_frenet_core asmc_planner_behavior_core asmc_planner_execution_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY src/Frenet/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/Frenet
  FILES_MATCHING PATTERN "*.hpp")
install(DIRECTORY src/Global/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/Global
  FILES_MATCHING PATTERN "*.hpp")
install(DIRECTORY src/Behavior src/Execution src/Planning
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp")

add_library(asmc_planner_behavior_module
  src/Behavior/Decision/behavior_decision.cpp
  src/Behavior/Decision/behavior_scorer.cpp
  src/Behavior/Decision/transition_condition.cpp
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
  asmc_planner_behavior_core asmc_planner_execution_core yaml-cpp ${catkin_LIBRARIES})

add_library(asmc_planner_ros_adapters
  src/Output/BehaviorMessages.cpp
  src/Output/PlanningFeedback.cpp
  src/Output/PlanningOutput.cpp
  src/Parser/BehaviorCommandAdapter.cpp
  src/Parser/BehaviorInput.cpp
  src/Parser/DetectedObjects.cpp
  src/Parser/MoraiObjects.cpp
  src/Parser/PlanningInput.cpp
  src/Parser/PlanningInputCallbacks.cpp
  src/Parser/RuntimeConfig.cpp
  src/Parser/Traffic/ros_traffic_light_provider.cpp
  src/Planning/LegacyCycleCompatibility.cpp
  src/Planning/PlanningPort.cpp
  src/Planning/ProductionRuntime.cpp
  src/Planning/GpsModeSource.cpp
  src/Planning/TargetProjection.cpp
  src/Visualization/Behavior/behavior_visualization.cpp
  src/Visualization/PlanningMarkers.cpp)
target_include_directories(asmc_planner_ros_adapters PUBLIC src ${catkin_INCLUDE_DIRS}
  PRIVATE ${COMPAT_DEBUG_INCLUDE})
add_dependencies(asmc_planner_ros_adapters asmc_planner_compat_debug_header ${catkin_EXPORTED_TARGETS})
target_link_libraries(asmc_planner_ros_adapters
  asmc_planner_frenet_core asmc_planner_behavior_module
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
add_dependencies(asmc_planner_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(asmc_planner_node asmc_planner_ros_adapters)
add_executable(asmc_control_selector_node
  src/Planning/control_selector_main.cpp src/Planning/ControlSelector.cpp)
target_include_directories(asmc_control_selector_node PRIVATE src ${catkin_INCLUDE_DIRS})
add_dependencies(asmc_control_selector_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(asmc_control_selector_node ${catkin_LIBRARIES})
install(TARGETS asmc_planner_behavior_module asmc_planner_ros_adapters asmc_planner_node
    asmc_control_selector_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY src/Parser src/Output src/Visualization
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp")
install(FILES ${COMPAT_DEBUG_HEADER}
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/behavior_planner)
install(DIRECTORY config launch msg DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(asmc_mode_detector_test tests/mode_detector_test.cpp)
  if(TARGET asmc_mode_detector_test)
    target_include_directories(asmc_mode_detector_test PRIVATE src ${catkin_INCLUDE_DIRS})
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
  # Explicit offline gate; preserve the existing registered regression set.
  add_executable(asmc_internal_flow_equivalence tests/internal_flow_equivalence.cpp)
  add_dependencies(asmc_internal_flow_equivalence ${catkin_EXPORTED_TARGETS})
  target_link_libraries(asmc_internal_flow_equivalence
    asmc_planner_execution_core asmc_planner_ros_adapters ${catkin_LIBRARIES})
endif()
