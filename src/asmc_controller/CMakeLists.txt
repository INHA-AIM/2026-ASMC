cmake_minimum_required(VERSION 3.0.2)
project(asmc_controller)

add_compile_options(-std=c++17 -O2 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS asmc_msgs morai_msgs roscpp std_msgs nav_msgs diagnostic_msgs)
find_package(yaml-cpp REQUIRED)

catkin_package(CATKIN_DEPENDS asmc_msgs morai_msgs roscpp std_msgs nav_msgs diagnostic_msgs)

include_directories(src ${catkin_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIRS})

set(MPC_CORE_SOURCES
  src/Global/Global.cpp
  src/Util/ParameterLoader.cpp
  src/MPC/Model/vehicle_model.cpp
  src/MPC/Model/steering_delay.cpp
  src/MPC/Cost/cost_function.cpp
  src/MPC/Constraints/constraints.cpp
  src/MPC/Solver/mpc_solver.cpp
  src/MPC/Reference/braking_reference.cpp
  src/MPC/Reference/path_resampler.cpp
  src/MPC/Reference/trajectory_timing.cpp
  src/MPC/Reference/reference_validator.cpp
  src/MPC/MPC.cpp)

add_library(mpc_core ${MPC_CORE_SOURCES})
target_link_libraries(mpc_core yaml-cpp)

add_library(mpc_ros_converter src/Util/MessageConverter.cpp)
add_dependencies(mpc_ros_converter ${catkin_EXPORTED_TARGETS})
target_link_libraries(mpc_ros_converter mpc_core ${catkin_LIBRARIES})

add_executable(mpc_controller_node src/main.cpp src/Control.cpp)
add_dependencies(mpc_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(mpc_controller_node
  mpc_core mpc_ros_converter yaml-cpp ${catkin_LIBRARIES})

install(TARGETS mpc_core mpc_ros_converter mpc_controller_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(control_binding_test test/control_binding_test.cpp)
  if(TARGET control_binding_test)
    add_dependencies(control_binding_test ${catkin_EXPORTED_TARGETS})
    target_link_libraries(control_binding_test ${catkin_LIBRARIES} gtest_main)
  endif()
  catkin_add_gtest(stop_acceleration_limit_test test/stop_acceleration_limit_test.cpp)
  if(TARGET stop_acceleration_limit_test)
    target_link_libraries(stop_acceleration_limit_test mpc_core gtest_main)
  endif()

  catkin_add_gtest(stop_brake_response_test test/stop_brake_response_test.cpp)
  if(TARGET stop_brake_response_test)
    target_link_libraries(stop_brake_response_test gtest_main)
  endif()

  catkin_add_gtest(straight_stability_test test/straight_stability_test.cpp)
  if(TARGET straight_stability_test)
    target_compile_definitions(straight_stability_test PRIVATE
      CONTROLLER_YAML_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/controller.yaml")
    target_link_libraries(straight_stability_test mpc_core gtest_main)
  endif()
  catkin_add_gtest(parameter_loader_test test/parameter_loader_test.cpp)
  if(TARGET parameter_loader_test)
    target_compile_definitions(parameter_loader_test PRIVATE
      CONTROLLER_YAML_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/controller.yaml")
    target_link_libraries(parameter_loader_test mpc_core gtest_main)
  endif()

  find_package(rostest REQUIRED)
  set(CONTROLLER_FAILSAFE_ROSTEST
    "${CMAKE_CURRENT_BINARY_DIR}/controller_failsafe_verification.test")
  file(WRITE "${CONTROLLER_FAILSAFE_ROSTEST}"
    "<launch>\n"
    "  <test test-name=\"controller_failsafe_verification\" "
    "pkg=\"asmc_controller\" type=\"controller_failsafe_verification_test\" "
    "time-limit=\"60\"/>\n"
    "</launch>\n")
  add_rostest_gtest(controller_failsafe_verification_test
    "${CONTROLLER_FAILSAFE_ROSTEST}"
    test/controller_failsafe_verification_test.cpp src/Control.cpp)
  if(TARGET controller_failsafe_verification_test)
    add_dependencies(controller_failsafe_verification_test
      ${catkin_EXPORTED_TARGETS})
    target_link_libraries(controller_failsafe_verification_test
      mpc_core mpc_ros_converter ${catkin_LIBRARIES})
  endif()

  catkin_add_gtest(offline_mpc_verification_test
    test/offline_mpc_verification_test.cpp)
  if(TARGET offline_mpc_verification_test)
    target_link_libraries(offline_mpc_verification_test mpc_core)
  endif()

  catkin_add_gtest(braking_warm_start_test test/braking_warm_start_test.cpp)
  if(TARGET braking_warm_start_test)
    target_link_libraries(braking_warm_start_test mpc_core gtest_main)
  endif()
  catkin_add_gtest(mpc_ros_converter_test test/message_converter_test.cpp)
  if(TARGET mpc_ros_converter_test)
    add_dependencies(mpc_ros_converter_test ${catkin_EXPORTED_TARGETS})
    target_link_libraries(mpc_ros_converter_test
      mpc_ros_converter mpc_core ${catkin_LIBRARIES})
  endif()

  catkin_add_gtest(braking_steering_test test/braking_steering_test.cpp)
  if(TARGET braking_steering_test)
    target_link_libraries(braking_steering_test mpc_core gtest_main)
  endif()

  catkin_add_gtest(mpc_controller_test test/mpc_controller_test.cpp)
  if(TARGET mpc_controller_test)
    target_link_libraries(mpc_controller_test mpc_core)
  endif()

  catkin_add_gtest(path_resampler_test test/path_resampler_test.cpp)
  if(TARGET path_resampler_test)
    target_link_libraries(path_resampler_test mpc_core)
  endif()

  catkin_add_gtest(acceleration_feedforward_test
    test/acceleration_feedforward_test.cpp)
  if(TARGET acceleration_feedforward_test)
    target_link_libraries(acceleration_feedforward_test mpc_core)
  endif()

  catkin_add_gtest(tracking_error_test test/tracking_error_test.cpp)
  if(TARGET tracking_error_test)
    target_link_libraries(tracking_error_test mpc_core)
  endif()

  catkin_add_gtest(steering_feedforward_test test/steering_feedforward_test.cpp)
  if(TARGET steering_feedforward_test)
    target_link_libraries(steering_feedforward_test mpc_core)
  endif()

  catkin_add_gtest(trajectory_timing_test test/trajectory_timing_test.cpp)
  if(TARGET trajectory_timing_test)
    target_link_libraries(trajectory_timing_test mpc_core)
  endif()

  catkin_add_gtest(reference_validator_test test/reference_validator_test.cpp)
  if(TARGET reference_validator_test)
    target_link_libraries(reference_validator_test mpc_core)
  endif()
  catkin_add_gtest(planner_stop_continuity_test test/planner_stop_continuity_test.cpp)
  if(TARGET planner_stop_continuity_test)
    target_link_libraries(planner_stop_continuity_test mpc_core gtest_main)
  endif()
endif()


add_executable(asmc_local_controller_node src/local/node.cpp)
add_dependencies(asmc_local_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(asmc_local_controller_node mpc_core mpc_ros_converter ${catkin_LIBRARIES})
install(TARGETS asmc_local_controller_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(local_execution_history_test test/local_execution_history_test.cpp)
  if(TARGET local_execution_history_test)
    target_link_libraries(local_execution_history_test mpc_core gtest_main)
  endif()
endif()

if(CATKIN_ENABLE_TESTING)
  add_executable(local_ownership_probe test/local_ownership_probe.cpp)
  target_include_directories(local_ownership_probe PRIVATE src ${catkin_INCLUDE_DIRS})
  target_link_libraries(local_ownership_probe ${catkin_LIBRARIES} pthread)
endif()
