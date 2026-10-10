cmake_minimum_required(VERSION 3.0.2)
project(behavior_planner)

add_compile_options(-std=c++17 -Wall -Wextra)
find_package(catkin REQUIRED COMPONENTS asmc_planner planning_inputs asmc_msgs roscpp std_msgs geometry_msgs visualization_msgs morai_msgs message_generation)
find_package(yaml-cpp REQUIRED)

add_message_files(FILES UrbanFeatureDebug.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS asmc_planner planning_inputs asmc_msgs roscpp std_msgs geometry_msgs visualization_msgs morai_msgs message_runtime)
# Preserve the requested src/<area> layout while exposing conventional
# behavior_planner/<area>/... include paths to this target.
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/include)
execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_BINARY_DIR}/include/behavior_planner)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/include ${catkin_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIRS})

add_executable(behavior_node src/main.cpp)
add_dependencies(behavior_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(behavior_node asmc_planner_ros_adapters ${catkin_LIBRARIES} yaml-cpp)

install(TARGETS behavior_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS
  scripts/behavior_dashboard_server.py
  scripts/morai_objects_adapter.py
  scripts/open_dashboard.py
  scripts/pedal_calibration_node.py
  scripts/steering_calibration_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch src/config web DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  add_executable(roundabout_transaction_test test/roundabout_transaction_test.cpp




    )
  add_dependencies(roundabout_transaction_test ${catkin_EXPORTED_TARGETS})
  target_link_libraries(roundabout_transaction_test ${catkin_LIBRARIES} yaml-cpp)
  add_test(NAME roundabout_transaction_test COMMAND roundabout_transaction_test)

  catkin_add_gtest(fsm_command_binding_test
    test/fsm_command_binding_test.cpp
    )
  if(TARGET fsm_command_binding_test)
    target_link_libraries(fsm_command_binding_test asmc_planner_behavior_core)
    target_include_directories(fsm_command_binding_test PRIVATE
      ${CMAKE_CURRENT_BINARY_DIR}/include)
  endif()

  catkin_add_gtest(detected_msg_converter_test
    test/detected_msg_converter_test.cpp
    )
  if(TARGET detected_msg_converter_test)
    add_dependencies(detected_msg_converter_test
      ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_include_directories(detected_msg_converter_test PRIVATE
      ${CMAKE_CURRENT_BINARY_DIR}/include ${catkin_INCLUDE_DIRS})
    target_link_libraries(detected_msg_converter_test ${catkin_LIBRARIES})
  endif()

  catkin_add_gtest(behavior_logic_test
    test/behavior_logic_test.cpp





    )
  if(TARGET behavior_logic_test)
    add_dependencies(behavior_logic_test
      ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_include_directories(behavior_logic_test PRIVATE
      ${CMAKE_CURRENT_BINARY_DIR}/include ${catkin_INCLUDE_DIRS})
    target_link_libraries(behavior_logic_test ${catkin_LIBRARIES})
  endif()

  catkin_add_gtest(route_signal_movement_test
    test/route_signal_movement_test.cpp


    )
  if(TARGET route_signal_movement_test)
    add_dependencies(route_signal_movement_test
      ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_include_directories(route_signal_movement_test PRIVATE
      ${CMAKE_CURRENT_BINARY_DIR}/include ${catkin_INCLUDE_DIRS})
    target_compile_definitions(route_signal_movement_test PRIVATE
      BEHAVIOR_PLANNER_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
    target_link_libraries(route_signal_movement_test
      ${catkin_LIBRARIES} yaml-cpp)
  endif()

  catkin_add_gtest(feature_builder_avoid_test
    test/feature_builder_avoid_test.cpp




    )
  if(TARGET feature_builder_avoid_test)
    add_dependencies(feature_builder_avoid_test
      ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_include_directories(feature_builder_avoid_test PRIVATE
      ${CMAKE_CURRENT_BINARY_DIR}/include ${catkin_INCLUDE_DIRS})
    target_compile_definitions(feature_builder_avoid_test PRIVATE
      BEHAVIOR_PLANNER_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
    target_link_libraries(feature_builder_avoid_test
      ${catkin_LIBRARIES} yaml-cpp)
  endif()
endif()
