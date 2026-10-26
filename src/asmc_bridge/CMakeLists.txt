cmake_minimum_required(VERSION 3.0.2)
project(asmc_bridge)

add_compile_options(-std=c++17 -O2)

find_package(catkin REQUIRED COMPONENTS
  asmc_geometry
  asmc_msgs
  diagnostic_msgs
  geometry_msgs
  morai_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES asmc_vehicle_state_adapter
  CATKIN_DEPENDS
    asmc_geometry asmc_msgs diagnostic_msgs geometry_msgs morai_msgs roscpp rospy
    sensor_msgs std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(asmc_vehicle_state_adapter
  src/vehicle_state/vehicle_state_adapter.cpp
)
add_dependencies(asmc_vehicle_state_adapter ${catkin_EXPORTED_TARGETS})
target_link_libraries(asmc_vehicle_state_adapter ${catkin_LIBRARIES})

add_executable(vehicle_state_node
  src/vehicle_state/vehicle_state_node.cpp)
add_dependencies(vehicle_state_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(vehicle_state_node
  asmc_vehicle_state_adapter ${catkin_LIBRARIES})

add_executable(ego_vehicle_state_dev_node
  src/vehicle_state/ego_vehicle_state_dev_node.cpp)
add_dependencies(ego_vehicle_state_dev_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(ego_vehicle_state_dev_node
  asmc_vehicle_state_adapter ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_install_python(PROGRAMS test/check_reporting_ros.py test/check_frame_timing_ros.py
    test/check_vehicle_state_ros.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  add_rostest(test/reporting.test)
  add_rostest(test/frame_timing.test)
  add_rostest(test/vehicle_state.test)
  catkin_add_nosetests(test/test_udp_bridge.py)
  catkin_add_nosetests(test/test_sensor_bridge.py)
  catkin_add_nosetests(test/test_topic_verifier.py)
  catkin_add_nosetests(test/test_telemetry.py)
  catkin_add_nosetests(test/test_frame_timing.py)

  add_executable(vehicle_state_adapter_test
    test/vehicle_state_adapter_test.cpp)
  add_dependencies(vehicle_state_adapter_test ${catkin_EXPORTED_TARGETS})
  target_link_libraries(vehicle_state_adapter_test
    asmc_vehicle_state_adapter ${catkin_LIBRARIES})
  add_test(NAME vehicle_state_adapter_test COMMAND vehicle_state_adapter_test)
endif()

catkin_install_python(PROGRAMS
  scripts/morai_udp_bridge.py
  scripts/morai_sensor_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
    asmc_vehicle_state_adapter
    vehicle_state_node
    ego_vehicle_state_dev_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
