cmake_minimum_required(VERSION 3.0.2)
project(vehicle_state)

add_compile_options(-std=c++17 -O2)

find_package(catkin REQUIRED COMPONENTS
  asmc_msgs
  morai_msgs
  roscpp
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vehicle_state_adapter
  CATKIN_DEPENDS asmc_msgs morai_msgs roscpp sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Canonical vehicle-state boundary. GPS/IMU is the competition source and
# EgoVehicleStatus is development-only. No runtime fallback connects them.
add_library(vehicle_state_adapter
  src/vehicle_state_adapter.cpp
  src/gps_transform.cpp
)
add_dependencies(vehicle_state_adapter ${catkin_EXPORTED_TARGETS})
target_link_libraries(vehicle_state_adapter ${catkin_LIBRARIES})

add_executable(vehicle_state_node
  src/vehicle_state_node.cpp)
add_dependencies(vehicle_state_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(vehicle_state_node
  vehicle_state_adapter ${catkin_LIBRARIES})

add_executable(ego_vehicle_state_dev_node
  src/ego_vehicle_state_dev_node.cpp)
add_dependencies(ego_vehicle_state_dev_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(ego_vehicle_state_dev_node
  vehicle_state_adapter ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  add_executable(vehicle_state_adapter_test tests/vehicle_state_adapter_test.cpp)
  add_dependencies(vehicle_state_adapter_test ${catkin_EXPORTED_TARGETS})
  target_link_libraries(vehicle_state_adapter_test
    vehicle_state_adapter ${catkin_LIBRARIES})
  add_test(NAME vehicle_state_adapter_test COMMAND vehicle_state_adapter_test)
endif()

install(TARGETS
    vehicle_state_adapter
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
