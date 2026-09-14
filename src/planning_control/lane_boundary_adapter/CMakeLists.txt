cmake_minimum_required(VERSION 3.0.2)
project(lane_boundary_adapter)

add_compile_options(-std=c++17 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  asmc_msgs
  diagnostic_msgs
  geometry_msgs
  roscpp
  std_msgs
  visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lane_boundary_adapter
  CATKIN_DEPENDS
    asmc_msgs
    diagnostic_msgs
    geometry_msgs
    roscpp
    std_msgs
    visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(lane_boundary_adapter
  src/lane_boundary_adapter.cpp
)
add_dependencies(lane_boundary_adapter ${catkin_EXPORTED_TARGETS})
target_link_libraries(lane_boundary_adapter ${catkin_LIBRARIES})

add_executable(lane_boundary_monitor_node
  src/lane_boundary_monitor_node.cpp
)
add_dependencies(lane_boundary_monitor_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(lane_boundary_monitor_node
  lane_boundary_adapter
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
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
endif()

install(TARGETS
    lane_boundary_adapter
    lane_boundary_monitor_node
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
