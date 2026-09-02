cmake_minimum_required(VERSION 3.0.2)
project(mpc_controller)

add_compile_options(-std=c++17 -O2 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS asmc_msgs morai_msgs roscpp)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)

catkin_package(CATKIN_DEPENDS asmc_msgs morai_msgs roscpp)

include_directories(src ${catkin_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIRS})

set(MPC_CORE_SOURCES
  src/global/global.cpp
  src/global/parameter_loader.cpp
  src/model/vehicle_model.cpp
  src/cost/cost_function.cpp
  src/constraints/constraints.cpp
  src/solver/mpc_solver.cpp
  src/runtime/path_resampler.cpp
  src/runtime/control_adapter.cpp)

add_library(mpc_core ${MPC_CORE_SOURCES})
target_link_libraries(mpc_core yaml-cpp)
target_compile_definitions(mpc_core PRIVATE
  MPC_PACKAGE_SRC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/src")

add_executable(mpc_node
  src/main.cpp
  src/udp_network/udp_manager.cpp
  src/udp_network/sender/sender.cpp
  src/udp_network/sender/ctrl_cmd_sender.cpp
  src/udp_network/receiver/receiver.cpp
  src/udp_network/receiver/planned_path_receiver.cpp)
target_link_libraries(mpc_node mpc_core yaml-cpp Threads::Threads)
target_compile_definitions(mpc_node PRIVATE
  MPC_PACKAGE_SRC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/src")

add_library(mpc_ros_converter src/ros/message_converter.cpp)
add_dependencies(mpc_ros_converter ${catkin_EXPORTED_TARGETS})
target_link_libraries(mpc_ros_converter mpc_core ${catkin_LIBRARIES})

add_executable(mpc_ros_node src/ros/main.cpp src/ros/mpc_ros_node.cpp)
add_dependencies(mpc_ros_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(mpc_ros_node
  mpc_core mpc_ros_converter yaml-cpp ${catkin_LIBRARIES})
target_compile_definitions(mpc_ros_node PRIVATE
  MPC_PACKAGE_SRC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/src")

install(TARGETS mpc_core mpc_ros_converter mpc_node mpc_ros_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config src/config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY src/udp_network/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/udp_network
  FILES_MATCHING PATTERN "*.yaml")

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(mpc_ros_converter_test test/message_converter_test.cpp)
  if(TARGET mpc_ros_converter_test)
    add_dependencies(mpc_ros_converter_test ${catkin_EXPORTED_TARGETS})
    target_link_libraries(mpc_ros_converter_test
      mpc_ros_converter mpc_core ${catkin_LIBRARIES})
  endif()
endif()
