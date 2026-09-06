cmake_minimum_required(VERSION 3.0.2)
project(mpc_controller)

add_compile_options(-std=c++17 -O2 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS asmc_msgs morai_msgs roscpp)
find_package(yaml-cpp REQUIRED)

catkin_package(CATKIN_DEPENDS asmc_msgs morai_msgs roscpp)

include_directories(src ${catkin_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIRS})

set(MPC_CORE_SOURCES
  src/config/parameter_loader.cpp
  src/model/vehicle_model.cpp
  src/cost/cost_function.cpp
  src/constraints/constraints.cpp
  src/solver/mpc_solver.cpp
  src/reference/path_resampler.cpp
  src/control/mpc_controller.cpp)

add_library(mpc_core ${MPC_CORE_SOURCES})
target_link_libraries(mpc_core yaml-cpp)
target_compile_definitions(mpc_core PRIVATE
  MPC_PACKAGE_SRC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/src")

add_library(mpc_ros_converter src/message_converter.cpp)
add_dependencies(mpc_ros_converter ${catkin_EXPORTED_TARGETS})
target_link_libraries(mpc_ros_converter mpc_core ${catkin_LIBRARIES})

add_executable(mpc_controller_node src/main.cpp src/mpc_controller_node.cpp)
add_dependencies(mpc_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(mpc_controller_node
  mpc_core mpc_ros_converter yaml-cpp ${catkin_LIBRARIES})
target_compile_definitions(mpc_controller_node PRIVATE
  MPC_PACKAGE_SRC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/src")

install(TARGETS mpc_core mpc_ros_converter mpc_controller_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config src/config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(mpc_ros_converter_test test/message_converter_test.cpp)
  if(TARGET mpc_ros_converter_test)
    add_dependencies(mpc_ros_converter_test ${catkin_EXPORTED_TARGETS})
    target_link_libraries(mpc_ros_converter_test
      mpc_ros_converter mpc_core ${catkin_LIBRARIES})
  endif()

  catkin_add_gtest(mpc_controller_test test/mpc_controller_test.cpp)
  if(TARGET mpc_controller_test)
    target_link_libraries(mpc_controller_test mpc_core)
  endif()
endif()
