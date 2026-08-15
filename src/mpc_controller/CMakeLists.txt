cmake_minimum_required(VERSION 3.10)
project(mpc_node)

# ROS/catkin 의존성이 없는 standalone CMake 프로젝트다.

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O2)

find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)

# ========================================
# include 경로
# ========================================
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${YAML_CPP_INCLUDE_DIR}
)

# ========================================
# 단일 실행 파일로 빌드
# ========================================
add_executable(mpc_node
  src/main.cpp
  src/global/global.cpp
  src/global/parameter_loader.cpp
  src/model/vehicle_model.cpp
  src/cost/cost_function.cpp
  src/constraints/constraints.cpp
  src/solver/mpc_solver.cpp
  src/udp_network/udp_manager.cpp
  src/udp_network/sender/sender.cpp
  src/udp_network/sender/ctrl_cmd_sender.cpp
  src/udp_network/receiver/receiver.cpp
  src/udp_network/receiver/planned_path_receiver.cpp
)

target_link_libraries(mpc_node
  yaml-cpp
  Threads::Threads
)

# 실행 위치(cwd)와 무관하게 config를 찾도록 절대경로를 컴파일 타임에 주입
target_compile_definitions(mpc_node PRIVATE
  MPC_PACKAGE_SRC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/src"
)

# ========================================
# Install
# ========================================
install(TARGETS mpc_node
  RUNTIME DESTINATION bin
)

install(DIRECTORY src/config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY src/udp_network/
  DESTINATION share/${PROJECT_NAME}/udp_network
  FILES_MATCHING PATTERN "*.yaml"
)
