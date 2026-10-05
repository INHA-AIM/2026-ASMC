cmake_minimum_required(VERSION 3.16)
project(lidar_height_review LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(PCL REQUIRED COMPONENTS common filters segmentation search)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
get_filename_component(REPO "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
set(PKG "${REPO}/src/asmc_lidar")
execute_process(COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/generate.py
  ${REPO} ${CMAKE_CURRENT_BINARY_DIR}/generated RESULT_VARIABLE generated)
if(NOT generated EQUAL 0)
  message(FATAL_ERROR "Pinned source generation failed")
endif()
add_library(h1_backend STATIC
  ${CMAKE_CURRENT_BINARY_DIR}/generated/src/patchworkpp.cpp
  ${CMAKE_CURRENT_BINARY_DIR}/generated/src/plane_fit.cpp
  ${PKG}/vendor/jaeho/Lidar/TunnelFilter/TunnelFilterCore.cpp
  ${PKG}/src/general.cpp ${PKG}/src/lshape.cpp)
target_include_directories(h1_backend PUBLIC
  ${CMAKE_CURRENT_BINARY_DIR}/generated/include ${PKG}/vendor/jaeho
  ${PKG}/include ${REPO}/tools/lidar ${PCL_INCLUDE_DIRS})
target_link_libraries(h1_backend PUBLIC Eigen3::Eigen ${PCL_LIBRARIES} yaml-cpp)
target_compile_options(h1_backend PRIVATE -O2 -Wall -Wextra)
add_executable(height_review review.cpp)
target_link_libraries(height_review PRIVATE h1_backend)
target_compile_options(height_review PRIVATE -O2 -Wall -Wextra)
enable_testing()
add_executable(height_test test.cpp)
target_link_libraries(height_test PRIVATE h1_backend)
target_compile_options(height_test PRIVATE -UNDEBUG)
add_test(NAME height_policy COMMAND height_test)
