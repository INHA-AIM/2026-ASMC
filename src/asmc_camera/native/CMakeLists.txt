cmake_minimum_required(VERSION 3.10)

# Can be built separately for an offline CPU benchmark without a ROS workspace.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(asmc_lane_association LANGUAGES CXX)
  set(ASMC_NATIVE_PYTHON_OUTPUT "${CMAKE_BINARY_DIR}/python/asmc_camera")
endif()

find_path(ASMC_PYBIND11_INCLUDE_DIR pybind11/pybind11.h)
if(NOT ASMC_PYBIND11_INCLUDE_DIR)
  if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    message(FATAL_ERROR "pybind11 headers required; set ASMC_PYBIND11_INCLUDE_DIR")
  endif()
  message(STATUS "asmc_camera: pybind11 headers absent; using Python association")
  return()
endif()

find_package(PythonInterp 3 REQUIRED)
find_package(PythonLibs ${PYTHON_VERSION_STRING} EXACT REQUIRED)

add_library(_lane_association MODULE lane_association.cpp)
target_include_directories(_lane_association PRIVATE ${ASMC_PYBIND11_INCLUDE_DIR} ${PYTHON_INCLUDE_DIRS})
target_link_libraries(_lane_association PRIVATE ${PYTHON_LIBRARIES})
target_compile_features(_lane_association PRIVATE cxx_std_14)
target_compile_options(_lane_association PRIVATE -O3 -fvisibility=hidden -ffp-contract=off)
set_target_properties(_lane_association PROPERTIES PREFIX "" LIBRARY_OUTPUT_DIRECTORY "${ASMC_NATIVE_PYTHON_OUTPUT}")
if(DEFINED CATKIN_PACKAGE_PYTHON_DESTINATION)
  install(TARGETS _lane_association LIBRARY DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})
endif()
