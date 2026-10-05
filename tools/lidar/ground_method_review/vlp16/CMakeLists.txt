cmake_minimum_required(VERSION 3.16)
project(asmc_linefit_vlp16_review LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(LINEFIT_SOURCE "" CACHE PATH "Pinned linefit_ground_segmentation package")
if(NOT EXISTS "${LINEFIT_SOURCE}/include/ground_segmentation/ground_segmentation.h")
  message(FATAL_ERROR "LINEFIT_SOURCE missing linefit include")
endif()
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters search segmentation visualization)
find_package(yaml-cpp REQUIRED)
add_executable(linefit_vlp16_review review.cpp
  "${LINEFIT_SOURCE}/src/ground_segmentation.cc"
  "${LINEFIT_SOURCE}/src/segment.cc"
  "${LINEFIT_SOURCE}/src/bin.cc"
  "${LINEFIT_SOURCE}/src/viewer.cc")
target_include_directories(linefit_vlp16_review PRIVATE "${LINEFIT_SOURCE}/include" ${PCL_INCLUDE_DIRS})
target_link_libraries(linefit_vlp16_review PRIVATE Eigen3::Eigen ${PCL_LIBRARIES} yaml-cpp pthread)
target_compile_options(linefit_vlp16_review PRIVATE -O2)
