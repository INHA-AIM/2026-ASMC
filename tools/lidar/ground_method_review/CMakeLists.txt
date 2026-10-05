cmake_minimum_required(VERSION 3.16)
project(asmc_ground_method_review LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(H1_GENERATED_SOURCE "" CACHE PATH "Generated fixed-height Patchwork++ review source")
set(LINEFIT_SOURCE "" CACHE PATH "Pinned linefit_ground_segmentation package")
if(NOT EXISTS "${H1_GENERATED_SOURCE}/include/patchwork/patchworkpp.h")
  message(FATAL_ERROR "H1_GENERATED_SOURCE missing generated/include/patchwork/patchworkpp.h")
endif()
if(NOT EXISTS "${LINEFIT_SOURCE}/include/ground_segmentation/ground_segmentation.h")
  message(FATAL_ERROR "LINEFIT_SOURCE must be the linefit_ground_segmentation package directory")
endif()
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters search segmentation visualization)
find_package(yaml-cpp REQUIRED)
add_executable(ground_method_review review.cpp
  "${H1_GENERATED_SOURCE}/src/patchworkpp.cpp"
  "${H1_GENERATED_SOURCE}/src/plane_fit.cpp"
  "${LINEFIT_SOURCE}/src/ground_segmentation.cc"
  "${LINEFIT_SOURCE}/src/segment.cc"
  "${LINEFIT_SOURCE}/src/bin.cc"
  "${LINEFIT_SOURCE}/src/viewer.cc")
target_include_directories(ground_method_review PRIVATE
  "${H1_GENERATED_SOURCE}/include" "${LINEFIT_SOURCE}/include" ${PCL_INCLUDE_DIRS})
target_link_libraries(ground_method_review PRIVATE Eigen3::Eigen ${PCL_LIBRARIES} yaml-cpp)
target_compile_options(ground_method_review PRIVATE -O2 -Wall -Wextra)
