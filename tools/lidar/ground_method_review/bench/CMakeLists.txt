cmake_minimum_required(VERSION 3.16)
project(asmc_ground_method_bench LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(H1_GENERATED_SOURCE "" CACHE PATH "Generated fixed-height Patchwork++ source")
set(LINEFIT_SOURCE "" CACHE PATH "Pinned linefit package")
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters search segmentation visualization)
add_executable(ground_method_bench bench.cpp
  "${H1_GENERATED_SOURCE}/src/patchworkpp.cpp"
  "${H1_GENERATED_SOURCE}/src/plane_fit.cpp"
  "${LINEFIT_SOURCE}/src/ground_segmentation.cc"
  "${LINEFIT_SOURCE}/src/segment.cc"
  "${LINEFIT_SOURCE}/src/bin.cc"
  "${LINEFIT_SOURCE}/src/viewer.cc")
target_include_directories(ground_method_bench PRIVATE
  "${H1_GENERATED_SOURCE}/include" "${LINEFIT_SOURCE}/include" ${PCL_INCLUDE_DIRS})
target_link_libraries(ground_method_bench PRIVATE Eigen3::Eigen ${PCL_LIBRARIES} pthread)
target_compile_options(ground_method_bench PRIVATE -O2)
