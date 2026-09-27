cmake_minimum_required(VERSION 3.16)
project(asmc_lidar_ground_benchmark LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(PATCHWORKPP_SOURCE "" CACHE PATH "Pinned, unmodified Patchwork++ checkout")
if(NOT EXISTS "${PATCHWORKPP_SOURCE}/cpp/patchworkpp/src/patchworkpp.cpp")
  message(FATAL_ERROR "Set PATCHWORKPP_SOURCE to the pinned source checkout")
endif()
# Verify all upstream compilation units/headers at revision
# 3e6903a1d5537a4cc2ace897b0bbb98a92d6014c, including archive-only snapshots.
function(check_source path expected)
  file(SHA256 "${PATCHWORKPP_SOURCE}/${path}" actual)
  if(NOT actual STREQUAL expected)
    message(FATAL_ERROR "Pinned Patchwork++ source mismatch: ${path}")
  endif()
endfunction()
check_source(cpp/patchworkpp/src/patchworkpp.cpp f7037532eea5027994f672fbdca34be9b7be1033fd8d6f23cf6e41d0912f9b2a)
check_source(cpp/patchworkpp/include/patchwork/patchworkpp.h 3803a5cd4c1064c71fa09f3388369e04ca912d18cdc2b8f52a796a6bbfc362f8)
check_source(cpp/common/src/plane_fit.cpp a9b507089e6946954136a23b4e5ceb91e599fdca4da9ea6b2dc5f97fd8e9370c)
check_source(cpp/common/include/patchwork/plane_fit.h 0f89a35178aa247bbdd80b36b983e8fd4ca85b338b9f8d882af99235b33fc7c2)
check_source(cpp/common/include/patchwork/types.h 4265f49849f598d731908d4ce5ad05a78362fdda8518cd7f712046173bf31816)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common sample_consensus segmentation)
add_executable(ground_benchmark benchmark.cpp
  "${PATCHWORKPP_SOURCE}/cpp/patchworkpp/src/patchworkpp.cpp"
  "${PATCHWORKPP_SOURCE}/cpp/common/src/plane_fit.cpp")
target_include_directories(ground_benchmark PRIVATE
  "${PATCHWORKPP_SOURCE}/cpp/patchworkpp/include"
  "${PATCHWORKPP_SOURCE}/cpp/common/include" ${PCL_INCLUDE_DIRS})
target_link_libraries(ground_benchmark PRIVATE Eigen3::Eigen ${PCL_LIBRARIES})
target_compile_definitions(ground_benchmark PRIVATE ${PCL_DEFINITIONS})
