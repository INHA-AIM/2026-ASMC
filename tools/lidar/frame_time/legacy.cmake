function(check_legacy path expected)
  file(SHA256 "${LEGACY_MAIN_SOURCE}/${path}" actual)
  if(NOT actual STREQUAL expected)
    message(FATAL_ERROR "Legacy probe requires unchanged Son 6435af1: ${path}")
  endif()
endfunction()
check_legacy(src/PlannerInterface/PlannerObjectAdapter.cpp 29d8281292ca4c225220ec89759c0c4c7f07ee0f22b5ac4ae144ff56713b0894)
check_legacy(src/PlannerInterface/PlannerObjectAdapter.hpp 576a4a40b0ae376ab502153175badb2797b3e52747db8d2aa9582ca149e9ef54)
check_legacy(src/Global/Global.hpp d96f607b6c1f025ce22ab35693d029161d6a38c028846535d8ea0bbe4c01fb95)
find_package(catkin REQUIRED COMPONENTS roscpp pcl_conversions morai_msgs Msg tf tf2_ros)
find_package(PCL REQUIRED COMPONENTS common)
find_package(OpenCV REQUIRED)
add_executable(legacy_adapter_probe legacy_probe.cpp
  "${LEGACY_MAIN_SOURCE}/src/PlannerInterface/PlannerObjectAdapter.cpp")
target_include_directories(legacy_adapter_probe PRIVATE "${LEGACY_MAIN_SOURCE}/src"
  ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(legacy_adapter_probe PRIVATE frame_time_core yaml-cpp
  ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})
