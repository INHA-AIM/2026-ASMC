# Build profiles are aggregates, never test runners. Keep the legacy `all`
# graph and test registrations intact for direct catkin/CMake users.
function(asmc_collect_build_targets directory output)
  get_property(local_targets DIRECTORY "${directory}" PROPERTY BUILDSYSTEM_TARGETS)
  set(result "")
  foreach(name IN LISTS local_targets)
    get_target_property(kind "${name}" TYPE)
    if(kind MATCHES "^(EXECUTABLE|STATIC_LIBRARY|SHARED_LIBRARY|MODULE_LIBRARY|OBJECT_LIBRARY)$"
        OR name MATCHES "_generate_messages$")
      list(APPEND result "${name}")
    endif()
  endforeach()
  get_property(children DIRECTORY "${directory}" PROPERTY SUBDIRECTORIES)
  foreach(child IN LISTS children)
    # catkin's bundled googletest is built through consumers, not every unused
    # gmock/example library. Every workspace package remains in the full set.
    if(child MATCHES "^${CMAKE_SOURCE_DIR}/")
      asmc_collect_build_targets("${child}" child_targets)
      list(APPEND result ${child_targets})
    endif()
  endforeach()
  set(${output} "${result}" PARENT_SCOPE)
endfunction()

asmc_collect_build_targets("${CMAKE_SOURCE_DIR}" asmc_full_targets)
list(REMOVE_DUPLICATES asmc_full_targets)
list(SORT asmc_full_targets)
add_custom_target(asmc_full DEPENDS ${asmc_full_targets})

set(asmc_runtime_targets
  asmc_planner_node mpc_controller_node asmc_control_selector_node
  asmc_local_planner_node asmc_shadow_localization_node asmc_local_controller_node
  asmc_competition_speed_state_node planning_vehicle_state_node planning_objects_adapter
  lidar_pipeline_node vehicle_state_node camera_bbox_lane_node camera_bbox_lane_change_node
  _lane_association asmc_camera_generate_messages asmc_hd_map_generate_messages
  lane_boundary_adapter_generate_messages)
set(asmc_missing_runtime "")
foreach(name IN LISTS asmc_runtime_targets)
  if(NOT TARGET "${name}")
    list(APPEND asmc_missing_runtime "${name}")
  endif()
endforeach()
if(NOT asmc_missing_runtime)
  add_custom_target(asmc_runtime DEPENDS ${asmc_runtime_targets})
endif()

set(ASMC_BUILD_REQUESTS "" CACHE STRING "Aggregate selected by scripts/build_ws.sh")
if("asmc_runtime" IN_LIST ASMC_BUILD_REQUESTS AND asmc_missing_runtime)
  message(FATAL_ERROR "Runtime targets missing: ${asmc_missing_runtime}. Check the package whitelist, PCL and pybind11 headers (pybind11-dev or ASMC_PYBIND11_INCLUDE_DIR).")
endif()
if("asmc_full" IN_LIST ASMC_BUILD_REQUESTS AND NOT CATKIN_ENABLE_TESTING)
  message(FATAL_ERROR "The full build requires CATKIN_ENABLE_TESTING=ON; no tests may be silently omitted.")
endif()
if("asmc_full" IN_LIST ASMC_BUILD_REQUESTS AND TARGET asmc_camera_generate_messages
    AND ASMC_CAMERA_NATIVE_ASSOCIATION AND NOT TARGET _lane_association)
  message(FATAL_ERROR "Full camera build requires native pybind11 headers; check pybind11-dev or ASMC_PYBIND11_INCLUDE_DIR.")
endif()
foreach(name IN LISTS ASMC_BUILD_REQUESTS)
  if(NOT TARGET "${name}" OR name STREQUAL "asmc_requested")
    message(FATAL_ERROR "Unknown/recursive requested build target: ${name}")
  endif()
endforeach()
add_custom_target(asmc_requested DEPENDS ${ASMC_BUILD_REQUESTS})

# A small inspectable build inventory; no test execution and no wall-clock stamp.
file(GENERATE OUTPUT "${CMAKE_BINARY_DIR}/asmc-build-targets.txt"
  CONTENT "runtime=${asmc_runtime_targets}\nmissing_runtime=${asmc_missing_runtime}\nfull=${asmc_full_targets}\nrequested=${ASMC_BUILD_REQUESTS}\n")
