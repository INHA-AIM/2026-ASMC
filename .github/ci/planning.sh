#!/usr/bin/env bash
# Unique checks from Planning integration; common input tests stay in lidar.sh.
source .github/ci/ros-env.sh
python3 src/planning_inputs/test/runtime_launch_contract_test.py
python3 src/planning_inputs/test/object_filter_contract_test.py
node src/asmc_planner/tests/behavior/dashboard_motion_test.js
node src/asmc_planner/tests/behavior/dashboard_shadow_test.js
python3 src/asmc_planner/tests/behavior/dashboard_observation_test.py
python3 src/asmc_planner/tests/behavior/dashboard_startup_test.py
rostest asmc_planner highway_runtime.test
for regression in command_avoid_target_test roundabout_execution_test avoid_road_constraints_test polygon_collision_test prepared_static_inputs_test; do
  test -x "${ASMC_CI_DEVEL:-devel}/lib/asmc_planner/$regression"
done
(
  cd "$ASMC_CI_BUILD/asmc_planner"
  ctest --output-on-failure -R '^(command_avoid_target_test|roundabout_execution_test|avoid_road_constraints_test|polygon_collision_test|prepared_static_inputs_test|_ctest_asmc_planner_gtest_detected_msg_converter_test|_ctest_asmc_planner_gtest_feature_builder_avoid_test|_ctest_asmc_planner_gtest_dynamic_object_policy_test)$'
)

# Opt-in Localization reuses the existing build graph and physical core.
"${ASMC_CI_DEVEL:-devel}/lib/asmc_planner/localization_input_test"
"${ASMC_CI_DEVEL:-devel}/lib/asmc_planner/shadow_localization_test" 15
"${ASMC_CI_DEVEL:-devel}/lib/asmc_controller/control_binding_test"
python3 tools/scen_runner/test_localization_launch_graph.py
rostest asmc_planner localization_handoff.test
rostest asmc_planner localization_seed.test
# LOCAL uses the same single build. Tests own private masters; no MORAI/GPU/bag required.
python3 tools/scen_runner/test_local_alignment.py
python3 tools/scen_runner/test_local_boundary.py
python3 tools/scen_runner/test_local_autonomy.py
python3 tools/scen_runner/test_local_handoff.py
python3 tools/scen_runner/test_local_entry_phase.py
python3 tools/scen_runner/test_local_ownership.py
python3 src/asmc_planner/tests/behavior/local_debug_cache_test.py
(
  cd "$ASMC_CI_BUILD/asmc_planner"
  ctest --output-on-failure -R '^(local_planner|jamming_policy)_regression$'
)

PYTHONPATH=tools/scen_runner python3 -m unittest discover -s tools/scen_runner/test/unit -p 'test_private_master.py'
PYTHONPATH=tools/scen_runner python3 -m unittest discover -s tools/scen_runner/test/unit -p 'test_local_delivery_metrics.py'
PYTHONPATH=tools/scen_runner python3 -m unittest discover -s tools/scen_runner/test/unit -p 'test_local_model_audit.py'
PYTHONPATH=tools/scen_runner python3 -m unittest discover -s tools/scen_runner/test/unit -p 'test_local_route_review.py'
PYTHONPATH=tools/scen_runner python3 -m unittest discover -s tools/scen_runner/test/unit -p 'test_local_margin_audit.py'
(
  cd "$ASMC_CI_BUILD/asmc_controller"
  ctest --output-on-failure -R 'local_execution_history_test'
)

PYTHONPATH=tools/scen_runner python3 -m unittest discover -s tools/scen_runner/test/unit -p 'test_trial_monitor.py'

# Recorder/outcome/gallery regressions retained from the historical LOCAL
# workflow. Main's fast PR graph remains the workflow owner.
PYTHONPATH=tools/scen_runner python3 -m unittest discover -s tools/scen_runner/test/unit -p 'test_sensor_recording.py'
PYTHONPATH=tools/scen_runner python3 -m unittest discover -s tools/scen_runner/test/unit -p 'test_shadow_analysis.py'
PYTHONPATH=tools/scen_runner python3 -m unittest discover -s tools/scen_runner/test/unit -p 'test_local_gallery.py'

# One prepared-command protocol under every selected planner identity.
for strategy in camera local localization; do
  rostest asmc_planner jamming_entry.test strategy:="$strategy"
  rostest asmc_planner jamming_handoff.test strategy:="$strategy"
done
"${ASMC_CI_DEVEL:-devel}/lib/asmc_planner/jamming_wall_scope_test"
