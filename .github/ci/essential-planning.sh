#!/usr/bin/env bash
set -eo pipefail
source .github/ci/ros-env.sh
for regression in planning_lidar_observation_test planning_group_conversion_test planning_lidar_companion_test planning_gps_recovery_test; do
  "${ASMC_CI_DEVEL:-devel}/lib/planning_inputs/$regression"
done
for regression in stop_brake_response_test reference_validator_test; do
  "${ASMC_CI_DEVEL:-devel}/lib/asmc_controller/$regression"
done
"${ASMC_CI_DEVEL:-devel}/lib/asmc_planner/asmc_mode_detector_test"
"${ASMC_CI_DEVEL:-devel}/lib/asmc_planner/highway_merge_preflight" src/asmc_planner/config/frenet/params.yaml
rostest planning_inputs lidar_observation.test

# Fast Localization core checks use the same essential production build.
"${ASMC_CI_DEVEL:-devel}/lib/asmc_planner/localization_input_test"
"${ASMC_CI_DEVEL:-devel}/lib/asmc_planner/shadow_localization_test" 15
"${ASMC_CI_DEVEL:-devel}/lib/asmc_controller/control_binding_test"
# Opt-in LOCAL ships with the same production runtime. Compile and exercise its
# geometry/physical speed and execution-history boundary on every PR.
"${ASMC_CI_DEVEL:-devel}/lib/asmc_planner/local_planner_test"
"${ASMC_CI_DEVEL:-devel}/lib/asmc_planner/jamming_policy_test"
"${ASMC_CI_DEVEL:-devel}/lib/asmc_controller/local_execution_history_test"
python3 tools/scen_runner/test_local_autonomy.py
python3 tools/scen_runner/test_local_handoff.py
python3 tools/scen_runner/test_local_ownership.py

python3 tools/scen_runner/test_localization_launch_graph.py
"${ASMC_CI_DEVEL:-devel}/lib/asmc_planner/jamming_wall_scope_test"
