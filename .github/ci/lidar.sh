#!/usr/bin/env bash
# Run from the repository root after the single workspace build.
source .github/ci/ros-env.sh
python3 tools/audit/check_lidar_canonicalization.py
python3 src/asmc_lidar/test/test_tracking_samples.py
python3 src/asmc_lidar/test/test_compare_traces.py
"${ASMC_CI_DEVEL:-devel}/lib/planning_inputs/planning_lidar_observation_test"
"${ASMC_CI_DEVEL:-devel}/lib/planning_inputs/planning_group_conversion_test"
"${ASMC_CI_DEVEL:-devel}/lib/planning_inputs/planning_lidar_companion_test"
"${ASMC_CI_DEVEL:-devel}/lib/planning_inputs/planning_gps_recovery_test"
python3 src/planning_inputs/test/lidar_launch_selection_test.py
rostest planning_inputs lidar_observation.test
(
  cd "$ASMC_CI_BUILD/asmc_lidar"
  ctest --output-on-failure -T Test --no-compress-output
)
python3 src/asmc_lidar/test/pipeline_ci_check.py --devel "${ASMC_CI_DEVEL:-devel}" --build "$ASMC_CI_BUILD/asmc_lidar" --results "$ASMC_CI_BUILD/test_results/asmc_lidar"
python3 src/planning_inputs/test/common_lidar_ros_test.py
python3 src/planning_inputs/test/ros_boundary_smoke_test.py
node src/asmc_lidar/test/monitor/test_monitor_view.js
