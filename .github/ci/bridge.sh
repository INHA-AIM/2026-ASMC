#!/usr/bin/env bash
# Run from the repository root after the single workspace build.
source .github/ci/ros-env.sh
# CTest runs all seven former catkin checks plus vehicle_state_adapter_test.
(
  cd "$ASMC_CI_BUILD/asmc_bridge"
  total=$(ctest -N | sed -n 's/^Total Tests: //p')
  test "${total:-0}" -ge 8
  ctest --output-on-failure
)
catkin_test_results --verbose "$ASMC_CI_BUILD/test_results/asmc_bridge"
