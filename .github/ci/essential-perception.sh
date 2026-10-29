#!/usr/bin/env bash
set -eo pipefail
source .github/ci/ros-env.sh
python3 tools/audit/check_git_history_assets.py
python3 tools/audit/validate_perception_contract.py
python3 -m unittest tools/audit/test_perception_contract.py
for regression in asmc_lidar_frame_time_core_test asmc_lidar_geometry_boundary_test asmc_lidar_test_tunnel_filter_core asmc_lidar_test_tunnel_route asmc_lidar_test_tunnel_runtime; do
  "${ASMC_CI_DEVEL:-devel}/lib/asmc_lidar/$regression"
done
"${ASMC_CI_DEVEL:-devel}/lib/lane_boundary_adapter/ground_projection_test"
python3 -c 'from asmc_camera import _lane_association; print(_lane_association.__file__)'
python3 -m unittest discover -s src/asmc_camera/test/bev -p 'test_native_lane_association.py'
python3 src/asmc_camera/test/runtime/test_camera_entrypoints.py
python3 src/asmc_camera/test/runtime/test_model_profiles.py
python3 -m unittest discover -s tools/perception -p 'test_prepare_local_camera_engines.py'
