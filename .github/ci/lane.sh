#!/usr/bin/env bash
# Run from the repository root after the single workspace build.
source .github/ci/ros-env.sh
"${ASMC_CI_DEVEL:-devel}/lib/lane_boundary_adapter/ground_projection_test"
for fixture_case in camera_bbox_lane camera_bbox_clipped camera_bbox_lane_change camera_bbox_ego_mask camera_bbox_imu_projection; do
  rostest lane_boundary_adapter "$fixture_case.test"
done
