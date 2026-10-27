#!/usr/bin/env bash
# Run from the repository root after the single workspace build.
source .github/ci/ros-env.sh
"${ASMC_CI_DEVEL:-devel}/lib/lane_boundary_adapter/ground_projection_test"
"${ASMC_CI_DEVEL:-devel}/lib/lane_boundary_adapter/contact_clearance_test"
"${ASMC_CI_DEVEL:-devel}/lib/lane_boundary_adapter/local_obstacle_support_test"
for fixture_case in camera_bbox_lane camera_bbox_clipped camera_bbox_lane_change camera_bbox_ego_mask camera_bbox_imu_projection camera_frame_bridge camera_bbox_inactive_lifecycle contact_clearance local_contact; do
  rostest lane_boundary_adapter "$fixture_case.test"
done
rostest asmc_planner dashboard_shadow.test
