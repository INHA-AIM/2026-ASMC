#pragma once
#include <asmc_lidar/ros/runtime.hpp>
#include <asmc_lidar/tracking/planning_velocity.hpp>
#include <asmc_msgs/LidarPlanningObservation.h>
#include <unordered_map>

namespace asmc::lidar::runtime {
using pipeline::PlanningVelocity;

inline asmc_msgs::LidarPlanningObservation PlanningOutput(
    const asmc_msgs::LidarObservation& observation, const pipeline::ScanResult& scan) {
  asmc_msgs::LidarPlanningObservation out;
  out.observation = observation;
  auto& array = out.observation.objects;
  if ((array.status == array.OBSERVED || array.status == array.OBSERVED_EMPTY) &&
      (!scan.context.map_pose.valid || scan.context.map_pose.query_stamp_ns != scan.context.time.stamp_ns)) {
    array.status = array.POSE_UNAVAILABLE;
    array.reason = "planning_map_pose_unavailable";
    array.objects.clear();
  }
  std::unordered_map<int, const pipeline::TrackedObject*> tracks;
  for (const auto& track : scan.objects) tracks.emplace(track.tracking.s32_Id, &track);
  for (auto& object : out.observation.objects.objects) {
    object.absolute_velocity_valid = object.dynamic_valid = object.is_dynamic = false;
    uint8_t state = out.UNKNOWN;
    const auto found = tracks.find(object.track_id);
    if (found != tracks.end()) {
      const auto velocity = PlanningVelocity(*found->second, scan);
      if (velocity) {
        object.absolute_velocity_valid = true;
        object.absolute_velocity_mps.x = velocity->x();
        object.absolute_velocity_mps.y = velocity->y();
        object.absolute_velocity_mps.z = velocity->z();
        // The diagnostic classifier has a separate maturity gate. Keep unknown
        // and pending distinct; never derive motion from camera class names.
        const auto& motion = found->second->tracking.motion;
        if (motion.motion_valid && motion.s32_MotionState >= out.LOW_THREAT &&
            motion.s32_MotionState <= out.DYNAMIC) state = motion.s32_MotionState;
        object.dynamic_valid = state == out.LOW_THREAT || state == out.DYNAMIC;
        object.is_dynamic = state == out.DYNAMIC;
      }
    }
    out.motion_states.push_back(state);
  }
  return out;
}
}  // namespace asmc::lidar::runtime
