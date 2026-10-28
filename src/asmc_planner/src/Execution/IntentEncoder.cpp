#include "Execution/IntentEncoder.hpp"

#include <algorithm>
#include <cmath>

namespace behavior_planner {
namespace {

using Command = asmc_msgs::BehaviorCommand;
using Feedback = asmc_msgs::FrenetFeedback;

void SetPoseFromYaw(double x, double y, double yaw,
                    geometry_msgs::Pose* pose) {
  pose->position.x = x;
  pose->position.y = y;
  pose->position.z = 0.0;
  pose->orientation.x = 0.0;
  pose->orientation.y = 0.0;
  pose->orientation.z = std::sin(0.5 * yaw);
  pose->orientation.w = std::cos(0.5 * yaw);
}

}  // namespace

asmc_msgs::BehaviorCommand V2CommandManager::makeCandidate(
    const BehaviorContextData& context, const ros::Time& now,
    double validity_seconds) {
  Command command;
  command.header.stamp = now;
  command.header.frame_id = "map";
  command.command_id = 0;
  command.execution_request = Command::EXECUTE;
  command.direction = Command::DIRECTION_NONE;
  command.merge_type = Command::MERGE_NONE;
  command.source_link_id = context.target.current_link_id;
  command.target_link_id.clear();
  command.has_target_pose = false;
  command.target_pose_map.orientation.w = 1.0;
  command.desired_speed_mps =
      std::isfinite(context.target.desired_speed)
          ? std::max(0.0, context.target.desired_speed)
          : -1.0;
  command.primary_object_id = -1;
  command.preferred_front_object_id = -1;
  command.preferred_rear_object_id = -1;
  command.use_custom_follow_gap = false;
  command.desired_time_gap_s = -1.0;
  command.minimum_gap_m = -1.0;
  command.valid_until = now + ros::Duration(std::max(0.05, validity_seconds));
  command.reason = context.debug.selected_reason;

  // force_stop has no geometric target.  The v1 boundary represented it with
  // a side-band boolean; v2 represents the same immediate safety request as
  // EMERGENCY_STOP so an invalid STOP-without-pose is never emitted.
  if (context.hard.force_stop ||
      context.selected_behavior == BehaviorType::EMERGENCY_STOP) {
    command.maneuver_type = Command::EMERGENCY_STOP;
    command.desired_speed_mps = 0.0;
    return command;
  }

  switch (context.selected_behavior) {
    case BehaviorType::KEEP:
      command.maneuver_type = Command::KEEP;
      break;
    case BehaviorType::MERGE:
      command.maneuver_type = Command::MERGE;
      {
        const auto highway=context.extra_flags.find("highway_merge");
        command.merge_type=highway!=context.extra_flags.end()&&highway->second
            ?Command::MERGE_HIGHWAY:Command::MERGE_ROUNDABOUT;
      }
      command.target_link_id = context.target.target_link_id;
      command.direction = Command::DIRECTION_NONE;
      command.execution_request = context.maneuver_phase == ManeuverPhase::PREPARE
          ? Command::PREPARE : Command::EXECUTE;
      command.has_target_pose = false;
      break;
    case BehaviorType::FOLLOW:
      command.maneuver_type = Command::FOLLOW;
      command.primary_object_id = context.target.lead_vehicle_id;
      command.use_custom_follow_gap = true;
      command.desired_time_gap_s = context.target.follow_time_gap;
      command.minimum_gap_m = context.target.follow_min_gap;
      break;
    case BehaviorType::LEFT_CHANGE:
    case BehaviorType::RIGHT_CHANGE:
      command.maneuver_type = Command::LANE_CHANGE;
      command.execution_request =
          context.maneuver_phase == ManeuverPhase::PREPARE
              ? Command::PREPARE
              : Command::EXECUTE;
      command.direction = context.selected_behavior == BehaviorType::LEFT_CHANGE
                              ? Command::DIRECTION_LEFT
                              : Command::DIRECTION_RIGHT;
      command.target_link_id = context.target.target_link_id;
      command.has_target_pose = context.target.has_target_pose;
      if (command.has_target_pose) {
        SetPoseFromYaw(context.target.target_x, context.target.target_y,
                       context.target.target_yaw, &command.target_pose_map);
      }
      break;
    case BehaviorType::STOP:
      command.maneuver_type = Command::STOP;
      command.has_target_pose = context.target.has_target_pose;
      if (command.has_target_pose) {
        SetPoseFromYaw(context.target.target_x, context.target.target_y,
                       context.target.target_yaw, &command.target_pose_map);
      }
      break;
    case BehaviorType::AVOID:
      command.maneuver_type = Command::AVOID;
      command.execution_request =
          context.maneuver_phase == ManeuverPhase::PREPARE
              ? Command::PREPARE
              : Command::EXECUTE;
      command.direction = Command::DIRECTION_NONE;
      command.primary_object_id = context.target.avoidance_object_id;
      break;
    case BehaviorType::YIELD:
      // YIELD has no v2 wire value.  It is currently not selected by the
      // scorer, but KEEP is the only structurally valid non-target fallback.
      command.maneuver_type = Command::KEEP;
      command.reason = "yield_fallback_keep";
      break;
    case BehaviorType::EMERGENCY_STOP:
      command.maneuver_type = Command::EMERGENCY_STOP;
      command.desired_speed_mps = 0.0;
      break;
  }
  return command;
}

asmc_msgs::BehaviorCommand V2CommandManager::update(
    const BehaviorContextData& context, const Feedback* feedback,
    const ros::Time& now, double validity_seconds) {
  const auto approach = context.extra_flags.find("intersection_approach_speed_limit");
  const auto recovery = context.extra_flags.find("signal_recovery_speed_limit");
  return request_transaction_.update(makeCandidate(context, now, validity_seconds),
      feedback, now, validity_seconds,
      context.maneuver_phase == ManeuverPhase::PREPARE,
      approach != context.extra_flags.end() && approach->second,
      recovery != context.extra_flags.end() && recovery->second);
}

}  // namespace behavior_planner
