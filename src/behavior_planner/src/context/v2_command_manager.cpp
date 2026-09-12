#include "behavior_planner/context/v2_command_manager.hpp"

#include <algorithm>
#include <cmath>

namespace behavior_planner {
namespace {

using Command = asmc_msgs::BehaviorCommand;
using Feedback = asmc_msgs::FrenetFeedback;

constexpr double kPoseTolerance = 1e-4;

bool NearlyEqual(double lhs, double rhs, double tolerance) {
  return std::isfinite(lhs) && std::isfinite(rhs) &&
         std::abs(lhs - rhs) <= tolerance;
}

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

bool V2CommandManager::usesHandshake(uint8_t maneuver_type) {
  return maneuver_type == Command::MERGE ||
         maneuver_type == Command::LANE_CHANGE ||
         maneuver_type == Command::AVOID;
}

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

bool V2CommandManager::sameIdentity(const Command& lhs, const Command& rhs) {
  if (lhs.maneuver_type != rhs.maneuver_type ||
      lhs.direction != rhs.direction || lhs.merge_type != rhs.merge_type ||
      lhs.source_link_id != rhs.source_link_id ||
      lhs.target_link_id != rhs.target_link_id ||
      lhs.has_target_pose != rhs.has_target_pose ||
      lhs.primary_object_id != rhs.primary_object_id ||
      lhs.preferred_front_object_id != rhs.preferred_front_object_id ||
      lhs.preferred_rear_object_id != rhs.preferred_rear_object_id ||
      lhs.use_custom_follow_gap != rhs.use_custom_follow_gap) {
    return false;
  }
  // Speed and gap values are immutable on the wire but are continuously
  // derived by the behavior scorer.  They are frozen in active_command_
  // instead of turning every small control update into a new maneuver.
  if (!lhs.has_target_pose) return true;
  return NearlyEqual(lhs.target_pose_map.position.x,
                     rhs.target_pose_map.position.x, kPoseTolerance) &&
         NearlyEqual(lhs.target_pose_map.position.y,
                     rhs.target_pose_map.position.y, kPoseTolerance) &&
         NearlyEqual(lhs.target_pose_map.position.z,
                     rhs.target_pose_map.position.z, kPoseTolerance) &&
         NearlyEqual(lhs.target_pose_map.orientation.x,
                     rhs.target_pose_map.orientation.x, kPoseTolerance) &&
         NearlyEqual(lhs.target_pose_map.orientation.y,
                     rhs.target_pose_map.orientation.y, kPoseTolerance) &&
         NearlyEqual(lhs.target_pose_map.orientation.z,
                     rhs.target_pose_map.orientation.z, kPoseTolerance) &&
         NearlyEqual(lhs.target_pose_map.orientation.w,
                     rhs.target_pose_map.orientation.w, kPoseTolerance);
}

asmc_msgs::BehaviorCommand V2CommandManager::update(
    const BehaviorContextData& context, const Feedback* feedback,
    const ros::Time& now, double validity_seconds) {
  Command candidate = makeCandidate(context, now, validity_seconds);
  const bool feedback_matches =
      feedback && has_active_command_ &&
      feedback->command_id == active_command_.command_id;
  const bool terminal_feedback =
      feedback_matches && usesHandshake(active_command_.maneuver_type) &&
      (feedback->execution_state == Feedback::COMPLETED ||
       feedback->execution_state == Feedback::FAILED);
  if (terminal_feedback) {
    active_transaction_locked_ = false;
  }
  if (feedback_matches && usesHandshake(active_command_.maneuver_type) &&
      (feedback->execution_state == Feedback::COMMITTED ||
       feedback->execution_state == Feedback::EXECUTING)) {
    active_transaction_locked_ = true;
  }
  const bool emergency = candidate.maneuver_type == Command::EMERGENCY_STOP;
  const auto preempt_it = context.extra_flags.find("stop_preempts_avoid");
  const bool planned_stop_preempts_avoid =
      candidate.maneuver_type == Command::STOP &&
      active_command_.maneuver_type == Command::AVOID &&
      preempt_it != context.extra_flags.end() && preempt_it->second;

  const bool restart_terminal_identity =
      terminal_feedback && context.maneuver_phase == ManeuverPhase::PREPARE &&
      sameIdentity(active_command_, candidate);
  if (!has_active_command_ || !sameIdentity(active_command_, candidate) ||
      restart_terminal_identity) {
    if (!active_transaction_locked_ || emergency ||
        planned_stop_preempts_avoid) {
      ++next_command_id_;
      if (next_command_id_ == 0) ++next_command_id_;
      candidate.command_id = next_command_id_;
      active_command_ = candidate;
      has_active_command_ = true;
      active_transaction_locked_ = false;
    }
  }

  // Refresh only mutable fields.  The immutable payload remains frozen for
  // the lifetime of the transaction, exactly matching Frenet's fingerprint.
  active_command_.header.stamp = now;
  active_command_.valid_until =
      now + ros::Duration(std::max(0.05, validity_seconds));
  active_command_.reason = candidate.reason;

  if (usesHandshake(active_command_.maneuver_type)) {
    if (candidate.execution_request == Command::EXECUTE) {
      active_command_.execution_request = Command::EXECUTE;
    }
    // EXECUTE is monotonic for the lifetime of this command_id.
  } else {
    active_command_.execution_request = Command::EXECUTE;
  }
  return active_command_;
}

}  // namespace behavior_planner
