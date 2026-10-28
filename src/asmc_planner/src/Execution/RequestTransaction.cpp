#include "Execution/RequestTransaction.hpp"
#include <algorithm>
#include <cmath>
namespace behavior_planner {
namespace {
using Command = asmc_msgs::BehaviorCommand;
using Feedback = asmc_msgs::FrenetFeedback;
constexpr double kPoseTolerance = 1e-4;
bool NearlyEqual(double lhs, double rhs, double tolerance) {
  return std::isfinite(lhs) && std::isfinite(rhs) && std::abs(lhs - rhs) <= tolerance;
}
}
bool RequestTransaction::usesHandshake(uint8_t maneuver_type) {
  return maneuver_type == Command::MERGE ||
         maneuver_type == Command::LANE_CHANGE ||
         maneuver_type == Command::AVOID;
}

bool RequestTransaction::sameIdentity(const Command& lhs, const Command& rhs) {
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
  // derived by the behavior scorer.  They are frozen in issued_request_
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

asmc_msgs::BehaviorCommand RequestTransaction::update(
    Command candidate, const Feedback* feedback, const ros::Time& now,
    double validity_seconds, bool prepare, bool intersection_approach,
    bool signal_recovery_speed) {
  const bool feedback_matches =
      feedback && has_issued_request_ &&
      feedback->command_id == issued_request_.command_id;
  const bool terminal_feedback =
      feedback_matches && usesHandshake(issued_request_.maneuver_type) &&
      (feedback->execution_state == Feedback::COMPLETED ||
       feedback->execution_state == Feedback::FAILED);

  // 진행 중인 AVOID에서 perception ID가 잠깐 바뀌어도 wire fingerprint와
  // command_id를 바꾸지 않는다. Frenet resolver가 기존 target geometry와
  // 유일하게 일치하는 새 관측 ID를 검증해 재연결한다. terminal feedback
  // 이후에는 새 target으로 정상 transaction을 시작할 수 있다.
  if (has_issued_request_ && !terminal_feedback &&
      issued_request_.maneuver_type == Command::AVOID &&
      candidate.maneuver_type == Command::AVOID) {
    candidate.source_link_id = issued_request_.source_link_id;
    candidate.primary_object_id = issued_request_.primary_object_id;
  }
  if (has_issued_request_ && !terminal_feedback &&
      issued_request_.maneuver_type == Command::MERGE && candidate.maneuver_type == Command::MERGE) {
    candidate = issued_request_;
    candidate.execution_request = prepare
        ? Command::PREPARE : Command::EXECUTE;
    candidate.reason = "roundabout_merge";
  }
  if(has_issued_request_&&!terminal_feedback&&
      issued_request_.maneuver_type==Command::LANE_CHANGE&&candidate.maneuver_type==Command::LANE_CHANGE&&
      issued_request_.direction==candidate.direction&&
      candidate.reason=="slow_leader_highway_lane_change") {
    const auto request=candidate.execution_request;const auto reason=candidate.reason;
    candidate=issued_request_;candidate.execution_request=request;candidate.reason=reason;
  }
  if (terminal_feedback) {
    request_identity_locked_ = false;
  }
  if (feedback_matches && usesHandshake(issued_request_.maneuver_type) &&
      (feedback->execution_state == Feedback::COMMITTED ||
       feedback->execution_state == Feedback::EXECUTING)) {
    request_identity_locked_ = true;
  }
  const bool emergency = candidate.maneuver_type == Command::EMERGENCY_STOP;
  // Context.selected_behavior is already the FSM's authoritative output.
  // A locked transaction may freeze identity changes *within the same
  // maneuver*, but it must not re-decide and reject an FSM mode transition.
  const bool fsm_changed_maneuver =
      has_issued_request_ &&
      candidate.maneuver_type != issued_request_.maneuver_type;

  const bool restart_terminal_identity =
      terminal_feedback && prepare &&
      sameIdentity(issued_request_, candidate);
  const bool candidate_intersection_approach = intersection_approach;
  // Speed is otherwise frozen for each wire transaction. Entering/leaving the
  // map stop-line approach zone is a discrete speed-policy transition, so a
  // non-committed command gets a new identity exactly at the zone boundary.
  // A committed lateral maneuver remains immutable and picks up the ceiling
  // when its transaction completes.
  const bool intersection_approach_policy_changed =
      has_issued_request_ && candidate.maneuver_type != Command::MERGE &&
      !(signal_recovery_speed && issued_signal_recovery_speed_) &&
      candidate_intersection_approach != issued_intersection_approach_;
  const bool signal_recovery_policy_changed=has_issued_request_ &&
      signal_recovery_speed!=issued_signal_recovery_speed_;
  if (!has_issued_request_ || !sameIdentity(issued_request_, candidate) ||
      restart_terminal_identity || intersection_approach_policy_changed || signal_recovery_policy_changed) {
    if (!request_identity_locked_ || emergency || fsm_changed_maneuver) {
      ++next_command_id_;
      if (next_command_id_ == 0) ++next_command_id_;
      candidate.command_id = next_command_id_;
      issued_request_ = candidate;
      has_issued_request_ = true;
      request_identity_locked_ = false;
      issued_intersection_approach_ = candidate_intersection_approach;
      issued_signal_recovery_speed_ = signal_recovery_speed;
    }
  }

  // Refresh only mutable fields.  The immutable payload remains frozen for
  // the lifetime of the transaction, exactly matching Frenet's fingerprint.
  issued_request_.header.stamp = now;
  issued_request_.valid_until =
      now + ros::Duration(std::max(0.05, validity_seconds));
  issued_request_.reason = candidate.reason;

  if (usesHandshake(issued_request_.maneuver_type)) {
    if (candidate.execution_request == Command::EXECUTE) {
      issued_request_.execution_request = Command::EXECUTE;
    }
    // EXECUTE is monotonic for the lifetime of this command_id.
  } else {
    issued_request_.execution_request = Command::EXECUTE;
  }
  return issued_request_;
}

}
