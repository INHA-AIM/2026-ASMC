#pragma once

#include <asmc_msgs/BehaviorCommand.h>
#include <asmc_msgs/FrenetFeedback.h>
#include <cmath>
#include <cstdint>

namespace behavior_planner {

// A READY notification asks Behavior to request execution. It is never a
// departure certificate: Planning must recheck today's candidate before COMMIT.
// The owner serializes issuedCommand/observe/consume with its feedback mutex.
class RoundaboutReadyNotice {
 public:
  void issuedCommand(const asmc_msgs::BehaviorCommand& command) {
    using Command = asmc_msgs::BehaviorCommand;
    const bool roundabout = command.command_id != 0 &&
        command.maneuver_type == Command::MERGE &&
        command.merge_type == Command::MERGE_ROUNDABOUT &&
        command.execution_request != Command::CANCEL;
    const auto id = roundabout ? command.command_id : 0;
    if (id != command_id_ || !roundabout) {
      command_id_ = id;
      pending_ = closed_ = false;
    }
  }

  void observe(const asmc_msgs::FrenetFeedback& feedback, double received_at) {
    using Feedback = asmc_msgs::FrenetFeedback;
    if (!command_id_ || feedback.command_id != command_id_) return;
    if (!std::isfinite(received_at) ||
        feedback.active_maneuver != asmc_msgs::BehaviorCommand::MERGE) {
      pending_ = false;
      return;
    }
    if (terminal(feedback.execution_state)) {
      pending_ = false;
      closed_ = true;
      return;
    }
    if (!feedback.command_accepted) {
      pending_ = false;
      return;
    }
    if (!closed_ && feedback.execution_state == Feedback::READY &&
        feedback.plan_valid) {
      pending_ = true;
      received_at_ = received_at;
    }
  }

  bool consume(const asmc_msgs::FrenetFeedback* latest, double now,
               double maximum_age) {
    if (!pending_) return false;
    const double age = now - received_at_;
    if (!std::isfinite(age) || !std::isfinite(maximum_age) || maximum_age <= 0 ||
        age < 0 || age > maximum_age) {
      pending_ = false;
      return false;
    }
    if (!latest) return false;
    if (latest->command_id != command_id_ || !latest->command_accepted ||
        latest->active_maneuver != asmc_msgs::BehaviorCommand::MERGE ||
        terminal(latest->execution_state)) {
      pending_ = false;
      return false;
    }
    pending_ = false;
    return true;
  }

 private:
  static bool terminal(std::uint8_t state) {
    using Feedback = asmc_msgs::FrenetFeedback;
    return state == Feedback::COMMITTED || state == Feedback::EXECUTING ||
           state == Feedback::COMPLETED || state == Feedback::FAILED;
  }
  std::uint32_t command_id_ = 0;
  double received_at_ = 0;
  bool pending_ = false, closed_ = false;
};

}  // namespace behavior_planner
