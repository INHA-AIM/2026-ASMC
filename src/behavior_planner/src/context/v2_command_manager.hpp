#pragma once

#include <cstdint>

#include <asmc_msgs/BehaviorCommand.h>
#include <asmc_msgs/FrenetFeedback.h>
#include <ros/time.h>

#include "behavior_planner/context/behavior_context.hpp"

namespace behavior_planner {

// Owns the BehaviorCommand transaction identity required by the Frenet v2
// contract.  It is deliberately independent of ROS publishers so the same
// behavior_node can expose either the v1 or v2 boundary.
class V2CommandManager {
 public:
  asmc_msgs::BehaviorCommand update(
      const BehaviorContextData& context,
      const asmc_msgs::FrenetFeedback* feedback, const ros::Time& now,
      double validity_seconds);

  uint32_t activeCommandId() const {
    return has_active_command_ ? active_command_.command_id : 0;
  }

 private:
  static asmc_msgs::BehaviorCommand makeCandidate(
      const BehaviorContextData& context, const ros::Time& now,
      double validity_seconds);
  static bool sameIdentity(const asmc_msgs::BehaviorCommand& lhs,
                           const asmc_msgs::BehaviorCommand& rhs);
  static bool usesHandshake(uint8_t maneuver_type);

  asmc_msgs::BehaviorCommand active_command_;
  uint32_t next_command_id_ = 0;
  bool has_active_command_ = false;
  bool active_transaction_locked_ = false;
  bool active_signal_speed_limit_ = false;
};

}  // namespace behavior_planner
