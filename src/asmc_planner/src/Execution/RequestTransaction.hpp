#pragma once
#include <cstdint>
#include <asmc_msgs/BehaviorCommand.h>
#include <asmc_msgs/FrenetFeedback.h>
#include <ros/time.h>
namespace behavior_planner {
// Request identity/handshake sequencing only. This never accepts a command or
// owns the maneuver actually executing in the downstream Execution owner.
class RequestTransaction {
 public:
  asmc_msgs::BehaviorCommand update(asmc_msgs::BehaviorCommand candidate,
      const asmc_msgs::FrenetFeedback* feedback, const ros::Time& now,
      double validity_seconds, bool prepare, bool intersection_approach,
      bool signal_recovery_speed=false);
  uint32_t issuedCommandId() const { return has_issued_request_ ? issued_request_.command_id : 0; }
 private:
  static bool sameIdentity(const asmc_msgs::BehaviorCommand&,const asmc_msgs::BehaviorCommand&);
  static bool usesHandshake(uint8_t);
  asmc_msgs::BehaviorCommand issued_request_;
  uint32_t next_command_id_ = 0;
  bool has_issued_request_ = false;
  bool request_identity_locked_ = false;
  bool issued_intersection_approach_ = false;
  bool issued_signal_recovery_speed_ = false;
};
}
