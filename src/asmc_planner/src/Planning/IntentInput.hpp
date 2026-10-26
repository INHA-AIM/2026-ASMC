#pragma once
#include "Planning/Planner.hpp"
#include "Execution/RequestedIntent.hpp"
#include <mutex>
#include <type_traits>
namespace frenet_planner_ros { class PlanningPort; }
namespace asmc_planner {
template<class RuntimePort>
void Planner::ReceiveIntent(RuntimePort& port, const RequestedIntent& request) {
  // Storage only. Validation, acceptance and lifecycle remain on the existing
  // 20 Hz timer, preserving independent requested and accepted command IDs.
  std::lock_guard<std::mutex> lock(port.input_mutex_);
  port.latest_v2_command_ = request.command;
  if constexpr(std::is_same_v<RuntimePort,frenet_planner_ros::PlanningPort>) {
    port.latest_control_request_=request.control_request ? *request.control_request : asmc_msgs::binding::Request{};
  }
  const auto bounded=request.decision.extra_flags.find("dynamic_in_lane_avoid");
  port.latest_v2_avoid_in_lane_only_=
      request.command.maneuver_type==asmc_msgs::BehaviorCommand::AVOID &&
      request.decision.selected_behavior==behavior_planner::BehaviorType::AVOID &&
      request.command.primary_object_id==request.decision.target.avoidance_object_id &&
      bounded!=request.decision.extra_flags.end() && bounded->second;
  port.has_v2_command_ = true;
}
}
