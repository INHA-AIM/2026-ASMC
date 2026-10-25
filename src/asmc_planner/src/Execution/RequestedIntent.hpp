#pragma once
#include "Behavior/Context/behavior_context.hpp"
#include <asmc_msgs/BehaviorCommand.h>
#include <asmc_msgs/control_binding.hpp>
namespace asmc_planner {
// Native semantic decision plus the exact existing transaction value. The
// generated command struct is retained as a compatibility value type: this
// object is passed by C++ reference, never serialized or sent over a topic.
struct RequestedIntent {
  const behavior_planner::BehaviorContextData& decision;
  const asmc_msgs::BehaviorCommand& command;
  const asmc_msgs::binding::Request* control_request=nullptr;
};
}
