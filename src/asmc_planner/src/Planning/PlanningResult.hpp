#pragma once
#include <asmc_msgs/Trajectory.h>
namespace asmc_planner {
// Compatibility trajectory value remains unchanged; no transport is required
// until the ROS output adapter publishes this immutable result view.
struct PlanningResult {
  const asmc_msgs::Trajectory& trajectory;
  bool previous_plan_still_valid;
};
}
