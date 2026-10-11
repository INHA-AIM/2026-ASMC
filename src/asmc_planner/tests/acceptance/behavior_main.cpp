#include <ros/ros.h>
#include "Parser/BehaviorInput.hpp"
// Isolated decision-loop acceptance, never a production composition entry point.
int main(int argc, char** argv) {
  ros::init(argc, argv, "traffic_light_acceptance_behavior");
  behavior_planner::BehaviorInput node(ros::NodeHandle(), ros::NodeHandle("~"));
  node.spin();
  return 0;
}
