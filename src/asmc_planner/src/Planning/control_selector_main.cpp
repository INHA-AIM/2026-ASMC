#include <ros/ros.h>
#include "Planning/ControlSelector.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "asmc_control_selector");
  asmc_planner::ControlSelector selector{ros::NodeHandle()};
  ros::spin();
  return 0;
}
