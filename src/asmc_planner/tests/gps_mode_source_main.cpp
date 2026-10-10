#include <ros/ros.h>
#include "Planning/GpsModeSource.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "gps_mode_source_test_node");
  asmc_planner::GpsModeSource source{ros::NodeHandle()};
  ros::spin();
  return 0;
}
