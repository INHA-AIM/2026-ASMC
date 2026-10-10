#include "Planning/ProductionRuntime.hpp"
#include <ros/master.h>
#include <algorithm>
int main(int argc, char** argv) {
  ros::init(argc, argv, "asmc_planner");
  ros::V_string nodes;
  if (!ros::master::getNodes(nodes)) return 1;
  for (const auto& legacy : {"/behavior_planner", "/frenet_planner"}) {
    if (std::find(nodes.begin(), nodes.end(), legacy) != nodes.end()) {
      ROS_ERROR("Stop compatibility node %s before starting asmc_planner", legacy);
      return 1;
    }
  }
  std::string interface_version;
  ros::NodeHandle("/behavior_planner").param<std::string>("interface_version", interface_version, "v1");
  if (interface_version != "v2") {
    ROS_ERROR("Use asmc_planner planner_v2.launch; candidate production integration requires interface_version=v2");
    return 1;
  }
  asmc_planner::ProductionRuntime runtime;
  runtime.start();
  ros::waitForShutdown();
  return 0;
}
