#pragma once
#include <ros/master.h>
#include <algorithm>
namespace asmc_planner {
inline bool LegacyRuntimeAllowed() {
  ros::V_string nodes;
  if (!ros::master::getNodes(nodes)) return false;
  if (std::find(nodes.begin(), nodes.end(), "/asmc_planner") != nodes.end()) {
    ROS_ERROR("Stop asmc_planner before starting a legacy compatibility planner");
    return false;
  }
  return true;
}
}
