#include <ros/ros.h>
#include "Output/RuntimeGuard.hpp"

#include "frenet_planner_node.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "frenet_planner");
    if (!asmc_planner::LegacyRuntimeAllowed()) return 1;
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    frenet_planner_ros::FrenetPlannerNode node(nh, pnh);
    ros::spin();
    return 0;
}
