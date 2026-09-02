#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "frenet_planner");
    ros::NodeHandle node;

    ROS_INFO("Frenet Planner ROS runtime initialized; planning shell is not connected yet.");
    ros::spin();
    return 0;
}
