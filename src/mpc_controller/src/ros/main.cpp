#include <ros/ros.h>

#include "ros/mpc_ros_node.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "mpc_controller");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    mpc_controller_ros::MpcRosNode node(nh, private_nh);
    ros::spin();
    return 0;
}
