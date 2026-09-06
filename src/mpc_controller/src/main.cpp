#include <ros/ros.h>

#include "mpc_controller_node.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "mpc_controller");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    mpc_controller_ros::MpcControllerNode node(nh, private_nh);
    ros::spin();
    return 0;
}
