#pragma once

#include <asmc_msgs/Trajectory.h>
#include <morai_msgs/CtrlCmd.h>
#include <ros/ros.h>

#include "control/mpc_controller.hpp"

namespace mpc_controller_ros {

class MpcControllerNode {
 public:
    MpcControllerNode(ros::NodeHandle nh, ros::NodeHandle private_nh);

 private:
    void trajectoryCallback(const asmc_msgs::Trajectory::ConstPtr& message);
    void controlTimerCallback(const ros::SteadyTimerEvent&);
    void publishStop(const char* reason);

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber trajectory_sub_;
    ros::Publisher control_pub_;
    ros::SteadyTimer control_timer_;

    asmc_msgs::Trajectory latest_trajectory_;
    bool has_trajectory_ = false;
    ros::SteadyTime trajectory_received_at_;
    double trajectory_timeout_ = 0.25;

    MpcController controller_;
};

}  // namespace mpc_controller_ros
