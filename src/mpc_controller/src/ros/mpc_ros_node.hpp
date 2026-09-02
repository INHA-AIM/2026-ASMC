#pragma once

#include <asmc_msgs/Trajectory.h>
#include <morai_msgs/CtrlCmd.h>
#include <ros/ros.h>

#include <vector>

#include "global/global.hpp"

namespace mpc_controller_ros {

class MpcRosNode {
 public:
    MpcRosNode(ros::NodeHandle nh, ros::NodeHandle private_nh);

 private:
    void trajectoryCallback(const asmc_msgs::Trajectory::ConstPtr& message);
    void controlTimerCallback(const ros::TimerEvent&);
    void publishStop(const char* reason);

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber trajectory_sub_;
    ros::Publisher control_pub_;
    ros::Timer control_timer_;

    asmc_msgs::Trajectory latest_trajectory_;
    bool has_trajectory_ = false;
    double trajectory_timeout_ = 0.25;
    std::vector<MPCControl> warm_start_;
};

}  // namespace mpc_controller_ros
