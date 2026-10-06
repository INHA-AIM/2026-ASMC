#pragma once

#include <asmc_msgs/Trajectory.h>
#include <asmc_msgs/BehaviorCommand.h>
#include <asmc_msgs/VehicleState.h>
#include <morai_msgs/CtrlCmd.h>
#include <ros/ros.h>

#include "MPC/MPC.hpp"

namespace mpc_controller_ros {

class Control {
 public:
    Control(const MPCParams& params, ros::Publisher control_pub,
            double trajectory_timeout, double vehicle_state_timeout);
    void trajectoryCallback(const asmc_msgs::Trajectory::ConstPtr& message);
    void vehicleStateCallback(const asmc_msgs::VehicleState::ConstPtr& message);
    void behaviorCommandCallback(
        const asmc_msgs::BehaviorCommand::ConstPtr& message);
    void ControlProcess(const ros::SteadyTimerEvent&);

 private:
    void publishStop(const char* reason);

    ros::Publisher control_pub_;
    asmc_msgs::Trajectory latest_trajectory_;
    bool has_trajectory_ = false;
    ros::SteadyTime trajectory_received_at_;
    double trajectory_timeout_ = 0.25;
    asmc_msgs::VehicleState latest_vehicle_state_;
    bool has_vehicle_state_ = false;
    ros::SteadyTime vehicle_state_received_at_;
    double vehicle_state_timeout_ = 0.20;
    double last_successful_steering_ = 0.0;
    ros::SteadyTime last_successful_control_at_;
    bool has_successful_control_ = false;
    MPC controller_;
    bool emergency_stop_active_ = false;
};

}  // namespace mpc_controller_ros
