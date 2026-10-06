#pragma once

#include "Global/Global.hpp"

namespace mpc_controller_ros {

void TrajectoryCallback(const asmc_msgs::Trajectory::ConstPtr& message);
void VehicleStateCallback(const asmc_msgs::VehicleState::ConstPtr& message);
void BehaviorCommandCallback(
    const asmc_msgs::BehaviorCommand::ConstPtr& message);
void PublishStop(const char* reason);
void ControlProcess(const ros::SteadyTimerEvent&);

}  // namespace mpc_controller_ros
