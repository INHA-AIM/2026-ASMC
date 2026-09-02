#pragma once

#include <asmc_msgs/Trajectory.h>
#include <morai_msgs/CtrlCmd.h>
#include <ros/time.h>

#include "global/global.hpp"
#include "udp_network/datatypes.hpp"

namespace mpc_controller_ros {

enum class TrajectoryStatus {
    OK = 0,
    STOP_TRAJECTORY,
    INVALID_MESSAGE,
    INVALID_FRAME,
    INVALID_TIMESTAMP,
    INVALID_DT,
    INVALID_EGO,
    INVALID_POINT,
};

const char* TrajectoryStatusName(TrajectoryStatus status);
bool IsTrajectoryFresh(const asmc_msgs::Trajectory& message,
                       const ros::Time& now, double timeout_seconds);
TrajectoryStatus ConvertTrajectory(const asmc_msgs::Trajectory& message,
                                   PlannedPath& path, MPCState& ego_state);
morai_msgs::CtrlCmd ToRosControlCommand(const CtrlCmd& command);

}  // namespace mpc_controller_ros
