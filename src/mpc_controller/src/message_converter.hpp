#pragma once

#include <asmc_msgs/Trajectory.h>
#include <morai_msgs/CtrlCmd.h>
#include <ros/time.h>

#include "common/mpc_control.hpp"
#include "common/mpc_params.hpp"
#include "common/mpc_state.hpp"
#include "reference/path_resampler.hpp"

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
bool IsTrajectoryFresh(const ros::SteadyTime& received_at,
                       const ros::SteadyTime& now, double timeout_seconds);
TrajectoryStatus ConvertTrajectory(const asmc_msgs::Trajectory& message,
                                   PlannedPath& path, MPCState& ego_state);
morai_msgs::CtrlCmd MakeRosStopCommand();
morai_msgs::CtrlCmd ToRosControlCommand(const MPCControl& control,
                                        const MPCParams& params);

}  // namespace mpc_controller_ros
