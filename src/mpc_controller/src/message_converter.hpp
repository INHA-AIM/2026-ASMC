#pragma once

#include <asmc_msgs/Trajectory.h>
#include <asmc_msgs/VehicleState.h>
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

// /vehicle/state -> MPCState(x0) 변환 상태. Trajectory.ego_at_plan과 같은
// 계약(frame_id=="map", finite 필드)을 검사하지만 별도 입력이므로 별도 enum을
// 쓴다 - 두 입력의 실패를 섞어 보고하지 않는다.
enum class VehicleStateStatus {
    OK = 0,
    INVALID_MESSAGE,
    INVALID_FRAME,
    INVALID_TIMESTAMP,
    NON_FINITE_VALUE,
};

const char* TrajectoryStatusName(TrajectoryStatus status);
const char* VehicleStateStatusName(VehicleStateStatus status);
bool IsTrajectoryFresh(const ros::SteadyTime& received_at,
                       const ros::SteadyTime& now, double timeout_seconds);
TrajectoryStatus ConvertTrajectory(const asmc_msgs::Trajectory& message,
                                   PlannedPath& path, MPCState& ego_state);

// Trajectory.ego_at_plan은 provenance(계획 당시 스냅샷) 검증에만 쓰고, 실제
// ControlProcess의 x0는 최신 /vehicle/state에서 만든다 (fix/yuntae-planner
// Commit B: MPC state/trajectory latency alignment).
VehicleStateStatus ConvertVehicleState(const asmc_msgs::VehicleState& message,
                                       MPCState& ego_state);

// trajectory가 생성된 source 시각과 최신 vehicle state의 source 시각 사이의
// age. 둘 다 message header.stamp(수신시각이 아닌 source/reference time,
// interfaces/README.md 4절)이므로 같은 timebase다. 음수(vehicle state가 더
// 오래됨)는 0으로 clamp한다 - reference 시작을 과거로 되돌리지 않는다.
// freshness(SteadyTime watchdog)와는 책임이 분리된 별도 계산이다.
double ComputeTrajectoryAge(const ros::Time& trajectory_stamp,
                            const ros::Time& vehicle_stamp);

morai_msgs::CtrlCmd MakeRosStopCommand();
morai_msgs::CtrlCmd MakeRosEmergencyStopCommand();
morai_msgs::CtrlCmd ToRosControlCommand(const MPCControl& control,
                                        const MPCParams& params);

}  // namespace mpc_controller_ros
