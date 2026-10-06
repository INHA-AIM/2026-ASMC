#pragma once

#include <asmc_msgs/Trajectory.h>
#include <asmc_msgs/VehicleState.h>
#include <morai_msgs/CtrlCmd.h>
#include <ros/time.h>

#include "Global/Global.hpp"
#include "MPC/Reference/path_resampler.hpp"

namespace mpc_controller_ros {

const char* TrajectoryStatusName(TrajectoryStatus status);
const char* VehicleStateStatusName(VehicleStateStatus status);

// receive-time(ros::SteadyTime) 기반 일반 staleness watchdog. trajectory와
// vehicle_state 모두 같은 정책을 쓴다 - 이름을 트래잭토리 전용으로 남기지
// 않는다(두 입력에 재사용되므로 일반화된 이름을 쓴다, Commit E).
bool IsMessageFresh(const ros::SteadyTime& received_at,
                    const ros::SteadyTime& now, double timeout_seconds);

// Trajectory.points -> PlannedPath만 변환한다. ego_at_plan의 provenance
// 계약(frame_id=="map", finite 필드, source timestamp 존재)은 여전히
// 검사하지만, 그 값을 MPCState로 변환해 반환하지 않는다 - ControlProcess의
// x0는 이 함수가 아니라 ConvertVehicleState(최신 /vehicle/state)가 만든다
// (Commit E: ego_at_plan -> MPCState coupling 제거. Commit B 참고 - x0가
// ego_at_plan에서 최신 vehicle state로 바뀐 뒤에도 이 변환 경로가 남아있던
// 것을 정리했다).
TrajectoryStatus ConvertTrajectory(const asmc_msgs::Trajectory& message,
                                   PlannedPath& path);

// /vehicle/state -> MPCState(x0) 변환. ControlProcess의 유일한 현재 상태
// source다.
VehicleStateStatus ConvertVehicleState(const asmc_msgs::VehicleState& message,
                                       MPCState& ego_state);

morai_msgs::CtrlCmd MakeRosStopCommand();
morai_msgs::CtrlCmd MakeRosEmergencyStopCommand();
morai_msgs::CtrlCmd ToRosControlCommand(const MPCControl& control,
                                        const MPCParams& params);

}  // namespace mpc_controller_ros
