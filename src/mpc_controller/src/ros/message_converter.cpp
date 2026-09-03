#include "ros/message_converter.hpp"

#include <cmath>
#include <utility>

#include "global/utils.hpp"

namespace mpc_controller_ros {
namespace {

bool IsFinite(double value) {
    return std::isfinite(value);
}

// 정상적인 전송/스케줄링 지터로 stamp가 조금 미래인 경우의 관용치 [s].
constexpr double kFutureToleranceSeconds = 0.05;

// 시계가 뒤로 점프했다고 볼 수 있는 미래 stamp의 한계 [s]. 이보다 더 미래인
// stamp는 시계 점프로 설명되지 않는 잘못된 입력으로 보고 거부한다.
// 실측 기준: WSL2/Docker 환경에서 약 28초 주기로 wall clock이 -1.2초 점프.
constexpr double kMaxClockJumpSeconds = 5.0;

}  // namespace

const char* TrajectoryStatusName(TrajectoryStatus status) {
    switch (status) {
        case TrajectoryStatus::OK: return "OK";
        case TrajectoryStatus::STOP_TRAJECTORY: return "STOP_TRAJECTORY";
        case TrajectoryStatus::INVALID_MESSAGE: return "INVALID_MESSAGE";
        case TrajectoryStatus::INVALID_FRAME: return "INVALID_FRAME";
        case TrajectoryStatus::INVALID_TIMESTAMP: return "INVALID_TIMESTAMP";
        case TrajectoryStatus::INVALID_DT: return "INVALID_DT";
        case TrajectoryStatus::INVALID_EGO: return "INVALID_EGO";
        case TrajectoryStatus::INVALID_POINT: return "INVALID_POINT";
    }
    return "UNKNOWN";
}

bool IsTrajectoryFresh(const asmc_msgs::Trajectory& message,
                       const ros::Time& now, double timeout_seconds) {
    if (message.header.stamp.isZero() || !IsFinite(timeout_seconds) ||
        timeout_seconds <= 0.0) {
        return false;
    }
    const double age = (now - message.header.stamp).toSec();
    if (!IsFinite(age)) return false;

    // 시계가 뒤로 점프하면 age가 크게 음수가 된다. 이건 궤적이 오래된 것이
    // 아니라 시각 기준 자체가 흔들린 것이므로 stale로 처리하면 안 된다.
    // stale로 보면 멀쩡한 궤적에 대해 매 점프마다 급정지하게 된다
    // (실측: 시계 점프 9회에 MPC fail-safe STOP 20회).
    if (age < -kFutureToleranceSeconds) {
        return age >= -kMaxClockJumpSeconds;
    }
    return age <= timeout_seconds;
}

TrajectoryStatus ConvertTrajectory(const asmc_msgs::Trajectory& message,
                                   PlannedPath& path, MPCState& ego_state) {
    if (!message.valid || message.points.empty()) {
        return TrajectoryStatus::INVALID_MESSAGE;
    }
    if (message.header.frame_id != "map") {
        return TrajectoryStatus::INVALID_FRAME;
    }
    if (message.header.stamp.isZero()) {
        return TrajectoryStatus::INVALID_TIMESTAMP;
    }
    if (!IsFinite(message.dt) || message.dt <= 0.0) {
        return TrajectoryStatus::INVALID_DT;
    }
    if (!message.ego_at_plan.valid ||
        message.ego_at_plan.header.frame_id != "map" ||
        !IsFinite(message.ego_at_plan.x) ||
        !IsFinite(message.ego_at_plan.y) ||
        !IsFinite(message.ego_at_plan.yaw) ||
        !IsFinite(message.ego_at_plan.speed)) {
        return TrajectoryStatus::INVALID_EGO;
    }

    PlannedPath converted;
    converted.dt = message.dt;
    converted.ego_x = message.ego_at_plan.x;
    converted.ego_y = message.ego_at_plan.y;
    converted.ego_yaw = message.ego_at_plan.yaw;
    converted.ego_v = message.ego_at_plan.speed;
    converted.ego_accel = message.ego_at_plan.acceleration;
    converted.x.reserve(message.points.size());
    converted.y.reserve(message.points.size());
    converted.yaw.reserve(message.points.size());
    converted.kappa.reserve(message.points.size());
    converted.v.reserve(message.points.size());
    converted.a.reserve(message.points.size());

    for (const auto& point : message.points) {
        if (!IsFinite(point.x) || !IsFinite(point.y) ||
            !IsFinite(point.yaw) || !IsFinite(point.curvature) ||
            !IsFinite(point.velocity) || point.velocity < 0.0 ||
            !IsFinite(point.acceleration)) {
            return TrajectoryStatus::INVALID_POINT;
        }
        converted.x.push_back(point.x);
        converted.y.push_back(point.y);
        converted.yaw.push_back(point.yaw);
        converted.kappa.push_back(point.curvature);
        converted.v.push_back(point.velocity);
        converted.a.push_back(point.acceleration);
    }

    MPCState converted_ego;
    converted_ego.x = converted.ego_x;
    converted_ego.y = converted.ego_y;
    converted_ego.yaw = converted.ego_yaw;
    converted_ego.vx = converted.ego_v;

    const bool stop_trajectory =
        converted.size() == 1 && converted.v.front() <= 1e-6;
    path = std::move(converted);
    ego_state = converted_ego;
    return stop_trajectory ? TrajectoryStatus::STOP_TRAJECTORY
                           : TrajectoryStatus::OK;
}

morai_msgs::CtrlCmd MakeRosStopCommand() {
    morai_msgs::CtrlCmd message;
    message.longlCmdType = 1;
    message.accel = 0.0;
    message.brake = 0.3;
    message.steering = 0.0;
    return message;
}

morai_msgs::CtrlCmd ToRosControlCommand(const MPCControl& control,
                                        const MPCParams& params) {
    morai_msgs::CtrlCmd message;
    message.longlCmdType = 1;
    if (control.accel >= 0.0) {
        message.accel = clip(control.accel / params.accel_max, 0.0, 1.0);
        message.brake = 0.0;
    } else {
        message.accel = 0.0;
        message.brake =
            clip(-control.accel / std::fabs(params.accel_min), 0.0, 1.0);
    }
    // morai_msgs/CtrlCmd.steering is radians. The MORAI bridge owns the
    // transport-specific conversion from radians to normalized UDP steering.
    message.steering =
        clip(control.delta, -params.steering_max, params.steering_max);
    return message;
}

}  // namespace mpc_controller_ros
