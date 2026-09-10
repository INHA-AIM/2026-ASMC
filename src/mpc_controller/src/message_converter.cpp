#include "message_converter.hpp"

#include <cmath>
#include <utility>

#include "math/utils.hpp"

namespace mpc_controller_ros {
namespace {

bool IsFinite(double value) {
    return std::isfinite(value);
}

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

const char* VehicleStateStatusName(VehicleStateStatus status) {
    switch (status) {
        case VehicleStateStatus::OK: return "OK";
        case VehicleStateStatus::INVALID_MESSAGE: return "INVALID_MESSAGE";
        case VehicleStateStatus::INVALID_FRAME: return "INVALID_FRAME";
        case VehicleStateStatus::INVALID_TIMESTAMP: return "INVALID_TIMESTAMP";
        case VehicleStateStatus::NON_FINITE_VALUE: return "NON_FINITE_VALUE";
    }
    return "UNKNOWN";
}

bool IsMessageFresh(const ros::SteadyTime& received_at,
                    const ros::SteadyTime& now, double timeout_seconds) {
    if (received_at.isZero() || !IsFinite(timeout_seconds) ||
        timeout_seconds <= 0.0) return false;
    const double age = (now - received_at).toSec();
    return IsFinite(age) && age >= 0.0 && age <= timeout_seconds;
}

TrajectoryStatus ConvertTrajectory(const asmc_msgs::Trajectory& message,
                                   PlannedPath& path) {
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
    // ego_at_plan은 provenance 계약이다 - trajectory가 어떤 Ego snapshot에서
    // 계획됐는지 검증한다. 이 값 자체를 MPCState로 변환해 반환하지는 않는다
    // (ControlProcess의 x0는 ConvertVehicleState/최신 /vehicle/state 전용).
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

    const bool stop_trajectory =
        converted.size() == 1 && converted.v.front() <= 1e-6;
    path = std::move(converted);
    return stop_trajectory ? TrajectoryStatus::STOP_TRAJECTORY
                           : TrajectoryStatus::OK;
}

VehicleStateStatus ConvertVehicleState(const asmc_msgs::VehicleState& message,
                                       MPCState& ego_state) {
    if (!message.valid) return VehicleStateStatus::INVALID_MESSAGE;
    if (message.header.frame_id != "map") {
        return VehicleStateStatus::INVALID_FRAME;
    }
    if (message.header.stamp.isZero()) {
        return VehicleStateStatus::INVALID_TIMESTAMP;
    }
    if (!IsFinite(message.x) || !IsFinite(message.y) ||
        !IsFinite(message.yaw) || !IsFinite(message.speed)) {
        return VehicleStateStatus::NON_FINITE_VALUE;
    }

    MPCState converted;
    converted.x = message.x;
    converted.y = message.y;
    converted.yaw = message.yaw;
    converted.vx = message.speed;
    ego_state = converted;
    return VehicleStateStatus::OK;
}

morai_msgs::CtrlCmd MakeRosStopCommand() {
    morai_msgs::CtrlCmd message;
    message.longlCmdType = 1;
    message.accel = 0.0;
    message.brake = 0.3;
    message.steering = 0.0;
    return message;
}

morai_msgs::CtrlCmd MakeRosEmergencyStopCommand() {
    morai_msgs::CtrlCmd message;
    message.longlCmdType = 1;
    message.accel = 0.0;
    message.brake = 1.0;
    message.steering = 0.0;
    return message;
}

namespace {

double InversePedalMap(double requested_response,
                       const std::vector<double>& pedal,
                       const std::vector<double>& response) {
    if (requested_response <= 0.0) return 0.0;
    for (size_t i = 1; i < response.size(); ++i) {
        if (requested_response <= response[i]) {
            const double ratio =
                (requested_response - response[i - 1]) /
                (response[i] - response[i - 1]);
            return clip(pedal[i - 1] + ratio * (pedal[i] - pedal[i - 1]),
                        0.0, 1.0);
        }
    }
    // 마지막 측정점 밖은 마지막 구간 기울기로 제한적으로 외삽한다.
    const size_t last = response.size() - 1;
    const double slope = (pedal[last] - pedal[last - 1]) /
                         (response[last] - response[last - 1]);
    return clip(pedal[last] +
                    (requested_response - response[last]) * slope,
                0.0, 1.0);
}

}  // namespace

morai_msgs::CtrlCmd ToRosControlCommand(const MPCControl& control,
                                        const MPCParams& params) {
    morai_msgs::CtrlCmd message;
    message.longlCmdType = 1;
    if (control.accel >= 0.0) {
        message.accel = params.use_calibrated_pedal_mapping
            ? InversePedalMap(control.accel, params.accel_pedal_lut,
                              params.accel_response_lut)
            : clip(control.accel / params.accel_max, 0.0, 1.0);
        message.brake = 0.0;
    } else {
        message.accel = 0.0;
        message.brake = params.use_calibrated_pedal_mapping
            ? InversePedalMap(-control.accel, params.brake_pedal_lut,
                              params.brake_response_lut)
            : clip(-control.accel / params.brake_full_decel, 0.0, 1.0);
    }
    // morai_msgs/CtrlCmd.steering is radians. The MORAI bridge owns the
    // transport-specific conversion from radians to normalized UDP steering.
    message.steering =
        clip(control.delta, -params.steering_max, params.steering_max);
    return message;
}

}  // namespace mpc_controller_ros
