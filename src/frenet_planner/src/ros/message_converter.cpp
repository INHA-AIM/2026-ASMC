#include "ros/message_converter.hpp"

#include <cmath>
#include <utility>

namespace frenet_planner_ros {
namespace {

bool IsMapFrame(const std::string& frame_id) {
    return frame_id == "map";
}

}  // namespace

const char* ConversionStatusName(ConversionStatus status) {
    switch (status) {
        case ConversionStatus::OK: return "OK";
        case ConversionStatus::INVALID_FLAG: return "INVALID_FLAG";
        case ConversionStatus::INVALID_FRAME: return "INVALID_FRAME";
        case ConversionStatus::INVALID_TIMESTAMP: return "INVALID_TIMESTAMP";
        case ConversionStatus::NON_FINITE_VALUE: return "NON_FINITE_VALUE";
        case ConversionStatus::UNSUPPORTED_BEHAVIOR:
            return "UNSUPPORTED_BEHAVIOR";
        case ConversionStatus::NON_EMPTY_OBJECTS_UNSUPPORTED:
            return "NON_EMPTY_OBJECTS_UNSUPPORTED";
        case ConversionStatus::INVALID_PATH: return "INVALID_PATH";
    }
    return "UNKNOWN_CONVERSION_STATUS";
}

ConversionStatus ConvertVehicleState(const asmc_msgs::VehicleState& message,
                                     CartesianState& state) {
    if (!message.valid) return ConversionStatus::INVALID_FLAG;
    if (!IsMapFrame(message.header.frame_id)) {
        return ConversionStatus::INVALID_FRAME;
    }
    if (message.header.stamp.isZero()) {
        return ConversionStatus::INVALID_TIMESTAMP;
    }
    if (!std::isfinite(message.x) || !std::isfinite(message.y) ||
        !std::isfinite(message.yaw) || !std::isfinite(message.speed) ||
        !std::isfinite(message.acceleration) ||
        !std::isfinite(message.yaw_rate)) {
        return ConversionStatus::NON_FINITE_VALUE;
    }

    CartesianState converted{};
    converted.x = message.x;
    converted.y = message.y;
    converted.yaw = message.yaw;
    converted.kappa = 0.0;
    converted.v = message.speed;
    converted.a = message.acceleration;
    state = converted;
    return ConversionStatus::OK;
}

ConversionStatus ConvertBehaviorContext(
    const asmc_msgs::BehaviorContext& message, PlannerCommand& command) {
    if (!message.valid) return ConversionStatus::INVALID_FLAG;
    if (message.header.stamp.isZero()) {
        return ConversionStatus::INVALID_TIMESTAMP;
    }
    if ((message.has_desired_speed && !std::isfinite(message.desired_speed)) ||
        (message.has_max_speed && !std::isfinite(message.max_speed))) {
        return ConversionStatus::NON_FINITE_VALUE;
    }

    PlannerCommand converted{};
    switch (message.selected_behavior) {
        case asmc_msgs::BehaviorContext::KEEP:
            converted.mode = LANE_KEEPING;
            break;
        default:
            return ConversionStatus::UNSUPPORTED_BEHAVIOR;
    }

    converted.target_lane = 0;
    converted.leader_id = -1;
    converted.has_target_speed = message.has_desired_speed;
    converted.target_speed = message.desired_speed;
    converted.has_max_speed = message.has_max_speed;
    converted.max_speed = message.max_speed;
    command = converted;
    return ConversionStatus::OK;
}

ConversionStatus ConvertDetectedObjects(
    const asmc_msgs::DetectedObjectArray& message,
    std::vector<ObjectInfo>& objects) {
    if (!IsMapFrame(message.header.frame_id)) {
        return ConversionStatus::INVALID_FRAME;
    }
    if (message.header.stamp.isZero()) {
        return ConversionStatus::INVALID_TIMESTAMP;
    }
    if (!message.objects.empty()) {
        return ConversionStatus::NON_EMPTY_OBJECTS_UNSUPPORTED;
    }

    objects.clear();
    return ConversionStatus::OK;
}

ConversionStatus ConvertCartesianPath(
    const CartesianPath& path, double dt,
    const asmc_msgs::VehicleState& ego_at_plan, const ros::Time& plan_stamp,
    asmc_msgs::Trajectory& trajectory) {
    const std::size_t count = path.x.size();
    if (count == 0 || path.y.size() != count || path.yaw.size() != count ||
        path.kappa.size() != count || path.v.size() != count ||
        path.a.size() != count || !std::isfinite(dt) || dt <= 0.0) {
        return ConversionStatus::INVALID_PATH;
    }
    asmc_msgs::Trajectory converted;
    converted.header.stamp = plan_stamp;
    converted.header.frame_id = "map";
    converted.valid = true;
    converted.failure_reason = asmc_msgs::Trajectory::FAILURE_NONE;
    converted.dt = dt;
    converted.ego_at_plan = ego_at_plan;
    converted.points.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        if (!std::isfinite(path.x[i]) || !std::isfinite(path.y[i]) ||
            !std::isfinite(path.yaw[i]) || !std::isfinite(path.kappa[i]) ||
            !std::isfinite(path.v[i]) || !std::isfinite(path.a[i])) {
            return ConversionStatus::NON_FINITE_VALUE;
        }
        asmc_msgs::TrajectoryPoint point;
        point.x = path.x[i];
        point.y = path.y[i];
        point.yaw = path.yaw[i];
        point.curvature = path.kappa[i];
        point.velocity = path.v[i];
        point.acceleration = path.a[i];
        converted.points.push_back(point);
    }
    trajectory = std::move(converted);
    return ConversionStatus::OK;
}

asmc_msgs::Trajectory MakeStopTrajectory(
    const asmc_msgs::VehicleState& ego_at_plan, double dt,
    uint8_t failure_reason, const ros::Time& plan_stamp) {
    asmc_msgs::Trajectory trajectory;
    trajectory.header.stamp = plan_stamp;
    trajectory.header.frame_id = "map";
    trajectory.valid = true;
    trajectory.failure_reason = failure_reason;
    trajectory.dt = dt;
    trajectory.ego_at_plan = ego_at_plan;
    asmc_msgs::TrajectoryPoint point;
    point.x = std::isfinite(ego_at_plan.x) ? ego_at_plan.x : 0.0;
    point.y = std::isfinite(ego_at_plan.y) ? ego_at_plan.y : 0.0;
    point.yaw = std::isfinite(ego_at_plan.yaw) ? ego_at_plan.yaw : 0.0;
    point.curvature = 0.0;
    point.velocity = 0.0;
    point.acceleration = 0.0;
    trajectory.points.push_back(point);
    return trajectory;
}

}  // namespace frenet_planner_ros
