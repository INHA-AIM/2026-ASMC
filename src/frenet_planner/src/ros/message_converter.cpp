#include "ros/message_converter.hpp"

#include <algorithm>
#include <cmath>

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
    if (message.has_desired_speed) {
        converted.target_speed = message.desired_speed;
    }
    if (message.has_max_speed) {
        converted.target_speed = std::min(converted.target_speed,
                                          message.max_speed);
    }
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

}  // namespace frenet_planner_ros
