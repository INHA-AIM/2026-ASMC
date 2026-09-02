#pragma once

#include <vector>

#include <asmc_msgs/BehaviorContext.h>
#include <asmc_msgs/DetectedObjectArray.h>
#include <asmc_msgs/VehicleState.h>

#include "frenet_planner/global/global.hpp"
#include "frenet_planner/math/frenet_converter.hpp"

namespace frenet_planner_ros {

enum class ConversionStatus {
    OK = 0,
    INVALID_FLAG,
    INVALID_FRAME,
    INVALID_TIMESTAMP,
    NON_FINITE_VALUE,
    UNSUPPORTED_BEHAVIOR,
    NON_EMPTY_OBJECTS_UNSUPPORTED,
};

const char* ConversionStatusName(ConversionStatus status);

ConversionStatus ConvertVehicleState(const asmc_msgs::VehicleState& message,
                                     CartesianState& state);
ConversionStatus ConvertBehaviorContext(
    const asmc_msgs::BehaviorContext& message, PlannerCommand& command);
ConversionStatus ConvertDetectedObjects(
    const asmc_msgs::DetectedObjectArray& message,
    std::vector<ObjectInfo>& objects);

}  // namespace frenet_planner_ros
