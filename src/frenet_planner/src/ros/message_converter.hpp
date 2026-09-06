#pragma once

#include <cstdint>
#include <vector>

#include <asmc_msgs/BehaviorContext.h>
#include <asmc_msgs/DetectedObjectArray.h>
#include <asmc_msgs/Trajectory.h>
#include <asmc_msgs/VehicleState.h>

#include "frenet_planner/Math/FrenetConverter.hpp"
#include "frenet_planner/Common/ObjectTypes.hpp"
#include "frenet_planner/Common/PlannerCommand.hpp"

namespace frenet_planner_ros {

enum class ConversionStatus {
    OK = 0,
    INVALID_FLAG,
    INVALID_FRAME,
    INVALID_TIMESTAMP,
    NON_FINITE_VALUE,
    UNSUPPORTED_BEHAVIOR,
    DUPLICATE_OBJECT_ID,
    INVALID_OBJECT_GEOMETRY,
    INVALID_PATH,
};

const char* ConversionStatusName(ConversionStatus status);

ConversionStatus ConvertVehicleState(const asmc_msgs::VehicleState& message,
                                     CartesianState& state);
ConversionStatus ConvertBehaviorContext(
    const asmc_msgs::BehaviorContext& message, PlannerCommand& command);
ConversionStatus ConvertDetectedObjects(
    const asmc_msgs::DetectedObjectArray& message,
    std::vector<TrackedObject>& objects,
    std::vector<TrackedObject>& collision_objects);
ConversionStatus ConvertCartesianPath(
    const CartesianPath& path, double dt,
    const asmc_msgs::VehicleState& ego_at_plan,
    const ros::Time& plan_stamp,
    asmc_msgs::Trajectory& trajectory);
asmc_msgs::Trajectory MakeStopTrajectory(
    const asmc_msgs::VehicleState& ego_at_plan, double dt,
    uint8_t failure_reason, const ros::Time& plan_stamp);

}  // namespace frenet_planner_ros
