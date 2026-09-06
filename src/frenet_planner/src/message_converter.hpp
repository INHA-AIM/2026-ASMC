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
// obb_corners[8](perception이 제공하는 authoritative geometry)이 유효한
// 8-corner OBB로 검증되지 않는 object는 INVALID_OBJECT_GEOMETRY로 거부한다
// - Planner는 center/width/length/yaw로 geometry를 재생성하지 않는다.
ConversionStatus ConvertDetectedObjects(
    const asmc_msgs::DetectedObjectArray& message,
    std::vector<TrackedObject>& objects);
ConversionStatus ConvertCartesianPath(
    const CartesianPath& path, double dt,
    const asmc_msgs::VehicleState& ego_at_plan,
    const ros::Time& plan_stamp,
    asmc_msgs::Trajectory& trajectory);
asmc_msgs::Trajectory MakeStopTrajectory(
    const asmc_msgs::VehicleState& ego_at_plan, double dt,
    uint8_t failure_reason, const ros::Time& plan_stamp);

}  // namespace frenet_planner_ros
