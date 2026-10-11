/**
 * @file output_publisher.cpp
 * @brief v1 trajectory/feedback 발행: publishResult/publishStop과 그 둘이
 *        의존하는 stopReferenceVehicle/trajectoryDt 유틸리티.
 *
 * R8-C1: frenet_planner_node.cpp에서 그대로 옮겼다 - 여전히 PlanningPort
 * 멤버 함수이고, trajectory_pub_/feedback_pub_ 등 ROS publisher 멤버도
 * PlanningPort가 그대로 소유한다. publishDebugMarkers 호출은 그대로 남아
 * 있다(정의는 debug_marker_publisher.cpp로 옮겼다) - 새 Publisher class를
 * 만들지 않는다.
 */

#include "Planning/PlanningPort.hpp"

#include <cmath>

#include "Parser/PlanningInput.hpp"

namespace frenet_planner_ros {

double PlanningPort::trajectoryDt() const {
    const double dt = planner_.sample_dt();
    return planner_initialized_ && std::isfinite(dt) && dt > 0.0 ? dt : 0.1;
}

asmc_msgs::VehicleState PlanningPort::stopReferenceVehicle(
    const InputSnapshot& snapshot) const {
    if (snapshot.has_vehicle && std::isfinite(snapshot.vehicle.x) &&
        std::isfinite(snapshot.vehicle.y) && std::isfinite(snapshot.vehicle.yaw)) {
        return snapshot.vehicle;
    }
    if (has_stop_reference_vehicle_) return last_stop_reference_vehicle_;

    asmc_msgs::VehicleState unknown;
    unknown.header.frame_id = "map";
    unknown.valid = false;
    return unknown;
}

void PlanningPort::publishResult(
    const asmc_msgs::Trajectory& trajectory, const InputSnapshot& snapshot,
    bool previous_plan_still_valid) {
    publishPlanningResult({trajectory, previous_plan_still_valid}, snapshot);
}

void PlanningPort::publishPlanningResult(
    const asmc_planner::PlanningResult& result, const InputSnapshot& snapshot) {
    const auto& trajectory = result.trajectory;
    const bool previous_plan_still_valid = result.previous_plan_still_valid;
    trajectory_pub_.publish(trajectory);
    publishDebugMarkers(trajectory, snapshot);

    asmc_msgs::PlanFeedback feedback;
    feedback.header.stamp = ros::Time::now();
    feedback.header.frame_id = "map";
    feedback.command_id = snapshot.has_behavior ? snapshot.behavior.command_id : 0;
    feedback.plan_valid = trajectory.valid;
    feedback.failure_reason = trajectory.failure_reason;
    feedback.selected_behavior = snapshot.has_behavior
        ? snapshot.behavior.selected_behavior
        : asmc_msgs::BehaviorContext::KEEP;
    feedback.selected_target_link_id = snapshot.has_behavior
        ? snapshot.behavior.target_link_id : "";
    feedback.previous_plan_still_valid = previous_plan_still_valid;
    feedback_pub_.publish(feedback);
}

void PlanningPort::publishStop(const InputSnapshot& snapshot,
                                    uint8_t failure_reason) {
    has_last_valid_trajectory_ = false;
    plan_fail_streak_ = 0;
    const asmc_msgs::Trajectory stop = MakeStopTrajectory(
        stopReferenceVehicle(snapshot), trajectoryDt(), failure_reason);
    publishResult(stop, snapshot, false);
}

}  // namespace frenet_planner_ros
