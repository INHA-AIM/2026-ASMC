/**
 * @file input_snapshot.cpp
 * @brief 입력 subscriber callback과 snapshot 조립: 저장만 한다, planning하지
 *        않는다.
 *
 * R8-A1: vehicleCallback/objectsCallback/behaviorCallback/takeSnapshot/
 * behaviorCommandV2Callback을 frenet_planner_node.cpp에서 그대로 옮겼다 -
 * 여전히 FrenetPlannerNode의 멤버 함수이고, 공유 mutex(input_mutex_)와 latest_*
 * 멤버도 FrenetPlannerNode가 그대로 소유한다(새 snapshot manager class를
 * 만들지 않는다). v2 명령 callback도 같은 이유로 여기 둔다 - validation/
 * transaction/planning은 20 Hz timer(runV2PlanningCycle)가 수행한다는 계약은
 * 그대로다.
 */

#include "frenet_planner_node.hpp"

namespace frenet_planner_ros {

void FrenetPlannerNode::vehicleCallback(
    const asmc_msgs::VehicleState::ConstPtr& message) {
    if (!message) return;
    std::lock_guard<std::mutex> lock(input_mutex_);
    latest_vehicle_ = *message;
    vehicle_received_at_ = ros::SteadyTime::now();
    has_vehicle_ = true;
}

void FrenetPlannerNode::objectsCallback(
    const planning_inputs::DetectedObjectArray::ConstPtr& message) {
    if (!message) return;
    std::lock_guard<std::mutex> lock(input_mutex_);
    latest_objects_ = *message;
    objects_received_at_ = ros::SteadyTime::now();
    has_objects_ = true;
}

void FrenetPlannerNode::behaviorCallback(
    const asmc_msgs::BehaviorContext::ConstPtr& message) {
    if (!message) return;
    std::lock_guard<std::mutex> lock(input_mutex_);
    latest_behavior_ = *message;
    behavior_received_at_ = ros::SteadyTime::now();
    has_behavior_ = true;
}

FrenetPlannerNode::InputSnapshot FrenetPlannerNode::takeSnapshot() const {
    std::lock_guard<std::mutex> lock(input_mutex_);
    InputSnapshot snapshot;
    snapshot.vehicle = latest_vehicle_;
    snapshot.objects = latest_objects_;
    snapshot.behavior = latest_behavior_;
    snapshot.vehicle_received_at = vehicle_received_at_;
    snapshot.objects_received_at = objects_received_at_;
    snapshot.behavior_received_at = behavior_received_at_;
    snapshot.has_vehicle = has_vehicle_;
    snapshot.has_objects = has_objects_;
    snapshot.has_behavior = has_behavior_;
    return snapshot;
}

void FrenetPlannerNode::behaviorCommandV2Callback(
    const asmc_msgs::BehaviorCommand::ConstPtr& message) {
    if (!message) return;
    // 저장만 한다. validation/transaction/planning은 20 Hz timer가 수행한다
    // (runV2PlanningCycle 주석 참고).
    std::lock_guard<std::mutex> lock(input_mutex_);
    latest_v2_command_ = *message;
    has_v2_command_ = true;
}

}  // namespace frenet_planner_ros
