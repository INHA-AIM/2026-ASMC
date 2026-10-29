/**
 * @file input_snapshot.cpp
 * @brief 입력 subscriber callback과 snapshot 조립: 저장만 한다, planning하지
 *        않는다.
 *
 * R8-A1: vehicleCallback/objectsCallback/behaviorCallback/takeSnapshot/
 * behaviorCommandV2Callback을 frenet_planner_node.cpp에서 그대로 옮겼다 -
 * 여전히 PlanningPort의 멤버 함수이고, 공유 mutex(input_mutex_)와 latest_*
 * 멤버도 PlanningPort가 그대로 소유한다(새 snapshot manager class를
 * 만들지 않는다). v2 명령 callback도 같은 이유로 여기 둔다 - validation/
 * transaction/planning은 20 Hz timer(runV2PlanningCycle)가 수행한다는 계약은
 * 그대로다.
 */

#include "Planning/PlanningPort.hpp"
#include "Planning/IntentInput.hpp"

namespace frenet_planner_ros {

void PlanningPort::vehicleCallback(
    const asmc_msgs::VehicleState::ConstPtr& message) {
    if (!message) return;
    std::lock_guard<std::mutex> lock(input_mutex_);
    latest_vehicle_ = *message;
    vehicle_received_at_ = ros::SteadyTime::now();
    has_vehicle_ = true;
}

void PlanningPort::objectsCallback(
    const planning_inputs::DetectedObjectArray::ConstPtr& message) {
    if (!message) return;
    std::lock_guard<std::mutex> lock(input_mutex_);
    latest_objects_ = *message;
    objects_received_at_ = ros::SteadyTime::now();
    has_objects_ = true;
}

void PlanningPort::behaviorCallback(
    const asmc_msgs::BehaviorContext::ConstPtr& message) {
    if (!message) return;
    std::lock_guard<std::mutex> lock(input_mutex_);
    latest_behavior_ = *message;
    behavior_received_at_ = ros::SteadyTime::now();
    has_behavior_ = true;
}

PlanningPort::InputSnapshot PlanningPort::takeSnapshot() const {
    InputSnapshot snapshot;
    {
        std::lock_guard<std::mutex> lock(input_mutex_);
        snapshot.vehicle = latest_vehicle_;
        snapshot.objects = latest_objects_;
        snapshot.behavior = latest_behavior_;
        snapshot.vehicle_received_at = vehicle_received_at_;
        snapshot.objects_received_at = objects_received_at_;
        snapshot.behavior_received_at = behavior_received_at_;
        snapshot.has_vehicle = has_vehicle_;
        snapshot.has_objects = has_objects_;
        snapshot.has_behavior = has_behavior_;
    }
    // The stored observation remains untouched. Match Behavior's per-cycle
    // policy before every Frenet consumer, including v1 and reference returns.
    if (static_mission_policy_ && snapshot.has_vehicle && snapshot.has_objects) {
      const auto& ego = snapshot.vehicle;
      static_mission_policy_->apply(ego.x, ego.y, ego.yaw, ego.valid, snapshot.objects);
    }
    return snapshot;
}

void PlanningPort::behaviorCommandV2Callback(
    const ros::MessageEvent<const asmc_msgs::BehaviorCommand>& event) {
    const auto message = event.getConstMessage();
    if (!message) return;
    if(control_binding_enabled_)return; // The explicit request channel owns this mode.
    // Internal intent is authoritative; do not consume its ROS mirror twice.
    // Commands from external peers still use this compatibility input adapter.
    if (feedback_sink_ && event.getPublisherName() == ros::this_node::getName()) return;
    // 저장만 한다. validation/transaction/planning은 20 Hz timer가 수행한다
    // (runV2PlanningCycle 주석 참고).
    const behavior_planner::BehaviorContextData external_intent{};
    receiveIntent({external_intent, *message});
}

void PlanningPort::controlRequestCallback(const ros::MessageEvent<const std_msgs::UInt8MultiArray>& event) {
    if(native_control_input_)return;
    asmc_msgs::binding::Request request;
    if(!event.getConstMessage() || !asmc_msgs::binding::Decode(*event.getConstMessage(),request) || !request.token.valid())return;
    const behavior_planner::BehaviorContextData external{};
    receiveIntent({external,request.command,&request});
}

void PlanningPort::receiveIntent(const asmc_planner::RequestedIntent& request) {
    coordinator_.ReceiveIntent(*this, request);
}

}  // namespace frenet_planner_ros
