#include "Planning/ProductionRuntime.hpp"
namespace asmc_planner {
ros::NodeHandle ProductionRuntime::handle(const std::string& ns, ros::CallbackQueue& queue) {
  ros::NodeHandle nh(ns);
  nh.setCallbackQueue(&queue);
  return nh;
}
ProductionRuntime::ProductionRuntime()
    : planning_spinner_(1, &planning_queue_), behavior_spinner_(1, &behavior_queue_) {
  planning_.reset(new frenet_planner_ros::PlanningPort(
      handle("", planning_queue_), handle("/frenet_planner", planning_queue_),
      [this](const asmc_msgs::FrenetFeedback& feedback) { behavior_->receiveFeedback(feedback); }));
  planning_->useNativeControlInput();
  behavior_.reset(new behavior_planner::BehaviorInput(
      handle("", behavior_queue_), handle("/behavior_planner", behavior_queue_),
      [this](const RequestedIntent& request) { planning_->receiveIntent(request); },
      planning_->dynamicEnvironment()));
  mode_source_.reset(new GpsModeSource(handle("", planning_queue_)));
}
ProductionRuntime::~ProductionRuntime() {
  behavior_spinner_.stop();
  planning_spinner_.stop();
}
void ProductionRuntime::start() {
  planning_spinner_.start();
  behavior_spinner_.start();
}
}
