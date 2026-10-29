#include "Planning/ProductionRuntime.hpp"
namespace asmc_planner {
ros::NodeHandle ProductionRuntime::handle(const std::string& ns, ros::CallbackQueue& queue) {
  ros::NodeHandle nh(ns);
  nh.setCallbackQueue(&queue);
  return nh;
}
ProductionRuntime::ProductionRuntime()
    : planning_spinner_(1, &planning_queue_), behavior_spinner_(1, &behavior_queue_) {
  std::shared_ptr<const StaticMissionPolicy> static_policy;
  ros::NodeHandle policy_params("~static_obstacle_policy");
  bool enabled = false;
  policy_params.param("enabled", enabled, false);
  if (enabled) {
    std::string metadata, data_root;
    double lateral = 10.0, heading = 1.0;
    policy_params.param("metadata_file", metadata, std::string());
    policy_params.param("data_root", data_root, std::string("/data"));
    policy_params.param("lateral_tolerance_m", lateral, lateral);
    policy_params.param("heading_tolerance_rad", heading, heading);
    // Invalid/mismatched assets must not silently broaden or disable the scope.
    static_policy = std::make_shared<const StaticMissionPolicy>(metadata, data_root, lateral, heading);
    ROS_INFO("Static mission stationary planning prior: official route %.3f-%.3f m (%s)",
             static_policy->start(), static_policy->end(), metadata.c_str());
  }
  planning_.reset(new frenet_planner_ros::PlanningPort(
      handle("", planning_queue_), handle("/frenet_planner", planning_queue_),
      [this](const asmc_msgs::FrenetFeedback& feedback) { behavior_->receiveFeedback(feedback); }));
  planning_->useNativeControlInput();
  behavior_.reset(new behavior_planner::BehaviorInput(
      handle("", behavior_queue_), handle("/behavior_planner", behavior_queue_),
      [this](const RequestedIntent& request) { planning_->receiveIntent(request); },
      planning_->dynamicEnvironment()));
  behavior_->setHighwayRoute(planning_->highwayRoute());
  planning_->setStaticMissionPolicy(static_policy);
  behavior_->setStaticMissionPolicy(static_policy);
  behavior_->setHighwayReferenceSource([this]{return planning_->highwayReferenceStatus();});
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
