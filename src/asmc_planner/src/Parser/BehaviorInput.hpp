#pragma once

#include <cstdint>
#include <memory>
#include <functional>
#include <mutex>
#include "Execution/RequestedIntent.hpp"
#include <asmc_msgs/BehaviorContext.h>
#include <asmc_msgs/BehaviorCommand.h>
#include <planning_inputs/DetectedObjectArray.h>
#include <asmc_msgs/FrenetFeedback.h>
#include <asmc_msgs/PlanFeedback.h>
#include <asmc_msgs/VehicleState.h>
#include <morai_msgs/ObjectStatusList.h>
#include <ros/ros.h>

#include "Behavior/Decision/behavior_decision.hpp"
#include "Execution/IntentEncoder.hpp"
#include "Behavior/Feature/feature_builder.hpp"
#include "Output/Behavior/data_logger.hpp"
#include "Behavior/Map/mgeo_loader.hpp"
#include "Parser/Traffic/mock_traffic_light_provider.hpp"
#include "Parser/Traffic/ros_traffic_light_provider.hpp"
#include "Visualization/Behavior/behavior_visualization.hpp"

namespace behavior_planner {

class BehaviorInput {
 public:
  using IntentSink = std::function<void(const asmc_planner::RequestedIntent&)>;
  BehaviorInput(ros::NodeHandle, ros::NodeHandle);
  BehaviorInput(ros::NodeHandle, ros::NodeHandle, IntentSink);
  void receiveFeedback(const asmc_msgs::FrenetFeedback&);
  void spin();

 private:
  void runOnce();
  void timerCallback(const ros::SteadyTimerEvent&);
  void egoCallback(const asmc_msgs::VehicleState::ConstPtr& message);
  void moraiObjectsCallback(
      const morai_msgs::ObjectStatusList::ConstPtr& message);
  void detectedObjectsCallback(
      const planning_inputs::DetectedObjectArray::ConstPtr& message);
  void feedbackCallback(const asmc_msgs::PlanFeedback::ConstPtr& message);
  void feedbackV2Callback(const asmc_msgs::FrenetFeedback::ConstPtr& message);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber ego_sub_;
  ros::Subscriber objects_sub_;
  ros::Subscriber feedback_sub_;
  ros::Publisher context_pub_;
  ros::Publisher command_v2_pub_;
  ros::Publisher debug_pub_;
  ros::Publisher marker_pub_;
  ros::SteadyTimer update_timer_;
  IntentSink intent_sink_;
  std::mutex feedback_mutex_;
  MGeoLoader map_;
  RouteManager route_;
  FeatureBuilder builder_;
  BehaviorDecision decision_;
  std::shared_ptr<TrafficLightProvider> lights_;
  BehaviorVisualization visualization_;
  DataLogger logger_;
  EgoState ego_;
  PerceptionObjects objects_;
  asmc_msgs::PlanFeedback feedback_;
  asmc_msgs::FrenetFeedback feedback_v2_;
  ros::SteadyTime feedback_v2_received_at_;
  ros::SteadyTime objects_received_at_;
  V2CommandManager v2_command_manager_;
  asmc_msgs::BehaviorContext last_command_;
  uint32_t current_command_id_ = 0;
  bool has_last_command_ = false;
  bool ego_received_ = false;
  bool use_v2_interface_ = false;
  bool feedback_v2_received_ = false;
  bool objects_received_ = false;
  double rate_ = 10.0;
  double command_validity_seconds_ = 0.5;
  double feedback_v2_stale_seconds_ = 1.0;
  double objects_stale_seconds_ = 0.3;
};

}  // namespace behavior_planner
