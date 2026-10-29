#pragma once
// Conditional owner permit; wire identity is checked separately by Controller.
#include "Signal/ReleaseCommit.hpp"
#include "Signal/ExecutionRecovery.hpp"

#include <cstdint>
#include <memory>
#include <functional>
#include <mutex>
#include <stdexcept>
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
#include "Behavior/Feature/DynamicObjectPolicy.hpp"
#include "Parser/StaticMissionPolicy.hpp"
#include "Execution/IntentEncoder.hpp"
#include "Behavior/Feature/feature_builder.hpp"
#include "Output/Behavior/data_logger.hpp"
#include "Behavior/Map/mgeo_loader.hpp"
#include "Parser/Traffic/mock_traffic_light_provider.hpp"
#include "Parser/Traffic/ros_traffic_light_provider.hpp"
#include "Visualization/Behavior/behavior_visualization.hpp"
#include "Signal/SignalShadow.hpp"

namespace behavior_planner {

class BehaviorInput {
 public:
  using IntentSink = std::function<void(const asmc_planner::RequestedIntent&)>;
  BehaviorInput(ros::NodeHandle, ros::NodeHandle);
  BehaviorInput(ros::NodeHandle, ros::NodeHandle, IntentSink);
  BehaviorInput(ros::NodeHandle, ros::NodeHandle, IntentSink, DynamicEnvironment);
  void receiveFeedback(const asmc_msgs::FrenetFeedback&);
  void spin();
  void setStaticMissionPolicy(std::shared_ptr<const asmc_planner::StaticMissionPolicy> policy) {
    static_mission_policy_ = std::move(policy);
  }
  void setHighwayReferenceSource(std::function<asmc_planner::lane_reference::HighwayReferenceStatus()> source) {
    highway_reference_source_=std::move(source);
  }
  void setHighwayRoute(std::shared_ptr<const HighwayMergeRoute> route) {
    if(route&&(!map_loaded_||!route_loaded_||!use_v2_interface_))
      throw std::runtime_error("highway requires a valid Behavior map, route and v2 interface");
    if(route) for(const auto& zone:route->zones) {
      if(!map_.links().count(zone.config.source_link))
        throw std::runtime_error("highway source missing from Behavior map: "+zone.config.source_link);
      for(const auto& id:zone.config.target_lanes) {
        const auto* lane=route->map->lane(id);
        if(!lane||!map_.links().count(lane->link_id))
          throw std::runtime_error("highway target missing from Behavior map: "+id);
      }
    }
    builder_.setHighwayRoute(std::move(route));
  }

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
  bool control_binding_enabled_=false;
  ros::Publisher control_request_pub_;
  ros::Subscriber controller_epoch_sub_;
  std::string controller_epoch_,behavior_epoch_=asmc_msgs::binding::NewEpoch();
  std::uint64_t control_request_revision_=0;
  asmc_msgs::binding::Request last_control_request_;
  ros::Subscriber control_status_sub_;
  asmc_msgs::binding::Status control_status_;
  ros::SteadyTime control_status_received_at_;
  std::uint64_t control_status_sequence_=0;
  std::uint64_t brake_generation_=0;
  unsigned brake_level_=0;
  asmc_msgs::binding::Request brake_baseline_;
  asmc_planner::signal::ReleaseCommit release_commit_;
  asmc_planner::signal::ExecutionRecovery execution_recovery_;
  ros::SteadyTimer update_timer_;
  IntentSink intent_sink_;
  std::function<asmc_planner::lane_reference::HighwayReferenceStatus()> highway_reference_source_;
  std::mutex feedback_mutex_;
  MGeoLoader map_;
  RouteManager route_;
  FeatureBuilder builder_;
  DynamicObjectPolicy dynamic_policy_;
  std::vector<::TrackedObject> canonical_objects_;
  std::shared_ptr<const asmc_planner::StaticMissionPolicy> static_mission_policy_;
  asmc_planner::StaticMissionPolicy::Observation static_mission_observation_;
  planning_inputs::DetectedObjectArray::ConstPtr raw_detected_objects_;
  double objects_source_time_ = 0;
  bool dynamic_enabled_ = true;
  ros::Publisher dynamic_diagnostics_pub_;
  ros::Publisher static_mission_diagnostics_pub_;
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
  ros::SteadyTime ego_received_at_;
  std::unique_ptr<SignalShadow> signal_shadow_;
  std::unique_ptr<SignalShadow> signal_active_;
  std::unique_ptr<RosTrafficLightProvider> signal_active_source_;
  V2CommandManager v2_command_manager_;
  asmc_msgs::BehaviorContext last_command_;
  uint32_t current_command_id_ = 0;
  bool has_last_command_ = false;
  bool ego_received_ = false;
  bool map_loaded_=false,route_loaded_=false;
  bool use_v2_interface_ = false;
  bool feedback_v2_received_ = false;
  bool objects_received_ = false;
  double rate_ = 10.0;
  double command_validity_seconds_ = 0.5;
  double feedback_v2_stale_seconds_ = 1.0;
  double objects_stale_seconds_ = 0.3;
};

}  // namespace behavior_planner
