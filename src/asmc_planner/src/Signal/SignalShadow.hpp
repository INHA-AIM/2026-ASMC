#pragma once
#include <memory>
#include <ros/ros.h>
#include <asmc_msgs/VehicleState.h>
#include "Signal/SignalPolicy.hpp"
#include "Signal/LocalizationContinuity.hpp"
#include "Behavior/Feature/urban_feature.hpp"
#include "Behavior/Map/route_manager.hpp"
#include "Parser/Traffic/ros_traffic_light_provider.hpp"

namespace behavior_planner {
// No IntentSink, command manager, execution owner, or Controller handle is
// accessible here. All IDs and policy state belong to this observer instance.
class SignalShadow {
 public:
  SignalShadow(ros::NodeHandle&,ros::NodeHandle&,bool operational=false);
  bool enabled() const { return enabled_; }
  void PushPose(const EgoState&,const RouteManager&);
  void Update(const EgoState&,bool state_fresh,bool risks_fresh,
              const UrbanFeature*,const RouteManager&,RosTrafficLightProvider*);
  // The operational evaluator is a DIFFERENT instance, journal and history.
  // The shadow instance never receives these proofs or changes live state.
  void ExecutionEvidence(const asmc_planner::signal::Evidence& e) {execution_evidence_=e;}
  bool inScope() const {return in_scope_;}
  const asmc_planner::signal::Output& output() const {return last_output_;}
  const asmc_planner::signal::Geometry& geometry() const {return current_.geometry;}
  const asmc_planner::signal::Config& config() const {return config_;}
  double targetOffset() const {return selected_offset_;}
  bool recoveryCandidate() const {return recovery_candidate_;}
  bool pathRecoveryCandidate() const {return path_recovery_candidate_;}
  bool pathRecoveryRequiresExit() const {return path_recovery_requires_exit_;}
 private:
  void LocalizationCallback(const ros::MessageEvent<asmc_msgs::VehicleState const>&);
  struct Binding {
    asmc_planner::signal::Geometry geometry;
    std::string node,frame,movement;
  };
  bool enabled_=true, source_clock_verified_=false;
  bool operational_=false,in_scope_=false,recovery_candidate_=false;
  bool episode_activated_=false;
  bool path_recovery_candidate_=false;
  bool path_recovery_requires_exit_=false;
  asmc_planner::signal::Evidence execution_evidence_;
  asmc_planner::signal::Output last_output_;
  asmc_planner::signal::Config config_;
  std::unique_ptr<asmc_planner::signal::Policy> policy_;
  std::unique_ptr<asmc_planner::signal::PoseHistory> history_;
  std::unique_ptr<asmc_planner::signal::LocalizationContinuity> localization_;
  ros::Subscriber localization_sub_;
  std::string localization_caller_;
  std::uint64_t last_localization_generation_=0;
  std::vector<Binding> bindings_;
  std::string scope_first_link_,scope_last_link_;
  Binding current_;
  std::uint64_t route_generation_=0, episode_generation_=1, source_epoch_=1;
  double last_ros_=0,last_steady_=0;
  double last_policy_pose_stamp_=0;
  double selected_offset_=5.5;
  asmc_planner::signal::StopCauses causes_;
  double collision_safe_since_=-1,pedestrian_safe_since_=-1,input_safe_since_=-1;
  asmc_planner::signal::Pose previous_pose_;
  std::deque<asmc_planner::signal::Observation> pending_;
  std::size_t published_events_=0;
  ros::Publisher publisher_;
};
}  // namespace behavior_planner
