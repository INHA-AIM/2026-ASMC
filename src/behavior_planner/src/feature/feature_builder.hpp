#pragma once
#include "behavior_planner/feature/object_projector.hpp"
#include "behavior_planner/signal/traffic_light_provider.hpp"
namespace behavior_planner {
struct SafetyConfig {
  double front_min_gap = 8.0;
  double emergency_ttc_enter = 2.0;
  double emergency_predicted_ttc = 2.0;
  double emergency_required_decel = 5.0;
  double pedestrian_stop_distance = 15.0;
};
struct FollowTargetConfig {
  // Keep these acceptance limits no wider than Frenet's FOLLOW target
  // validation. Behavior owns target selection, so it must not select a
  // target which Frenet will immediately reject.
  double max_distance = 60.0;
  double max_lateral_offset = 1.75;
};
struct AvoidTargetConfig {
  double max_distance = 120.0;
  // Legacy acquisition ceiling. The actual AVOID corridor is narrower and is
  // derived from ego half width + path_margin_m.
  double max_lateral_offset = 1.75;
  double ego_width = 1.85;
  double path_margin_m = 0.4;
};
class FeatureBuilder {
 public:
  void setStopTargetOffset(double offset_m) {
    stop_target_offset_m_ = offset_m > 0.0 ? offset_m : 0.0;
  }
  void setSafetyConfig(const SafetyConfig& config) { safety_ = config; }
  void setFollowTargetConfig(const FollowTargetConfig& config) {
    follow_target_ = config;
  }
  void setAvoidTargetConfig(const AvoidTargetConfig& config) {
    avoid_target_ = config;
  }
  UrbanFeature build(const EgoState&, const PerceptionObjects&,
                     const RouteManager&, const TrafficLightProvider&);

 private:
  int frame_ = 0;
  int last_lead_vehicle_id_ = -1;
  double stop_target_offset_m_ = 0.0;
  SafetyConfig safety_;
  FollowTargetConfig follow_target_;
  AvoidTargetConfig avoid_target_;
  ObjectProjector projector_;
};
}
