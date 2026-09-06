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
class FeatureBuilder { public: void setStopTargetOffset(double offset_m) { stop_target_offset_m_ = offset_m > 0.0 ? offset_m : 0.0; } void setSafetyConfig(const SafetyConfig& config) { safety_ = config; } UrbanFeature build(const EgoState&,const PerceptionObjects&,const RouteManager&,const TrafficLightProvider&); private:int frame_=0;int last_lead_vehicle_id_=-1;double stop_target_offset_m_=0.0;SafetyConfig safety_;ObjectProjector projector_; };
}
