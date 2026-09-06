#pragma once
#include "behavior_planner/feature/object_projector.hpp"
#include "behavior_planner/signal/traffic_light_provider.hpp"
namespace behavior_planner { class FeatureBuilder { public: void setStopTargetOffset(double offset_m) { stop_target_offset_m_ = offset_m > 0.0 ? offset_m : 0.0; } UrbanFeature build(const EgoState&,const PerceptionObjects&,const RouteManager&,const TrafficLightProvider&); private:int frame_=0;int last_lead_vehicle_id_=-1;double stop_target_offset_m_=0.0;ObjectProjector projector_; }; }
