#pragma once

#include <ros/ros.h>

#include "Behavior/Context/behavior_context.hpp"
#include "Behavior/Feature/urban_feature.hpp"
#include "Behavior/Map/route_manager.hpp"
#include "Parser/Traffic/traffic_light_provider.hpp"

namespace behavior_planner {

class BehaviorVisualization {
 public:
  BehaviorVisualization() = default;
  explicit BehaviorVisualization(ros::Publisher publisher)
      : publisher_(std::move(publisher)) {}
  void publish(const UrbanFeature& feature,
               const BehaviorContextData& context,
               const RouteManager& route,
               const MGeoLoader& map,
               const TrafficLightProvider& traffic_lights) const;

 private:
  ros::Publisher publisher_;
};

}  // namespace behavior_planner
