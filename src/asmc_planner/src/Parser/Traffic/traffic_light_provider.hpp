#pragma once

#include <string>

#include "Behavior/Context/behavior_types.hpp"

namespace behavior_planner {

class TrafficLightProvider {
 public:
  virtual ~TrafficLightProvider() = default;
  virtual TrafficLightState getState(
      const std::string& traffic_light_id,
      SignalMovement movement = SignalMovement::UNKNOWN) const = 0;
  virtual bool hasRecentDetection(
      const std::string& traffic_light_id) const = 0;
  virtual std::string getRawLabel() const { return {}; }
};

}  // namespace behavior_planner
