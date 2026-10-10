#pragma once

#include <unordered_map>

#include "Parser/Traffic/traffic_light_provider.hpp"

namespace behavior_planner {

class MockTrafficLightProvider : public TrafficLightProvider {
 public:
  bool load(const std::string& path);
  TrafficLightState getState(
      const std::string& traffic_light_id,
      SignalMovement movement = SignalMovement::UNKNOWN) const override;
  bool hasRecentDetection(
      const std::string& traffic_light_id) const override;

 private:
  TrafficLightState fallback_ = TrafficLightState::GREEN;
  std::unordered_map<std::string, TrafficLightState> states_;
};

}  // namespace behavior_planner
