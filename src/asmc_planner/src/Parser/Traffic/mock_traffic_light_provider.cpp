#include "Parser/Traffic/mock_traffic_light_provider.hpp"

#include <yaml-cpp/yaml.h>

namespace behavior_planner {
namespace {

TrafficLightState ParseState(const std::string& state) {
  if (state == "RED") return TrafficLightState::RED;
  if (state == "YELLOW") return TrafficLightState::YELLOW;
  if (state == "GREEN") return TrafficLightState::GREEN;
  return TrafficLightState::UNKNOWN;
}

}  // namespace

bool MockTrafficLightProvider::load(const std::string& path) {
  try {
    const auto yaml = YAML::LoadFile(path);
    fallback_ =
        ParseState(yaml["default_state"].as<std::string>("GREEN"));
    if (yaml["traffic_lights"]) {
      for (const auto& item : yaml["traffic_lights"]) {
        states_[item.first.as<std::string>()] =
            ParseState(item.second.as<std::string>());
      }
    }
    return true;
  } catch (...) {
    return false;
  }
}

TrafficLightState MockTrafficLightProvider::getState(
    const std::string& traffic_light_id, SignalMovement) const {
  const auto found = states_.find(traffic_light_id);
  return found == states_.end() ? fallback_ : found->second;
}

bool MockTrafficLightProvider::hasRecentDetection(
    const std::string& traffic_light_id) const {
  return getState(traffic_light_id) != TrafficLightState::UNKNOWN;
}

}  // namespace behavior_planner
