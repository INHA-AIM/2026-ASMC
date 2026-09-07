#include "behavior_planner/rule/hard_rule_filter.hpp"

namespace behavior_planner {

HardConstraint HardRuleFilter::apply(const UrbanFeature& feature,
                                     std::string* reason) const {
  HardConstraint hard;
  std::string result;

  hard.forbid_left_change = !feature.route.lane_change_left_allowed;
  hard.forbid_right_change = !feature.route.lane_change_right_allowed;
  hard.forbid_lane_change =
      hard.forbid_left_change && hard.forbid_right_change;
  hard.max_speed =
      feature.route.speed_limit > 0.0 ? feature.route.speed_limit : 1e9;

  if (feature.signal.has_stop_line &&
      feature.signal.distance_to_stop_line >= 0.0 &&
      feature.signal.distance_to_stop_line <= 30.0) {
    hard.stop_line_constraint = true;
    hard.stop_before_s = feature.signal.stop_line_s;

    // MGeo right_unprotected is a route turn, not a RIGHT_CHANGE maneuver.
    // Ignore only the vehicle signal; pedestrian/collision stops remain below.
    const bool unprotected_right =
        feature.signal.has_required_movement &&
        feature.signal.required_movement == SignalMovement::RIGHT_UNPROTECTED;
    const bool stop_aspect =
        feature.signal.light_state == TrafficLightState::RED ||
        feature.signal.light_state == TrafficLightState::YELLOW;
    if (stop_aspect && !unprotected_right) {
      hard.planned_stop = true;
      result += "traffic_light;";
    }
  }

  if (feature.risk.pedestrian_risk) {
    hard.force_stop = true;
    result += "pedestrian_risk;";
  }
  if (feature.risk.emergency_risk) {
    hard.emergency_stop = true;
    hard.force_stop = true;
    result += "emergency_ttc;";
  }

  if (result.empty()) result = "no_hard_constraint";
  if (reason) *reason = result;
  return hard;
}

}  // namespace behavior_planner
