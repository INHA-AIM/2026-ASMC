#include "behavior_planner/rule/hard_rule_filter.hpp"

#include <algorithm>
#include <cmath>

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

  // light_state remains UNKNOWN when the camera label is stale or
  // not_detected. A non-UNKNOWN state therefore means a signal was actually
  // observed and associated with the next map stop line. Apply only a normal
  // speed ceiling here: green must not request STOP/E-STOP.
  const bool visible_route_signal =
      feature.signal.has_stop_line &&
      feature.signal.light_state != TrafficLightState::UNKNOWN &&
      std::isfinite(feature.signal.distance_to_stop_line) &&
      feature.signal.distance_to_stop_line >= 0.0;
  if (visible_route_signal &&
      std::isfinite(traffic_stop_config_.visible_signal_speed_limit) &&
      traffic_stop_config_.visible_signal_speed_limit > 0.0) {
    hard.max_speed = std::min(
        hard.max_speed, traffic_stop_config_.visible_signal_speed_limit);
    hard.signal_approach_speed_limit = true;
    result += "visible_signal_speed_limit;";
  }

  // distance_to_stop_line은 물리 정지선까지의 거리지만 Frenet에 전달되는
  // 목표는 target_offset만큼 뒤이고, Frenet도 stop_buffer를 남긴다. 따라서
  // 현재 속도에서 편안하게 감속하는 데 필요한 거리뿐 아니라 두 여유 거리와
  // 제동 진입 margin까지 물리 정지선 기준으로 모두 더한다.
  const double speed = std::max(0.0, feature.ego.speed);
  const double decel =
      std::max(0.1, traffic_stop_config_.comfortable_decel);
  const double dynamic_stop_distance =
      speed * speed / (2.0 * decel) +
      std::max(0.0, traffic_stop_config_.braking_margin) +
      std::max(0.0, traffic_stop_config_.target_offset) +
      std::max(0.0, traffic_stop_config_.planner_stop_buffer);
  const double stop_detection_distance = std::max(
      std::max(0.0, traffic_stop_config_.minimum_detection_distance),
      dynamic_stop_distance);

  if (feature.signal.has_stop_line &&
      feature.signal.distance_to_stop_line >= 0.0 &&
      feature.signal.distance_to_stop_line <= stop_detection_distance) {
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
