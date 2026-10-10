#include "Behavior/Rule/hard_rule_filter.hpp"

#include <algorithm>
#include <cmath>

namespace behavior_planner {

double ComputeIntersectionApproachDistance(
    double current_speed, double approach_speed,
    double comfortable_decel, double margin) {
  if (!std::isfinite(current_speed) || !std::isfinite(approach_speed) ||
      !std::isfinite(comfortable_decel) || !std::isfinite(margin) ||
      current_speed < 0.0 || approach_speed <= 0.0 ||
      comfortable_decel <= 0.0) {
    return 0.0;
  }
  const double braking_distance = current_speed > approach_speed
      ? (current_speed * current_speed - approach_speed * approach_speed) /
            (2.0 * comfortable_decel)
      : 0.0;
  return braking_distance + std::max(0.0, margin);
}

HardConstraint HardRuleFilter::apply(const UrbanFeature& feature,
                                     std::string* reason) {
  HardConstraint hard;
  std::string result;

  hard.forbid_left_change = !feature.route.lane_change_left_allowed;
  hard.forbid_right_change = !feature.route.lane_change_right_allowed;
  hard.forbid_lane_change =
      hard.forbid_left_change && hard.forbid_right_change;
  hard.max_speed =
      feature.route.speed_limit > 0.0 ? feature.route.speed_limit : 1e9;

  // Start the transition from the measured ego speed so a vehicle already
  // travelling slowly is not constrained tens of metres too early. Once the
  // transition starts, the physical stop line is latched through the 5 m exit
  // zone; this prevents the shrinking current-speed distance from releasing
  // the cap while Frenet is decelerating.
  const double ego_speed = std::max(0.0, feature.ego.speed);
  hard.intersection_approach_speed_limit =
      traffic_stop_config_.approach_speed_limit;
  hard.intersection_approach_activation_distance =
      ComputeIntersectionApproachDistance(
          ego_speed, traffic_stop_config_.approach_speed_limit,
          traffic_stop_config_.approach_comfortable_decel,
          traffic_stop_config_.approach_margin);
  const bool reached_dynamic_approach =
      feature.signal.has_stop_line &&
      std::isfinite(feature.signal.stop_line_s) &&
      std::isfinite(feature.signal.distance_to_stop_line) &&
      feature.signal.distance_to_stop_line >= 0.0 &&
      feature.signal.distance_to_stop_line <=
          hard.intersection_approach_activation_distance;
  const bool inside_intersection_exit =
      feature.signal.has_passed_stop_line &&
      std::isfinite(feature.signal.distance_since_stop_line) &&
      feature.signal.distance_since_stop_line >= 0.0 &&
      feature.signal.distance_since_stop_line <=
          std::max(0.0, traffic_stop_config_.post_stopline_distance);
  const bool current_stop_matches_latch =
      approach_latched_ && feature.signal.has_stop_line &&
      std::isfinite(feature.signal.stop_line_s) &&
      std::abs(feature.signal.stop_line_s - latched_stop_line_s_) <= 0.5 &&
      std::isfinite(feature.signal.distance_to_stop_line) &&
      feature.signal.distance_to_stop_line >= 0.0;

  if (approach_latched_ &&
      !current_stop_matches_latch && !inside_intersection_exit) {
    approach_latched_ = false;
    latched_stop_line_s_ = 1e9;
  }
  if (!approach_latched_ && reached_dynamic_approach) {
    approach_latched_ = true;
    latched_stop_line_s_ = feature.signal.stop_line_s;
  }

  const bool inside_intersection_speed_zone =
      approach_latched_ || inside_intersection_exit;
  if (inside_intersection_speed_zone &&
      std::isfinite(traffic_stop_config_.approach_speed_limit) &&
      traffic_stop_config_.approach_speed_limit > 0.0) {
    hard.max_speed = std::min(
        hard.max_speed, traffic_stop_config_.approach_speed_limit);
    hard.intersection_approach_active = true;
    if (inside_intersection_exit && !current_stop_matches_latch) {
      result += "intersection_exit_speed_limit;";
    } else if (current_stop_matches_latch && !reached_dynamic_approach) {
      result += "intersection_approach_latched_speed_limit;";
    } else {
      result += "intersection_approach_speed_limit;";
    }
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
