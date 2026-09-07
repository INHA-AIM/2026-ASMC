#include "behavior_planner/decision/behavior_decision.hpp"

#include <algorithm>
#include <array>

namespace behavior_planner {

BehaviorContextData BehaviorDecision::decide(const UrbanFeature& feature,
                                             const asmc_msgs::PlanFeedback&) {
  BehaviorContextData context;
  context.timestamp = feature.timestamp;
  context.hard =
      rules_.apply(feature, &context.debug.hard_rule_reason);
  const bool raw_emergency = context.hard.emergency_stop ||
                             context.hard.force_stop;
  if (fsm_.updateEmergency(raw_emergency, feature.risk.min_ttc,
                           feature.timestamp)) {
    context.hard.emergency_stop = true;
    context.hard.force_stop = true;
    if (!raw_emergency) context.debug.hard_rule_reason += "emergency_latched;";
  }
  context.score = scorer_.score(feature, context.hard);
  context.enable.keep = true;
  context.enable.follow = feature.surrounding.has_front_vehicle;
  context.enable.left_change = !context.hard.forbid_left_change &&
                               feature.surrounding.left_lane_safe;
  context.enable.right_change = !context.hard.forbid_right_change &&
                                feature.surrounding.right_lane_safe;
  context.enable.stop = context.hard.planned_stop || context.hard.force_stop;
  context.enable.emergency_stop = context.hard.emergency_stop;

  context.target.current_link_id = feature.route.current_link_id;
  context.target.route_required_link_id =
      feature.route.route_required_link_id;
  context.target.desired_speed =
      std::min(feature.route.speed_limit > 0.0 ? feature.route.speed_limit
                                               : 13.9,
               context.hard.max_speed);
  context.target.stop_before_s = context.hard.stop_before_s;
  if (context.hard.planned_stop && feature.signal.stop_line_valid) {
    context.target.has_stop_target = true;
    context.target.stop_x = feature.signal.stop_line_x;
    context.target.stop_y = feature.signal.stop_line_y;
    context.target.has_target_pose = true;
    context.target.target_x = feature.signal.stop_line_x;
    context.target.target_y = feature.signal.stop_line_y;
    context.target.target_yaw = feature.signal.stop_line_yaw;
  }
  if (feature.surrounding.has_front_vehicle) {
    const ObjectFeature& lead = feature.surrounding.front_vehicle;
    context.target.lead_vehicle_id = lead.id;
    context.extra_values["front_gap"] = lead.gap;
    context.extra_values["front_relative_speed"] = lead.relative_speed;
    context.extra_values["front_ttc"] = lead.ttc;
  }
  if (context.enable.left_change) {
    context.target.candidate_link_ids.push_back(
        feature.route.left_target_link_id);
  }
  if (context.enable.right_change) {
    context.target.candidate_link_ids.push_back(
        feature.route.right_target_link_id);
  }

  struct Candidate {
    double score;
    BehaviorType behavior;
    bool enabled;
  };
  const std::array<Candidate, 6> candidates{{
      {context.score.keep, BehaviorType::KEEP, context.enable.keep},
      {context.score.follow, BehaviorType::FOLLOW, context.enable.follow},
      {context.score.left_change, BehaviorType::LEFT_CHANGE,
       context.enable.left_change},
      {context.score.right_change, BehaviorType::RIGHT_CHANGE,
       context.enable.right_change},
      {context.score.stop, BehaviorType::STOP, context.enable.stop},
      {context.score.emergency_stop, BehaviorType::EMERGENCY_STOP,
       context.enable.emergency_stop},
  }};
  Candidate best = candidates.front();
  for (const Candidate& candidate : candidates) {
    if (candidate.enabled && candidate.score > best.score) best = candidate;
  }
  context.selected_behavior = best.behavior;

  switch (best.behavior) {
    case BehaviorType::FOLLOW:
      context.state = BehaviorState::FOLLOW;
      // Behavior owns the maneuver and its cruise-speed ceiling.  The leader
      // ID and requested gap are sent to Frenet, which alone owns the
      // cycle-by-cycle FOLLOW speed and trajectory calculation.
      break;
    case BehaviorType::LEFT_CHANGE:
      context.state = BehaviorState::LEFT_CHANGE_PREPARE;
      context.target.target_link_id = feature.route.left_target_link_id;
      context.target.has_target_pose = feature.route.left_target_pose_valid;
      context.target.target_x = feature.route.left_target_x;
      context.target.target_y = feature.route.left_target_y;
      context.target.target_yaw = feature.route.left_target_yaw;
      break;
    case BehaviorType::RIGHT_CHANGE:
      context.state = BehaviorState::RIGHT_CHANGE_PREPARE;
      context.target.target_link_id = feature.route.right_target_link_id;
      context.target.has_target_pose = feature.route.right_target_pose_valid;
      context.target.target_x = feature.route.right_target_x;
      context.target.target_y = feature.route.right_target_y;
      context.target.target_yaw = feature.route.right_target_yaw;
      break;
    case BehaviorType::STOP:
      context.state = BehaviorState::STOP;
      // A planned stop retains the cruise target: Frenet lowers it according
      // to remaining braking distance. force_stop bypasses planning anyway.
      if (!context.hard.planned_stop) context.target.desired_speed = 0.0;
      break;
    case BehaviorType::EMERGENCY_STOP:
      context.state = BehaviorState::EMERGENCY_STOP;
      context.target.desired_speed = 0.0;
      break;
    default:
      context.state = BehaviorState::KEEP_LANE;
      context.target.target_link_id = feature.route.current_link_id;
      break;
  }
  context.debug.selected_reason = "highest_enabled_score";
  context.debug.cost_terms = {
      {"keep", context.score.keep},
      {"follow", context.score.follow},
      {"left_change", context.score.left_change},
      {"right_change", context.score.right_change},
      {"stop", context.score.stop},
      {"emergency_stop", context.score.emergency_stop},
  };
  return context;
}

}  // namespace behavior_planner
