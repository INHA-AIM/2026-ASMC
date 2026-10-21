#include "Behavior/Decision/behavior_decision.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace behavior_planner {

bool BehaviorDecision::setAbsoluteSpeedLimit(double mps) {
  if(!std::isfinite(mps)||mps<=0)return false;
  absolute_speed_limit_=mps;return true;
}

bool BehaviorDecision::setNormalCruiseConfig(const NormalCruiseConfig& config) {
  normal_cruise_ = {};
  if (!config.enabled) return true;
  if (!std::isfinite(config.target_speed) || !std::isfinite(config.max_speed) ||
      !std::isfinite(config.minimum_map_speed) || config.minimum_map_speed <= 0.0 ||
      config.target_speed < config.minimum_map_speed ||
      config.max_speed < config.target_speed) return false;
  for (const auto& entry : config.link_speed_caps) {
    if (entry.first.empty() || !std::isfinite(entry.second) ||
        entry.second <= 0.0 || entry.second > config.max_speed) return false;
  }
  normal_cruise_ = config;
  return true;
}

BehaviorContextData BehaviorDecision::decide(const UrbanFeature& feature,
                                             const asmc_msgs::PlanFeedback&) {
  // The v1 feedback contract has no PREPARE/READY/COMMITTED lifecycle.
  // Preserve legacy behavior selection without synthesizing v2 transitions.
  return decide(feature, PlannerFeedbackData{}, 0);
}

BehaviorContextData BehaviorDecision::decide(
    const UrbanFeature& feature, const PlannerFeedbackData& planner_feedback,
    uint32_t active_command_id) {
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
  const int32_t follow_target_id = fsm_.updateFollowTarget(
      feature.surrounding.has_front_vehicle,
      feature.surrounding.has_front_vehicle
          ? feature.surrounding.front_vehicle.id
          : -1,
      feature.timestamp);
  const bool follow_release_grace =
      fsm_.followReleaseGraceActive() && follow_target_id >= 0;
  context.score =
      scorer_.score(feature, context.hard, follow_release_grace);
  context.enable.keep = true;
  context.enable.follow = follow_target_id >= 0;
  context.enable.left_change = !context.hard.forbid_left_change &&
                               feature.surrounding.left_lane_safe;
  context.enable.right_change = !context.hard.forbid_right_change &&
                                feature.surrounding.right_lane_safe;
  const bool dynamic_avoid=feature.dynamic.action==DynamicAction::AVOID &&
      !feature.risk.static_obstacle_ahead;
  context.enable.avoid = (feature.risk.static_obstacle_ahead &&
                         feature.risk.avoidance_object_id >= 0) || dynamic_avoid;
  if(dynamic_avoid) context.score.avoid=90.0;
  context.extra_flags["dynamic_in_lane_avoid"]=dynamic_avoid;
  context.enable.stop = context.hard.planned_stop || context.hard.force_stop;
  context.enable.emergency_stop = context.hard.emergency_stop;

  context.target.current_link_id = feature.route.current_link_id;
  context.target.route_required_link_id =
      feature.route.route_required_link_id;
  context.target.desired_speed =
      std::min(feature.route.speed_limit > 0.0 ? feature.route.speed_limit
                                               : 13.9,
               context.hard.max_speed);
  if (context.hard.intersection_approach_active) {
    context.extra_flags["intersection_approach_speed_limit"] = true;
    context.extra_values["intersection_approach_speed_mps"] =
        context.hard.max_speed;
    context.extra_values["intersection_approach_activation_distance_m"] =
        context.hard.intersection_approach_activation_distance;
  }
  context.target.stop_before_s = context.hard.stop_before_s;
  const bool signal_stop_aspect=feature.signal.light_state==TrafficLightState::RED ||
      feature.signal.light_state==TrafficLightState::YELLOW;
  const bool signal_exempt=feature.signal.has_required_movement &&
      feature.signal.required_movement==SignalMovement::RIGHT_UNPROTECTED;
  if (context.hard.planned_stop && feature.signal.stop_line_valid && signal_stop_aspect && !signal_exempt) {
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
  if (follow_release_grace) {
    context.target.lead_vehicle_id = follow_target_id;
    context.extra_flags["follow_release_grace"] = true;
  }
  if(feature.dynamic.action==DynamicAction::STOP &&
      (!context.target.has_stop_target || feature.dynamic.stop_s<feature.signal.stop_target_s)) {
    context.target.has_stop_target=context.target.has_target_pose=true;
    context.target.stop_x=context.target.target_x=feature.dynamic.stop_x;
    context.target.stop_y=context.target.target_y=feature.dynamic.stop_y;
    context.target.target_yaw=feature.dynamic.stop_yaw;
  }
  if (context.enable.avoid) {
    context.target.avoidance_object_id = dynamic_avoid?feature.dynamic.object_id:feature.risk.avoidance_object_id;
    context.extra_values["avoidance_object_id"] = context.target.avoidance_object_id;
  }

  const bool follow_owns_avoid_target =
      follow_target_id >= 0 && context.enable.avoid &&
      (feature.risk.avoidance_object_id == follow_target_id ||
       feature.risk.avoid_target_matches_front_lead ||
       (follow_release_grace &&
        feature.risk.avoid_target_matches_recent_lead));
  if (follow_owns_avoid_target) {
    context.extra_flags["fsm_follow_owns_avoid_target"] = true;
  }

  const bool cancel_reclassified_avoid =
      feature.risk.avoid_target_reclassified_moving &&
      !context.enable.avoid;
  if (cancel_reclassified_avoid) {
    context.extra_flags["cancel_reclassified_avoid"] = true;
  }

  // STOP과 AVOID가 동시에 필요하면 검출 순서나 고정 score가 아니라
  // global-path s상의 첫 제약을 우선한다. STOP은 이미 후륜축 기준으로
  // 보정된 목표 s, 장애물은 OBB 전면 s를 비교한다.
  bool stop_preempts_avoid = false;
  bool avoid_precedes_stop = false;
  if (context.hard.planned_stop && context.enable.avoid &&
      feature.signal.stop_line_valid &&
      std::isfinite(feature.signal.stop_target_s) &&
      std::isfinite(feature.risk.nearest_obstacle_s) &&
      std::isfinite(feature.risk.nearest_obstacle_length)) {
    const double obstacle_front_s =
        feature.risk.nearest_obstacle_s -
        0.5 * std::max(0.0, feature.risk.nearest_obstacle_length);
    stop_preempts_avoid = feature.signal.stop_target_s <= obstacle_front_s;
    avoid_precedes_stop = !stop_preempts_avoid;
    context.extra_values["stop_target_s"] = feature.signal.stop_target_s;
    context.extra_values["avoid_obstacle_front_s"] = obstacle_front_s;
    context.extra_flags["stop_preempts_avoid"] = stop_preempts_avoid;
    context.extra_flags["avoid_precedes_stop"] = avoid_precedes_stop;
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
  const std::array<Candidate, 7> candidates{{
      {context.score.keep, BehaviorType::KEEP, context.enable.keep},
      {context.score.follow, BehaviorType::FOLLOW, context.enable.follow},
      {context.score.left_change, BehaviorType::LEFT_CHANGE,
       context.enable.left_change},
      {context.score.right_change, BehaviorType::RIGHT_CHANGE,
       context.enable.right_change},
      {context.score.avoid, BehaviorType::AVOID, context.enable.avoid},
      {context.score.stop, BehaviorType::STOP, context.enable.stop},
      {context.score.emergency_stop, BehaviorType::EMERGENCY_STOP,
       context.enable.emergency_stop},
  }};
  Candidate best = candidates.front();
  for (const Candidate& candidate : candidates) {
    if (candidate.enabled && candidate.score > best.score) best = candidate;
  }
  if (stop_preempts_avoid) {
    best = {context.score.stop, BehaviorType::STOP, true};
  } else if (avoid_precedes_stop) {
    best = {context.score.avoid, BehaviorType::AVOID, true};
  }
  if (feature.roundabout.enabled && feature.roundabout.approach &&
      !context.hard.planned_stop && !context.hard.force_stop && !context.hard.emergency_stop) {
    best = {0.0, BehaviorType::MERGE, true};
  }
  const Fsm::ManeuverOutput fsm_output = fsm_.updateManeuver(
      best.behavior, planner_feedback, active_command_id,
      stop_preempts_avoid, cancel_reclassified_avoid,
      follow_owns_avoid_target);
  context.selected_behavior = fsm_output.behavior;
  context.state = fsm_output.state;
  context.maneuver_phase = fsm_output.phase;

  switch (context.selected_behavior) {
    case BehaviorType::MERGE:
      context.target.current_link_id = feature.roundabout.source_link_id;
      context.target.target_link_id = feature.roundabout.target_link_id;
      context.target.desired_speed = 25.0/3.6;
      context.target.has_target_pose = false;
      context.extra_flags["roundabout_merge"] = true;
      break;
    case BehaviorType::FOLLOW:
      // Behavior owns the maneuver and its cruise-speed ceiling.  The leader
      // ID and requested gap are sent to Frenet, which alone owns the
      // cycle-by-cycle FOLLOW speed and trajectory calculation.
      break;
    case BehaviorType::LEFT_CHANGE:
      context.target.target_link_id = feature.route.left_target_link_id;
      context.target.has_target_pose = feature.route.left_target_pose_valid;
      context.target.target_x = feature.route.left_target_x;
      context.target.target_y = feature.route.left_target_y;
      context.target.target_yaw = feature.route.left_target_yaw;
      break;
    case BehaviorType::RIGHT_CHANGE:
      context.target.target_link_id = feature.route.right_target_link_id;
      context.target.has_target_pose = feature.route.right_target_pose_valid;
      context.target.target_x = feature.route.right_target_x;
      context.target.target_y = feature.route.right_target_y;
      context.target.target_yaw = feature.route.right_target_yaw;
      break;
    case BehaviorType::STOP:
      // A planned stop retains the cruise target: Frenet lowers it according
      // to remaining braking distance. force_stop bypasses planning anyway.
      if (!context.hard.planned_stop) context.target.desired_speed = 0.0;
      break;
    case BehaviorType::AVOID:
      break;
    case BehaviorType::EMERGENCY_STOP:
      context.target.desired_speed = 0.0;
      break;
    default:
      context.target.target_link_id = feature.route.current_link_id;
      break;
  }
  // Apply after FSM arbitration so committed avoidance/follow/merge cannot
  // inherit the ordinary cruise request. Every cycle starts from the original
  // map constraints above; leaving KEEP immediately restores those constraints.
  const bool roundabout_zone = feature.roundabout.enabled &&
      (feature.roundabout.approach ||
       feature.route.current_link_id == feature.roundabout.source_link_id ||
       feature.route.current_link_id == feature.roundabout.target_link_id);
  const auto link_cap = normal_cruise_.link_speed_caps.find(
      feature.route.current_link_id);
  const bool explicit_link = link_cap != normal_cruise_.link_speed_caps.end();
  const bool ordinary_cruise = normal_cruise_.enabled &&
      feature.ego.valid && feature.route.current_link_valid &&
      context.selected_behavior == BehaviorType::KEEP &&
      !context.hard.intersection_approach_active &&
      !context.hard.planned_stop && !context.hard.force_stop &&
      !context.hard.emergency_stop && !roundabout_zone &&
      !feature.risk.static_obstacle_ahead &&
      !feature.risk.avoid_target_reclassified_moving &&
      !feature.risk.pedestrian_risk && !feature.risk.emergency_risk &&
      !feature.surrounding.has_front_vehicle && !follow_release_grace &&
      std::isfinite(feature.route.speed_limit) &&
      feature.route.speed_limit > 0.0 &&
      (explicit_link ||
       feature.route.speed_limit >= normal_cruise_.minimum_map_speed) &&
      std::abs(context.hard.max_speed - feature.route.speed_limit) < 1e-6;
  if (ordinary_cruise) {
    context.hard.max_speed = explicit_link ? link_cap->second
                                          : normal_cruise_.max_speed;
    context.target.desired_speed = std::min(normal_cruise_.target_speed,
                                          context.hard.max_speed);
    context.extra_flags["normal_cruise_active"] = true;
    context.debug.hard_rule_reason += ";normal_cruise_simulation_profile";
  }
  // Keep the approach cap throughout the configured dynamic section instead
  // of reaccelerating immediately after the stop-line exit buffer. Lower
  // signal/object limits and execution safety still take precedence.
  if(feature.dynamic.in_scope) {
    context.hard.max_speed=std::min(context.hard.max_speed,feature.dynamic.section_speed_limit);
    context.target.desired_speed=std::min(context.target.desired_speed,context.hard.max_speed);
    context.debug.hard_rule_reason+=";dynamic_section_speed_limit";
  }
  // Final arbitration cap covers KEEP/FOLLOW/AVOID/MERGE and lane changes.
  // Initial-state continuity and all lower signal/obstacle limits remain.
  context.hard.max_speed=std::min(context.hard.max_speed,absolute_speed_limit_);
  context.target.desired_speed=std::min(context.target.desired_speed,context.hard.max_speed);
  if (follow_owns_avoid_target &&
      context.selected_behavior == BehaviorType::FOLLOW) {
    context.debug.selected_reason = "fsm_follow_owns_same_target";
  } else if (stop_preempts_avoid) {
    context.debug.selected_reason = "stop_before_avoid";
  } else if (avoid_precedes_stop) {
    context.debug.selected_reason = "avoid_before_stop";
  } else if (follow_release_grace &&
             context.selected_behavior == BehaviorType::FOLLOW) {
    context.debug.selected_reason = "follow_release_grace";
  } else {
    context.debug.selected_reason = "highest_enabled_score";
  }
  context.debug.cost_terms = {
      {"keep", context.score.keep},
      {"follow", context.score.follow},
      {"left_change", context.score.left_change},
      {"right_change", context.score.right_change},
      {"avoid", context.score.avoid},
      {"stop", context.score.stop},
      {"emergency_stop", context.score.emergency_stop},
  };
  return context;
}

}  // namespace behavior_planner

