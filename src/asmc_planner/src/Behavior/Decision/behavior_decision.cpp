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

double BehaviorDecision::ordinarySpeedCeiling(const std::string& link,double map_cap) const {
  double cap=map_cap;
  const auto it=normal_cruise_.link_speed_caps.find(link);
  if(normal_cruise_.enabled && (it!=normal_cruise_.link_speed_caps.end() || map_cap>=normal_cruise_.minimum_map_speed))
    cap=it==normal_cruise_.link_speed_caps.end()?normal_cruise_.max_speed:it->second;
  return std::min(cap,absolute_speed_limit_);
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
  if(feature.lane_change.network&&driving_epoch_!=feature.lane_change.reference_epoch) {
    // A grace-period leader belongs to its old driving reference. Do not
    // inherit it as a new-lane FOLLOW target after an atomic handover.
    fsm_.updateFollowTarget(false,-1,NAN);
    driving_epoch_=feature.lane_change.reference_epoch;
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
    const double target_x = feature.signal.stop_target_pose_valid ?
        feature.signal.stop_target_x : feature.signal.stop_line_x;
    const double target_y = feature.signal.stop_target_pose_valid ?
        feature.signal.stop_target_y : feature.signal.stop_line_y;
    context.target.stop_x = target_x;
    context.target.stop_y = target_y;
    context.target.has_target_pose = true;
    context.target.target_x = target_x;
    context.target.target_y = target_y;
    context.target.target_yaw = feature.signal.stop_target_pose_valid ?
        feature.signal.stop_target_yaw : feature.signal.stop_line_yaw;
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
  if (feature.highway.valid && feature.highway.approach && !feature.roundabout.approach &&
      !context.hard.planned_stop && !context.hard.force_stop && !context.hard.emergency_stop &&
      best.behavior != BehaviorType::AVOID) {
    best = {0.0, BehaviorType::MERGE, true};
  }
  updateHighwayLaneChange(feature,planner_feedback,active_command_id,context,best.behavior);
  const bool merge_left_preparation=best.behavior==BehaviorType::LEFT_CHANGE&&
      feature.highway.approach&&feature.lane_change.network&&
      asmc_planner::lane_reference::HighwayLeftChangeSupportsMerge(*feature.lane_change.network,
          feature.lane_change.left.source_link,feature.lane_change.left.target_link);
  const Fsm::ManeuverOutput fsm_output = fsm_.updateManeuver(
      best.behavior, planner_feedback, active_command_id,
      stop_preempts_avoid, cancel_reclassified_avoid,
      follow_owns_avoid_target,merge_left_preparation);
  if(fsm_output.behavior==BehaviorType::MERGE&&best.behavior==BehaviorType::LEFT_CHANGE) {
    // A committed mandatory merge retains ownership. Do not freeze an
    // unpublished optional offer against that merge's subsequent feedback.
    lc_active_=lc_committed_=false;lc_since_=-1;
  }
  context.selected_behavior = fsm_output.behavior;
  context.state = fsm_output.state;
  context.maneuver_phase = fsm_output.phase;

  switch (context.selected_behavior) {
    case BehaviorType::MERGE:
      if(feature.highway.valid) {
        context.target.current_link_id=feature.highway.source_link;
        context.target.target_link_id=feature.highway.target_link;
        context.target.desired_speed=std::min(context.hard.max_speed,feature.highway.speed_cap_mps);
        context.target.has_target_pose=false;
        context.extra_flags["highway_merge"]=true;
        break;
      }
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
      if(lc_active_) {
        context.target.current_link_id=lc_offer_.source_link;
        context.target.target_link_id=lc_offer_.target_link;context.target.has_target_pose=true;
        context.target.target_x=lc_offer_.x;context.target.target_y=lc_offer_.y;context.target.target_yaw=lc_offer_.yaw;
        context.target.desired_speed=std::min(feature.highway.speed_cap_mps,context.hard.max_speed);
        context.extra_flags["highway_lane_change"]=true;break;
      }
      context.target.target_link_id = feature.route.left_target_link_id;
      context.target.has_target_pose = feature.route.left_target_pose_valid;
      context.target.target_x = feature.route.left_target_x;
      context.target.target_y = feature.route.left_target_y;
      context.target.target_yaw = feature.route.left_target_yaw;
      break;
    case BehaviorType::RIGHT_CHANGE:
      if(lc_active_) {
        context.target.current_link_id=lc_offer_.source_link;
        context.target.target_link_id=lc_offer_.target_link;context.target.has_target_pose=true;
        context.target.target_x=lc_offer_.x;context.target.target_y=lc_offer_.y;context.target.target_yaw=lc_offer_.yaw;
        context.target.desired_speed=std::min(feature.highway.speed_cap_mps,context.hard.max_speed);
        context.extra_flags["highway_lane_change"]=true;break;
      }
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
  if(feature.highway.cruising &&
     (context.selected_behavior==BehaviorType::KEEP || context.selected_behavior==BehaviorType::FOLLOW)) {
    context.target.desired_speed=std::min(feature.highway.speed_cap_mps,context.hard.max_speed);
    context.extra_flags["highway_cruise_active"]=true;
  }
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
      !feature.highway.cruising &&
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
    context.hard.max_speed = ordinarySpeedCeiling(feature.route.current_link_id, feature.route.speed_limit);
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
  const bool highway_speed_policy=feature.highway.cruising ||
      context.extra_flags["highway_merge"] ||
      context.extra_flags["highway_lane_change"];
  const double selected_ceiling=highway_speed_policy?feature.highway.speed_cap_mps:absolute_speed_limit_;
  // A153 exit preview only lowers a speed ceiling; it never creates STOP,
  // signal approach or a permission to accelerate before a faster link.
  const auto& route=feature.route;
  if(feature.timestamp<boundary_last_time_ || route.route_generation!=boundary_generation_ ||
     !feature.ego.valid || !route.lower_boundary_valid ||
     feature.ego.s<route.lower_boundary_s-20. || feature.ego.s>route.next_boundary_s+1.)boundary_cap_=0;
  boundary_last_time_=feature.timestamp;boundary_generation_=route.route_generation;
  if(feature.ego.valid&&route.lower_boundary_valid&&std::isfinite(route.next_speed_limit)&&route.next_speed_limit>0&&
     feature.ego.s>=route.lower_boundary_s-10. && feature.ego.s<=route.next_boundary_s+1.) {
    const double next=ordinarySpeedCeiling(route.next_speed_link,route.next_speed_limit);
    if(next<selected_ceiling)boundary_cap_=boundary_cap_>0?std::min(boundary_cap_,next):next;
  }
  context.hard.max_speed=std::min(context.hard.max_speed,selected_ceiling);
  if(boundary_cap_>0){context.hard.max_speed=std::min(context.hard.max_speed,boundary_cap_);
    context.extra_flags["lower_next_link_speed_cap"]=true;
    context.debug.hard_rule_reason+=";A153_lower_next_link_speed_cap";}

  context.target.desired_speed=std::min(context.target.desired_speed,context.hard.max_speed);
  if((context.selected_behavior==BehaviorType::LEFT_CHANGE||context.selected_behavior==BehaviorType::RIGHT_CHANGE)&&
      context.extra_flags["highway_lane_change"]) {
    context.debug.selected_reason="slow_leader_highway_lane_change";
  } else if (follow_owns_avoid_target &&
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

namespace {
// Same endpoint and achievable speed response for both alternatives. Both
// alternatives move during the change; its duration is not subtracted twice.
double travelTime(double distance,double initial,double target,double accel,double decel) {
  if(distance<=0)return 0;
  if(target<=.1)return 1e9;
  const double a=initial<=target?accel:-decel;
  const double time=(target-initial)/a;
  const double response_distance=(initial+target)*time*.5;
  if(distance<=response_distance)
    return 2*distance/(initial+std::sqrt(std::max(0.0,initial*initial+2*a*distance)));
  return time+(distance-response_distance)/target;
}
}
void BehaviorDecision::updateHighwayLaneChange(const UrbanFeature& f,
    const PlannerFeedbackData& feedback,uint32_t active_id,BehaviorContextData& c,BehaviorType& best) {
  const auto network=f.lane_change.network;
  if(!network||!network->policy.enabled)return;
  const auto& policy=network->policy;
  const double now=f.lane_change.evaluation_stamp>0?f.lane_change.evaluation_stamp:f.timestamp;
  const auto interrupted=f.extra_flags.find("highway_transition_interrupted");
  if(c.hard.emergency_stop||(interrupted!=f.extra_flags.end()&&interrupted->second)) {
    // Emergency clears the FSM lifecycle too. Its old frozen source/target
    // must not become a new transaction when the emergency is released.
    lc_active_=lc_committed_=false;lc_since_=-1;lc_last_lead_observation_=-1;
    c.enable.left_change=c.enable.right_change=false;return;
  }
  c.extra_flags["lane_change_inputs_fresh"]=f.lane_change.fresh;
  c.extra_values["lane_change_observation_age_s"]=f.lane_change.observation_stamp>0?
      now-f.lane_change.observation_stamp:-1;
  c.enable.left_change=f.lane_change.left.available&&!c.hard.forbid_left_change;
  c.enable.right_change=f.lane_change.right.available&&!c.hard.forbid_right_change;
  const bool matches=lc_active_&&active_id&&feedback.valid&&feedback.command_id==active_id;
  if(matches&&(feedback.execution_state==PlannerExecutionState::COMMITTED||
               feedback.execution_state==PlannerExecutionState::EXECUTING))lc_committed_=true;
  if(matches&&(feedback.execution_state==PlannerExecutionState::COMPLETED||
               feedback.execution_state==PlannerExecutionState::FAILED)) {
    const bool executed=lc_committed_;
    lc_active_=lc_committed_=false;lc_since_=-1;
    if(executed)lc_cooldown_until_=now+policy.cooldown_s;
  }
  if(lc_active_&&lc_committed_) {
    // FSM owns committed maneuver precedence. Keep its frozen pose available
    // even if the leader disappears or the target is now physically occupied by ego.
    if(best!=BehaviorType::EMERGENCY_STOP)best=lc_direction_;
    return;
  }
  const auto& lead=f.surrounding.front_vehicle;
  const double free=std::min(f.lane_change.free_speed,c.hard.max_speed);
  const double lead_speed=std::hypot(lead.vx,lead.vy);
  const bool left_supports_merge=f.lane_change.left.available&&
      asmc_planner::lane_reference::HighwayLeftChangeSupportsMerge(*network,
          f.lane_change.left.source_link,f.lane_change.left.target_link);
  const bool merge_left=f.highway.approach&&left_supports_merge;
  const bool driving_allowed=(best==BehaviorType::KEEP||best==BehaviorType::FOLLOW||
      (best==BehaviorType::MERGE&&merge_left))&&
      f.ego.valid&&f.highway.cruising&&(!f.highway.approach||merge_left)&&
      f.lane_change.fresh&&!c.hard.planned_stop&&!c.hard.force_stop&&
      !c.hard.emergency_stop&&!f.risk.static_obstacle_ahead&&
      !f.risk.pedestrian_risk&&now>=lc_cooldown_until_;
  const bool observed_lead=f.surrounding.has_front_vehicle&&lead.valid;
  const bool lead_benefit=observed_lead&&lead.gap>0&&lead.gap<60&&
      std::isfinite(lead_speed)&&free-lead_speed>=policy.deficit_mps;
  // Preserve an EXISTING preparation through one short detection loss. This
  // does not create a request on an empty road or preserve a safety approval:
  // the offer/epoch/freshness checks below and Frenet's current certificate
  // still apply. A real new observation can withdraw the benefit immediately.
  const bool lead_grace=lc_active_&&!observed_lead&&
      lc_last_lead_observation_>0&&now>=lc_last_lead_observation_&&
      now-lc_last_lead_observation_<=std::min(.3,network->source_max_age);
  const bool eligible=driving_allowed&&(lead_benefit||lead_grace);
  if(driving_allowed&&lead_benefit)
    lc_last_lead_observation_=f.lane_change.observation_stamp;
  c.extra_flags["lane_change_lead_observation_grace"]=eligible&&lead_grace;
  const HighwayLaneChangeOffer* selected=nullptr;
  BehaviorType direction=BehaviorType::KEEP;
  if(eligible) {
    if(f.lane_change.left.available&&!c.hard.forbid_left_change&&
       std::min(free,f.lane_change.left.free_speed)>lead_speed) {
      selected=&f.lane_change.left;direction=BehaviorType::LEFT_CHANGE;
    } else if(!f.highway.approach&&f.lane_change.right.available&&!c.hard.forbid_right_change&&
       std::min(free,f.lane_change.right.free_speed)>lead_speed) {
      selected=&f.lane_change.right;direction=BehaviorType::RIGHT_CHANGE;
    }
  }
  if(lc_active_) {
    selected=lc_direction_==BehaviorType::LEFT_CHANGE?&f.lane_change.left:&f.lane_change.right;
    if(!eligible||!selected->available||(f.highway.approach&&lc_direction_!=BehaviorType::LEFT_CHANGE)||
       network->indexForLink(selected->source_link)!=network->indexForLink(lc_offer_.source_link)||
       network->indexForLink(selected->target_link)!=network->indexForLink(lc_offer_.target_link)||
       f.lane_change.reference_epoch!=lc_epoch_) {
      // No lateral maneuver was executed. Reconfirm the request instead of
      // applying the post-change cooldown to a refused preparation.
      lc_active_=false;lc_since_=-1;return;
    }
    best=lc_direction_;return;
  }
  if(!selected){lc_since_=-1;lc_leader_=-1;return;}
  const double acceleration=network->forward_accel;
  const double target_speed=std::min(free,selected->free_speed);
  const double change_distance=std::max(3.0*f.ego.speed,
      std::sqrt(6*selected->lateral_distance*std::max(free*free,f.ego.speed*f.ego.speed)/
          network->lanes.front()->options().max_lateral_accel));
  const double reserve=asmc_planner::lane_reference::HighwayLaneChangeReserve(*network,f.ego.speed,
      network->lanes.front()->options().vehicle.front_m+network->lanes.front()->options().vehicle.rear_m,
      direction==BehaviorType::LEFT_CHANGE&&left_supports_merge);
  const double useful=std::max(0.0,f.lane_change.remaining_m-reserve);
  const double gain=travelTime(useful,f.ego.speed,lead_speed,acceleration,network->comfortable_decel)-
      travelTime(useful,f.ego.speed,target_speed,acceleration,network->comfortable_decel);
  c.extra_values["lane_change_estimated_gain_s"]=gain;
  c.extra_values["lane_change_remaining_m"]=f.lane_change.remaining_m;
  if(useful<change_distance||gain<policy.min_gain_s){lc_since_=-1;return;}
  const bool new_observation=f.lane_change.observation_stamp>lc_observation_;
  if(f.lane_change.observation_stamp<lc_observation_||
     f.lane_change.observation_stamp-lc_observation_>network->source_max_age||
     lc_leader_!=lead.id||lc_epoch_!=f.lane_change.reference_epoch)lc_since_=-1;
  if(!new_observation)return;
  lc_observation_=f.lane_change.observation_stamp;lc_leader_=lead.id;lc_epoch_=f.lane_change.reference_epoch;
  if(lc_since_<0)lc_since_=lc_observation_;
  if(lc_observation_-lc_since_+1e-9<policy.confirmation_s)return;
  lc_offer_=*selected;lc_direction_=direction;lc_active_=true;lc_committed_=false;
  best=direction;
  if(direction==BehaviorType::LEFT_CHANGE)c.score.left_change=c.score.follow+gain;
  else c.score.right_change=c.score.follow+gain;
  c.debug.flags["optional_highway_lane_change"]=true;
}

}  // namespace behavior_planner

