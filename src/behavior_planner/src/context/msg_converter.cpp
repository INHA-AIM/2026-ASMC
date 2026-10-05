#include "behavior_planner/context/msg_converter.hpp"

#include <cmath>
#include <ros/ros.h>

namespace behavior_planner {
namespace {

constexpr double kCommandValueTolerance = 1e-3;

bool NearlyEqual(double lhs, double rhs) {
  return std::abs(lhs - rhs) <= kCommandValueTolerance;
}

}  // namespace

bool SameCommandIdentity(const asmc_msgs::BehaviorContext& lhs,
                         const asmc_msgs::BehaviorContext& rhs) {
  return lhs.valid == rhs.valid &&
         lhs.selected_behavior == rhs.selected_behavior &&
         lhs.force_stop == rhs.force_stop &&
         lhs.has_desired_speed == rhs.has_desired_speed &&
         (!lhs.has_desired_speed ||
          NearlyEqual(lhs.desired_speed, rhs.desired_speed)) &&
         lhs.has_max_speed == rhs.has_max_speed &&
         (!lhs.has_max_speed || NearlyEqual(lhs.max_speed, rhs.max_speed)) &&
         lhs.has_target_lane == rhs.has_target_lane &&
         (!lhs.has_target_lane ||
          lhs.target_lane_offset == rhs.target_lane_offset) &&
         lhs.has_target_d == rhs.has_target_d &&
         (!lhs.has_target_d || NearlyEqual(lhs.target_d, rhs.target_d)) &&
         lhs.has_stop_target == rhs.has_stop_target &&
         (!lhs.has_stop_target ||
          (NearlyEqual(lhs.stop_x, rhs.stop_x) &&
           NearlyEqual(lhs.stop_y, rhs.stop_y))) &&
         lhs.has_lead_vehicle == rhs.has_lead_vehicle &&
         (!lhs.has_lead_vehicle ||
          lhs.lead_vehicle_id == rhs.lead_vehicle_id) &&
         lhs.target_link_id == rhs.target_link_id;
}

asmc_msgs::BehaviorContext MsgConverter::toMsg(
    const BehaviorContextData& context, uint32_t command_id) {
  asmc_msgs::BehaviorContext message;
  message.header.stamp = ros::Time(context.timestamp);
  message.header.frame_id = "map";
  message.command_id = command_id;
  message.valid = std::isfinite(context.timestamp) && context.timestamp > 0.0;
  switch (context.selected_behavior) {
    case BehaviorType::KEEP: message.selected_behavior = asmc_msgs::BehaviorContext::KEEP; break;
    case BehaviorType::FOLLOW: message.selected_behavior = asmc_msgs::BehaviorContext::FOLLOW; break;
    case BehaviorType::LEFT_CHANGE: message.selected_behavior = asmc_msgs::BehaviorContext::LEFT_CHANGE; break;
    case BehaviorType::RIGHT_CHANGE: message.selected_behavior = asmc_msgs::BehaviorContext::RIGHT_CHANGE; break;
    case BehaviorType::STOP: message.selected_behavior = asmc_msgs::BehaviorContext::STOP; break;
    case BehaviorType::YIELD: message.selected_behavior = asmc_msgs::BehaviorContext::YIELD; break;
    case BehaviorType::EMERGENCY_STOP: message.selected_behavior = asmc_msgs::BehaviorContext::EMERGENCY_STOP; break;
    case BehaviorType::AVOID: message.selected_behavior = asmc_msgs::BehaviorContext::AVOID; break;
    default:
      message.valid = false;
      message.selected_behavior = asmc_msgs::BehaviorContext::KEEP;
      break;
  }
  message.force_stop = context.hard.force_stop;
  message.has_desired_speed = std::isfinite(context.target.desired_speed) && context.target.desired_speed >= 0.0;
  message.desired_speed = context.target.desired_speed;
  message.has_max_speed = std::isfinite(context.hard.max_speed) && context.hard.max_speed >= 0.0 && context.hard.max_speed < 1e9;
  message.max_speed = context.hard.max_speed;
  message.has_target_lane = false;
  message.has_target_d = false;
  message.has_stop_target = context.target.has_stop_target;
  message.stop_x = context.target.stop_x;
  message.stop_y = context.target.stop_y;
  const int object_id = context.selected_behavior == BehaviorType::AVOID
                            ? context.target.avoidance_object_id
                            : context.target.lead_vehicle_id;
  message.has_lead_vehicle = object_id >= 0;
  message.lead_vehicle_id = object_id;
  message.target_link_id = context.target.target_link_id;
  return message;
}

UrbanFeatureDebug MsgConverter::toDebugMsg(
    const UrbanFeature& feature, const BehaviorContextData& context) {
  UrbanFeatureDebug message;
  message.header.stamp = ros::Time(feature.timestamp);
  message.header.frame_id = "map";
  message.selected_behavior = static_cast<uint8_t>(context.selected_behavior);
  message.behavior_state = static_cast<uint8_t>(context.state);
  message.enable_keep = context.enable.keep;
  message.enable_follow = context.enable.follow;
  message.enable_left_change = context.enable.left_change;
  message.enable_right_change = context.enable.right_change;
  message.enable_avoid = context.enable.avoid;
  message.enable_stop = context.enable.stop;
  message.enable_emergency_stop = context.enable.emergency_stop;
  message.planned_stop = context.hard.planned_stop;
  message.force_stop = context.hard.force_stop;
  message.emergency_stop = context.hard.emergency_stop;
  message.forbid_left_change = context.hard.forbid_left_change;
  message.forbid_right_change = context.hard.forbid_right_change;
  message.desired_speed = context.target.desired_speed;
  message.has_max_speed = std::isfinite(context.hard.max_speed) &&
                          context.hard.max_speed < 1e9;
  message.max_speed = context.hard.max_speed;
  message.has_stop_target = context.target.has_stop_target;
  message.stop_x = context.target.stop_x;
  message.stop_y = context.target.stop_y;
  message.target_link_id = context.target.target_link_id;
  message.candidate_link_ids = context.target.candidate_link_ids;
  message.keep_score = context.score.keep;
  message.follow_score = context.score.follow;
  message.left_change_score = context.score.left_change;
  message.right_change_score = context.score.right_change;
  message.avoid_score = context.score.avoid;
  message.stop_score = context.score.stop;
  message.emergency_stop_score = context.score.emergency_stop;
  message.decision_reason = context.debug.selected_reason;
  message.hard_rule_reason = context.debug.hard_rule_reason;
  message.current_link_id = feature.route.current_link_id;
  message.route_required_link_id = feature.route.route_required_link_id;
  message.ego_s = feature.ego.s;
  message.ego_d = feature.ego.d;
  message.ego_speed = feature.ego.speed;
  message.vehicle_state_source = feature.ego.source;
  message.vehicle_state_valid = feature.ego.state_valid;
  message.has_front_vehicle = feature.surrounding.has_front_vehicle;
  message.front_gap = feature.surrounding.front_vehicle.gap;
  message.front_relative_speed =
      feature.surrounding.front_vehicle.relative_speed;
  message.front_ttc = feature.surrounding.front_vehicle.ttc;
  message.left_lane_safe = feature.surrounding.left_lane_safe;
  message.right_lane_safe = feature.surrounding.right_lane_safe;
  message.left_front_gap = feature.surrounding.left_front_gap;
  message.left_rear_gap = feature.surrounding.left_rear_gap;
  message.right_front_gap = feature.surrounding.right_front_gap;
  message.right_rear_gap = feature.surrounding.right_rear_gap;
  message.has_stop_line = feature.signal.has_stop_line;
  message.distance_to_stop_line = feature.signal.distance_to_stop_line;
  message.intersection_approach_active =
      context.hard.intersection_approach_active;
  message.intersection_approach_activation_distance =
      context.hard.intersection_approach_activation_distance;
  message.intersection_approach_speed_limit =
      context.hard.intersection_approach_speed_limit;
  message.traffic_light_id = feature.signal.traffic_light_id;
  message.traffic_light_state =
      static_cast<uint8_t>(feature.signal.light_state);
  message.traffic_light_raw_label = feature.signal.raw_label;
  message.traffic_light_recently_visible =
      feature.signal.recently_visible;
  message.has_required_movement = feature.signal.has_required_movement;
  message.required_movement =
      static_cast<uint8_t>(feature.signal.required_movement);
  message.related_signal = feature.signal.related_signal;
  message.signal_controlled_link_id = feature.signal.controlled_link_id;
  message.distance_to_signal_movement = feature.signal.distance_to_movement;
  message.pedestrian_risk = feature.risk.pedestrian_risk;
  message.static_obstacle_ahead = feature.risk.static_obstacle_ahead;
  message.avoidance_object_id = feature.risk.avoidance_object_id;
  message.emergency_risk = feature.risk.emergency_risk;
  message.min_ttc = feature.risk.min_ttc;
  message.predicted_front_ttc =
      feature.extra_values.count("predicted_front_ttc")
          ? feature.extra_values.at("predicted_front_ttc")
          : 1e9;
  message.required_deceleration =
      feature.extra_values.count("required_deceleration")
          ? feature.extra_values.at("required_deceleration")
          : 0.0;
  return message;
}

}  // namespace behavior_planner
