#include "behavior_planner/context/msg_converter.hpp"

#include <cmath>
#include <ros/ros.h>

namespace behavior_planner {

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
  // stop_before_s cannot be represented as a Cartesian stop point yet.
  message.has_target_lane = false;
  message.has_target_d = false;
  message.has_stop_target = false;
  message.has_lead_vehicle = context.target.lead_vehicle_id >= 0;
  message.lead_vehicle_id = context.target.lead_vehicle_id;
  message.target_link_id = context.target.target_link_id;
  return message;
}

UrbanFeatureDebug MsgConverter::toDebugMsg(const UrbanFeature& feature) {
  UrbanFeatureDebug message;
  message.header.stamp = ros::Time(feature.timestamp);
  message.header.frame_id = "map";
  message.current_link_id = feature.route.current_link_id;
  message.route_required_link_id = feature.route.route_required_link_id;
  message.ego_s = feature.ego.s;
  message.ego_d = feature.ego.d;
  message.ego_speed = feature.ego.speed;
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
  message.traffic_light_id = feature.signal.traffic_light_id;
  message.traffic_light_state =
      static_cast<uint8_t>(feature.signal.light_state);
  message.pedestrian_risk = feature.risk.pedestrian_risk;
  message.static_obstacle_ahead = feature.risk.static_obstacle_ahead;
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
