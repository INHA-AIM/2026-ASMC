#include "Visualization/Behavior/behavior_visualization.hpp"

#include <cmath>
#include <sstream>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>

namespace behavior_planner {
namespace {

const char* BehaviorName(BehaviorType behavior) {
  switch (behavior) {
    case BehaviorType::KEEP: return "KEEP";
    case BehaviorType::FOLLOW: return "FOLLOW";
    case BehaviorType::LEFT_CHANGE: return "LEFT_CHANGE";
    case BehaviorType::RIGHT_CHANGE: return "RIGHT_CHANGE";
    case BehaviorType::STOP: return "STOP";
    case BehaviorType::YIELD: return "YIELD";
    case BehaviorType::EMERGENCY_STOP: return "EMERGENCY_STOP";
    case BehaviorType::AVOID: return "AVOID";
    case BehaviorType::MERGE: return "MERGE";
  }
  return "UNKNOWN";
}

const char* MovementName(SignalMovement movement) {
  switch (movement) {
    case SignalMovement::STRAIGHT: return "STRAIGHT";
    case SignalMovement::LEFT: return "LEFT";
    case SignalMovement::LEFT_UNPROTECTED: return "LEFT_UNPROTECTED";
    case SignalMovement::RIGHT_UNPROTECTED: return "RIGHT_UNPROTECTED";
    case SignalMovement::UTURN: return "UTURN";
    default: return "UNKNOWN";
  }
}

const char* LightName(TrafficLightState state) {
  switch (state) {
    case TrafficLightState::RED: return "RED";
    case TrafficLightState::YELLOW: return "YELLOW";
    case TrafficLightState::GREEN: return "GREEN";
    default: return "UNKNOWN";
  }
}

const char* VehicleStateSourceName(uint8_t source) {
  switch (source) {
    case 1: return "GPS_IMU";
    case 2: return "EGO_STATUS";
    default: return "UNKNOWN";
  }
}

void SetLightColor(TrafficLightState state,
                   std_msgs::ColorRGBA* color) {
  color->a = 1.0;
  if (state == TrafficLightState::RED) color->r = 1.0;
  else if (state == TrafficLightState::YELLOW) {
    color->r = 1.0;
    color->g = 0.85;
  } else if (state == TrafficLightState::GREEN) color->g = 1.0;
  else color->r = color->g = color->b = 0.55;
}

void AppendLinkPoints(const LinkInfo& link, double z,
                      visualization_msgs::Marker* marker) {
  for (const Point2D& point : link.centerline) {
    geometry_msgs::Point p;
    p.x = point.x;
    p.y = point.y;
    p.z = z;
    marker->points.push_back(p);
  }
}

}  // namespace

void BehaviorVisualization::publish(const UrbanFeature& feature,
                                    const BehaviorContextData& context,
                                    const RouteManager& route,
                                    const MGeoLoader& map,
                                    const TrafficLightProvider&) const {
  if (!publisher_) return;
  const ros::Time stamp = ros::Time::now();
  visualization_msgs::MarkerArray array;

  visualization_msgs::Marker clear;
  clear.action = visualization_msgs::Marker::DELETEALL;
  array.markers.push_back(clear);

  visualization_msgs::Marker route_line;
  route_line.header.frame_id = "map";
  route_line.header.stamp = stamp;
  route_line.ns = "behavior_route";
  route_line.id = 0;
  route_line.type = visualization_msgs::Marker::LINE_STRIP;
  route_line.action = visualization_msgs::Marker::ADD;
  route_line.pose.orientation.w = 1.0;
  route_line.scale.x = 0.35;
  route_line.color.r = 0.1;
  route_line.color.g = 0.8;
  route_line.color.b = 1.0;
  route_line.color.a = 1.0;
  for (const Point2D& point : route.referenceLine()) {
    geometry_msgs::Point ros_point;
    ros_point.x = point.x;
    ros_point.y = point.y;
    ros_point.z = 0.15;
    route_line.points.push_back(ros_point);
  }
  array.markers.push_back(route_line);

  visualization_msgs::Marker ego;
  ego.header = route_line.header;
  ego.ns = "behavior_ego";
  ego.id = 0;
  ego.type = visualization_msgs::Marker::ARROW;
  ego.action = visualization_msgs::Marker::ADD;
  ego.pose.position.x = feature.ego.x;
  ego.pose.position.y = feature.ego.y;
  ego.pose.position.z = 0.4;
  ego.pose.orientation.z = std::sin(feature.ego.yaw * 0.5);
  ego.pose.orientation.w = std::cos(feature.ego.yaw * 0.5);
  ego.scale.x = 3.0;
  ego.scale.y = 1.0;
  ego.scale.z = 1.0;
  ego.color.g = 1.0;
  ego.color.r = feature.ego.state_valid ? 0.0 : 1.0;
  ego.color.a = 1.0;
  array.markers.push_back(ego);

  visualization_msgs::Marker text;
  text.header = route_line.header;
  text.ns = "behavior_text";
  text.id = 0;
  text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text.action = visualization_msgs::Marker::ADD;
  text.pose.position.x = feature.ego.x;
  text.pose.position.y = feature.ego.y;
  text.pose.position.z = 3.0;
  text.pose.orientation.w = 1.0;
  text.scale.z = 1.2;
  text.color.r = 1.0;
  text.color.g = 1.0;
  text.color.a = 1.0;
  std::ostringstream summary;
  summary << BehaviorName(context.selected_behavior)
          << "  v=" << feature.ego.speed
          << " -> " << context.target.desired_speed << " m/s\n"
          << "state=/vehicle/state "
          << VehicleStateSourceName(feature.ego.source)
          << " valid=" << (feature.ego.state_valid ? "true" : "false") << "\n"
          << "link=" << feature.route.current_link_id;
  if (feature.signal.has_required_movement) {
    summary << "\nsignal=" << LightName(feature.signal.light_state)
            << " for " << MovementName(feature.signal.required_movement)
            << " (" << feature.signal.related_signal << ")";
  }
  if (feature.signal.has_stop_line) {
    summary << "  stop=" << feature.signal.distance_to_stop_line << "m";
  }
  if (context.hard.planned_stop) summary << "  PLANNED_STOP";
  if (context.hard.force_stop) summary << "  FORCE_STOP";
  text.text = summary.str();
  array.markers.push_back(text);

  int signal_marker_id = 0;
  for (const StopLineInfo& map_stop : map.stopLines()) {
    if (!map_stop.valid || map_stop.traffic_light_id.empty()) continue;
    visualization_msgs::Marker signal;
    signal.header = route_line.header;
    signal.ns = "behavior_map_signals";
    signal.id = signal_marker_id++;
    signal.type = visualization_msgs::Marker::SPHERE;
    signal.action = visualization_msgs::Marker::ADD;
    signal.pose.position.x = map_stop.point.x;
    signal.pose.position.y = map_stop.point.y;
    signal.pose.position.z = 0.35;
    signal.pose.orientation.w = 1.0;
    signal.scale.x = 0.8;
    signal.scale.y = 0.8;
    signal.scale.z = 0.8;
    // The camera topic has no signal ID. Only the active route signal can be
    // assigned its observed state; all other map signals stay gray.
    const bool active = map_stop.traffic_light_id == feature.signal.traffic_light_id;
    SetLightColor(active ? feature.signal.light_state
                         : TrafficLightState::UNKNOWN,
                  &signal.color);
    signal.color.a = active ? 1.0 : 0.35;
    array.markers.push_back(signal);
  }

  if (feature.signal.has_stop_line) {
    visualization_msgs::Marker stop;
    stop.header = route_line.header;
    stop.ns = "behavior_next_stop_line";
    stop.id = 0;
    stop.type = visualization_msgs::Marker::SPHERE;
    stop.action = visualization_msgs::Marker::ADD;
    stop.pose.position.x = feature.signal.stop_line_x;
    stop.pose.position.y = feature.signal.stop_line_y;
    stop.pose.position.z = 0.6;
    stop.pose.orientation.w = 1.0;
    stop.scale.x = 1.4;
    stop.scale.y = 1.4;
    stop.scale.z = 1.4;
    SetLightColor(feature.signal.light_state, &stop.color);
    array.markers.push_back(stop);

    visualization_msgs::Marker stop_text = text;
    stop_text.ns = "behavior_next_stop_text";
    stop_text.pose.position.x = feature.signal.stop_line_x;
    stop_text.pose.position.y = feature.signal.stop_line_y;
    stop_text.pose.position.z = 2.0;
    stop_text.scale.z = 0.8;
    stop_text.text = std::string("STOP LINE  ") +
                     LightName(feature.signal.light_state) + " / " +
                     MovementName(feature.signal.required_movement) + "\n" +
                     feature.signal.traffic_light_id;
    array.markers.push_back(stop_text);

    visualization_msgs::Marker connector;
    connector.header = route_line.header;
    connector.ns = "behavior_stop_distance";
    connector.id = 0;
    connector.type = visualization_msgs::Marker::LINE_STRIP;
    connector.action = visualization_msgs::Marker::ADD;
    connector.pose.orientation.w = 1.0;
    connector.scale.x = 0.12;
    connector.color = stop.color;
    geometry_msgs::Point ego_point;
    ego_point.x = feature.ego.x;
    ego_point.y = feature.ego.y;
    ego_point.z = 0.25;
    geometry_msgs::Point stop_point = ego_point;
    stop_point.x = feature.signal.stop_line_x;
    stop_point.y = feature.signal.stop_line_y;
    connector.points = {ego_point, stop_point};
    array.markers.push_back(connector);
  }

  LinkInfo controlled_link;
  if (feature.signal.has_required_movement &&
      map.getLink(feature.signal.controlled_link_id, &controlled_link)) {
    visualization_msgs::Marker link = route_line;
    link.ns = "behavior_signal_controlled_link";
    link.id = 0;
    link.points.clear();
    link.scale.x = 0.75;
    link.color.r = 1.0;
    link.color.g = 0.5;
    link.color.b = 0.0;
    link.color.a = 0.9;
    AppendLinkPoints(controlled_link, 0.3, &link);
    array.markers.push_back(link);
  }

  LinkInfo target_link;
  if (!context.target.target_link_id.empty() &&
      map.getLink(context.target.target_link_id, &target_link)) {
    visualization_msgs::Marker link = route_line;
    link.ns = "behavior_target_link";
    link.id = 0;
    link.points.clear();
    link.scale.x = 0.65;
    link.color.r = 1.0;
    link.color.b = 0.85;
    link.color.a = 0.9;
    AppendLinkPoints(target_link, 0.35, &link);
    array.markers.push_back(link);
  }

  if (feature.surrounding.has_front_vehicle) {
    const ObjectFeature& front = feature.surrounding.front_vehicle;
    visualization_msgs::Marker lead;
    lead.header = route_line.header;
    lead.ns = "behavior_lead_vehicle";
    lead.id = 0;
    lead.type = visualization_msgs::Marker::CUBE;
    lead.action = visualization_msgs::Marker::ADD;
    lead.pose.position.x = front.x;
    lead.pose.position.y = front.y;
    lead.pose.position.z = 0.8;
    lead.pose.orientation.z = std::sin(front.yaw * 0.5);
    lead.pose.orientation.w = std::cos(front.yaw * 0.5);
    lead.scale.x = front.length > 0.0 ? front.length : 4.5;
    lead.scale.y = front.width > 0.0 ? front.width : 1.8;
    lead.scale.z = 1.5;
    lead.color.r = 1.0;
    lead.color.g = 0.55;
    lead.color.a = 0.75;
    array.markers.push_back(lead);
  }

  publisher_.publish(array);
}

}  // namespace behavior_planner

