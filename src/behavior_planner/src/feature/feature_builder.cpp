#include "behavior_planner/feature/feature_builder.hpp"

#include <algorithm>
#include <cmath>

namespace behavior_planner {

UrbanFeature FeatureBuilder::build(
    const EgoState& ego, const PerceptionObjects& objects,
    const RouteManager& route_manager,
    const TrafficLightProvider& traffic_light_provider) {
  UrbanFeature feature;
  feature.signal.raw_label = traffic_light_provider.getRawLabel();
  feature.timestamp = ego.stamp;
  feature.frame_count = ++frame_;
  feature.ego.x = ego.x;
  feature.ego.y = ego.y;
  feature.ego.yaw = ego.yaw;
  feature.ego.speed = ego.speed;
  feature.ego.source = ego.source;
  feature.ego.state_valid = ego.valid;

  const bool route_projection_valid = route_manager.projectEgo(
      ego.x, ego.y, &feature.ego.s, &feature.ego.d,
      &feature.route.route_required_link_id);
  LinkProjection map_projection;
  const bool map_match_valid =
      route_manager.matchMapLink(ego.x, ego.y, ego.yaw, &map_projection);
  feature.ego.valid = route_projection_valid && map_match_valid;
  if (map_match_valid) {
    feature.route.current_link_id = map_projection.link_id;
    feature.ego.map_s = map_projection.link_s;
  }

  LinkInfo current_link;
  if (route_manager.getCurrentLinkInfo(feature.route.current_link_id,
                                       &current_link)) {
    feature.route.current_link_valid = true;
    feature.route.speed_limit = current_link.max_speed;
    feature.route.left_target_link_id = current_link.left_dst_link_id;
    feature.route.right_target_link_id = current_link.right_dst_link_id;
    feature.route.lane_change_left_allowed =
        current_link.can_move_left && !current_link.left_dst_link_id.empty();
    feature.route.lane_change_right_allowed =
        current_link.can_move_right && !current_link.right_dst_link_id.empty();
    Point2D target;
    if (feature.route.lane_change_left_allowed &&
        route_manager.getLinkTargetPose(feature.route.left_target_link_id,
                                        ego.x, ego.y, 20.0, &target,
                                        &feature.route.left_target_yaw)) {
      feature.route.left_target_pose_valid = true;
      feature.route.left_target_x = target.x;
      feature.route.left_target_y = target.y;
    }
    if (feature.route.lane_change_right_allowed &&
        route_manager.getLinkTargetPose(feature.route.right_target_link_id,
                                        ego.x, ego.y, 20.0, &target,
                                        &feature.route.right_target_yaw)) {
      feature.route.right_target_pose_valid = true;
      feature.route.right_target_x = target.x;
      feature.route.right_target_y = target.y;
    }
  }

  RouteSignalMovement signal_movement;
  if (route_manager.findNextSignalMovement(feature.ego.s, 200.0,
                                           &signal_movement)) {
    feature.signal.has_required_movement = true;
    feature.signal.required_movement = signal_movement.movement;
    feature.signal.related_signal = signal_movement.related_signal;
    feature.signal.controlled_link_id = signal_movement.link_id;
    feature.signal.movement_link_s = signal_movement.route_s;
    feature.signal.distance_to_movement = signal_movement.distance;
  }

  StopLineInfo stop_line;
  double stop_distance = 1e9;
  if (route_manager.findNextMapStopLine(
          feature.route.current_link_id, feature.ego.map_s, 200.0,
          &stop_line, &stop_distance)) {
    feature.signal.has_stop_line = true;
    feature.signal.stop_line_valid = true;
    feature.signal.stop_line_s = feature.ego.s + stop_distance;
    feature.signal.distance_to_stop_line = stop_distance;
    feature.signal.stop_line_x = stop_line.point.x;
    feature.signal.stop_line_y = stop_line.point.y;
    // VehicleState x/y is the rear-axle GPS reference.  Move the commanded
    // stop target backward along the route so the vehicle front, plus a small
    // safety clearance, remains before the physical stop line.  Converting
    // route s back to x/y also keeps the offset correct on curved approaches.
    Point2D adjusted_stop;
    const double adjusted_stop_s =
        std::max(0.0, feature.signal.stop_line_s - stop_target_offset_m_);
    feature.signal.stop_target_s = adjusted_stop_s;
    if (stop_target_offset_m_ > 0.0 &&
        route_manager.getRoutePose(adjusted_stop_s, &adjusted_stop,
                                   &feature.signal.stop_line_yaw)) {
      feature.signal.stop_line_x = adjusted_stop.x;
      feature.signal.stop_line_y = adjusted_stop.y;
    } else {
      Point2D stop_pose;
      route_manager.getRoutePose(feature.signal.stop_line_s, &stop_pose,
                                 &feature.signal.stop_line_yaw);
    }
    feature.signal.traffic_light_id = stop_line.traffic_light_id;
    feature.signal.light_state = traffic_light_provider.getState(
        stop_line.traffic_light_id, feature.signal.required_movement);
  }

  std::vector<ObjectFeature> projected_objects;
  projected_objects.reserve(objects.vehicles.size() + objects.pedestrians.size() +
                            objects.obstacles.size());
  for (const auto& object : objects.vehicles) {
    projected_objects.push_back(
        projector_.project(object, feature.ego, route_manager));
  }
  for (const auto& object : objects.pedestrians) {
    projected_objects.push_back(
        projector_.project(object, feature.ego, route_manager));
  }
  for (const auto& object : objects.obstacles) {
    projected_objects.push_back(
        projector_.project(object, feature.ego, route_manager));
  }

  constexpr double kLaneWidth = 3.5;
  const double front_lookahead = follow_target_.max_distance;
  double best_lead_score = 1e9;
  double best_static_obstacle_gap = 1e9;
  for (ObjectFeature& object : projected_objects) {
    if (!object.valid) continue;
    const double relative_d = object.d - feature.ego.d;
    const bool left_lane =
        (!feature.route.left_target_link_id.empty() &&
         object.link_id == feature.route.left_target_link_id) ||
        std::abs(relative_d - kLaneWidth) < kLaneWidth * 0.5;
    const bool right_lane =
        (!feature.route.right_target_link_id.empty() &&
         object.link_id == feature.route.right_target_link_id) ||
        std::abs(relative_d + kLaneWidth) < kLaneWidth * 0.5;

    // Route-s is appropriate on the global route, but becomes ambiguous when
    // Ego and an object are on an off-route lane near another route segment.
    // Same/adjacent physical links share direction and comparable origins, so
    // use their map-link arc length for longitudinal front/rear classification.
    const bool physical_neighbor_lane =
        object.link_id == feature.route.current_link_id ||
        object.link_id == feature.route.left_target_link_id ||
        object.link_id == feature.route.right_target_link_id;
    if (physical_neighbor_lane) {
      object.gap = object.link_s - feature.ego.map_s;
      object.ttc = object.gap > 0.0 && object.relative_speed > 0.0
                       ? object.gap / object.relative_speed
                       : 1e9;
    }

    const double dx = object.x - feature.ego.x;
    const double dy = object.y - feature.ego.y;
    const double cos_yaw = std::cos(feature.ego.yaw);
    const double sin_yaw = std::sin(feature.ego.yaw);
    const double ego_longitudinal = cos_yaw * dx + sin_yaw * dy;
    const double euclidean_distance = std::hypot(dx, dy);
    const bool same_direction =
        std::cos(object.yaw - feature.ego.yaw) > 0.5;
    double follow_topology_gap = 1e9;
    const bool follow_forward_in_topology = route_manager.forwardDistance(
        feature.route.current_link_id, feature.ego.map_s, object.link_id,
        object.link_s, front_lookahead, &follow_topology_gap);
    const bool follow_geometry_front_gate =
        ego_longitudinal > -2.0 && euclidean_distance <= front_lookahead &&
        std::abs(object.d) <= follow_target_.max_lateral_offset &&
        same_direction;
    const bool current_lane_front =
        follow_forward_in_topology && follow_geometry_front_gate;

    // A static obstacle has no meaningful travel heading. Applying the
    // vehicle same-direction gate to MORAI obstacle heading (commonly zero)
    // makes a real obstacle disappear on north/south roads. Use route
    // topology and geometry only, with a horizon no shorter than Frenet's
    // collision lookahead.
    const double avoid_route_gap = object.s - feature.ego.s;
    // Gate the nearest obstacle footprint, not only its center. A wide object
    // can overlap the route even when its center is just outside the nominal
    // lateral acquisition band. Project the oriented box half extents onto
    // the route normal represented by Frenet d.
    double route_yaw = feature.ego.yaw;
    Point2D obstacle_route_pose;
    route_manager.getRoutePose(object.s, &obstacle_route_pose, &route_yaw);
    const double obstacle_yaw_delta = object.yaw - route_yaw;
    const double obstacle_lateral_half_extent =
        0.5 * std::abs(object.length * std::sin(obstacle_yaw_delta)) +
        0.5 * std::abs(object.width * std::cos(obstacle_yaw_delta));
    const double obstacle_longitudinal_half_extent =
        0.5 * std::abs(object.length * std::cos(obstacle_yaw_delta)) +
        0.5 * std::abs(object.width * std::sin(obstacle_yaw_delta));
    const double obstacle_nearest_lateral_distance =
        std::max(0.0, std::abs(object.d) - obstacle_lateral_half_extent);
    const double path_corridor_half_width = std::min(
        avoid_target_.max_lateral_offset,
        0.5 * avoid_target_.ego_width + avoid_target_.path_margin_m);
    const double obstacle_nearest_forward_gap =
        avoid_route_gap - obstacle_longitudinal_half_extent;
    const bool obstacle_blocks_route_corridor =
        avoid_route_gap > 0.0 &&
        obstacle_nearest_forward_gap <= avoid_target_.max_distance &&
        ego_longitudinal > -2.0 &&
        euclidean_distance <=
            avoid_target_.max_distance + obstacle_longitudinal_half_extent &&
        obstacle_nearest_lateral_distance <=
            path_corridor_half_width;

    if (current_lane_front) {
      object.gap = follow_topology_gap;
      object.ttc = object.gap > 0.0 && object.relative_speed > 0.0
                       ? object.gap / object.relative_speed
                       : 1e9;
    }

    if (object.type == ObjectType::VEHICLE && current_lane_front &&
        object.gap > 0.0) {
      // A small continuity bonus prevents lead selection from flickering when
      // two vehicles have nearly equal gaps around a link boundary.
      const double lead_score =
          object.gap - (object.id == last_lead_vehicle_id_ ? 4.0 : 0.0);
      if (lead_score < best_lead_score) {
        best_lead_score = lead_score;
        feature.surrounding.has_front_vehicle = true;
        feature.surrounding.front_vehicle = object;
      }
    }
    if (object.type == ObjectType::PEDESTRIAN && object.gap > 0.0 &&
        object.gap < safety_.pedestrian_stop_distance &&
        std::abs(relative_d) < kLaneWidth) {
      feature.risk.pedestrian_risk = true;
      feature.risk.nearest_pedestrian_s =
          std::min(feature.risk.nearest_pedestrian_s, object.s);
    }
    const bool avoidable_obstacle =
        object.type == ObjectType::STATIC_OBSTACLE ||
        object.type == ObjectType::UNKNOWN;
    if (avoidable_obstacle && obstacle_blocks_route_corridor) {
      object.gap = std::max(0.0, obstacle_nearest_forward_gap);
      if (!feature.risk.static_obstacle_ahead ||
          object.gap < best_static_obstacle_gap) {
        feature.risk.static_obstacle_ahead = true;
        feature.risk.avoidance_object_id = object.id;
        feature.risk.nearest_obstacle_s = object.s;
        feature.risk.nearest_obstacle_length =
            2.0 * obstacle_longitudinal_half_extent;
        best_static_obstacle_gap = object.gap;
      }
    }
    // Do not feed the selected static AVOID target into the generic TTC AEB.
    // Otherwise the same object first selects AVOID, then preempts it with
    // E-STOP as TTC drops; after stopping TTC becomes infinite, E-STOP
    // releases, and the cycle repeats. Static-object path safety belongs to
    // Frenet's candidate collision check and SAFE_STOP fallback. Behavior's
    // TTC AEB remains reserved for a moving lead vehicle; pedestrian risk is
    // handled independently by the hard-rule stop above.
    if (object.type == ObjectType::VEHICLE && current_lane_front &&
        object.gap > 0.0) {
      feature.risk.min_ttc = std::min(feature.risk.min_ttc, object.ttc);
    }

    if (left_lane) {
      if (object.gap >= 0.0) {
        feature.surrounding.left_front_gap =
            std::min(feature.surrounding.left_front_gap, object.gap);
      } else {
        feature.surrounding.left_rear_gap =
            std::min(feature.surrounding.left_rear_gap, -object.gap);
      }
    }
    if (right_lane) {
      if (object.gap >= 0.0) {
        feature.surrounding.right_front_gap =
            std::min(feature.surrounding.right_front_gap, object.gap);
      } else {
        feature.surrounding.right_rear_gap =
            std::min(feature.surrounding.right_rear_gap, -object.gap);
      }
    }
  }

  last_lead_vehicle_id_ = feature.surrounding.has_front_vehicle
                              ? feature.surrounding.front_vehicle.id
                              : -1;

  feature.surrounding.left_lane_safe =
      feature.route.lane_change_left_allowed &&
      feature.surrounding.left_front_gap >= 12.0 &&
      feature.surrounding.left_rear_gap >= 15.0;
  feature.surrounding.right_lane_safe =
      feature.route.lane_change_right_allowed &&
      feature.surrounding.right_front_gap >= 12.0 &&
      feature.surrounding.right_rear_gap >= 15.0;
  double predicted_front_ttc = 1e9;
  double required_deceleration = 0.0;
  if (feature.surrounding.has_front_vehicle) {
    const ObjectFeature& lead = feature.surrounding.front_vehicle;
    const double closing_speed =
        std::max(0.0, feature.ego.speed - lead.speed);
    const double available_braking_distance =
        std::max(0.5, lead.gap - safety_.front_min_gap);
    required_deceleration =
        closing_speed * closing_speed / (2.0 * available_braking_distance);

    constexpr double kPredictionHorizon = 1.0;
    const double predicted_lead_speed =
        std::max(0.0, lead.speed +
                          std::min(0.0, lead.accel) * kPredictionHorizon);
    const double predicted_closing_speed =
        feature.ego.speed - predicted_lead_speed;
    if (predicted_closing_speed > 0.0) {
      predicted_front_ttc = lead.gap / predicted_closing_speed;
      feature.risk.min_ttc =
          std::min(feature.risk.min_ttc, predicted_front_ttc);
    }
  }
  feature.extra_values["predicted_front_ttc"] = predicted_front_ttc;
  feature.extra_values["required_deceleration"] = required_deceleration;
  feature.risk.emergency_risk =
      feature.risk.min_ttc < safety_.emergency_ttc_enter ||
      predicted_front_ttc < safety_.emergency_predicted_ttc ||
      required_deceleration > safety_.emergency_required_decel;
  return feature;
}

}  // namespace behavior_planner
