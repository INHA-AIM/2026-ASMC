#include "behavior_planner/feature/feature_builder.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace behavior_planner {
namespace {

bool IsStaticAvoidCandidate(const ObjectFeature& object) {
  const bool static_semantics =
      object.type == ObjectType::STATIC_OBSTACLE ||
      object.type == ObjectType::UNKNOWN;
  const bool stationary_motion_state =
      object.motion_state == MotionState::UNKNOWN ||
      object.motion_state == MotionState::LOW_THREAT;
  return static_semantics && stationary_motion_state;
}

bool IsLongitudinalLeadCandidate(const ObjectFeature& object) {
  if (object.type == ObjectType::VEHICLE) return true;
  return object.type == ObjectType::UNKNOWN &&
         (object.motion_state == MotionState::DYNAMIC_PENDING ||
          object.motion_state == MotionState::DYNAMIC);
}

bool ObbsOverlap(const ObjectFeature& lhs, const ObjectFeature& rhs) {
  if (!std::isfinite(lhs.x) || !std::isfinite(lhs.y) ||
      !std::isfinite(lhs.yaw) || !std::isfinite(lhs.length) ||
      !std::isfinite(lhs.width) || !std::isfinite(rhs.x) ||
      !std::isfinite(rhs.y) || !std::isfinite(rhs.yaw) ||
      !std::isfinite(rhs.length) || !std::isfinite(rhs.width) ||
      lhs.length <= 0.0 || lhs.width <= 0.0 || rhs.length <= 0.0 ||
      rhs.width <= 0.0) {
    return false;
  }

  const double lhs_cos = std::cos(lhs.yaw);
  const double lhs_sin = std::sin(lhs.yaw);
  const double rhs_cos = std::cos(rhs.yaw);
  const double rhs_sin = std::sin(rhs.yaw);
  const double axes[4][2] = {{lhs_cos, lhs_sin},
                             {-lhs_sin, lhs_cos},
                             {rhs_cos, rhs_sin},
                             {-rhs_sin, rhs_cos}};
  const double dx = rhs.x - lhs.x;
  const double dy = rhs.y - lhs.y;
  constexpr double kOverlapToleranceM = 0.05;
  for (const auto& axis : axes) {
    const double center_distance =
        std::abs(dx * axis[0] + dy * axis[1]);
    const double lhs_radius =
        0.5 * lhs.length * std::abs(lhs_cos * axis[0] + lhs_sin * axis[1]) +
        0.5 * lhs.width * std::abs(-lhs_sin * axis[0] + lhs_cos * axis[1]);
    const double rhs_radius =
        0.5 * rhs.length * std::abs(rhs_cos * axis[0] + rhs_sin * axis[1]) +
        0.5 * rhs.width * std::abs(-rhs_sin * axis[0] + rhs_cos * axis[1]);
    if (center_distance > lhs_radius + rhs_radius + kOverlapToleranceM) {
      return false;
    }
  }
  return true;
}

bool IsSpatiallySameTrack(const ObjectFeature& observation,
                          const ObjectFeature& reference,
                          const AvoidTargetConfig& config) {
  const double observation_long_side =
      std::max(observation.length, observation.width);
  const double observation_short_side =
      std::min(observation.length, observation.width);
  const double reference_long_side =
      std::max(reference.length, reference.width);
  const double reference_short_side =
      std::min(reference.length, reference.width);
  return std::abs(observation.s - reference.s) <=
             config.reconnect_max_longitudinal_delta_m &&
         std::abs(observation.d - reference.d) <=
             config.reconnect_max_lateral_delta_m &&
         std::abs(observation_long_side - reference_long_side) <=
             config.reconnect_max_size_delta_m &&
         std::abs(observation_short_side - reference_short_side) <=
             config.reconnect_max_size_delta_m &&
         ObbsOverlap(observation, reference);
}

// LiDAR가 한 차량의 반사점을 둘로 나누면, 정상 선두 OBB 옆에 작은
// UNKNOWN/LOW_THREAT OBB가 동시에 생길 수 있다. 단순 중심거리만 쓰면
// 선두 바로 옆의 실제 장애물을 삼킬 수 있으므로 다음을 모두 요구한다.
//   1. fragment가 정적 AVOID semantic이고,
//   2. 면적이 선두 OBB의 절반 이하이며,
//   3. 선두 OBB의 두 축에서 실제 점유가 겹친다.
// 높이는 AVOID corridor 판정과 마찬가지로 사용하지 않는다.
bool IsSmallOverlappingLeadFragment(const ObjectFeature& fragment,
                                    const ObjectFeature& lead) {
  if (!fragment.valid || !lead.valid || fragment.id == lead.id ||
      !IsStaticAvoidCandidate(fragment)) {
    return false;
  }
  if (!std::isfinite(fragment.length) || !std::isfinite(fragment.width) ||
      !std::isfinite(lead.length) || !std::isfinite(lead.width) ||
      fragment.length <= 0.0 || fragment.width <= 0.0 ||
      lead.length <= 0.0 || lead.width <= 0.0) {
    return false;
  }

  const double fragment_area = fragment.length * fragment.width;
  const double lead_area = lead.length * lead.width;
  constexpr double kMaxFragmentAreaRatio = 0.50;
  if (fragment_area > kMaxFragmentAreaRatio * lead_area) return false;

  const double dx = fragment.x - lead.x;
  const double dy = fragment.y - lead.y;
  const double cosine = std::cos(lead.yaw);
  const double sine = std::sin(lead.yaw);
  const double local_x = cosine * dx + sine * dy;
  const double local_y = -sine * dx + cosine * dy;
  const double yaw_delta = fragment.yaw - lead.yaw;
  const double delta_cosine = std::abs(std::cos(yaw_delta));
  const double delta_sine = std::abs(std::sin(yaw_delta));
  const double fragment_half_x =
      0.5 * (fragment.length * delta_cosine +
             fragment.width * delta_sine);
  const double fragment_half_y =
      0.5 * (fragment.length * delta_sine +
             fragment.width * delta_cosine);

  // 센서 양자화 때문에 두 OBB가 수 cm 벌어지는 경우만 허용한다. 별개의
  // 물체를 묶는 일반 근접거리 조건이 아니라 사실상 점유 중첩 조건이다.
  constexpr double kOverlapToleranceM = 0.05;
  return std::abs(local_x) <=
             0.5 * lead.length + fragment_half_x + kOverlapToleranceM &&
         std::abs(local_y) <=
             0.5 * lead.width + fragment_half_y + kOverlapToleranceM;
}

}  // namespace

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

  feature.roundabout.enabled = roundabout_enabled_;
  if (roundabout_enabled_) {
    StopLineInfo stop;
    if (route_manager.findStopLineByNode(roundabout_stop_node_, &stop)) {
      feature.roundabout.source_link_id=roundabout_source_;
      feature.roundabout.target_link_id=roundabout_target_;
      feature.roundabout.distance_to_stop=stop.route_s-feature.ego.s;
      // Begin early enough to stop from the measured speed, then hold a
      // constant 25 km/h maneuver ceiling throughout the transaction.
      double preview=std::max(45.0,feature.ego.speed*feature.ego.speed/(2*1.5)+4);
      feature.roundabout.approach=feature.roundabout.distance_to_stop>=-.5 &&
          feature.roundabout.distance_to_stop<=preview;
    }
  }

  StopLineInfo stop_line;
  double stop_distance = 1e9;
  if (route_manager.findNextSignalizedStopLine(
          feature.ego.s, 200.0, &stop_line, &stop_distance)) {
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
    feature.signal.recently_visible =
        traffic_light_provider.hasRecentDetection(stop_line.traffic_light_id);
  }

  StopLineInfo passed_stop_line;
  double distance_since_stop_line = 1e9;
  if (route_manager.findPreviousSignalizedStopLine(
          feature.ego.s, &passed_stop_line, &distance_since_stop_line)) {
    feature.signal.has_passed_stop_line = true;
    feature.signal.distance_since_stop_line = distance_since_stop_line;
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

  // AVOID overlap history must follow the physical object, not a short-lived
  // tracker ID. Reconnect only a one-to-one spatial match; if an old state or
  // a new observation has multiple possible partners, leave it unmatched.
  // This keeps boundary hysteresis through benign LiDAR re-identification
  // without attaching a committed target to the wrong nearby obstacle.
  std::unordered_map<int, int> current_id_counts;
  for (const ObjectFeature& object : projected_objects) {
    if (object.valid) ++current_id_counts[object.id];
  }

  // 먼저 현재 FOLLOW 선두를 확정한다. 기존 단일 pass에서는 정적 OBB를
  // AVOID 후보로 평가한 뒤에야 선두가 정해져, 같은 frame에 선두 본체와
  // fragment가 함께 들어온 경우 둘의 소유 관계를 판단할 수 없었다.
  constexpr double kLaneWidth = 3.5;
  const double front_lookahead = follow_target_.max_distance;
  double best_lead_score = 1e9;
  for (ObjectFeature& object : projected_objects) {
    if (!object.valid || !IsLongitudinalLeadCandidate(object)) continue;

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
    const double ego_longitudinal =
        std::cos(feature.ego.yaw) * dx + std::sin(feature.ego.yaw) * dy;
    const double euclidean_distance = std::hypot(dx, dy);
    const double object_motion_speed = std::hypot(object.vx, object.vy);
    const double object_motion_yaw = object_motion_speed > 0.5
                                         ? std::atan2(object.vy, object.vx)
                                         : object.yaw;
    const bool same_direction =
        std::cos(object_motion_yaw - feature.ego.yaw) > 0.5;
    double topology_gap = 1e9;
    const bool forward_in_topology = route_manager.forwardDistance(
        feature.route.current_link_id, feature.ego.map_s, object.link_id,
        object.link_s, front_lookahead, &topology_gap);
    const bool geometry_front_gate =
        ego_longitudinal > 0.0 && euclidean_distance <= front_lookahead &&
        std::abs(object.d) <= follow_target_.max_lateral_offset &&
        same_direction;
    if (!forward_in_topology || !geometry_front_gate || topology_gap <= 0.0) {
      continue;
    }

    object.gap = topology_gap;
    object.ttc = object.relative_speed > 0.0
                     ? object.gap / object.relative_speed
                     : 1e9;
    const double lead_score =
        object.gap - (object.id == last_lead_vehicle_id_ ? 4.0 : 0.0);
    if (lead_score < best_lead_score) {
      best_lead_score = lead_score;
      feature.surrounding.has_front_vehicle = true;
      feature.surrounding.front_vehicle = object;
    }
  }

  // 현재 선두가 직전 frame의 선두와 이어질 때만 작은 동시 관측을 fragment로
  // 인정한다. 따라서 처음 본 정적 장애물이 우연히 차량 가까이에 있다는
  // 이유만으로 즉시 무시되지 않는다. ID가 바뀌어도 위치/크기가 연속이면
  // 동일 선두로 볼 수 있다.
  std::unordered_set<int> current_follow_fragment_ids;
  if (feature.surrounding.has_front_vehicle && recent_lead_.geometry_valid &&
      recent_lead_.missing_cycles <= avoid_target_.reconnect_grace_cycles) {
    const ObjectFeature& lead = feature.surrounding.front_vehicle;
    const double lead_long_side = std::max(lead.length, lead.width);
    const double lead_short_side = std::min(lead.length, lead.width);
    const double recent_long_side =
        std::max(recent_lead_.length, recent_lead_.width);
    const double recent_short_side =
        std::min(recent_lead_.length, recent_lead_.width);
    const bool stable_lead =
        lead.id == recent_lead_.id ||
        (std::abs(lead.s - recent_lead_.s) <=
             avoid_target_.reconnect_max_longitudinal_delta_m &&
         std::abs(lead.d - recent_lead_.d) <=
             avoid_target_.reconnect_max_lateral_delta_m &&
         std::abs(lead_long_side - recent_long_side) <=
             avoid_target_.reconnect_max_size_delta_m &&
         std::abs(lead_short_side - recent_short_side) <=
             avoid_target_.reconnect_max_size_delta_m);
    if (stable_lead) {
      for (const ObjectFeature& object : projected_objects) {
        if (current_id_counts[object.id] == 1 &&
            IsSmallOverlappingLeadFragment(object, lead)) {
          current_follow_fragment_ids.insert(object.id);
          // 이미 정적 overlap latch가 생겼더라도 현재 선두 본체와의 연결이
          // 확인되면 stale AVOID 후보로 남기지 않는다.
          avoid_overlap_states_.erase(object.id);
        }
      }
    }
  }
  feature.extra_values["follow_fragment_count"] =
      static_cast<double>(current_follow_fragment_ids.size());

  // A moving lead and a static-looking duplicate can coexist in the same
  // LiDAR frame with different tracker IDs.  Provide only one-to-one spatial
  // ownership evidence here; Behavior/FSM remains responsible for deciding
  // whether FOLLOW preempts AVOID.  Requiring OBB overlap in addition to the
  // normal re-identification limits prevents a nearby real obstacle from being
  // absorbed into the lead vehicle.
  int current_lead_static_match_id = -1;
  int current_lead_static_match_count = 0;
  if (feature.surrounding.has_front_vehicle) {
    const ObjectFeature& lead = feature.surrounding.front_vehicle;
    for (const ObjectFeature& object : projected_objects) {
      if (!object.valid || !IsStaticAvoidCandidate(object) ||
          current_id_counts[object.id] != 1 ||
          current_follow_fragment_ids.count(object.id) != 0U ||
          !IsSpatiallySameTrack(object, lead, avoid_target_)) {
        continue;
      }
      current_lead_static_match_id = object.id;
      ++current_lead_static_match_count;
    }
  }
  if (current_lead_static_match_count != 1) {
    current_lead_static_match_id = -1;
  }

  // ID가 바뀌었더라도 직전 FOLLOW 대상과 같은 물리 객체로 유일하게 연결되는
  // 정적 관측을 찾는다. 여기서는 공간적 동일성만 제공하고 FOLLOW를 유지할지
  // AVOID로 바꿀지는 FSM이 결정한다. 같은 위치에 후보가 둘 이상이면 잘못된
  // 연결보다 보수적으로 "불명"을 선택한다.
  int recent_lead_static_match_id = -1;
  int recent_lead_static_match_count = 0;
  if (recent_lead_.geometry_valid &&
      recent_lead_.missing_cycles <= avoid_target_.reconnect_grace_cycles) {
    for (const ObjectFeature& object : projected_objects) {
      if (!object.valid || !IsStaticAvoidCandidate(object) ||
          current_id_counts[object.id] != 1 ||
          current_follow_fragment_ids.count(object.id) != 0U) {
        continue;
      }
      const bool same_id = object.id == recent_lead_.id;
      const double object_long_side = std::max(object.length, object.width);
      const double object_short_side = std::min(object.length, object.width);
      const double lead_long_side =
          std::max(recent_lead_.length, recent_lead_.width);
      const double lead_short_side =
          std::min(recent_lead_.length, recent_lead_.width);
      const bool unique_spatial_match =
          std::abs(object.s - recent_lead_.s) <=
              avoid_target_.reconnect_max_longitudinal_delta_m &&
          std::abs(object.d - recent_lead_.d) <=
              avoid_target_.reconnect_max_lateral_delta_m &&
          std::abs(object_long_side - lead_long_side) <=
              avoid_target_.reconnect_max_size_delta_m &&
          std::abs(object_short_side - lead_short_side) <=
              avoid_target_.reconnect_max_size_delta_m;
      if (same_id || unique_spatial_match) {
        recent_lead_static_match_id = object.id;
        ++recent_lead_static_match_count;
      }
    }
  }
  if (recent_lead_static_match_count != 1) {
    recent_lead_static_match_id = -1;
  }
  if (avoid_target_.reconnect_grace_cycles > 0) {
    std::unordered_map<int, std::vector<int>> old_ids_by_new_id;
    std::unordered_map<int, int> new_match_count_by_old_id;
    const auto dimensions_match = [this](const ObjectFeature& object,
                                         const AvoidOverlapState& state) {
      const double object_long_side = std::max(object.length, object.width);
      const double object_short_side = std::min(object.length, object.width);
      const double state_long_side = std::max(state.last_length,
                                              state.last_width);
      const double state_short_side = std::min(state.last_length,
                                               state.last_width);
      return std::abs(object_long_side - state_long_side) <=
                 avoid_target_.reconnect_max_size_delta_m &&
             std::abs(object_short_side - state_short_side) <=
                 avoid_target_.reconnect_max_size_delta_m;
    };

    for (const ObjectFeature& object : projected_objects) {
      if (!object.valid || current_id_counts[object.id] != 1 ||
          avoid_overlap_states_.count(object.id) != 0U) {
        continue;
      }
      const bool avoidable_observation = IsStaticAvoidCandidate(object);
      if (!avoidable_observation) continue;

      for (const auto& entry : avoid_overlap_states_) {
        const int old_id = entry.first;
        const AvoidOverlapState& state = entry.second;
        if (current_id_counts.count(old_id) != 0U ||
            !state.geometry_valid ||
            state.missing_cycles > avoid_target_.reconnect_grace_cycles ||
            std::abs(object.s - state.last_s) >
                avoid_target_.reconnect_max_longitudinal_delta_m ||
            std::abs(object.d - state.last_d) >
                avoid_target_.reconnect_max_lateral_delta_m ||
            !dimensions_match(object, state)) {
          continue;
        }
        old_ids_by_new_id[object.id].push_back(old_id);
        ++new_match_count_by_old_id[old_id];
      }
    }

    std::vector<std::pair<int, int>> reconnects;
    for (const auto& entry : old_ids_by_new_id) {
      if (entry.second.size() != 1U) continue;
      const int old_id = entry.second.front();
      if (new_match_count_by_old_id[old_id] == 1) {
        reconnects.emplace_back(old_id, entry.first);
      }
    }
    for (const auto& reconnect : reconnects) {
      const auto old_state = avoid_overlap_states_.find(reconnect.first);
      if (old_state == avoid_overlap_states_.end() ||
          avoid_overlap_states_.count(reconnect.second) != 0U) {
        continue;
      }
      AvoidOverlapState state = old_state->second;
      state.missing_cycles = 0;
      avoid_overlap_states_.erase(old_state);
      avoid_overlap_states_.emplace(reconnect.second, state);
    }

    // A LiDAR track can change ID at the same time its motion/classification
    // state matures.  Do not expose the old, latched static track as a ghost
    // AVOID while the new observation is already a longitudinal lead.  Apply
    // the same unique spatial/dimension checks used by static re-connection;
    // ambiguous nearby observations deliberately do not erase anything.
    std::unordered_map<int, std::vector<int>> moving_old_ids_by_new_id;
    std::unordered_map<int, int> moving_match_count_by_old_id;
    for (const ObjectFeature& object : projected_objects) {
      if (!object.valid || !IsLongitudinalLeadCandidate(object) ||
          current_id_counts[object.id] != 1 ||
          avoid_overlap_states_.count(object.id) != 0U) {
        continue;
      }
      for (const auto& entry : avoid_overlap_states_) {
        const int old_id = entry.first;
        const AvoidOverlapState& state = entry.second;
        if (current_id_counts.count(old_id) != 0U ||
            !state.geometry_valid ||
            state.missing_cycles > avoid_target_.reconnect_grace_cycles ||
            std::abs(object.s - state.last_s) >
                avoid_target_.reconnect_max_longitudinal_delta_m ||
            std::abs(object.d - state.last_d) >
                avoid_target_.reconnect_max_lateral_delta_m ||
            !dimensions_match(object, state)) {
          continue;
        }
        moving_old_ids_by_new_id[object.id].push_back(old_id);
        ++moving_match_count_by_old_id[old_id];
      }
    }
    std::vector<int> reclassified_old_ids;
    for (const auto& entry : moving_old_ids_by_new_id) {
      if (entry.second.size() != 1U) continue;
      const int old_id = entry.second.front();
      if (moving_match_count_by_old_id[old_id] == 1) {
        reclassified_old_ids.push_back(old_id);
      }
    }
    for (const int old_id : reclassified_old_ids) {
      const auto old_state = avoid_overlap_states_.find(old_id);
      if (old_state == avoid_overlap_states_.end()) continue;
      feature.risk.avoid_target_reclassified_moving |=
          old_state->second.latched;
      avoid_overlap_states_.erase(old_state);
    }
  }

  double best_static_obstacle_gap = 1e9;
  std::unordered_set<int> avoid_overlap_ids_seen;
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
    // 움직이는 객체는 OBB 방향이 아니라 perception의 절대속도 방향으로
    // 진행 방향을 판정한다. 정적/저속 객체에만 OBB yaw를 fallback으로 쓴다.
    const double object_motion_speed = std::hypot(object.vx, object.vy);
    const double object_motion_yaw = object_motion_speed > 0.5
                                         ? std::atan2(object.vy, object.vx)
                                         : object.yaw;
    const bool same_direction =
        std::cos(object_motion_yaw - feature.ego.yaw) > 0.5;
    double follow_topology_gap = 1e9;
    const bool follow_forward_in_topology = route_manager.forwardDistance(
        feature.route.current_link_id, feature.ego.map_s, object.link_id,
        object.link_s, front_lookahead, &follow_topology_gap);
    const bool follow_geometry_front_gate =
        ego_longitudinal > 0.0 && euclidean_distance <= front_lookahead &&
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
    const double physical_corridor_half_width =
        0.5 * avoid_target_.ego_width;
    const double path_corridor_half_width = std::min(
        avoid_target_.max_lateral_offset,
        physical_corridor_half_width + avoid_target_.path_margin_m);
    const double obstacle_lateral_min =
        object.d - obstacle_lateral_half_extent;
    const double obstacle_lateral_max =
        object.d + obstacle_lateral_half_extent;
    const auto interval_overlap = [](double lhs_min, double lhs_max,
                                     double rhs_min, double rhs_max) {
      return std::max(0.0,
                      std::min(lhs_max, rhs_max) -
                          std::max(lhs_min, rhs_min));
    };
    const double physical_overlap_m = interval_overlap(
        obstacle_lateral_min, obstacle_lateral_max,
        -physical_corridor_half_width, physical_corridor_half_width);
    const double margin_overlap_m = interval_overlap(
        obstacle_lateral_min, obstacle_lateral_max,
        -path_corridor_half_width, path_corridor_half_width);
    const double obstacle_projected_width =
        2.0 * obstacle_lateral_half_extent;
    const double margin_overlap_ratio = obstacle_projected_width > 1e-6
                                            ? margin_overlap_m /
                                                  obstacle_projected_width
                                            : 0.0;
    // "중심점이 corridor 안인가"만 보면 중심이 경계에서 흔들릴 때 AVOID가
    // 매 frame 켜졌다 꺼진다. 실제로 ego swept width를 얼마나 가리는지를
    // 사용한다. 작은 물체가 분모 때문에 불리해지지 않도록 OBB 투영 폭과
    // ego 폭 중 작은 쪽을 기준 폭으로 삼는다.
    const double blocking_reference_width = std::min(
        obstacle_projected_width, 2.0 * physical_corridor_half_width);
    const double physical_blocking_ratio = blocking_reference_width > 1e-6
                                               ? physical_overlap_m /
                                                     blocking_reference_width
                                               : 0.0;
    const double obstacle_nearest_forward_gap =
        avoid_route_gap - obstacle_longitudinal_half_extent;
    const bool obstacle_is_in_acquisition_range =
        avoid_route_gap > 0.0 &&
        obstacle_nearest_forward_gap <= avoid_target_.max_distance &&
        ego_longitudinal > 0.0 &&
        euclidean_distance <=
            avoid_target_.max_distance + obstacle_longitudinal_half_extent;

    if (current_lane_front) {
      object.gap = follow_topology_gap;
      object.ttc = object.gap > 0.0 && object.relative_speed > 0.0
                       ? object.gap / object.relative_speed
                       : 1e9;
    }

    const bool followable_object = IsLongitudinalLeadCandidate(object);
    if (object.type == ObjectType::PEDESTRIAN && object.gap > 0.0 &&
        object.gap < safety_.pedestrian_stop_distance &&
        std::abs(relative_d) < kLaneWidth) {
      feature.risk.pedestrian_risk = true;
      feature.risk.nearest_pedestrian_s =
          std::min(feature.risk.nearest_pedestrian_s, object.s);
    }
    // UNKNOWN/LOW_THREAT만 정적 회피 대상으로 본다. DYNAMIC_PENDING부터는
    // 횡방향 AVOID를 시작하지 않고, 전방 동일 진행 객체면 FOLLOW, 그 외에는
    // KEEP으로 보낸다. 같은 ID의 기존 정적 latch도 즉시 제거한다.
    const bool avoidable_obstacle =
        IsStaticAvoidCandidate(object) &&
        current_follow_fragment_ids.count(object.id) == 0U;
    // 10 cm 절대 기준은 정상 크기 OBB의 한 voxel 수준 접촉을 제거한다.
    // OBB 자체가 그보다 작은 경우에는 폭의 절반까지만 요구해 작은 실제
    // 장애물이 영원히 제외되는 것을 막는다.
    const double required_physical_overlap = std::min(
        avoid_target_.min_physical_overlap_m,
        0.5 * blocking_reference_width);
    const bool measurable_physical_overlap =
        blocking_reference_width > 1e-6 &&
        physical_overlap_m + 1e-9 >= required_physical_overlap;
    const bool immediate_physical_block =
        measurable_physical_overlap &&
        physical_blocking_ratio + 1e-9 >=
            avoid_target_.immediate_blocking_ratio;
    const bool physical_overlap_candidate =
        measurable_physical_overlap &&
        physical_blocking_ratio + 1e-9 >=
            avoid_target_.blocking_enter_ratio;
    const bool margin_overlap_candidate =
        !measurable_physical_overlap &&
        margin_overlap_m + 1e-9 >= avoid_target_.min_margin_overlap_m &&
        margin_overlap_ratio + 1e-9 >= avoid_target_.min_overlap_ratio;
    bool confirmed_corridor_block = false;
    if (avoidable_obstacle && obstacle_is_in_acquisition_range) {
      avoid_overlap_ids_seen.insert(object.id);
      AvoidOverlapState& state = avoid_overlap_states_[object.id];
      state.missing_cycles = 0;
      if (immediate_physical_block) {
        state.latched = true;
        state.enter_cycles = avoid_target_.overlap_confirm_cycles;
        state.exit_cycles = 0;
      } else if (physical_overlap_candidate || margin_overlap_candidate) {
        state.exit_cycles = 0;
        state.enter_cycles = std::min(
            state.enter_cycles + 1, avoid_target_.overlap_confirm_cycles);
        if (state.enter_cycles >= avoid_target_.overlap_confirm_cycles) {
          state.latched = true;
        }
      } else if (state.latched && measurable_physical_overlap &&
                 physical_blocking_ratio + 1e-9 >=
                     avoid_target_.blocking_exit_ratio) {
        // 진입 기준 아래로 잠깐 내려가도 해제 기준 이상이면 유지한다.
        state.exit_cycles = 0;
      } else if (state.latched) {
        state.exit_cycles = std::min(
            state.exit_cycles + 1, avoid_target_.overlap_release_cycles);
        if (state.exit_cycles >= avoid_target_.overlap_release_cycles) {
          state = AvoidOverlapState{};
        }
      } else {
        state.enter_cycles = 0;
        state.exit_cycles = 0;
      }
      // Always keep the newest measured geometry. Only the overlap latch is
      // carried across a re-ID; the old OBB itself is never frozen over a new
      // observation.
      state.geometry_valid = true;
      state.last_s = object.s;
      state.last_d = object.d;
      state.last_x = object.x;
      state.last_y = object.y;
      state.last_length = object.length;
      state.last_width = object.width;
      state.last_projected_longitudinal_length =
          2.0 * obstacle_longitudinal_half_extent;
      confirmed_corridor_block = state.latched;
    } else {
      const auto previous_state = avoid_overlap_states_.find(object.id);
      if (!avoidable_obstacle && IsLongitudinalLeadCandidate(object) &&
          previous_state != avoid_overlap_states_.end()) {
        feature.risk.avoid_target_reclassified_moving |=
            previous_state->second.latched;
      }
      avoid_overlap_states_.erase(object.id);
    }
    const bool obstacle_blocks_route_corridor =
        obstacle_is_in_acquisition_range &&
        confirmed_corridor_block;
    if (avoidable_obstacle && obstacle_blocks_route_corridor) {
      object.gap = std::max(0.0, obstacle_nearest_forward_gap);
      if (!feature.risk.static_obstacle_ahead ||
          object.gap < best_static_obstacle_gap) {
        feature.risk.static_obstacle_ahead = true;
        feature.risk.avoidance_object_id = object.id;
        feature.risk.nearest_obstacle_s = object.s;
        feature.risk.nearest_obstacle_length =
            2.0 * obstacle_longitudinal_half_extent;
        feature.risk.avoid_target_matches_front_lead =
            object.id == current_lead_static_match_id;
        feature.risk.avoid_target_matches_recent_lead =
            object.id == recent_lead_static_match_id;
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
    if (followable_object && current_lane_front &&
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

  for (auto it = avoid_overlap_states_.begin();
       it != avoid_overlap_states_.end();) {
    if (avoid_overlap_ids_seen.count(it->first) == 0U) {
      AvoidOverlapState& state = it->second;
      ++state.missing_cycles;
      const double route_gap = state.last_s - feature.ego.s;
      const double nearest_forward_gap =
          route_gap - 0.5 * state.last_projected_longitudinal_length;
      const double euclidean_distance =
          std::hypot(state.last_x - feature.ego.x,
                     state.last_y - feature.ego.y);
      const bool still_in_range =
          state.geometry_valid && feature.ego.valid && route_gap > 0.0 &&
          nearest_forward_gap <= avoid_target_.max_distance &&
          euclidean_distance <=
              avoid_target_.max_distance +
                  0.5 * state.last_projected_longitudinal_length;
      if (state.missing_cycles > avoid_target_.reconnect_grace_cycles ||
          !still_in_range) {
        it = avoid_overlap_states_.erase(it);
        continue;
      }

      // Keep an already confirmed target through a very short sensor dropout.
      // Pending candidates retain only their confirmation history and are not
      // exposed as AVOID until an observation confirms them again.
      if (state.latched) {
        const double gap = std::max(0.0, nearest_forward_gap);
        if (!feature.risk.static_obstacle_ahead ||
            gap < best_static_obstacle_gap) {
          feature.risk.static_obstacle_ahead = true;
          feature.risk.avoidance_object_id = it->first;
          feature.risk.nearest_obstacle_s = state.last_s;
          feature.risk.nearest_obstacle_length =
              state.last_projected_longitudinal_length;
          feature.risk.avoid_target_matches_front_lead = false;
          feature.risk.avoid_target_matches_recent_lead = false;
          best_static_obstacle_gap = gap;
        }
      }
    }
    ++it;
  }

  if (feature.surrounding.has_front_vehicle) {
    const ObjectFeature& lead = feature.surrounding.front_vehicle;
    recent_lead_.geometry_valid = true;
    recent_lead_.id = lead.id;
    recent_lead_.missing_cycles = 0;
    recent_lead_.s = lead.s;
    recent_lead_.d = lead.d;
    recent_lead_.length = lead.length;
    recent_lead_.width = lead.width;
  } else if (recent_lead_.geometry_valid) {
    ++recent_lead_.missing_cycles;
    if (recent_lead_.missing_cycles >
        avoid_target_.reconnect_grace_cycles) {
      recent_lead_ = RecentLeadState{};
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

