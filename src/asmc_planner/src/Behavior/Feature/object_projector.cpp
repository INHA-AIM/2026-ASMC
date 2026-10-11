#include "Behavior/Feature/object_projector.hpp"

namespace behavior_planner {

ObjectFeature ObjectProjector::project(const TrackedObject& object,
                                       const EgoFeature& ego,
                                       const RouteManager& route) const {
  ObjectFeature feature;
  feature.type = object.type;
  feature.motion_state = object.motion_state;
  feature.id = object.id;
  feature.x = object.x;
  feature.y = object.y;
  feature.yaw = object.yaw;
  feature.vx = object.vx;
  feature.vy = object.vy;
  feature.speed = object.speed;
  feature.accel = object.accel;
  feature.length = object.length;
  feature.width = object.width;

  std::string route_link;
  const bool route_valid = route.projectEgo(
      object.x, object.y, &feature.s, &feature.d, &route_link);
  LinkProjection map_projection;
  const bool map_valid =
      route.matchMapLink(object.x, object.y, object.yaw, &map_projection);

  // A static object's heading is not a travel direction. MORAI commonly
  // reports zero, which can map it to a nearby crossing/opposite link even
  // though its global-path s/d projection is correct. Frenet validates AVOID
  // targets in global Frenet coordinates, so Behavior uses that same boundary.
  feature.valid = route_valid &&
                  (object.type == ObjectType::STATIC_OBSTACLE ||
                   object.type == ObjectType::UNKNOWN || map_valid);
  if (map_valid) {
    feature.link_id = map_projection.link_id;
    feature.link_s = map_projection.link_s;
  } else if (route_valid) {
    feature.link_id = route_link;
  }

  feature.gap = feature.s - ego.s;
  feature.relative_speed = ego.speed - object.speed;
  feature.ttc = feature.gap > 0.0 && feature.relative_speed > 0.0
                    ? feature.gap / feature.relative_speed
                    : 1e9;
  return feature;
}

}  // namespace behavior_planner
