#pragma once

#include "Map/HdLaneMap.hpp"
#include "Frenet/Geometry/ObjectTypes.hpp"
#include "Frenet/Geometry/PolygonUtils.hpp"
#include <cmath>

namespace asmc_planner::lane_reference {

// Admission for the opt-in, empty-traffic reference handover trial only.
// This does not authorize traffic gaps or remove objects from collision checks.
// LOW_THREAT is the input classifier's settled, static-equivalent state; raw
// velocity noise is not reclassified here using a second speed threshold.
inline bool TrialObjectsAllowStart(const LaneRegion& corridor,
    const std::vector<TrackedObject>& objects,std::string& detail) {
  detail.clear();
  for (const auto& object:objects) {
    const auto reject=[&](const char* reason){
      detail="trial_object_blocks_start: id="+std::to_string(object.id)+" reason="+reason;
      return false;
    };
    if (!object.geometry_valid || !object.z_valid ||
        !geometry::ValidateCorners8(object.corners).valid ||
        !std::isfinite(object.vx) || !std::isfinite(object.vy))
      return reject("invalid_object_geometry_or_velocity");
    if (object.type!=int(ObjectType::UNKNOWN) && object.type!=int(ObjectType::STATIC_OBSTACLE))
      return reject("traffic_actor_or_unknown_class");
    if (object.motion_state!=MotionState::LOW_THREAT)
      return reject("motion_not_settled_low_threat");
    const auto hull=geometry::ComputeConvexHull(geometry::ProjectCornersToXY(object.corners));
    std::vector<Point2D> polygon;polygon.reserve(hull.size());
    for (const auto& point:hull) polygon.push_back({point.x,point.y});
    if (corridor.intersectsPolygon(polygon))
      return reject("overlaps_source_target_corridor");
  }
  return true;
}

}  // namespace asmc_planner::lane_reference
