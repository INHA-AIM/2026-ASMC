#include "planning_inputs/converter.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <sstream>
#include <unordered_set>
#include <utility>
#include <vector>

#include <planning_inputs/DetectedObject.h>

namespace planning_inputs {
namespace {

constexpr double kDuplicatePointEpsilon = 1e-6;
constexpr double kMinimumObbArea = 1e-6;
constexpr double kMinimumObbHeight = 1e-6;

struct Point2 {
  double x = 0.0;
  double y = 0.0;
};

bool IsFinite(double value) { return std::isfinite(value); }

bool IsFinitePoint(const geometry_msgs::Point& point) {
  return IsFinite(point.x) && IsFinite(point.y) && IsFinite(point.z);
}

bool IsFiniteVector(const geometry_msgs::Vector3& vector) {
  return IsFinite(vector.x) && IsFinite(vector.y) && IsFinite(vector.z);
}

double Cross(const Point2& origin, const Point2& a, const Point2& b) {
  return (a.x - origin.x) * (b.y - origin.y) -
         (a.y - origin.y) * (b.x - origin.x);
}

bool NearlySame(const Point2& a, const Point2& b) {
  return std::hypot(a.x - b.x, a.y - b.y) <= kDuplicatePointEpsilon;
}

std::vector<Point2> ConvexHull(std::vector<Point2> points) {
  std::sort(points.begin(), points.end(), [](const Point2& a, const Point2& b) {
    if (a.x != b.x) return a.x < b.x;
    return a.y < b.y;
  });
  points.erase(std::unique(points.begin(), points.end(), NearlySame),
               points.end());
  if (points.size() < 3) return {};

  std::vector<Point2> hull;
  hull.reserve(points.size() * 2);
  for (const Point2& point : points) {
    while (hull.size() >= 2 &&
           Cross(hull[hull.size() - 2], hull.back(), point) <= 0.0) {
      hull.pop_back();
    }
    hull.push_back(point);
  }
  const std::size_t lower_size = hull.size();
  for (auto it = points.rbegin() + 1; it != points.rend(); ++it) {
    while (hull.size() > lower_size &&
           Cross(hull[hull.size() - 2], hull.back(), *it) <= 0.0) {
      hull.pop_back();
    }
    hull.push_back(*it);
  }
  if (!hull.empty()) hull.pop_back();
  return hull;
}

double PolygonArea(const std::vector<Point2>& polygon) {
  if (polygon.size() < 3) return 0.0;
  double twice_area = 0.0;
  for (std::size_t i = 0; i < polygon.size(); ++i) {
    const Point2& a = polygon[i];
    const Point2& b = polygon[(i + 1) % polygon.size()];
    twice_area += a.x * b.y - a.y * b.x;
  }
  return 0.5 * std::fabs(twice_area);
}

template <class Object>
bool ValidateObb(const Object& object, std::string* reason) {
  std::vector<Point2> projected;
  projected.reserve(object.obb_corners.size());
  double min_z = std::numeric_limits<double>::infinity();
  double max_z = -std::numeric_limits<double>::infinity();
  for (const auto& corner : object.obb_corners) {
    if (!IsFinitePoint(corner)) {
      if (reason) *reason = "OBB contains a non-finite corner";
      return false;
    }
    projected.push_back({corner.x, corner.y});
    min_z = std::min(min_z, corner.z);
    max_z = std::max(max_z, corner.z);
  }
  const std::vector<Point2> hull = ConvexHull(std::move(projected));
  if (hull.size() < 3 || !IsFinite(PolygonArea(hull)) ||
      PolygonArea(hull) < kMinimumObbArea) {
    if (reason) *reason = "OBB has degenerate XY geometry";
    return false;
  }
  if (!IsFinite(min_z) || !IsFinite(max_z) ||
      max_z - min_z <= kMinimumObbHeight) {
    if (reason) *reason = "OBB has degenerate Z geometry";
    return false;
  }
  return true;
}

std::string NormalizeClassification(std::string value) {
  value.erase(value.begin(),
              std::find_if(value.begin(), value.end(), [](unsigned char c) {
                return !std::isspace(c);
              }));
  value.erase(std::find_if(value.rbegin(), value.rend(),
                           [](unsigned char c) { return !std::isspace(c); })
                  .base(),
              value.end());
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char c) {
                   return static_cast<char>(std::tolower(c));
                 });
  return value;
}

std::uint8_t ConvertClassification(const std::string& source,
                                   bool* supported) {
  const std::string label = NormalizeClassification(source);
  if (label == "car" || label == "vehicle" || label == "truck" ||
      label == "bus") {
    if (supported) *supported = true;
    return kObjectVehicle;
  }
  if (label == "person" || label == "pedestrian") {
    if (supported) *supported = true;
    return kObjectPedestrian;
  }
  // This adapter currently emits only vehicle/pedestrian/unknown; the planner
  // also defines STATIC_OBSTACLE=3, but this is NOT a mapping of mission groups.
  // Both vehicle and dynamic_obstacle can be stationary. static_obstacle is a
  // competition asset category, not proof of zero velocity. Never derive
  // motion_state/velocity validity from these names (or vice versa).
  // Keep all geometry and motion for the two obstacle groups. Group and raw
  // class are copied losslessly from the SAME incoming observation into the
  // private DetectedObject. No independent tracks_map join or class-derived
  // velocity is used. Existing numeric behavior policy is deliberately kept.
  // Planning must keep UNKNOWN geometry as a collision candidate. The enum
  // cannot recover a source class or decide whether an object actually moves;
  // preserve original class/group separately from the pedestrian policy type.
  // See docs/handoffs/perception-to-planning.md (consumer acceptance).
  if (label == "dynamic_obstacle" || label == "static_obstacle") {
    if (supported) *supported = true;
    return kObjectUnknown;
  }
  if (label.empty() || label == "unknown" || label == "unclassified") {
    if (supported) *supported = true;
    return kObjectUnknown;
  }
  if (supported) *supported = false;
  return kObjectUnknown;
}

ConversionStatus Fail(ConversionStatus status, std::size_t index, int id,
                      const std::string& reason, std::string* detail) {
  if (detail) {
    std::ostringstream stream;
    stream << "object_index=" << index << " id=" << id << ": " << reason;
    *detail = stream.str();
  }
  return status;
}

}  // namespace

const char* ConversionStatusName(ConversionStatus status) {
  switch (status) {
    case ConversionStatus::OK:
      return "OK";
    case ConversionStatus::INVALID_ARGUMENT:
      return "INVALID_ARGUMENT";
    case ConversionStatus::INVALID_FRAME:
      return "INVALID_FRAME";
    case ConversionStatus::INVALID_TIMESTAMP:
      return "INVALID_TIMESTAMP";
    case ConversionStatus::INVALID_ID:
      return "INVALID_ID";
    case ConversionStatus::DUPLICATE_ID:
      return "DUPLICATE_ID";
    case ConversionStatus::NON_FINITE_VALUE:
      return "NON_FINITE_VALUE";
    case ConversionStatus::INVALID_DIMENSIONS:
      return "INVALID_DIMENSIONS";
    case ConversionStatus::INVALID_OBB:
      return "INVALID_OBB";
    case ConversionStatus::INVALID_MOTION_STATE:
      return "INVALID_MOTION_STATE";
    case ConversionStatus::INVALID_OBSERVATION:
      return "INVALID_OBSERVATION";
    case ConversionStatus::INVALID_VELOCITY:
      return "INVALID_VELOCITY";
  }
  return "UNKNOWN_STATUS";
}

ConversionStatus ConvertPlanningGeometry(
    const asmc_msgs::LidarObjectArray& array,
    planning_inputs::DetectedObjectArray* output, ConversionStats* stats,
    std::string* detail) {
  if (!output) return Fail(ConversionStatus::INVALID_ARGUMENT, 0, -1, "null output", detail);
  if (array.header.frame_id != "map")
    return Fail(ConversionStatus::INVALID_FRAME, 0, -1, "global planning requires map", detail);
  if (array.header.stamp.isZero())
    return Fail(ConversionStatus::INVALID_TIMESTAMP, 0, -1, "zero source stamp", detail);
  DetectedObjectArray converted;
  converted.header = array.header;
  ConversionStats counts;
  counts.total = array.objects.size();
  std::unordered_set<std::int32_t> ids;
  for (std::size_t i = 0; i < array.objects.size(); ++i) {
    const auto& object = array.objects[i];
    if (object.track_id < 0 || !ids.insert(object.track_id).second ||
        object.observation_state != object.OBSERVED || object.miss_count)
      return Fail(ConversionStatus::INVALID_ID, i, object.track_id, "duplicate, mismatched or non-observed ID", detail);
    if (!IsFinitePoint(object.center) || !IsFinite(object.yaw))
      return Fail(ConversionStatus::NON_FINITE_VALUE, i, object.track_id, "non-finite geometry", detail);
    if (!IsFinite(object.length) || object.length <= 0 || !IsFinite(object.width) ||
        object.width <= 0 || !IsFinite(object.height) || object.height <= 0)
      return Fail(ConversionStatus::INVALID_DIMENSIONS, i, object.track_id, "invalid dimensions", detail);
    std::string reason;
    if (!ValidateObb(object, &reason))
      return Fail(ConversionStatus::INVALID_OBB, i, object.track_id, reason, detail);
    if (object.absolute_velocity_valid && (!IsFiniteVector(object.absolute_velocity_mps) ||
        !IsFinite(3.6 * std::hypot(object.absolute_velocity_mps.x, object.absolute_velocity_mps.y))))
      return Fail(ConversionStatus::INVALID_VELOCITY, i, object.track_id, "non-finite qualified velocity", detail);
    bool supported = true;
    DetectedObject target;
    target.id = object.track_id;
    target.classification = object.classification;
    target.camera_class = object.camera_class;
    target.type = ConvertClassification(object.classification == "dynamic_obstacle" &&
        object.camera_class == "person" ? "person" : object.classification, &supported);
    target.velocity_unavailable = !object.absolute_velocity_valid;
    target.center = object.center;
    target.yaw = object.yaw;
    if (object.absolute_velocity_valid) {
      target.vx = object.absolute_velocity_mps.x;
      target.vy = object.absolute_velocity_mps.y;
    }
    // Preserve PR #114's stateless yaw-rate policy; OBB yaw derivative is not measured.
    target.yaw_rate = 0.;
    target.width = object.width; target.length = object.length; target.height = object.height;
    target.obb_corners = object.obb_corners;
    if (!supported) ++counts.unsupported_classification;
    if (target.type == kObjectVehicle) ++counts.vehicle;
    else if (target.type == kObjectPedestrian) ++counts.pedestrian;
    else ++counts.unknown;
    converted.objects.push_back(std::move(target));
  }
  *output = std::move(converted);
  if (stats) *stats = counts;
  if (detail) detail->clear();
  return ConversionStatus::OK;
}

ConversionStatus ConvertLidarObservation(
    const asmc_msgs::LidarTrackingObservation& input,
    planning_inputs::DetectedObjectArray* output, ConversionStats* stats,
    std::string* detail) {
  if (!output) return Fail(ConversionStatus::INVALID_ARGUMENT, 0, -1, "null output", detail);
  const auto& observation = input.observation;
  const auto& array = observation.objects;
  if (array.header.frame_id != "map")
    return Fail(ConversionStatus::INVALID_FRAME, 0, -1, "global planning requires map", detail);
  if (array.header.stamp.isZero())
    return Fail(ConversionStatus::INVALID_TIMESTAMP, 0, -1, "zero source stamp", detail);
  if (observation.producer_session_id.empty() || !observation.generation ||
      !observation.scan_sequence || observation.config_id.empty() || observation.calibration_id.empty() ||
      input.tracking.size() != array.objects.size() ||
      (array.status != array.OBSERVED && array.status != array.OBSERVED_EMPTY) ||
      ((array.status == array.OBSERVED_EMPTY) != array.objects.empty()))
    return Fail(ConversionStatus::INVALID_OBSERVATION, 0, -1, "invalid identity/status/tracking alignment", detail);
  DetectedObjectArray converted;
  converted.header = array.header;
  CopyObservationIdentity(observation, &converted);
  ConversionStats counts;
  counts.total = array.objects.size();
  std::unordered_set<std::int32_t> ids;
  for (std::size_t i = 0; i < array.objects.size(); ++i) {
    const auto& object = array.objects[i];
    const auto& tracking = input.tracking[i];
    if (object.track_id < 0 || !ids.insert(object.track_id).second ||
        object.track_id != tracking.track_id || object.observation_state != object.OBSERVED || object.miss_count)
      return Fail(ConversionStatus::INVALID_ID, i, object.track_id, "duplicate, mismatched or non-observed ID", detail);
    if (!IsFinitePoint(object.center) || !IsFinitePoint(tracking.cluster_center) || !IsFinite(object.yaw))
      return Fail(ConversionStatus::NON_FINITE_VALUE, i, object.track_id, "non-finite geometry", detail);
    if (!IsFinite(object.length) || object.length <= 0 || !IsFinite(object.width) ||
        object.width <= 0 || !IsFinite(object.height) || object.height <= 0)
      return Fail(ConversionStatus::INVALID_DIMENSIONS, i, object.track_id, "invalid dimensions", detail);
    std::string reason;
    if (!ValidateObb(object, &reason))
      return Fail(ConversionStatus::INVALID_OBB, i, object.track_id, reason, detail);
    if (!object.absolute_velocity_valid || !IsFiniteVector(object.absolute_velocity_mps))
      return Fail(ConversionStatus::INVALID_VELOCITY, i, object.track_id, "no representable absolute estimate", detail);
    const auto state = tracking.motion_state;
    if (state > kMotionDynamic || (object.dynamic_valid != (state != kMotionUnknown)) ||
        object.is_dynamic != (state == kMotionDynamicPending || state == kMotionDynamic))
      return Fail(ConversionStatus::INVALID_MOTION_STATE, i, object.track_id, "inconsistent motion state/validity", detail);
    bool supported = true;
    DetectedObject target;
    target.id = object.track_id;
    target.classification = object.classification;
    target.camera_class = object.camera_class;
    target.type = ConvertClassification(object.classification == "dynamic_obstacle" &&
        object.camera_class == "person" ? "person" : object.classification, &supported);
    target.motion_state = state;
    target.center = object.center;
    target.yaw = object.yaw;
    target.vx = object.absolute_velocity_mps.x;
    target.vy = object.absolute_velocity_mps.y;
    // Legacy producer did not estimate yaw rate; preserve that interface.
    // Do not substitute a moving-frame OBB angle derivative.
    target.yaw_rate = object.yaw_rate_valid ? object.yaw_rate_rad_s : 0.;
    if (!IsFinite(target.yaw_rate))
      return Fail(ConversionStatus::NON_FINITE_VALUE, i, object.track_id, "non-finite yaw rate", detail);
    target.width = object.width; target.length = object.length; target.height = object.height;
    target.obb_corners = object.obb_corners;
    if (!supported) ++counts.unsupported_classification;
    if (target.type == kObjectVehicle) ++counts.vehicle;
    else if (target.type == kObjectPedestrian) ++counts.pedestrian;
    else ++counts.unknown;
    converted.objects.push_back(std::move(target));
  }
  *output = std::move(converted);
  if (stats) *stats = counts;
  if (detail) detail->clear();
  return ConversionStatus::OK;
}

}  // namespace planning_inputs
