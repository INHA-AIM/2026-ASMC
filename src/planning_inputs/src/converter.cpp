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

bool ValidateObb(const asmc_msgs::TrackedObject& object, std::string* reason) {
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
  // The frozen numeric enum has only vehicle/pedestrian/unknown. Keep all
  // geometry and motion for the two obstacle groups; raw strings remain on
  // tracked_objects_map and the validated tracked companion topic.
  // Planning must keep UNKNOWN geometry as a collision candidate. The enum
  // cannot recover a source class or decide whether an object actually moves;
  // use the tracked companion and motion validity for those policies.
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
  }
  return "UNKNOWN_STATUS";
}

ConversionStatus ConvertTrackedObjects(
    const asmc_msgs::TrackedObjectArray& input,
    planning_inputs::DetectedObjectArray* output,
    ConversionStats* stats,
    std::string* detail) {
  if (!output) {
    if (detail) *detail = "output pointer is null";
    return ConversionStatus::INVALID_ARGUMENT;
  }
  if (input.header.frame_id != "map") {
    if (detail) *detail = "header.frame_id must be exactly 'map'";
    return ConversionStatus::INVALID_FRAME;
  }
  if (input.header.stamp.isZero()) {
    if (detail) *detail = "header.stamp must be non-zero";
    return ConversionStatus::INVALID_TIMESTAMP;
  }

  planning_inputs::DetectedObjectArray converted;
  converted.header = input.header;
  converted.objects.reserve(input.objects.size());
  ConversionStats converted_stats;
  converted_stats.total = input.objects.size();
  std::unordered_set<std::int32_t> ids;

  for (std::size_t i = 0; i < input.objects.size(); ++i) {
    const asmc_msgs::TrackedObject& source = input.objects[i];
    if (source.id < 0) {
      return Fail(ConversionStatus::INVALID_ID, i, source.id,
                  "ID must be non-negative", detail);
    }
    if (!ids.insert(source.id).second) {
      return Fail(ConversionStatus::DUPLICATE_ID, i, source.id,
                  "duplicate ID in one source frame", detail);
    }
    if (source.age < 0 || source.miss_count < 0) {
      return Fail(ConversionStatus::INVALID_ID, i, source.id,
                  "age and miss_count must be non-negative", detail);
    }
    if (!IsFinitePoint(source.cluster_center) ||
        !IsFinitePoint(source.obb_center) ||
        !IsFinite(source.obb_yaw) ||
        !IsFiniteVector(source.velocity_mps) ||
        !IsFinite(source.speed_kph)) {
      return Fail(ConversionStatus::NON_FINITE_VALUE, i, source.id,
                  "center, yaw, velocity, or display speed is non-finite",
                  detail);
    }
    if (!IsFinite(source.length) || source.length <= 0.0 ||
        !IsFinite(source.width) || source.width <= 0.0 ||
        !IsFinite(source.height) || source.height <= 0.0) {
      return Fail(ConversionStatus::INVALID_DIMENSIONS, i, source.id,
                  "length, width, and height must be finite and positive",
                  detail);
    }
    std::string obb_reason;
    if (!ValidateObb(source, &obb_reason)) {
      return Fail(ConversionStatus::INVALID_OBB, i, source.id, obb_reason,
                  detail);
    }

    bool supported_classification = true;
    const std::uint8_t object_type =
        ConvertClassification(source.classification == "dynamic_obstacle" &&
                                  source.yolo_class == "person" ? "person" : source.classification,
                              &supported_classification);
    if (!supported_classification) {
      ++converted_stats.unsupported_classification;
    }
    if (object_type == kObjectVehicle) {
      ++converted_stats.vehicle;
    } else if (object_type == kObjectPedestrian) {
      ++converted_stats.pedestrian;
    } else {
      ++converted_stats.unknown;
    }

    planning_inputs::DetectedObject target;
    target.id = source.id;
    target.type = object_type;
    target.motion_state = planning_inputs::DetectedObject::MOTION_UNKNOWN;
    target.center = source.obb_center;
    target.yaw = source.obb_yaw;
    target.vx = source.velocity_mps.x;
    target.vy = source.velocity_mps.y;
    // The fusion message does not measure yaw rate. Do not derive a value
    // from OBB orientation in this stateless boundary adapter.
    target.yaw_rate = 0.0;
    target.width = source.width;
    target.length = source.length;
    target.height = source.height;
    target.obb_corners = source.obb_corners;
    converted.objects.push_back(std::move(target));
  }

  *output = std::move(converted);
  if (stats) *stats = converted_stats;
  if (detail) detail->clear();
  return ConversionStatus::OK;
}

}  // namespace planning_inputs
