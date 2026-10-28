#pragma once

#include "Global/Global.hpp"
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace asmc_planner::lane_reference {

struct MapIdentity {
  std::string frame_id, build_id, map_id, geometry_sha256;
  bool valid() const;
  bool operator==(const MapIdentity& other) const;
  bool operator!=(const MapIdentity& other) const { return !(*this == other); }
};

struct LaneData {
  std::string id, link_id, left_adjacent, right_adjacent;
  std::vector<std::string> predecessors, successors;
  std::vector<Point2D> centerline, polygon;
  bool traversable = false;
};

enum class PaintPattern { UNKNOWN, SOLID, BROKEN };
struct MarkingData {
  std::string id;
  std::vector<Point2D> points;
  double width_m = 0.0;
  PaintPattern pattern = PaintPattern::UNKNOWN;
  bool stop_line = false;
  bool virtual_line = false;
};
struct AreaData { std::string id; std::vector<Point2D> polygon; bool validated = false; };
struct LaneMapData {
  MapIdentity identity;
  bool complete = false;
  std::vector<LaneData> lanes;
  std::vector<MarkingData> markings;
  std::vector<AreaData> areas;
};

// Distances from base_link/rear axle, all positive. No vehicle defaults here.
struct FootprintShape { double front_m = 0.0, rear_m = 0.0, half_width_m = 0.0; };

class LaneRegion {
 public:
  bool coversPoint(double x, double y) const;
  bool coversFootprint(double x, double y, double yaw,
                       const FootprintShape& shape, double margin_m) const;
  // Boundary contact counts as overlap. Invalid queries conservatively overlap.
  // Uses the original region parts; no materialized global union is required.
  bool intersectsPolygon(const std::vector<Point2D>& polygon) const;
  // Exact containment of authoritative observed geometry; gaps remain holes.
  bool coversPolygon(const std::vector<Point2D>& polygon) const;
 private:
  struct Impl;
  std::shared_ptr<const Impl> impl_;
  explicit LaneRegion(std::shared_ptr<const Impl> impl) : impl_(std::move(impl)) {}
  friend class HdLaneMap;
};

// One immutable, validated global snapshot. A failed new snapshot never edits
// the currently held shared_ptr. ROS transport is an independent input adapter.
class HdLaneMap {
 public:
  static std::shared_ptr<const HdLaneMap> create(LaneMapData data, std::string& error);
  const MapIdentity& identity() const;
  const LaneData* lane(const std::string& id) const;
  // Heading of the containing traversable lane, aligned with observed motion.
  // Defines actor motion axes without changing authoritative OBB geometry.
  std::optional<double> laneHeading(double x,double y,double vx,double vy) const;
  std::size_t laneCount() const;
  std::shared_ptr<const LaneRegion> road() const;
  std::shared_ptr<const LaneRegion> corridor(const std::vector<std::string>& lane_ids,
                                           std::string& error) const;
  // Query all actual paint geometries, not a single per-lane boundary ID.
  bool touchesSolid(double x, double y, double yaw, const FootprintShape& shape,
                    double margin_m, std::string* marking_id = nullptr) const;
 private:
  struct Impl;
  std::shared_ptr<const Impl> impl_;
  explicit HdLaneMap(std::shared_ptr<const Impl> impl) : impl_(std::move(impl)) {}
};

bool ValidFootprint(const FootprintShape& shape);

}  // namespace asmc_planner::lane_reference
