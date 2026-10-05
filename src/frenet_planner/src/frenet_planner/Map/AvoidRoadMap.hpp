#pragma once
#include <string>
#include <vector>
#include <utility>
#include <unordered_map>
#include <cstdint>
#include "frenet_planner/Common/CartesianTypes.hpp"
#include "frenet_planner/Evaluation/CollisionChecker.hpp"

// Pinned reviewed map-frame polygons/lanes and their official 0.2m raster.
// Raster footprint containment; paint is a separate policy layer:
// solid lane-boundary paint is protected; broken/guide/stop paint is not a wall.
class AvoidRoadMap {
 public:
  bool Load(const std::string& areas, const std::string& lines, std::string& error);
  bool enabled() const { return enabled_; }
  bool PoseClear(double x, double y, double yaw, const VehicleShape& shape) const;
  bool PathClear(const CartesianPath& path, const VehicleShape& shape) const;
 private:
  struct Point { double x, y; };
  struct Polygon { std::vector<Point> points; double xmin, ymin, xmax, ymax; };
  struct Segment { Point a, b; double xmin, ymin, xmax, ymax; };
  bool Inside(double x, double y) const;
  bool enabled_ = false;
  double grid_resolution_ = 0.0, grid_x_ = 0.0, grid_y_ = 0.0;
  int grid_width_ = 0, grid_height_ = 0;
  std::vector<std::vector<std::pair<int,int>>> grid_rows_;
  std::vector<Polygon> areas_;
  std::vector<Segment> protected_lines_;
  std::unordered_map<std::uint64_t,std::vector<std::size_t>> line_bins_;
};
