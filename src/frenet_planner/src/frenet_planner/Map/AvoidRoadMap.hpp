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
// Solid paint contributes a duration cost; later road departures remain hard; minor initial overlap has bounded recovery.
class AvoidRoadMap {
 public:
  bool Load(const std::string& areas, const std::string& lines, std::string& error);
  bool enabled() const { return enabled_; }
  bool PoseClear(double x, double y, double yaw, const VehicleShape& shape) const;
  struct PathEvaluation {
    bool drivable = false;
    double paint_contact_seconds = 0.0;
    double longest_paint_contact_seconds = 0.0;
    double initial_overlap_m = 0.0;
    double maximum_overlap_m = 0.0;
    double recovery_seconds = 0.0;
  };
  // PoseClear retains strict paint checking for diagnostics. Path evaluation
  // separates paint from hard road containment, so an existing paint overlap
  // never traps all recovery candidates at their shared initial pose.
  PathEvaluation EvaluatePath(const CartesianPath& path, const VehicleShape& shape,
                              double dt) const;
  // Only a small pre-existing overlap may recover. Must enter the original
  // grid within the distance budget / 5 s, then stay inside for the entire path.
  PathEvaluation EvaluateRecoveryPath(const CartesianPath& path, const VehicleShape& shape,
                                      double dt, double distance_budget_m = 6.0,
                                      double overlap_limit_m = .35, bool continuing_recovery = false) const;
  double FootprintOverlapDepth(double x, double y, double yaw, const VehicleShape& shape) const {
    return enabled_ ? FootprintOverlap(x,y,yaw,shape).maximum : 0.0;
  }
  bool FootprintInside(double x, double y, double yaw, const VehicleShape& shape) const {
    return !enabled_ || DrivablePoseClear(x, y, yaw, shape);
  }
  bool PathClear(const CartesianPath& path, const VehicleShape& shape) const;
  static double PaintCost(const PathEvaluation& evaluation);
 private:
  struct Point { double x, y; };
  struct Polygon { std::vector<Point> points; double xmin, ymin, xmax, ymax; };
  struct Segment { Point a, b; double xmin, ymin, xmax, ymax; };
  struct Overlap { double maximum = 0.0; double fraction = 0.0; };
  Overlap FootprintOverlap(double x, double y, double yaw, const VehicleShape& shape) const;
  PathEvaluation Evaluate(const CartesianPath& path, const VehicleShape& shape,
                          double dt, double recovery_distance_m, double overlap_limit_m,
                          bool continuing_recovery) const;
  bool Inside(double x, double y) const;
  bool DrivablePoseClear(double x, double y, double yaw, const VehicleShape& shape) const;
  bool TouchesPaint(double x, double y, double yaw, const VehicleShape& shape) const;
  bool enabled_ = false;
  double grid_resolution_ = 0.0, grid_x_ = 0.0, grid_y_ = 0.0;
  int grid_width_ = 0, grid_height_ = 0;
  std::vector<std::vector<std::pair<int,int>>> grid_rows_;
  std::vector<Polygon> areas_;
  std::vector<Segment> protected_lines_;
  std::unordered_map<std::uint64_t,std::vector<std::size_t>> line_bins_;
};
