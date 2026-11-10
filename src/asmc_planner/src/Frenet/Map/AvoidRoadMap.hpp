#pragma once
#include <string>
#include <vector>
#include <utility>
#include <unordered_map>
#include <cstdint>
#include "Frenet/Common/CartesianTypes.hpp"
#include "Frenet/Evaluation/CollisionChecker.hpp"

// Pinned reviewed map-frame polygons/lanes and their official 0.2m raster.
// Raster footprint containment is hard. AVOID additionally bounds predicted
// continuous solid-paint contact; dashed markings are not protected.
class AvoidRoadMap {
 public:
  bool Load(const std::string& areas, const std::string& lines, std::string& error);
  bool enabled() const { return enabled_; }
  const std::string& product_sha256()const{return product_sha256_;}
  void SetPaintExemption(bool value) { paint_exempt_=value; }
  // Private Localization map layer. It is active only with the scoped shadow
  // permission; the canonical raster and every default caller remain intact.
  bool SetShadowWallOverlay(const std::vector<geometry::Polygon2D>& polygons);
  // Full canonical XY footprint against the original raster; unknown is false.
  bool IntersectsDrivable(const geometry::Polygon2D& footprint) const;
  bool PoseClear(double x, double y, double yaw, const VehicleShape& shape) const;
  struct PathEvaluation {
    bool drivable = false;
    double paint_contact_seconds = 0.0;
    double longest_paint_contact_seconds = 0.0;
    // An already-running contact exceeded the target, but this road-safe path
    // exits the paint instead of holding the vehicle on it.
    bool best_effort_paint_exit = false;
    double initial_overlap_m = 0.0;
    double maximum_overlap_m = 0.0;
    double recovery_seconds = 0.0;
  };
  // AVOID bounds continuous body/solid-paint contact below the 3 s referee
  // window when budget_paint=true. Other maneuvers retain their paint policy.
  PathEvaluation EvaluatePath(const CartesianPath& path, const VehicleShape& shape,
                              double dt, bool budget_paint = false,
                              double prior_contact_s = 0.0) const;
  // Only a small pre-existing overlap may recover. Must enter the original
  // grid within the distance budget / 5 s, then stay inside for the entire path.
  PathEvaluation EvaluateRecoveryPath(const CartesianPath& path, const VehicleShape& shape,
                                      double dt, double distance_budget_m = 6.0,
                                      double overlap_limit_m = .35, bool continuing_recovery = false,
                                      double settled_travel_m = 0.0,
                                      bool budget_paint = false,
                                      double prior_contact_s = 0.0) const;
  // MERGE-only short bridge over a raster corner. Original grid stays intact;
  // a full return and settled clearance are mandatory within the supplied budget.
  PathEvaluation EvaluateMergeBoundaryBridge(const CartesianPath& path,
      const VehicleShape& shape, double dt, double remaining_distance_m,
      double remaining_time_s, double settled_travel_m = 0.0) const;
  double FootprintOverlapDepth(double x, double y, double yaw, const VehicleShape& shape) const {
    return enabled_ ? FootprintOverlap(x,y,yaw,shape).maximum : 0.0;
  }
  bool FootprintInside(double x, double y, double yaw, const VehicleShape& shape) const {
    return !enabled_ || DrivablePoseClear(x, y, yaw, shape);
  }
  bool PoseTouchesSolidPaint(double x, double y, double yaw,
                             const VehicleShape& shape) const;
  bool PathClear(const CartesianPath& path, const VehicleShape& shape) const;
  static double PaintCost(const PathEvaluation& evaluation);
 private:
  struct Point { double x, y; };
  struct Polygon { std::vector<Point> points; double xmin, ymin, xmax, ymax; };
  struct Segment { Point a, b; double xmin, ymin, xmax, ymax, half_width; bool legacy_boundary; };
  struct Overlap { double maximum = 0.0; double fraction = 0.0; };
  Overlap FootprintOverlap(double x, double y, double yaw, const VehicleShape& shape) const;
  PathEvaluation Evaluate(const CartesianPath& path, const VehicleShape& shape,
                          double dt, double recovery_distance_m, double overlap_limit_m,
                          bool continuing_recovery, double settled_travel_m = 0.0,
                          bool allow_initial_excursion = false, double time_budget_s = 5.0,
                          bool budget_paint = false,
                          double prior_contact_s = 0.0) const;
  bool Inside(double x, double y) const;
  bool DrivablePoseClear(double x, double y, double yaw, const VehicleShape& shape) const;
  bool TouchesPaint(double x, double y, double yaw, const VehicleShape& shape,
                    bool all_solids = false) const;
  bool enabled_ = false;
  bool paint_exempt_ = false;
  double grid_resolution_ = 0.0, grid_x_ = 0.0, grid_y_ = 0.0;
  int grid_width_ = 0, grid_height_ = 0;
  std::vector<std::vector<std::pair<int,int>>> grid_rows_;
  std::vector<Polygon> areas_;
  std::vector<Polygon> shadow_wall_areas_;
  std::string product_sha256_;
  std::vector<Segment> protected_lines_;
  std::unordered_map<std::uint64_t,std::vector<std::size_t>> line_bins_;
};
