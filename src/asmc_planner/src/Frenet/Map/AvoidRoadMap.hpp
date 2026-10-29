#pragma once
#include <string>
#include <vector>
#include <utility>
#include <unordered_map>
#include <cstdint>
#include "Frenet/Common/CartesianTypes.hpp"
#include "Frenet/Evaluation/CollisionChecker.hpp"

// Pinned reviewed map-frame polygons/lanes and their official 0.2m raster.
// Raster footprint containment is hard. AVOID additionally scores predicted
// continuous solid-paint contact; dashed markings are not protected.
class AvoidRoadMap {
 public:
  bool Load(const std::string& areas, const std::string& lines, std::string& error);
  bool enabled() const { return enabled_; }
  const std::string& product_sha256()const{return product_sha256_;}
  void SetPaintExemption(bool value) { paint_exempt_=value; }
  // Private Localization map layer. It is active only with the scoped shadow
  // permission; the canonical raster and every default caller remain intact.
  void SetShadowWallPermission(bool value){wall_permission_=value;}
  bool SetShadowWallOverlay(const std::vector<geometry::Polygon2D>& polygons);
  // Full canonical XY footprint against the original raster; unknown is false.
  bool IntersectsDrivable(const geometry::Polygon2D& footprint,
                          bool clip_to_grid = false) const;
  bool PoseClear(double x, double y, double yaw, const VehicleShape& shape) const;
  struct PathEvaluation {
    bool drivable = false;
    double paint_contact_seconds = 0.0;
    double longest_paint_contact_seconds = 0.0;
    // An already-running contact exits the paint and stays clear. Diagnostic
    // only: paint duration is a cost, never a road/collision exemption.
    bool best_effort_paint_exit = false;
    double initial_overlap_m = 0.0;
    double maximum_overlap_m = 0.0;
    double recovery_seconds = 0.0;
    double overlap_exposure_m_s = 0.0;
    double outside_fraction_seconds = 0.0;
  };
  // AVOID scores continuous body/solid-paint contact when budget_paint=true.
  // Include the observed contact age; there is no paint-duration rejection.
  // A stopped endpoint on paint is scored over at least five seconds, so a
  // one-point hold cannot appear cheaper than leaving the same paint.
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
  // Explicit low-speed departure policy for an already partially outside body.
  // No depth/fraction/distance/time cutoff. Full separation is forbidden;
  // a complete settled return is mandatory. Existing maneuver policy is unchanged.
  PathEvaluation EvaluateDepartureRecoveryPath(const CartesianPath& path,
      const VehicleShape& shape, double dt, bool continuing_recovery = false,
      double settled_travel_m = 0.0) const;
  static double DepartureRecoveryCost(const PathEvaluation& evaluation);
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
  Overlap FootprintOverlap(double x, double y, double yaw, const VehicleShape& shape,
                           bool full_footprint_search = false) const;
  PathEvaluation Evaluate(const CartesianPath& path, const VehicleShape& shape,
                          double dt, double recovery_distance_m, double overlap_limit_m,
                          bool continuing_recovery, double settled_travel_m = 0.0,
                          bool allow_initial_excursion = false, double time_budget_s = 5.0,
                          bool budget_paint = false,
                          double prior_contact_s = 0.0,
                          bool partial_departure = false) const;
  bool Inside(double x, double y) const;
  bool DrivablePoseClear(double x, double y, double yaw, const VehicleShape& shape) const;
  bool TouchesPaint(double x, double y, double yaw, const VehicleShape& shape,
                    bool all_solids = false) const;
  bool enabled_ = false;
  bool wall_permission_=false;
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
