#pragma once
#include "Frenet/Geometry/PolygonUtils.hpp"
#include "Frenet/Speed/SpeedProfiler.hpp"
#include <array>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace local_planning {
struct Pose {
  double x = 0, y = 0, yaw = 0, k = 0, s = 0;
};
struct Obstacle {
  int id = -1;
  geometry::Polygon2D polygon;
  std::string camera_class, classification;
  double source_age = 0, uncertainty = 0;
  double motion_budget = 5.;
  bool retained = false, moving = false;
  // LOCAL short relative-pose evidence, independent of semantic group/map GPS.
  // Original measured polygon and public LiDAR motion/velocity remain intact.
  geometry::Polygon2D motion_sweep;
  double local_vx = 0, local_vy = 0;
  double extra_clearance = 0; // Vulnerable-object space, never a motion assertion.
};
struct Config {
  double length = 4.635, width = 2.160835, rear = .79, margin = .10;
  double speed_cap = 25. / 3.6, decel = 1.5, accel = 1., jerk = 4.8;
  double controlled_decel = 1.5; // Candidate actual-state brake; cruise stays at decel.
  double preferred_clearance = .40;
  double low_speed_rate_floor = 1.;
  double maneuver_reserve = 0.; // Keep turn room if a verified prefix ends at an obstacle.
  double wheelbase = 3., gain = .844, max_k = .23, max_steer_rate = 1.;
  double lateral_accel = 2.6, delay = .5, max_range = 20., budget_ms = 40.;
  bool blind_assist = false, recovery = false;
  bool recovery_geometry = false; // Internal near-field policy; never a launch switch.
  int beam = 40;
};
struct Grid {
  int rows = 0, cols = 0;
  double resolution = .1, xmax = 20., ymax = 8.;
  std::vector<uint8_t> probability, coverage;
  std::vector<std::vector<int>> blocked_columns;
  std::vector<uint8_t> exterior_nonroad; // Current row exterior, not interior occlusion holes.
  std::vector<float>
      recent_road_age; // Original observation age;0 means no evidence.
  // base_link ground -> gravity-aligned planning plane, no second projection.
  std::array<double, 9> rotation{{1, 0, 0, 0, 1, 0, 0, 0, 1}};
  double ground_z = -.36;
  // Source level frame -> current planning level frame (x, y, yaw). The raster
  // remains in original source pixels; no second projection/resampling.
  std::array<double, 3> source_in_plan{{0, 0, 0}};
};
struct Scene {
  Grid grid;
  std::vector<Obstacle> obstacles;
  // Current family support: x, left painted line, right painted line. Its
  // longitudinal support is diagnostic. Painted lines are NOT
  // physical walls or footprint clipping bounds. LOCAL-2B uses right-home
  // support for navigation and the full semantic body sweep for physical range.
  std::vector<std::array<double, 3>> corridor;
  std::vector<std::array<double, 2>>
      home; // soft right-home reference, not a discrete lane/target selection
  double speed = 0, yaw_rate = 0, age = 0, acceleration = 0;
  bool aligned_to_plan = false;
  std::vector<Pose> previous;
  bool previous_recovery = false;
};
struct Result {
  bool valid = false, assisted = false, recovery = false;
  std::string reason = "no_path", speed_reason = "unavailable";
  std::vector<Pose> path;
  std::vector<std::array<double, 7>> trajectory; // x y yaw k v a t
  std::map<std::string, unsigned> rejected;
  unsigned expanded = 0;
  int blocking_id = -1;
  double blocked_x = 0, blocked_y = 0, minimum_clearance = 1e9, elapsed_ms = 0;
  double raw_cap = 0, curve_cap = 0, space_cap = 0, chosen_cap = 0;
  double stopping_cap = 0;
  double required_stopping_m = 0, available_stopping_m = 0;
  double comfort_stopping_m = 0, braking_limit = 1.5;
  double recovery_extension_m = 0;
  double maneuver_reserve_m = 0;
  bool budget_exhausted = false;
  double reference_end = 0, search_horizon = 0;
  std::string path_source = "new_search";
  unsigned remembered_cells = 0;
  unsigned soft_margin_cells = 0; // Sample visits, not unique physical cells.
  std::vector<std::array<double, 2>> goals;
  struct Rejected {
    Pose pose;
    std::string reason;
    int object_id;
  };
  std::vector<Rejected> rejected_examples;
  double alignment_ms = 0, search_ms = 0, speed_ms = 0, index_ms = 0;
  struct Candidate {
    std::vector<Pose> path;
    std::string reason;
    double lateral_target = 0, transition = 0;
  };
  std::vector<Candidate> candidates;
  unsigned candidates_omitted = 0;
  double recovery_cost = 0;
  unsigned recovery_obb_overlap = 0;
};
double ReferenceAt(const Scene &, double x); // Soft home cue, never a collision boundary.
// Distance-normalized final score; does not certify collision validity.
double SegmentQualityCost(const Scene &, const Config &, const Pose &, const Pose &, unsigned soft_cells);
void BuildGridIndex(Grid &); // Current raster index; never changes probability/coverage.
Result Plan(Scene, const Config &);
bool CheckFootprint(const Scene &, const Config &, const Pose &, Result &);
std::array<double, 3> Rotate(const std::array<double, 9> &, double, double,
                             double);
std::array<double, 9> GroundRotation(double dzdx, double dzdy);
std::array<double, 9> AttitudeRotation(double roll, double pitch);
std::array<double, 3> GridToPlan(const Grid &, double bx, double by);
std::array<double, 3> PlanToGrid(const Grid &, double x, double y);
bool CheckSweep(const Scene &, const Config &, const Pose &, const Pose &,
                Result &);
} // namespace local_planning
