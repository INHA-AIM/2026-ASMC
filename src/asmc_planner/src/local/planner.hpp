#pragma once
#include "Frenet/Geometry/PolygonUtils.hpp"
#include "Frenet/Geometry/ObjectTypes.hpp"
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
  TrackedObject observation; // optional authoritative 8-corner research input
  double stamp = 0;
};
struct Config {
  double length = 4.635, width = 2.160835, rear = .79, margin = .10;
  double speed_cap = 25. / 3.6, decel = 1.5, accel = 2., jerk = 4.8;
  double wheelbase = 3., gain = .844, max_k = .23, max_steer_rate = 1.;
  double lateral_accel = 2.6, delay = .5, max_range = 20., budget_ms = 40.;
  bool blind_assist = false, recovery = false;
  int beam = 40;
};
struct Grid {
  int rows = 0, cols = 0;
  double resolution = .1, xmax = 20., ymax = 8.;
  std::vector<uint8_t> probability, coverage;
  std::vector<std::vector<int>> blocked_columns;
  // base_link ground -> gravity-aligned planning plane, no second projection.
  std::array<double, 9> rotation{{1, 0, 0, 0, 1, 0, 0, 0, 1}};
  double ground_z = -.36;
};
struct Scene {
  Grid grid;
  std::vector<Obstacle> obstacles;
  // Current family support: x, left painted line, right painted line. Only its
  // longitudinal support currently limits the horizon. Painted lines are NOT
  // physical walls or footprint clipping bounds; qualified free road can extend
  // beyond them. Decoupling horizon validity from all three roles is LOCAL-2A/B.
  std::vector<std::array<double, 3>> corridor;
  std::vector<std::array<double, 2>>
      home; // soft right-home reference, not a discrete lane/target selection
  double speed = 0, yaw_rate = 0, age = 0, acceleration = 0;
  std::vector<Pose> previous;
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
  bool budget_exhausted = false;
  std::vector<std::array<double, 2>> goals;
  struct Rejected {
    Pose pose;
    std::string reason;
    int object_id;
  };
  std::vector<Rejected> rejected_examples;
  double alignment_ms = 0, search_ms = 0, speed_ms = 0;
  struct Candidate {
    std::vector<Pose> path;
    std::string reason;
  };
  std::vector<Candidate> candidates;
  unsigned candidates_omitted = 0;
  double recovery_cost = 0;
  unsigned recovery_obb_overlap = 0;
};
Result Plan(const Scene &, const Config &);
// Shared timing/braking owner, called only after full local swept validation.
Result TimeParameterize(const Scene &, const Config &, Result,
                       const std::vector<double>& speed_limits = {});
bool CheckFootprint(const Scene &, const Config &, const Pose &, Result &);
std::array<double, 3> Rotate(const std::array<double, 9> &, double, double,
                             double);
std::array<double, 9> GroundRotation(double dzdx, double dzdy);
std::array<double, 9> AttitudeRotation(double roll, double pitch);
bool CheckSweep(const Scene &, const Config &, const Pose &, const Pose &,
                Result &);
} // namespace local_planning
