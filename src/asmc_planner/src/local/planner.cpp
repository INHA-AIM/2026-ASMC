#include "local/planner.hpp"
#include "Frenet/Evaluation/CollisionChecker.hpp"
#include "Frenet/Math/Polynomial.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <tuple>

namespace local_planning {
std::array<double, 3> Rotate(const std::array<double, 9> &r, double x, double y,
                             double z) {
  return {r[0] * x + r[1] * y + r[2] * z, r[3] * x + r[4] * y + r[5] * z,
          r[6] * x + r[7] * y + r[8] * z};
}
std::array<double, 9> AttitudeRotation(double roll, double pitch) {
  const double cr = std::cos(roll), sr = std::sin(roll), cp = std::cos(pitch),
               sp = std::sin(pitch);
  // Heading-free R_y(pitch) R_x(roll). Unlike a minimal normal rotation this
  // preserves the same heading convention as yaw-rate integration.
  return {cp, sp * sr, sp * cr, 0, cr, -sr, -sp, cp * sr, cp * cr};
}
std::array<double, 9> GroundRotation(double a, double b) {
  return AttitudeRotation(std::atan2(-b, 1.),
                          std::atan2(a, std::sqrt(1 + b * b)));
}
namespace {
// One affine transform per footprint check; no trigonometry per raster cell.
// The raster stays in its original camera plane, including unknown cells.
struct GridTransform {
  const Grid &g;
  double c, s;
  explicit GridTransform(const Grid &grid)
      : g(grid), c(std::cos(g.source_in_plan[2])),
        s(std::sin(g.source_in_plan[2])) {}
  std::array<double, 3> toPlan(double bx, double by) const {
    const auto &R = g.rotation;
    const double bz = (g.ground_z - R[6] * bx - R[7] * by) / R[8];
    const auto q = Rotate(R, bx, by, bz);
    return {g.source_in_plan[0] + c * q[0] - s * q[1],
            g.source_in_plan[1] + s * q[0] + c * q[1], q[2]};
  }
  std::array<double, 3> toGrid(double x, double y) const {
    const double dx = x - g.source_in_plan[0], dy = y - g.source_in_plan[1];
    const double sx = c * dx + s * dy, sy = -s * dx + c * dy;
    const auto &R = g.rotation;
    return {R[0] * sx + R[3] * sy + R[6] * g.ground_z,
            R[1] * sx + R[4] * sy + R[7] * g.ground_z,
            R[2] * sx + R[5] * sy + R[8] * g.ground_z};
  }
};
} // namespace
std::array<double, 3> GridToPlan(const Grid &g, double x, double y) {
  return GridTransform(g).toPlan(x, y);
}
std::array<double, 3> PlanToGrid(const Grid &g, double x, double y) {
  return GridTransform(g).toGrid(x, y);
}
namespace {
double elapsed(std::chrono::steady_clock::time_point t) {
  return std::chrono::duration<double, std::milli>(
             std::chrono::steady_clock::now() - t)
      .count();
}
double homeAt(const Scene &s, double x) {
  if (s.home.empty())
    return 0.;
  for (size_t i = 1; i < s.home.size(); ++i)
    if (s.home[i][0] >= x) {
      auto a = s.home[i - 1], b = s.home[i];
      // Navigation tangent may reach the axle from a near-field paint gap.
      // This does not fill a single road/coverage cell.
      double q = std::min(1., (x - a[0]) / std::max(1e-6, b[0] - a[0]));
      return a[1] + q * (b[1] - a[1]);
    }
  return s.home.back()[1];
}
double homeSlope(const Scene &s, double x) {
  return (homeAt(s, x + .25) - homeAt(s, x - .25)) / .5;
}
double homeSecond(const Scene &s, double x) {
  return (homeAt(s, x + .25) - 2 * homeAt(s, x) + homeAt(s, x - .25)) / .0625;
}
// Exact constant-jerk ramp from current acceleration to the configured brake.
// Positive acceleration is part of stopping distance, not instantaneous
// braking.
double brakingDistance(double v, double a, const Config &c) {
  if (v <= 0)
    return 0.;
  a = std::max(-c.decel, a);
  double ramp = (a + c.decel) / c.jerk;
  double zero = (a + std::sqrt(a * a + 2 * c.jerk * v)) / c.jerk;
  double t = std::min(ramp, zero);
  double d = v * t + .5 * a * t * t - c.jerk * t * t * t / 6.;
  double rest = std::max(0., v + a * t - .5 * c.jerk * t * t);
  return d + rest * rest / (2 * c.decel);
}
double spaceSpeed(double distance, double acceleration, double delay,
                  const Config &c) {
  double lo = 0., hi = std::max(15., c.speed_cap);
  for (int i = 0; i < 24; ++i) {
    double v = .5 * (lo + hi);
    if (v * delay + brakingDistance(v, acceleration, c) <=
        std::max(0., distance))
      lo = v;
    else
      hi = v;
  }
  return lo;
}
bool fail(Result &r, const char *reason, double x, double y, int id = -1) {
  ++r.rejected[reason];
  r.blocked_x = x;
  r.blocked_y = y;
  r.blocking_id = id;
  return false;
}
} // namespace
bool CheckFootprint(const Scene &s, const Config &c, const Pose &p, Result &r) {
  CartesianState state{p.x, p.y, p.yaw, p.k, 0, 0};
  VehicleShape shape{c.width, c.length, c.rear};
  const auto physical = MakeEgoFootprint(state, shape, 0.);
  const double clearance_margin =
      c.margin + std::min(.25, .015 * s.speed * s.speed);
  for (const auto &o : s.obstacles) {
    double distance = geometry::ConvexPolygonDistance(physical, o.polygon);
    r.minimum_clearance = std::min(r.minimum_clearance, distance);
    if (distance <= clearance_margin + o.uncertainty)
      return fail(r, "local_obb", p.x, p.y, o.id);
  }
  const auto &g = s.grid;
  const GridTransform transform(g);
  if (g.rows <= 0 || g.cols <= 0 ||
      g.probability.size() != size_t(g.rows * g.cols) ||
      g.coverage.size() != g.probability.size())
    return fail(r, "grid_invalid", p.x, p.y);
  const double cs = std::cos(p.yaw), sn = std::sin(p.yaw);
  // Cell half-diagonal covers discretisation; the physical footprint is not
  // shrunk. A measured plane is undone only for querying its original raster.
  const double pad = clearance_margin + g.resolution * std::sqrt(.5);
  const double front = c.length - c.rear + pad, rear = c.rear + pad,
               half = c.width * .5 + pad;
  int row0 = g.rows, row1 = 0, col0 = g.cols, col1 = 0;
  for (double a : {-rear, front})
    for (double b : {-half, half}) {
      double x = p.x + cs * a - sn * b, y = p.y + sn * a + cs * b;
      const auto source = transform.toGrid(x, y);
      double bx = source[0], by = source[1];
      int row = std::lround((g.xmax - bx) / g.resolution),
          col = std::lround((g.ymax - by) / g.resolution);
      if (row < 0 || row >= g.rows || col < 0 || col >= g.cols)
        return fail(r, "grid_extent", x, y);
      row0 = std::min(row0, row);
      row1 = std::max(row1, row);
      col0 = std::min(col0, col);
      col1 = std::max(col1, col);
    }
  for (int row = row0; row <= row1; ++row) {
    std::vector<int> fallback;
    if (g.blocked_columns.empty()) {
      for (int col = col0; col <= col1; ++col)
        if (g.coverage[row * g.cols + col] != 255 ||
            g.probability[row * g.cols + col] < 153)
          fallback.push_back(col);
    }
    const auto &columns =
        g.blocked_columns.empty() ? fallback : g.blocked_columns[row];
    for (auto it = std::lower_bound(columns.begin(), columns.end(), col0);
         it != columns.end() && *it <= col1; ++it) {
      int col = *it;
      double bx = g.xmax - row * g.resolution, by = g.ymax - col * g.resolution;
      // The published grid's points lie on n.p=ground_z. Restore their Z before
      // changing coordinates, rather than applying camera projection twice.
      auto q = transform.toPlan(bx, by);
      double x = q[0], y = q[1];
      double dx = x - p.x, dy = y - p.y, a = cs * dx + sn * dy,
             b = -sn * dx + cs * dy;
      if (a < -rear || a > front || std::abs(b) > half)
        continue;
      const int index = row * g.cols + col;
      auto cover = g.coverage[index];
      // Current occupied ego footprint is a geometric fact, not future
      // free-space. Half a cell covers quantization at the occupied body edge.
      bool current_body = x >= -c.rear - .5 * g.resolution &&
                          x <= c.length - c.rear + .5 * g.resolution &&
                          std::abs(y) <= c.width * .5 + .5 * g.resolution;
      if (cover != 255) {
        if (current_body)
          continue;
        // Unknown clearance padding is not a measured collision. The physical
        // swept body still requires evidence; observed nonroad/OBB retain
        // margin.
        if (a < -c.rear || a > c.length - c.rear || std::abs(b) > c.width * .5)
          continue;
        if (g.recent_road_age.size() == g.coverage.size() &&
            g.recent_road_age[index] > 0 && g.recent_road_age[index] <= 5.) {
          ++r.remembered_cells;
          continue;
        }
        bool assist = c.blind_assist && bx >= -c.rear - .35 && bx <= 6.5 &&
                      ((cover == 128 && std::abs(by) <= 2.5) ||
                       (cover == 0 && std::abs(by) <= 1.6));
        if (assist) {
          r.assisted = true;
          continue;
        }
        return fail(r, cover == 128 ? "ego_occlusion" : "unobserved", x, y);
      }
      if (g.probability[index] < 153) {
        // Keep physical body +10cm +cell quantization hard. Additional
        // speed-dependent semantic clearance is a preference, not collision.
        // Otherwise a single bonnet-edge pixel30cm outside the body can
        // truncate a clear15m path to40cm at speed. LiDAR veto is unchanged.
        const double hard = c.margin + g.resolution * std::sqrt(.5);
        if (a >= -c.rear - hard && a <= c.length - c.rear + hard &&
            std::abs(b) <= c.width * .5 + hard)
          return fail(r, "observed_nonroad", x, y);
        ++r.soft_margin_cells;
      }
    }
  }
  return true;
}
bool CheckSweep(const Scene &s, const Config &c, const Pose &a, const Pose &b,
                Result &r) {
  const double d = std::hypot(b.x - a.x, b.y - a.y), turn = b.yaw - a.yaw;
  const int steps = std::max(1, int(std::ceil((d + std::abs(turn) * c.length) /
                                              (.5 * s.grid.resolution))));
  for (int i = 0; i <= steps; ++i) {
    double q = double(i) / steps;
    Pose m{a.x + q * (b.x - a.x), a.y + q * (b.y - a.y), a.yaw + q * turn,
           a.k + q * (b.k - a.k), 0};
    if (!CheckFootprint(s, c, m, r))
      return false;
  }
  return true;
}
// A common distance integral for analytic, lattice and rechecked paths.
// Goal heuristics guide search only; they cannot bias final cross-method rank.
double SegmentQualityCost(const Scene &s, const Config &c, const Pose &a,
                          const Pose &b, unsigned soft_cells) {
  const double distance = std::hypot(b.x-a.x, b.y-a.y);
  if (distance < 1e-8) return 0.;
  auto previous_y = [&](double x, double fallback) {
    for (size_t i=1; i<s.previous.size(); ++i) {
      const auto &p=s.previous[i-1], &q=s.previous[i];
      if (x >= p.x && x <= q.x && q.x > p.x)
        return p.y + (q.y-p.y)*(x-p.x)/(q.x-p.x);
    }
    // No previous support at this x is not evidence to hold its endpoint.
    return fallback;
  };
  auto density = [&](double t) {
    const double x=a.x+t*(b.x-a.x), y=a.y+t*(b.y-a.y), k=a.k+t*(b.k-a.k);
    return .04*std::pow(y-homeAt(s,x),2) + .2*k*k +
           .20*std::pow(y-previous_y(x,y),2);
  };
  const int samples = 1 + std::max(1, int(std::ceil(
      (distance+std::abs(b.yaw-a.yaw)*c.length)/(.5*s.grid.resolution))));
  // Soft margin is a mean sample density, not a raw count dependent on
  // segment length. Curvature change is squared derivative per metre.
  return distance*(density(0)+4*density(.5)+density(1))/6. +
         std::pow(b.k-a.k,2)/distance + .002*distance*soft_cells/samples;
}
namespace {
// Simulation recovery requested for LOCAL-1: fresh semantic evidence continues
// to guide a short 3km/h path even if a conservative swept-footprint gate
// fails. This is explicitly NOT collision-free certification. OBB intrusion is
// costly, retained in the trace, and any physical contact fails the experiment.
bool Recovery(const Scene &s, const Config &c, Result &out) {
  if (!c.recovery)
    return false;
  const auto &g = s.grid;
  auto cell = [&](double x, double y) {
    const auto source = PlanToGrid(g, x, y);
    double bx = source[0], by = source[1];
    int row = std::lround((g.xmax - bx) / g.resolution),
        col = std::lround((g.ymax - by) / g.resolution);
    return row >= 0 && row < g.rows && col >= 0 && col < g.cols
               ? row * g.cols + col
               : -1;
  };
  // Find distinct near-field observed road intervals. HMAAC's near-row
  // weighting is used as guidance, but its first/last-pixel midpoint is not
  // copied: that midpoint can lie in an obstacle between disjoint components.
  std::vector<std::array<double, 2>> goals;
  for (double x : {4.5, 6., 8.}) {
    bool open = false;
    double first = 0, last = 0;
    for (int column = 0; column <= 100; ++column) {
      const double y = -5. + .1 * column;
      int i = cell(x, y);
      bool road = i >= 0 && g.coverage[i] == 255 && g.probability[i] >= 166;
      if (road) {
        if (!open)
          first = y;
        last = y;
        open = true;
      }
      if (open && (!road || column == 100)) {
        if (last - first >= c.width) {
          goals.push_back({x, .5 * (first + last)});
          // A broad connected road should not pull an empty-road vehicle
          // away from its original lane merely to reach the whole-road mean.
          const double preferred = std::clamp(homeAt(s, x), first+c.width*.5,
                                                          last-c.width*.5);
          if (std::abs(preferred - .5 * (first + last)) > .15)
            goals.push_back({x, preferred});
        }
        open = false;
      }
    }
  }
  if (goals.empty())
    return false;
  double best_cost = std::numeric_limits<double>::infinity();
  std::vector<Pose> best;
  unsigned best_overlap = 0;
  for (const auto &goal : goals) {
    const double requested =
        std::clamp(2 * goal[1] / (goal[0] * goal[0] + goal[1] * goal[1]),
                   -c.max_k, c.max_k);
    Pose p;
    p.k =
        s.speed > .3 ? std::clamp(s.yaw_rate / s.speed, -c.max_k, c.max_k) : 0.;
    std::vector<Pose> path{p};
    double cost = 0.;
    unsigned overlap = 0;
    int known = 0;
    bool physical_collision = false;
    for (int step = 1; step <= 40; ++step) {
      const double delta=std::atan(c.wheelbase*p.k/c.gain),
                   desired=std::atan(c.wheelbase*requested/c.gain),
                   span=c.max_steer_rate*.1/std::max(1.,s.speed);
      double k = c.gain*std::tan(std::clamp(desired,delta-span,delta+span))/c.wheelbase,
             turn = .05 * (p.k + k);
      p.x += .1 * std::cos(p.yaw + turn * .5);
      p.y += .1 * std::sin(p.yaw + turn * .5);
      p.yaw += turn;
      p.s += .1;
      p.k = k;
      if (std::abs(p.yaw) > .65)
        break;
      path.push_back(p);
      CartesianState state{p.x, p.y, p.yaw, p.k, 0, 0};
      auto body =
          MakeEgoFootprint(state, VehicleShape{c.width, c.length, c.rear}, 0.);
      for (auto &o : s.obstacles) {
        double d = geometry::ConvexPolygonDistance(body, o.polygon);
        // Recovery may soften unobserved near-body raster/motion padding,
        // but cannot select a path through an actual measured OBB. Evaluate
        // every10cm pose, not only the coarse semantic scoring samples.
        if (d<=1e-9) { physical_collision=true;break; }
        if (d <= o.uncertainty) {
          cost += 50.;
          ++overlap;
        } else
          cost += .05 / std::max(.05, d);
      }
      if (physical_collision) break;
      if (step % 4) continue;
      for (double a : {0., c.length - c.rear})
        for (double b : {-c.width * .5, 0., c.width * .5}) {
          double x = p.x + std::cos(p.yaw) * a - std::sin(p.yaw) * b,
                 y = p.y + std::sin(p.yaw) * a + std::cos(p.yaw) * b;
          int i = cell(x, y);
          if (i < 0) {
            cost += 10.;
            continue;
          }
          if (g.coverage[i] == 255) {
            ++known;
            cost += g.probability[i] < 153 ? 4. : 0.;
          } else if (g.coverage[i] != 128)
            cost += .8;
          // The known hood/body mask has no left/right obstacle evidence.
          // Penalising it as nonroad steers an empty-road car towards the
          // visible sides. This neutrality is recovery-only, not free space.
        }
      cost += .02 * std::pow(p.y - homeAt(s, p.x), 2) + .1 * p.k * p.k;
      if (!s.previous.empty()) {
        auto old = std::min_element(
            s.previous.begin(), s.previous.end(), [&](auto &a, auto &b) {
              return std::abs(a.x - p.x) < std::abs(b.x - p.x);
            });
        cost += .02 * std::pow(p.y - old->y, 2);
      }
    }
    cost += .3 * std::abs(goal[1] - homeAt(s, goal[0])) + .2 * (8. - goal[0]);
    if (!physical_collision && path.size() > 10 && known >= 10 && cost < best_cost) {
      best_cost = cost;
      best = std::move(path);
      best_overlap = overlap;
    }
  }
  if (best.empty())
    return false;
  out.path = std::move(best);
  out.goals = std::move(goals);
  out.assisted = true;
  out.recovery = true;
  out.recovery_cost = best_cost;
  out.recovery_obb_overlap = best_overlap;
  return true;
}
} // namespace
static Result PlanPrepared(const Scene &, const Config &);
Result Plan(Scene indexed, const Config &c) {
  const auto started = std::chrono::steady_clock::now();
  Result out;
  auto &grid = indexed.grid;
  if (grid.rows <= 0 || grid.cols <= 0 || grid.rows > 1000 ||
      grid.cols > 1000 || !std::isfinite(grid.resolution) ||
      grid.resolution <= 0 ||
      grid.probability.size() != size_t(grid.rows * grid.cols) ||
      grid.coverage.size() != grid.probability.size()) {
    out.reason = "grid_invalid";
    return out;
  }
  grid.blocked_columns.assign(grid.rows, {});
  for (int row = 0; row < grid.rows; ++row)
    for (int col = 0; col < grid.cols; ++col) {
      int index = row * grid.cols + col;
      if (grid.coverage[index] != 255 || grid.probability[index] < 153)
        grid.blocked_columns[row].push_back(col);
    }
  const double index_ms = elapsed(started);
  auto out_prepared = PlanPrepared(indexed, c);
  out_prepared.index_ms = index_ms;
  out_prepared.elapsed_ms = elapsed(started);
  return out_prepared;
}
static Result PlanPrepared(const Scene &s, const Config &c) {
  const auto start = std::chrono::steady_clock::now();
  Result out;
  if (!std::isfinite(s.speed + s.age + s.yaw_rate + s.acceleration) ||
      s.speed < 0 || s.age < 0 || s.age > .35 || s.home.size() < 2) {
    out.reason = "invalid_scene";
    return out;
  }
  struct Node {
    std::vector<Pose> path;
    double cost = 0;
    bool assisted = false;
    size_t goal = 0;
    unsigned soft_margin_cells = 0;
    double quality = 0; // final distance-normalized ranking, no goal heuristic
  };
  Pose origin;
  origin.k =
      s.speed > .3 ? std::clamp(s.yaw_rate / s.speed, -c.max_k, c.max_k) : 0.;
  // Paint is a rear-axle navigation reference, not the end of free road.
  // The full-body grid/OBB check independently limits physical progress.
  const double margin = c.margin + std::min(.25, .015 * s.speed * s.speed) +
                        s.grid.resolution * std::sqrt(.5);
  const double raster_end =
      std::min(GridToPlan(s.grid, s.grid.xmax, -c.width * .5)[0],
               GridToPlan(s.grid, s.grid.xmax, c.width * .5)[0]) -
      (c.length - c.rear) - margin;
  const double horizon = std::min({c.max_range, s.home.back()[0], raster_end});
  out.reference_end = s.home.back()[0];
  out.search_horizon = horizon;
  // Separate connected lateral free intervals; never average across an
  // obstacle. Goals guide the search only. Every connecting swept body is still
  // checked.
  for (double gx : {6., 9., 12.}) {
    if (gx > horizon)
      continue;
    bool open = false;
    double first = 0, last = 0;
    for (double gy = -6.; gy <= 6.0001; gy += .1) {
      Result probe;
      Pose pose{gx, gy, 0, 0, 0};
      const bool free = CheckFootprint(s, c, pose, probe);
      if (free) {
        if (!open)
          first = gy;
        last = gy;
        open = true;
      }
      if (open && (!free || gy >= 6.)) {
        double preferred = std::clamp(homeAt(s, gx), first, last);
        out.goals.push_back({gx, preferred});
        if (std::abs(preferred - .5 * (first + last)) > .15)
          out.goals.push_back({gx, .5 * (first + last)});
        open = false;
      }
    }
  }
  if (out.goals.empty())
    out.goals.push_back(
        {std::max(1., horizon), homeAt(s, std::max(1., horizon))});
  // Bound both spatial alternatives and debug output.
  if (out.goals.size() > 12)
    out.goals.resize(12);
  std::vector<Node> beam;
  for (size_t i = 0; i < out.goals.size(); ++i)
    beam.push_back({{origin}, 0, false, i});
  Node best = beam[0];
  // Seed the bounded search with existing Frenet quintics. This is a family
  // of continuous rear-axle paths, not discrete lane selection. Dense sweep and
  // the same steering limits validate every seed before it can beat the
  // lattice.
  Node analytic_best = best;
  std::vector<double> lateral_targets;
  for (auto &goal : out.goals)
    if (std::none_of(
            lateral_targets.begin(), lateral_targets.end(), [&](double y) {
              return std::abs(y - (goal[1] - homeAt(s, goal[0]))) < .05;
            }))
      lateral_targets.push_back(goal[1] - homeAt(s, goal[0]));
  // Round-robin lateral alternatives at each transition distance. Previously
  // all12 straight-reference seeds could consume the analytic time slice
  // before either bypass side was tried. Geometry/collision/ranking stay the
  // same; no extra search budget or rollout is added.
  for (double transition :
       {3., 4., 5., 6., 7., 8., 9., 10., 11., 12., 14., 16.})
    for (double lateral : lateral_targets) {
      if (elapsed(start) > c.budget_ms * .6)
        break;
      if (transition > horizon - .3)
        continue;
      // The reference is the measured home strip, not a global straight line.
      // Meet its tangent/curvature, then follow it. A scalar target with yaw=0
      // after transition used to understeer curved tunnel lanes.
      auto polynomial = MakeQuintic(
          0., 0., origin.k, homeAt(s, transition) + lateral,
          homeSlope(s, transition), homeSecond(s, transition), transition);
      Node candidate{{origin}, 0, false, 0};
      Result check;
      for (double x = .2; x <= horizon; x += .2) {
        double t = std::min(x, transition),
               y = x < transition ? EvalPos(polynomial, t)
                                  : homeAt(s, x) + lateral,
               d = x < transition ? EvalVel(polynomial, t) : homeSlope(s, x),
               dd = x < transition ? EvalAcc(polynomial, t) : homeSecond(s, x);
        Pose b{x, y, std::atan(d), dd / std::pow(1 + d * d, 1.5), 0};
        const auto &a = candidate.path.back();
        double distance = std::hypot(b.x - a.x, b.y - a.y);
        b.s = a.s + distance;
        double steering_change =
            std::abs(std::atan(c.wheelbase * b.k / c.gain) -
                     std::atan(c.wheelbase * a.k / c.gain));
        if (std::abs(b.k) > c.max_k || std::abs(b.yaw) > .65 ||
            steering_change >
                c.max_steer_rate * distance / std::max(1., s.speed))
          break;
        ++out.expanded;
        const auto soft_before = check.soft_margin_cells;
        if (!CheckSweep(s, c, a, b, check))
          break;
        candidate.assisted |= check.assisted;
        out.remembered_cells += check.remembered_cells;
        candidate.quality += SegmentQualityCost(s,c,a,b,check.soft_margin_cells-soft_before);
        candidate.path.push_back(b);
        candidate.cost = candidate.quality;
        candidate.soft_margin_cells = check.soft_margin_cells;
      }
      if (out.candidates.size() < 12) {
        Result::Candidate example;
        example.lateral_target=lateral;
        example.transition=transition;
        example.reason =
            candidate.path.back().x >= horizon - .21
                ? "sweep_passed"
                : (!check.rejected.empty() ? check.rejected.begin()->first
                                           : "kinematic_limit");
        for (size_t k = 0; k < candidate.path.size(); k += 5)
          example.path.push_back(candidate.path[k]);
        example.path.push_back(candidate.path.back());
        out.candidates.push_back(std::move(example));
      } else
        ++out.candidates_omitted;
      for (auto &entry : check.rejected)
        out.rejected[entry.first] += entry.second;
      if (candidate.path.back().x > analytic_best.path.back().x + .1 ||
          (candidate.path.back().x >= analytic_best.path.back().x - .1 &&
           candidate.cost < analytic_best.cost))
        analytic_best = std::move(candidate);
    }
  const double ds = .5;
  for (int depth = 0;
       analytic_best.path.back().x < horizon - .21 && depth < int(horizon / ds);
       ++depth) {
    std::vector<Node> next;
    std::map<std::tuple<int, int, int, size_t>, double> seen;
    for (const auto &node : beam)
      for (double change : {-.06, -.03, 0., .03, .06}) {
        if (elapsed(start) > c.budget_ms) {
          out.budget_exhausted = true;
          break;
        }
        auto a = node.path.back();
        double k = std::clamp(a.k + change, -c.max_k, c.max_k);
        double steering_change =
            std::abs(std::atan(c.wheelbase * k / c.gain) -
                     std::atan(c.wheelbase * a.k / c.gain));
        if (steering_change > c.max_steer_rate * ds / std::max(1., s.speed)) {
          ++out.rejected["steering_rate"];
          continue;
        }
        Pose b = a;
        b.k = k;
        double turn = .5 * (a.k + k) * ds;
        b.x += ds * std::cos(a.yaw + turn * .5);
        b.y += ds * std::sin(a.yaw + turn * .5);
        b.yaw += turn;
        b.s += ds;
        if (b.x < 0 || std::abs(b.yaw) > .65) {
          ++out.rejected["heading"];
          continue;
        }
        // Reuse the same continuous swept-footprint test for new and old paths.
        Result check;
        bool clear = CheckSweep(s, c, a, b, check);
        ++out.expanded;
        for (const auto &entry : check.rejected)
          out.rejected[entry.first] += entry.second;
        if (!clear) {
          if (out.rejected_examples.size() < 32)
            out.rejected_examples.push_back(
                {b, check.rejected.begin()->first, check.blocking_id});
          out.blocking_id = check.blocking_id;
          out.blocked_x = check.blocked_x;
          out.blocked_y = check.blocked_y;
          continue;
        }
        const auto goal = out.goals[node.goal];
        double target = homeAt(s, b.x);
        // A look-ahead goal supplies an early lateral preference before a
        // blocked straight branch terminates. Beyond the opening return
        // preference wins.
        double desired =
            goal[1] * std::clamp(b.x / std::max(1., goal[0]), 0., 1.);
        double goal_cost =
            b.x <= goal[0] + 2. ? .15 * (b.y - desired) * (b.y - desired) : 0.;
        double cost = node.cost + goal_cost + .0001 * check.soft_margin_cells +
                      .04 * (b.y - target) * (b.y - target) + .2 * k * k +
                      2 * change * change;
        if (!s.previous.empty()) {
          const auto old = std::min_element(
              s.previous.begin(), s.previous.end(), [&](auto &p, auto &q) {
                return std::abs(p.x - b.x) < std::abs(q.x - b.x);
              });
          cost += .08 * (b.y - old->y) * (b.y - old->y);
        }
        auto key = std::make_tuple(int(std::lround(b.y / .15)),
                                   int(std::lround(b.yaw / .06)),
                                   int(std::lround(k / .03)), node.goal);
        if (seen.count(key) && seen[key] <= cost)
          continue;
        seen[key] = cost;
        Node candidate = node;
        candidate.path.push_back(b);
        candidate.cost = cost;
        candidate.quality += SegmentQualityCost(s,c,a,b,check.soft_margin_cells);
        candidate.assisted |= check.assisted;
        candidate.soft_margin_cells += check.soft_margin_cells;
        next.push_back(std::move(candidate));
      }
    if (next.empty())
      break;
    std::sort(next.begin(), next.end(),
              [](auto &a, auto &b) { return a.cost < b.cost; });
    std::vector<Node> diverse;
    std::vector<int> per_goal(out.goals.size(), 0);
    const int quota = std::max(3, c.beam / int(out.goals.size()));
    for (auto &node : next)
      if (per_goal[node.goal]++ < quota)
        diverse.push_back(std::move(node));
    next = std::move(diverse);
    best = *std::min_element(next.begin(), next.end(), [](const auto &a, const auto &b) {
      return a.quality < b.quality;
    });
    beam = std::move(next);
    if (out.budget_exhausted)
      break;
  }
  if (analytic_best.path.back().x > best.path.back().x + .1 ||
      (analytic_best.path.back().x >= best.path.back().x - .1 &&
       analytic_best.quality < best.quality))
    best = std::move(analytic_best);
  out.path = best.path;
  out.assisted = best.assisted;
  out.soft_margin_cells = best.soft_margin_cells;
  // Recheck previous feasible prefix as a complete candidate; never replay
  // stale control or an unchecked geometry merely because search exhausted its
  // budget.
  if (s.previous.size() > 1) {
    // Reuse only a geometrically continuous suffix. A stale arc-length or a
    // shortcut from today's axle to yesterday's path must never extend free
    // range.
    auto nearest = std::min_element(
        s.previous.begin(), s.previous.end(), [](auto &a, auto &b) {
          return std::hypot(a.x, a.y) < std::hypot(b.x, b.y);
        });
    std::vector<Pose> prefix{origin};
    Result check;
    double old_cost = 0.;
    if (std::hypot(nearest->x, nearest->y) < .20 &&
        std::abs(nearest->yaw) < .08) {
      for (auto it = nearest; it != s.previous.end(); ++it) {
        auto p = *it;
        if (p.x <= .1)
          continue;
        const auto &a = prefix.back();
        double ds = std::hypot(p.x - a.x, p.y - a.y), turn = p.yaw - a.yaw;
        const auto soft_before = check.soft_margin_cells;
        const double steering_change = std::abs(std::atan(c.wheelbase*p.k/c.gain)-
                                                std::atan(c.wheelbase*a.k/c.gain));
        if (ds > .80 || p.x > horizon || std::abs(p.k)>c.max_k ||
            steering_change > c.max_steer_rate*ds/std::max(1.,s.speed) ||
            std::abs(turn) > c.max_k * ds + .015 || !CheckSweep(s, c, a, p, check))
          break;
        p.s = a.s + ds;
        old_cost += SegmentQualityCost(s,c,a,p,check.soft_margin_cells-soft_before);
        prefix.push_back(p);
      }
      const bool similar = prefix.size() > 1 &&
                           prefix.back().x >= out.path.back().x - .2 &&
                           old_cost <= best.quality * 1.1 + .005;
      if (prefix.size() > 1 &&
          (prefix.back().x > out.path.back().x + .5 || similar)) {
        out.path_source = "rechecked_previous";
        out.path = std::move(prefix);
        out.assisted = check.assisted;
        out.soft_margin_cells = check.soft_margin_cells;
      }
    }
  }
  const double prefix_lag=c.delay+(s.aligned_to_plan?0.:s.age);
  const bool short_for_current_speed=out.path.size()>1 &&
      s.speed*prefix_lag+brakingDistance(s.speed,s.acceleration,c)>out.path.back().s;
  // A valid near-zero-speed prefix can otherwise keep a stationary car
  // stationary forever: it no longer violates current stopping distance.
  // Seek a longer alternative, never raise the speed of that short prefix.
  const bool no_crawl_progress=out.path.size()>1 &&
      std::max(0.,spaceSpeed(out.path.back().s-.1,c.accel,prefix_lag,c)-.35)<
      std::min(c.speed_cap,3./3.6);
  if ((out.path.size() < 3 || short_for_current_speed || no_crawl_progress) && c.recovery) {
    // Prefer a full-horizon swept path with explicitly bounded near-camera
    // unknown assistance over a short centroid arc. Known nonroad and OBBs
    // still veto this candidate, so a distant barrier causes early avoidance.
    // Also try it before rejecting a short valid prefix: drive01 had2m of
    // strict road at10km/h while a long crawl sweep remained available. This
    // is replanning/deceleration, not ignoring insufficient stopping space.
    Config aided = c;
    aided.recovery = false;
    aided.blind_assist = true;
    aided.speed_cap = std::min(c.speed_cap, 3. / 3.6);
    aided.budget_ms = std::max(1., c.budget_ms - elapsed(start));
    auto full = PlanPrepared(
        s, aided); // Reuse indexed scene; no second raster copy/index.
    if (full.valid && full.path.back().x >= 6.) {
      full.recovery = true;
      full.reason = "near_field_assisted_sweep";
      full.speed_reason = "near_field_recovery_3kph";
      full.raw_cap = c.speed_cap;
      full.elapsed_ms = elapsed(start);
      return full;
    }
  }
  // The full-sweep crawl alternative can still fail at a semantic shadow.
  // Try the separately labelled3km/h near-field recovery before turning a
  // short prefix into full STOP. Its actual-OBB veto above is unconditional;
  // terminal speed integration below must still fit the new path length.
  const bool recovered=(out.path.size()<3 || short_for_current_speed || no_crawl_progress) &&
      Recovery(s,c,out);
  if (out.path.size() < 3 && !recovered) {
    out.reason = "no_verified_prefix";
    out.elapsed_ms = elapsed(start);
    return out;
  }
  out.search_ms = elapsed(start);
  double maxk = 0;
  for (auto &p : out.path)
    maxk = std::max(maxk, std::abs(p.k));
  out.raw_cap = c.speed_cap;
  out.curve_cap = std::sqrt(c.lateral_accel * .95 / std::max(1e-4, maxk));
  // Path s is already rear-axle travel after a full-body collision check; do
  // not subtract the front overhang a second time. Reserve jerk ramp +
  // execution lag.
  double distance = out.path.back().s;
  // In aligned scenes the vehicle's source->plan travel already shortens the
  // available sweep. Keep execution/jerk delay, do not charge that travel
  // twice.
  double lag = c.delay + (s.aligned_to_plan ? 0. : s.age);
  // A braking frame must not instantly raise the cruise cap and restart the
  // throttle/brake cycle. Keep a positive-acceleration envelope, plus measured
  // LOCAL-2B tracking overshoot reserve (0.35m/s). Actual-state stopping
  // feasibility below remains separate and is never relaxed by this governor.
  out.stopping_cap = spaceSpeed(distance - .1, s.acceleration, lag, c);
  out.space_cap =
      std::max(0., spaceSpeed(distance - .1, c.accel, lag, c) - .35);
  out.chosen_cap = std::min({out.raw_cap, out.curve_cap, out.space_cap});
  out.speed_reason = out.chosen_cap == out.space_cap ? "verified_stopping_space"
                     : out.chosen_cap == out.curve_cap ? "curvature"
                                                       : "requested_cap";
  if (out.assisted && !out.recovery && out.chosen_cap > 5. / 3.6) {
    out.chosen_cap = 5. / 3.6;
    out.speed_reason = "experimental_near_blind_assist";
  }
  if (out.recovery) {
    out.chosen_cap = std::min(3. / 3.6, out.chosen_cap);
    out.speed_reason = "near_field_recovery_3kph";
  }
  if (!out.recovery &&
      s.speed * lag + brakingDistance(s.speed, s.acceleration, c) > distance) {
    out.reason = "insufficient_stopping_space";
    out.elapsed_ms = elapsed(start);
    return out;
  }
  double travelled = 0, v = s.speed, a = std::max(-c.decel, s.acceleration);
  size_t index = 1;
  for (int i = 0; i < 400; ++i) {
    while (index < out.path.size() - 1 && out.path[index].s < travelled)
      ++index;
    const auto &left = out.path[index - 1], &right = out.path[index];
    double q = std::clamp(
        (travelled - left.s) / std::max(1e-6, right.s - left.s), 0., 1.);
    out.trajectory.push_back({left.x + q * (right.x - left.x),
                              left.y + q * (right.y - left.y),
                              left.yaw + q * (right.yaw - left.yaw),
                              left.k + q * (right.k - left.k), v, a, i * .05});
    double remaining = std::max(0., distance - travelled);
    double cap =
        std::min(out.chosen_cap, spaceSpeed(remaining - .1, a, lag, c));
    // LOCAL-2C: scalar SpeedProfiler clips a lower cap immediately and is
    // deliberately unsuitable for a physical v/a trajectory. Use its shared
    // analytical transition, retaining endpoint acceleration (not average
    // delta-v/dt). Otherwise a tiny overspeed provokes a brake/reaccelerate
    // loop.
    const auto step =
        AdvancePhysicalSpeed({cap, v, a, .05}, {c.accel, c.decel, c.jerk});
    const double new_v = step.target_speed;
    a = step.target_acceleration;
    travelled += .5 * (v + new_v) * .05;
    v = new_v;
    if (travelled > distance + .05) {
      out.reason = "terminal_braking_violation";
      out.trajectory.clear();
      out.elapsed_ms = elapsed(start);
      return out;
    }
    if (i > 40 && v < .03 && remaining < .4) {
      out.trajectory.back()[4] = 0;
      break;
    }
  }
  out.valid = true;
  out.reason = out.recovery ? "near_field_recovery" : "ok";
  out.elapsed_ms = elapsed(start);
  out.speed_ms = out.elapsed_ms - out.search_ms;
  return out;
}
} // namespace local_planning
