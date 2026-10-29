#include "local/planner.hpp"
#include "jamming_common/geometry.hpp"
#include "local/speed_profile.hpp"
#include "local/recovery.hpp"
#include "Frenet/Evaluation/CollisionChecker.hpp"
#include "Frenet/Math/Polynomial.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <set>
#include <tuple>

namespace local_planning {
double ReferenceAt(const Scene &s, double x) {
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
namespace {
double elapsed(std::chrono::steady_clock::time_point t) {
  return std::chrono::duration<double, std::milli>(
             std::chrono::steady_clock::now() - t)
      .count();
}
double homeSlope(const Scene &s, double x) {
  return (ReferenceAt(s, x + .25) - ReferenceAt(s, x - .25)) / .5;
}
double homeSecond(const Scene &s, double x) {
  return (ReferenceAt(s, x + .25) - 2 * ReferenceAt(s, x) + ReferenceAt(s, x - .25)) / .0625;
}
} // namespace
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
    return .04*std::pow(y-ReferenceAt(s,x),2) + .2*k*k +
           // Preserve near-term control continuity, not an old avoidance
           // offset over the whole horizon. A constant5x-home history weight
           // kept drive16 about1.9m off home even after80m with no obstacle.
           .20*std::exp(-std::max(0.,x)/3.)*std::pow(y-previous_y(x,y),2);
  };
  const int samples = 1 + std::max(1, int(std::ceil(
      (distance+std::abs(b.yaw-a.yaw)*c.length)/(.5*s.grid.resolution))));
  // Soft margin is a mean sample density, not a raw count dependent on
  // segment length. Curvature change is squared derivative per metre.
  // Prefer a wider measured gap when one exists, without inflating the hard
  // collision envelope a second time or forcing a large-margin deadlock.
  const auto body=MakeEgoFootprint(CartesianState{(a.x+b.x)*.5,(a.y+b.y)*.5,
      (a.yaw+b.yaw)*.5,0,0,0},VehicleShape{c.width,c.length,c.rear},0.);
  double gap=1e9;
  for(const auto &o:s.obstacles)gap=std::min(gap,geometry::ConvexPolygonDistance(body,o.polygon));
  const double near=std::max(0.,c.preferred_clearance-gap);
  return distance*(density(0)+4*density(.5)+density(1))/6. + .4*distance*near*near +
         std::pow(b.k-a.k,2)/distance + .002*distance*soft_cells/samples;
}
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
  BuildGridIndex(grid);
  const double index_ms = elapsed(started);
  auto out_prepared = PlanPrepared(indexed, c);
  if(out_prepared.valid) {
    // Measure the selected geometry, not the minimum across rejected probes.
    Result measured;
    for(size_t i=1;i<out_prepared.path.size();++i) {
      const auto &p=out_prepared.path[i];
      const auto body=MakeEgoFootprint(CartesianState{p.x,p.y,p.yaw,0,0,0},VehicleShape{c.width,c.length,c.rear},0.);
      for(const auto &o:indexed.obstacles)
        measured.minimum_clearance=std::min(measured.minimum_clearance,geometry::ConvexPolygonDistance(body,o.polygon));
    }
    out_prepared.minimum_clearance=measured.minimum_clearance;
  }
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
      InitialCurvature(s,c);
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
        double preferred = std::clamp(ReferenceAt(s, gx), first, last);
        out.goals.push_back({gx, preferred});
        if (std::abs(preferred - .5 * (first + last)) > .15)
          out.goals.push_back({gx, .5 * (first + last)});
        open = false;
      }
    }
  }
  const bool observed_opening = !out.goals.empty();
  if (out.goals.empty())
    out.goals.push_back(
        {std::max(1., horizon), ReferenceAt(s, std::max(1., horizon))});
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
  // Full-body goal probes can all fail across an object's semantic occlusion
  // before a usable bypass is even tried. Add measured OBB-face guidance;
  // these are search seeds only, and every connecting body sweep is unchanged.
  // Prioritize objects intersecting the navigation reference before roadside
  // clutter can consume the bounded seed list. This only proposes goals: the
  // existing full-body sweep and observed-road checks still decide validity.
  std::vector<const Obstacle*> guidance;
  for(const auto &o:s.obstacles) guidance.push_back(&o);
  auto priority=[&](const Obstacle& o) {
    double x=1e9,lo=1e9,hi=-1e9;
    for(const auto& p:o.polygon) {x=std::min(x,p.x);lo=std::min(lo,p.y);hi=std::max(hi,p.y);}
    const double home=ReferenceAt(s,std::clamp(x,0.,horizon));
    const double pad=c.width*.5+c.margin+o.uncertainty;
    return std::make_tuple(home>=lo-pad && home<=hi+pad ? 0 : 1,x);
  };
  std::stable_sort(guidance.begin(),guidance.end(),[&](const auto* a,const auto* b){return priority(*a)<priority(*b);});
  for(const auto *source:guidance) {
    const auto &o=*source;
    double minx=1e9,lo=1e9,hi=-1e9;
    for(auto p:o.polygon) {minx=std::min(minx,p.x);lo=std::min(lo,p.y);hi=std::max(hi,p.y);}
    if(minx<4. || minx>s.grid.xmax)continue;
    double gx=std::min(minx,horizon),pad=c.width*.5+c.margin+o.uncertainty+o.extra_clearance+.25;
    if(gx<4.)continue;
    for(double side:{hi+pad,lo-pad}) {
      if(std::abs(side)>6.)continue;
      auto q=PlanToGrid(s.grid,gx,side);
      int row=std::lround((s.grid.xmax-q[0])/s.grid.resolution),col=std::lround((s.grid.ymax-q[1])/s.grid.resolution);
      if(row<0 || row>=s.grid.rows || col<0 || col>=s.grid.cols)continue;
      auto i=row*s.grid.cols+col;
      const double target=side-ReferenceAt(s,gx);
      if(s.grid.coverage[i]==255 && s.grid.probability[i]>=166 &&
         std::none_of(lateral_targets.begin(),lateral_targets.end(),[&](double y){return std::abs(y-target)<.10;}))
        lateral_targets.push_back(target);
    }
    if(lateral_targets.size()>=6)break;
  }
  for (auto &goal : out.goals)
    if (std::none_of(
            lateral_targets.begin(), lateral_targets.end(), [&](double y) {
              return std::abs(y - (goal[1] - ReferenceAt(s, goal[0]))) < .05;
            }))
      lateral_targets.push_back(goal[1] - ReferenceAt(s, goal[0]));
  // Round-robin lateral alternatives at each transition distance. Previously
  // all12 straight-reference seeds could consume the analytic time slice
  // before either bypass side was tried. Geometry/collision/ranking stay the
  // same; no extra search budget or rollout is added.
  for (double transition :
       {6., 8., 10., 4., 12., 5., 3., 7., 9., 11., 14., 16.})
    for (double lateral : lateral_targets) {
      if (elapsed(start) > c.budget_ms * .6)
        break;
      if (transition > horizon - .3)
        continue;
      // The reference is the measured home strip, not a global straight line.
      // Meet its tangent/curvature, then follow it. A scalar target with yaw=0
      // after transition used to understeer curved tunnel lanes.
      auto polynomial = MakeQuintic(
          0., 0., origin.k, ReferenceAt(s, transition) + lateral,
          homeSlope(s, transition), homeSecond(s, transition), transition);
      Node candidate{{origin}, 0, false, 0};
      Result check;
      for (double x = .2; x <= horizon; x += .2) {
        double t = std::min(x, transition),
               y = x < transition ? EvalPos(polynomial, t)
                                  : ReferenceAt(s, x) + lateral,
               d = x < transition ? EvalVel(polynomial, t) : homeSlope(s, x),
               dd = x < transition ? EvalAcc(polynomial, t) : homeSecond(s, x);
        Pose b{x, y, std::atan(d), dd / std::pow(1 + d * d, 1.5), 0};
        const auto &a = candidate.path.back();
        double distance = std::hypot(b.x - a.x, b.y - a.y);
        b.s = a.s + distance;
        double steering_change =
            std::abs(std::atan(c.wheelbase * b.k / c.gain) -
                     std::atan(c.wheelbase * a.k / c.gain));
        if(std::abs(b.k)>c.max_k) {++out.rejected["curvature_limit"];break;}
        if(std::abs(b.yaw)>.65) {++out.rejected["heading_limit"];break;}
        if(steering_change>c.max_steer_rate*distance/std::max(c.low_speed_rate_floor,s.speed)) {
          ++out.rejected["steering_rate"];break;
        }
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
        if (steering_change > c.max_steer_rate * ds / std::max(c.low_speed_rate_floor, s.speed)) {
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
        double target = ReferenceAt(s, b.x);
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
          cost += .08 * std::exp(-std::max(0.,b.x)/3.) * (b.y - old->y) * (b.y - old->y);
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
  if(s.previous_recovery && c.recovery && !observed_opening)++out.rejected["recovery_recheck_no_current_opening"];
  const int previous_passes=s.previous_recovery && c.recovery && observed_opening?2:1;
  for(int policy=0;policy<previous_passes;++policy) if (s.previous.size() > 1) {
    if(policy==1 && out.path_source=="rechecked_previous")continue;
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
        // Keep the generated10cm recovery waypoint. Dropping it compared
        // a two-step steering change against its shorter chord and falsely
        // rejected a path generated exactly at the same steering-rate bound.
        if (p.x <= .001)
          continue;
        const auto &a = prefix.back();
        double ds = std::hypot(p.x - a.x, p.y - a.y), turn = p.yaw - a.yaw;
        const auto soft_before = check.soft_margin_cells;
        const double steering_change = std::abs(std::atan(c.wheelbase*p.k/c.gain)-
                                                std::atan(c.wheelbase*a.k/c.gain));
        p.s = a.s + ds;
        Config current_policy=c;
        // A previously labelled recovery is re-evaluated with current OBBs,
        // current exterior and current road-opening evidence. Do not silently
        // switch it to a strict bonnet-visibility gate on the next frame.
        current_policy.recovery_geometry=policy==1 && p.s<=4.;
        if (ds > .80 || p.x > horizon || std::abs(p.k)>c.max_k ||
            steering_change > c.max_steer_rate*ds/std::max(c.low_speed_rate_floor,s.speed)+1e-8 ||
            std::abs(turn) > c.max_k * ds + .015 || !CheckSweep(s, current_policy, a, p, check))
        {
          if(policy==1) ++out.rejected[check.rejected.empty()?"recovery_recheck_kinematic":"recovery_recheck_"+check.rejected.begin()->first];
          break;
        }
        p.s = a.s + ds;
        old_cost += SegmentQualityCost(s,c,a,p,check.soft_margin_cells-soft_before);
        prefix.push_back(p);
      }
      const bool similar = prefix.size() > 1 &&
                           prefix.back().x >= out.path.back().x - .2 &&
                           old_cost <= best.quality * 1.1 + .005;
      if (prefix.size() > 1 &&
          (prefix.back().x > out.path.back().x + .5 || similar)) {
        out.path_source = policy==1?"rechecked_recovery":"rechecked_previous";
        out.recovery = policy==1;
        out.path = std::move(prefix);
        out.assisted = policy==1 || check.assisted;
        out.soft_margin_cells = check.soft_margin_cells;
      }
    }
  }
  const double prefix_lag=c.delay+(s.aligned_to_plan?0.:s.age);
  const bool short_for_current_speed=out.path.size()>1 &&
      s.speed*prefix_lag+BrakingDistance(s.speed,s.acceleration,c)>out.path.back().s;
  // A valid near-zero-speed prefix can otherwise keep a stationary car
  // stationary forever: it no longer violates current stopping distance.
  // Seek a longer alternative, never raise the speed of that short prefix.
  const bool no_crawl_progress=out.path.size()>1 &&
      std::max(0.,SpaceSpeed(out.path.back().s-.1,c.accel,prefix_lag,c)-.35)<
      std::min(c.speed_cap,c.recovery_speed);
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
    aided.speed_cap = std::min(c.speed_cap,c.recovery_speed);
    aided.budget_ms = std::max(1., c.budget_ms - elapsed(start));
    auto full = PlanPrepared(
        s, aided); // Reuse indexed scene; no second raster copy/index.
    if(!full.valid) ++out.rejected["aided_"+full.reason];
    // BuildSpeedProfile has already checked this candidate against actual
    // speed/acceleration and lag. An arbitrary6m gate discarded feasible5m
    // paths and selected an infeasible4m recovery instead (LOCAL-2F fixture).
    // A short, physically feasible prefix can reposition a stopped vehicle
    // before a near obstacle; demanding4m trapped drive07 indefinitely. It
    // still needs distinct CURRENT road cells at the future nose outside the
    // already occupied body. An all-unknown raster grants no crawl permission.
    std::set<int> front_support;
    if(full.valid && full.path.back().x >= .6) {
      const auto &g=s.grid;
      for(const auto &p:full.path) for(double side:{-.45,0.,.45}) {
        const double nose=c.length-c.rear;
        const double x=p.x+std::cos(p.yaw)*nose-std::sin(p.yaw)*c.width*side,
                     y=p.y+std::sin(p.yaw)*nose+std::cos(p.yaw)*c.width*side;
        if(x<=nose+.5*g.resolution && std::abs(y)<=c.width*.5+.5*g.resolution) continue;
        const auto q=PlanToGrid(g,x,y);
        const int row=std::lround((g.xmax-q[0])/g.resolution),
                  col=std::lround((g.ymax-q[1])/g.resolution);
        if(row>=0 && row<g.rows && col>=0 && col<g.cols) {
          const int i=row*g.cols+col;
          if(g.coverage[i]==255 && g.probability[i]>=166)front_support.insert(i);
        }
      }
    }
    const bool short_supported=full.valid && full.path.back().x>=.6 &&
        full.chosen_cap>.1 && front_support.size()>=3;
    if (full.valid && ((full.path.back().x >= 4. && full.chosen_cap > .3) || short_supported)) {
      full.recovery = true;
      full.reason = full.path.back().x>=4.?"near_field_assisted_sweep":"near_field_assisted_prefix";
      if(full.path.back().x<4.)full.path_source="bounded_recovery_prefix";
      full.speed_reason = full.braking_limit>c.decel?"near_field_controlled_braking":c.recovery_speed>3./3.6?"near_field_recovery_5kph":"near_field_recovery_3kph";
      full.raw_cap = c.speed_cap;
      full.elapsed_ms = elapsed(start);
      return full;
    }
  }
  // The full-sweep crawl alternative can still fail at a semantic shadow.
  // Try the separately labelled3km/h near-field recovery before turning a
  // short prefix into full STOP. Its actual-OBB veto above is unconditional;
  // terminal speed integration below must still fit the new path length.
  const bool recovery_attempted=out.path.size()<3 || short_for_current_speed || no_crawl_progress;
  const bool recovered=recovery_attempted &&
      BuildRecoveryPath(s,c,out);
  if (out.path.size() < 3 && !recovered) {
    out.reason = "no_verified_prefix";
    out.elapsed_ms = elapsed(start);
    return out;
  }
  out.search_ms = elapsed(start);
  if (!BuildSpeedProfile(s, c, out)) {
    const auto failure=out.reason;
    // Geometric ranking precedes actual-state speed/slew validation. A single
    // failed winner must not suppress a currently measured, fully swept crawl
    // alternative. Do not hold the old command or weaken freshness/collision
    // checks; recovery constructs and profiles a new candidate from this scene.
    if(c.recovery && !recovery_attempted && (failure=="initial_steering_slew_infeasible" ||
                      failure=="insufficient_stopping_space") &&
        BuildRecoveryPath(s,c,out) && BuildSpeedProfile(s,c,out)) {
      ++out.rejected["speed_replan_"+failure];
      out.valid=true;
      out.reason="near_field_recovery";
      out.path_source="speed_feasible_recovery";
    }
    out.elapsed_ms=elapsed(start);
    out.speed_ms=out.elapsed_ms-out.search_ms;
    return out;
  }
  out.valid = true;
  out.reason = out.recovery ? "near_field_recovery" : "ok";
  out.elapsed_ms = elapsed(start);
  out.speed_ms = out.elapsed_ms - out.search_ms;
  return out;
}
} // namespace local_planning
