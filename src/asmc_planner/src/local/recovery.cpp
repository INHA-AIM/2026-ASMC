#include "local/recovery.hpp"
#include "local/speed_profile.hpp"
#include "Frenet/Evaluation/CollisionChecker.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace local_planning {
// Simulation recovery requested for LOCAL-1: fresh semantic evidence continues
// to guide a short 3km/h path even if a conservative swept-footprint gate
// fails. Internal occlusion evidence remains explicitly uncertain. Actual OBB
// and exterior-road sweeps veto; configured clearance/age uncertainty and
// short confirmed object-motion sweeps also remain hard checks.
// Any physical contact fails the experiment.
bool BuildRecoveryPath(const Scene &s, const Config &c, Result &out) {
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
          const double preferred = std::clamp(ReferenceAt(s, x), first+c.width*.5,
                                                          last-c.width*.5);
          if (std::abs(preferred - .5 * (first + last)) > .15)
            goals.push_back({x, preferred});
          // A connected strip can have its centroid and home BOTH to the
          // right while its left edge is the only way around a near drum.
          // Sample both inset edges instead of treating a centroid's sign as
          // proof that the opposite opening is unavailable. Every resulting
          // arc still passes the unchanged current body/OBB/exterior sweep.
          const double inset=.5*c.width+c.margin+g.resolution;
          if(last-first>=2*inset)
            for(double y:{first+inset,last-inset})
              if(std::none_of(goals.begin(),goals.end(),[&](const auto &p){return p[0]==x && std::abs(p[1]-y)<.15;}))
                goals.push_back({x,y});
        }
        open = false;
      }
    }
  }
  if (goals.empty()) {++out.rejected["recovery_no_observed_opening"];return false;}
  if(goals.size()>12)goals.resize(12); // Bounded near-first rollouts, plus the two directional probes below.
  double best_cost = std::numeric_limits<double>::infinity();
  std::vector<Pose> best;
  unsigned best_overlap = 0;
  const double lag = c.delay + (s.aligned_to_plan ? 0. : s.age);
  const double required = s.speed * lag + BrakingDistance(s.speed,s.acceleration,c);
  // A fixed4m arc could not contain an accelerating12km/h state. Extend
  // only as far as today's full-body road/OBB sweep verifies. The original
  // near-field soft policy is never extended beyond its existing4m range.
  const double target = std::max(std::min(c.recovery_target,std::max(4.,s.home.back()[0])), required + .25);
  if (target > 8.) {++out.rejected["recovery_stopping_range"];return false;}
  std::vector<std::pair<std::array<double,2>,double>> requests;
  for(const auto &goal:goals)
    requests.push_back({goal,std::clamp(2*goal[1]/(goal[0]*goal[0]+goal[1]*goal[1]),-c.max_k,c.max_k)});
  // Tight near obstacles may need a stronger initial turn than a distant
  // centroid arc. Only add directions with a CURRENT observed opening.
  for(double sign:{-1.,1.}) {
    auto it=std::find_if(goals.begin(),goals.end(),[&](const auto &g){return sign*g[1]>.3;});
    if(it!=goals.end())requests.push_back({*it,sign*c.max_k});
  }
  // The occupied starting body must also be clear of measured geometry.
  const auto initial=MakeEgoFootprint(CartesianState{0,0,0,0,0,0},VehicleShape{c.width,c.length,c.rear},0.);
  for(const auto &o:s.obstacles)
    if(geometry::ConvexPolygonDistance(initial,o.polygon)<=1e-9) {
      ++out.rejected["recovery_origin_obb"];out.blocking_id=o.id;return false;
    }
  double best_length=0.;
  for (const auto &request : requests) {
    const auto &goal=request.first;
    const double requested=request.second;
    Pose p;
    p.k =
        InitialCurvature(s,c);
    std::vector<Pose> path{p};
    double cost = 0.;
    unsigned overlap = 0;
    int known = 0;
    bool physical_collision = false;
    for (int step = 1; step <= int(std::ceil(target/.1)); ++step) {
      const Pose before = p;
      const double delta=std::atan(c.wheelbase*p.k/c.gain),
                   desired=std::atan(c.wheelbase*requested/c.gain),
                   span=c.max_steer_rate*.1/std::max(c.low_speed_rate_floor,s.speed);
      double k = c.gain*std::tan(std::clamp(desired,delta-span,delta+span))/c.wheelbase,
             turn = .05 * (p.k + k);
      p.x += .1 * std::cos(p.yaw + turn * .5);
      p.y += .1 * std::sin(p.yaw + turn * .5);
      p.yaw += turn;
      p.s += .1;
      p.k = k;
      if (std::abs(p.yaw) > .65)
        break;
      if (step > 40) {
        Result check;
        Config strict=c; strict.blind_assist=false;
        if (!CheckSweep(s,strict,before,p,check)) break;
      }
      Config recovery_geometry=c;recovery_geometry.recovery_geometry=true;
      Result physical;
      if(!CheckSweep(s,recovery_geometry,before,p,physical)) {
        for(const auto &v:physical.rejected)out.rejected[v.first]+=v.second;
        break;
      }
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
      if (physical_collision) {++out.rejected["recovery_physical_obb"];break;}
      path.push_back(p); // Never retain the first actually colliding pose.
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
      cost += .02 * std::pow(p.y - ReferenceAt(s, p.x), 2) + .1 * p.k * p.k;
      if (!s.previous.empty()) {
        auto old = std::min_element(
            s.previous.begin(), s.previous.end(), [&](auto &a, auto &b) {
              return std::abs(a.x - p.x) < std::abs(b.x - p.x);
            });
        cost += .02 * std::exp(-std::max(0.,p.x)/3.) * std::pow(p.y - old->y, 2);
      }
    }
    cost += .3 * std::abs(goal[1] - ReferenceAt(s, goal[0])) + .2 * (8. - goal[0]);
    Result candidate;
    candidate.path=path; candidate.assisted=true; candidate.recovery=true;
    const bool short_reposition=s.speed<=1./3.6 && path.back().s>=.6;
    // At a stopped bonnet blind spot, the opening itself supplies current
    // directional evidence. Do not demand an entire4m arc before allowing
    // a braking-feasible short reposition. This remains labelled recovery.
    const bool feasible = path.size()>6 && (known>=10 || short_reposition) &&
        BuildSpeedProfile(s,c,candidate) && candidate.chosen_cap>.02;
    const double quality=cost/std::max(.1,path.back().s);
    if (feasible && (path.back().s>best_length+.1 ||
        (path.back().s>=best_length-.1 && quality<best_cost))) {
      best_length=path.back().s; cost=quality;
      best_cost = cost;
      best = std::move(path);
      best_overlap = overlap;
    }
  }
  if (best.empty()) {++out.rejected["recovery_no_braking_candidate"];return false;}
  // An already rechecked recovery is current geometry, not stale control.
  // A shorter newly sampled arc must not overwrite a longer feasible one.
  if(out.path_source=="rechecked_recovery" && !out.path.empty() &&
      out.path.back().s>=best.back().s) {
    // Length alone does not make yesterday's rechecked geometry executable.
    // A changed initial curvature/acceleration can fail its speed/slew profile
    // while today's shorter arc is feasible. Retain the old preference only
    // after the same actual-state speed checks, with no relaxed bounds.
    Result existing=out;
    if(BuildSpeedProfile(s,c,existing))return false;
  }
  out.path = std::move(best);
  out.goals = std::move(goals);
  out.assisted = true;
  out.recovery = true;
  out.recovery_cost = best_cost;
  out.recovery_obb_overlap = best_overlap;
  out.recovery_extension_m = std::max(0.,out.path.back().s-4.);
  if(out.path.back().s<target-.05)out.path_source="bounded_recovery_prefix";
  return true;
}
} // namespace local_planning
