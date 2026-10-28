#include "local/journal.hpp"
#include "local/motion.hpp"
#include "local/planner.hpp"
#include "local/road_memory.hpp"
#include <cmath>
#include <cstdio>
#include <iostream>
#include <stdexcept>
#include <unistd.h>
using namespace local_planning;
void expect(bool ok, const char *message) {
  if (!ok)
    throw std::runtime_error(message);
}
Scene empty() {
  Scene s;
  s.grid.rows = 250;
  s.grid.cols = 160;
  s.grid.probability.assign(40000, 255);
  s.grid.coverage.assign(40000, 255);
  for (int i = 0; i <= 40; ++i) {
    s.corridor.push_back({i * .5, 5, -5});
    s.home.push_back({i * .5, 0});
  }
  return s;
}
int main() {
  try {
    // Tiny overspeed must converge, not become a full-brake trough. The old
    // scalar-cap + average-acceleration loop created an unnecessary trough.
    for (double initial_a : {-1., 0., 1.}) {
      double v = 4.3, a = initial_a;
      for (int i = 0; i < 100; ++i) {
        auto next =
            AdvancePhysicalSpeed({15. / 3.6, v, a, .05}, {1., 1.5, 4.8});
        expect(std::abs(next.target_acceleration - a) <= 4.8 * .05 + 1e-8,
               "physical speed jerk bound");
        v = next.target_speed;
        a = next.target_acceleration;
        expect(v >= 4.14, "small overspeed cannot create braking trough");
      }
      expect(std::abs(v - 15. / 3.6) < 1e-8 && std::abs(a) < 1e-8,
             "physical profile settles at cap with zero acceleration");
    }
    MotionHistory h;
    expect(h.add(1, 1, .1), "first motion");
    expect(h.add(1.1, 1, .1), "second motion");
    auto m = h.at(1.05);
    expect(m && std::abs(m->x - .05) < .001, "motion interpolation");
    expect(!h.at(.99), "cannot use future sample for old state");
    auto epoch = h.epoch;
    h.add(.9, 1, 0);
    expect(h.epoch == epoch + 1, "regression resets epoch");
    expect(!h.at(1.1), "prior epoch gone");
    epoch = h.epoch;
    h.add(2, 1, 0);
    expect(h.epoch == epoch + 1, "gap resets epoch");
    MotionHistory reverse;
    reverse.add(10., -1., 0.);
    const auto reverse_epoch = reverse.epoch;
    reverse.add(10.1, -1., 0.);
    expect(reverse.epoch == reverse_epoch && reverse.at(10.1)->x < -.099,
           "reverse observations preserve signed motion and epoch");
    expect(reverse.at(10.19) && !reverse.at(10.201),
           "motion forecast bounded at100ms, beyond valid83ms IMU cadence");
    expect(!reverse.add(10.1, -1., 0.) && reverse.epoch == reverse_epoch,
           "duplicate state does not reset history");
    Grid aligned;
    aligned.rotation = GroundRotation(.1, -.04);
    aligned.source_in_plan = {-1.2, .15, -.08};
    for (double x : {-1., 3., 15.})
      for (double y : {-3., 0., 4.}) {
        const auto q = GridToPlan(aligned, x, y);
        const auto back = PlanToGrid(aligned, q[0], q[1]);
        expect(std::hypot(back[0] - x, back[1] - y) < 1e-10,
               "tilt and relative translation/yaw are inverted once");
      }
    auto r = GroundRotation(.1, -.05);
    auto normal = Rotate(r, -.1, .05, 1);
    expect(std::hypot(normal[0], normal[1]) < 1e-10,
           "plane levelling exactly once");
    Config c;
    c.budget_ms = 500.;
    Scene s = empty();
    Result check;
    expect(CheckFootprint(s, c, {}, check), "empty free scene");
    Obstacle o;
    o.id = 42;
    o.polygon = {{4, -.2}, {4.4, -.2}, {4.4, .2}, {4, .2}};
    s.obstacles.push_back(o);
    Pose p;
    p.x = 1.;
    expect(!CheckFootprint(s, c, p, check) && check.blocking_id == 42,
           "all eight-corner hull veto independent of semantic free");
    s = empty();
    int idx = std::lround((20 - 4.) / .1) * 160 + 80;
    s.grid.coverage[idx] = 0;
    check = {};
    expect(!CheckFootprint(s, c, p, check) &&
               check.rejected.count("unobserved"),
           "unknown is not free");
    // The same source nonroad cell moves behind the current footprint after
    // a 6m relative displacement; no raster resampling changes its evidence.
    s.grid.source_in_plan = {-6., 0., 0.};
    check = {};
    expect(CheckFootprint(s, c, p, check), "source cell aligned away from ego");
    s.grid.source_in_plan = {0., 0., 0.};
    c.blind_assist = true;
    check = {};
    expect(CheckFootprint(s, c, p, check) && check.assisted,
           "explicit bounded assist");
    s.grid.coverage[idx] = 255;
    s.grid.probability[idx] = 0;
    check = {};
    expect(!CheckFootprint(s, c, p, check),
           "assist cannot erase observed nonroad");
    c.blind_assist = false;
    s = empty();
    auto result = Plan(s, c);
    std::cerr << "empty valid=" << result.valid << " reason=" << result.reason
              << " length=" << result.path.back().s
              << " ms=" << result.elapsed_ms << " caps=" << result.chosen_cap
              << "\n";
    expect(result.valid, "empty scene yields kinematic path");
    expect(result.chosen_cap <= 25. / 3.6, "25 is ceiling");
    expect(result.speed_reason == "verified_stopping_space",
           "limited range limits speed");
    expect(result.path.front().x == 0 && result.path.front().y == 0,
           "start is rear axle not free-space centroid");
    for (const auto &q : result.path)
      expect(std::abs(q.k) <= c.max_k + 1e-9, "curvature limit");
    // A distant curved home lane must retain its tangent after the initial
    // connection; a straight scalar lateral target used to understeer here.
    s = empty();
    for (auto &q : s.home)
      q[1] = .005 * q[0] * q[0];
    auto curved = Plan(s, c);
    expect(curved.valid && curved.path.back().x > 12.,
           "curved family yields long path");
    const auto tail = curved.path.back();
    expect(std::abs(tail.y - .005 * tail.x * tail.x) < .15 && tail.yaw > .08,
           "family tangent and lateral position survive the transition");
    // An obstacle splits free-space: do not target the mean through its centre.
    s = empty();
    // Family paint is a navigation reference, not the physical road edge. The
    // measured free road below is wider; avoidance may cross either white line.
    for (auto &paint : s.corridor) {
      paint[1] = 1.75;
      paint[2] = -1.75;
    }
    s.obstacles.push_back({9, {{8.5, -.4}, {9.3, -.4}, {9.3, .4}, {8.5, .4}}});
    for (int row = 0; row < 250; ++row)
      for (int col = 0; col < 160; ++col)
        if (std::abs(8. - col * .1) > 3.5)
          s.grid.probability[row * 160 + col] = 0;
    auto avoid = Plan(s, c);
    std::cerr << "obstacle valid=" << avoid.valid
              << " length=" << avoid.path.back().s << " ms=" << avoid.elapsed_ms
              << " end=" << avoid.path.back().x << "," << avoid.path.back().y
              << "," << avoid.path.back().yaw << " goals=" << avoid.goals.size()
              << "\n";
    for (auto g : avoid.goals)
      std::cerr << "goal=" << g[0] << "," << g[1] << "\n";
    expect(avoid.valid && avoid.path.back().x > 10.,
           "bounded kinematic search must pass isolated central obstacle");
    for (size_t k = 1; k < avoid.path.size(); ++k) {
      Result swept;
      expect(CheckSweep(s, c, avoid.path[k - 1], avoid.path[k], swept),
             "selected path continuous collision validation");
    }
    expect(avoid.goals.size() >= 2, "split openings produce separate goals");
    bool beyond_paint = false;
    for (const auto &q : avoid.path)
      beyond_paint |= std::abs(q.y) + c.width * .5 > 1.75;
    expect(beyond_paint,
           "valid swept avoidance can cross paint inside measured free road");
    // A shorter unused left boundary must not truncate a right-home path.
    s = empty();
    s.corridor.resize(16);
    auto support = Plan(s, c);
    expect(support.valid && support.path.back().x > 14.,
           "paint support is not a physical stopping boundary");
    RoadMemory memory;
    auto observed = empty().grid;
    memory.observe(observed, Pose{}, 10.);
    auto hidden = observed;
    hidden.coverage.assign(40000, 128);
    hidden.probability.assign(40000, 0);
    memory.apply(hidden, Pose{1., 0, 0}, 11.);
    const int near = 150 * 160 + 80; // x5, beyond the physical bonnet
    expect(hidden.recent_road_age[near] > 0,
           "past actually observed road is aligned into near ego occlusion");
    expect(hidden.recent_road_age[50 * 160 + 80] == 0,
           "memory cannot fill distant unobserved road");
    hidden.coverage[near] = 255;
    memory.apply(hidden, Pose{1., 0, 0}, 11.);
    expect(hidden.recent_road_age[near] == 0,
           "current observed nonroad always overrides old road");
    hidden.coverage[near] = 128;
    memory.apply(hidden, Pose{1., 0, 0}, 16.);
    expect(hidden.recent_road_age[near] == 0,
           "expired evidence is not renewed");
    memory.clear();
    memory.apply(hidden, Pose{1., 0, 0}, 11.);
    expect(hidden.recent_road_age[near] == 0, "epoch reset erases road memory");
    memory.observe(observed, Pose{}, 10.);
    auto obstruction = observed;
    obstruction.probability[near] = 0;
    memory.observe(obstruction, Pose{}, 10.5);
    memory.apply(hidden, Pose{}, 11.);
    expect(hidden.recent_road_age[near] == 0,
           "newer observed obstruction blocks older free road");
    s = empty();
    s.grid.coverage[idx] = 128;
    s.grid.recent_road_age.assign(40000, 0.);
    s.grid.recent_road_age[idx] = .5;
    check = {};
    expect(CheckFootprint(s, c, p, check),
           "causal near road can support footprint");
    s.obstacles.push_back(o);
    check = {};
    expect(!CheckFootprint(s, c, p, check),
           "fresh obstacle vetoes remembered road");
    s = empty();
    s.home.resize(16); // 7.5m rear-axle path cannot stop from25km/h.
    s.speed = 25. / 3.6;
    s.acceleration = .8;
    expect(!Plan(s, c).valid,
           "positive acceleration cannot invent emergency stopping space");
    s = empty();
    s.speed = 5.;
    const int soft_edge = 160 * 160 + 94; // x4, y-1.4: outside hard envelope
    s.grid.probability[soft_edge] = 0;
    check = {};
    expect(CheckFootprint(s, c, p, check) && check.soft_margin_cells > 0,
           "speed comfort margin is a cost, not physical collision");
    s.grid.probability[160 * 160 + 91] = 0; // y-1.1: hard footprint margin
    check = {};
    expect(!CheckFootprint(s, c, p, check),
           "observed nonroad inside body and minimum margin still vetoes");
    // Reusing an old path must not carry a stale 100m arc-length into speed
    // cap.
    s = empty();
    s.previous = {{0, 0, 0, 0, 100}, {.5, 0, 0, 0, 100.5}, {1, 0, 0, 0, 101}};
    auto reuse = Plan(s, c);
    expect(reuse.path.back().s < 20 && reuse.space_cap < 25. / 3.6,
           "old arc length cannot invent stopping room");
    // A swept segment detects a narrow obstacle between clear endpoints.
    s = empty();
    s.obstacles.push_back({10, {{4.6, -.1}, {4.7, -.1}, {4.7, .1}, {4.6, .1}}});
    Result sweep;
    expect(!CheckSweep(s, c, Pose{0, 0, 0, 0, 0}, Pose{8, 0, 0, 0, 8}, sweep),
           "no tunnelling through sparse old path points");
    s = empty();
    s.age = .4;
    expect(!Plan(s, c).valid, "stale cannot plan");
    s = empty();
    s.grid.probability.assign(40000, 0);
    expect(!Plan(s, c).valid, "all blocked cannot plan");
    // The recovery experiment may relax geometric rejection, but cannot
    // invent road evidence when every cell is blocked/unknown/stale.
    c.recovery = true;
    expect(!Plan(s, c).valid, "recovery cannot invent all-blocked road");
    s = empty();
    s.grid.coverage.assign(40000, 0);
    expect(!Plan(s, c).valid, "recovery cannot invent all-unknown road");
    s = empty();
    // Near hood mask prevents strict sweep; broad visible interval ends at
    // search boundary. Integer endpoint flush must retain that component.
    for (int row = 140; row < 220; ++row)
      for (int col = 65; col < 95; ++col)
        s.grid.coverage[row * 160 + col] = 128;
    auto creep = Plan(s, c);
    expect(creep.valid && creep.recovery && creep.chosen_cap <= 3. / 3.6,
           "visible near-field recovery is explicitly capped at 3km/h");
    expect(std::abs(creep.path.back().y) < .01,
           "broad free road retains home lane instead of off-centre mean");
    // With only the bounded camera hood blind spot, distant barriers must
    // influence a full-length swept candidate, rather than a four-metre arc.
    s = empty();
    for (int row = 137; row < 220; ++row)
      for (int col = 70; col < 90; ++col)
        s.grid.coverage[row * 160 + col] = 128;
    s.obstacles.push_back({73, {{10., -1.}, {11., -1.}, {11., 1.}, {10., 1.}}});
    auto ahead = Plan(s, c);
    expect(ahead.valid && ahead.recovery && ahead.path.back().x > 11.,
           "near-field assistance keeps distant obstacle lookahead");
    Config aided = c;
    aided.blind_assist = true;
    for (size_t k = 1; k < ahead.path.size(); ++k) {
      Result check;
      expect(CheckSweep(s, aided, ahead.path[k - 1], ahead.path[k], check),
             "assisted long path never softens a known OBB collision");
    }
    s.age = .4;
    expect(!Plan(s, c).valid, "recovery cannot mask stale perception");
    c.recovery = false;
    s = empty();
    s.speed = 25. / 3.6;
    auto fast = Plan(s, c);
    expect(!fast.valid && fast.reason == "insufficient_stopping_space",
           "unsafe speed cannot be made safe by cap alone");
    std::string file =
        "/tmp/local-journal-" + std::to_string(getpid()) + ".jsonl";
    {
      Journal j(file);
      j.push("{\"test\":1}");
    }
    std::ifstream f(file);
    std::string line;
    std::getline(f, line);
    expect(line == "{\"test\":1}", "shutdown drains journal");
    bool refused = false;
    try {
      Journal j(file);
    } catch (...) {
      refused = true;
    }
    expect(refused, "journal cannot overwrite");
    std::remove(file.c_str());
    std::cout << "LOCAL-1 core regression passed\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "FAIL " << e.what() << '\n';
    return 1;
  }
}
