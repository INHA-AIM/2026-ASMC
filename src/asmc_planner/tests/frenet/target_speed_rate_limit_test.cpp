// TargetSpeedResolver now owns spatial/upstream raw-cap composition only.
// Cycle-to-cycle continuity moved to SpeedProfiler so the two components do
// not smooth the same value twice.

#include "Frenet/Speed/TargetSpeedResolver.hpp"
#include "local/preparation_limit.hpp"

#include <cmath>
#include <iostream>
#include <string>

namespace {

int g_failures = 0;

void Expect(bool condition, const std::string& name) {
    if (condition) std::cout << "[PASS] " << name << '\n';
    else { std::cerr << "[FAIL] " << name << '\n'; ++g_failures; }
}

RefLine MakeFlatRefLine() {
    RefLine ref;
    for (double s = 0.0; s <= 200.0; s += 1.0) {
        RefPoint point{};
        point.x = s;
        point.s = s;
        point.kappa = 0.0;
        ref.points.push_back(point);
    }
    ref.length = 200.0;
    return ref;
}

CurveSpeedConfig MakeCurveConfig(double target_speed) {
    CurveSpeedConfig config{};
    config.target_vel = target_speed;
    config.curve_lookahead_m = 15.0;
    config.lateral_accel_safety_factor = 0.8;
    config.comfortable_decel_factor = 0.5;
    config.min_usable_decel = 0.5;
    return config;
}

KinematicLimits MakeLimits() {
    KinematicLimits limits{};
    limits.max_lateral_accel = 3.0;
    limits.max_longitudinal_accel = 3.0;
    limits.max_curvature = 0.3;
    limits.max_longitudinal_jerk = 6.0;
    return limits;
}

double Resolve(const TargetSpeedInput& input, double reference_cap) {
    return ResolveTargetSpeed(MakeFlatRefLine(), input,
                              MakeCurveConfig(reference_cap), MakeLimits());
}

}  // namespace

int main() {
    PlannerCommand entry{};
    asmc_planner::local::ApplyPreparationLimit(entry,false,true,0.,10.);
    Expect(!entry.has_max_speed,"default NORMAL/camera policy unchanged");
    asmc_planner::local::ApplyPreparationLimit(entry,true,true,.51,10.);
    Expect(!entry.has_max_speed,"stale preparation cannot limit speed");
    asmc_planner::local::ApplyPreparationLimit(entry,true,true,.1,10.);
    Expect(entry.has_max_speed && std::abs(entry.max_speed-10./3.6)<1e-9,
           "fresh LOCAL preparation sets upstream cap");
    entry.max_speed=1.;
    asmc_planner::local::ApplyPreparationLimit(entry,true,true,.1,10.);
    Expect(entry.max_speed==1.,"preparation never raises a stricter existing limit");
    TargetSpeedInput unconstrained{};
    unconstrained.ego_speed = 10.0;
    const double reference_cap = Resolve(unconstrained, 12.0);
    Expect(std::fabs(reference_cap - 12.0) < 1e-9,
           "straight reference cap passes through without temporal history");

    TargetSpeedInput behavior_target = unconstrained;
    behavior_target.has_external_target_speed = true;
    behavior_target.external_target_speed = 8.33;
    Expect(std::fabs(Resolve(behavior_target, 12.0) - 8.33) < 1e-9,
           "J Behavior target cap never gets raised by reference speed");

    TargetSpeedInput both_caps = behavior_target;
    both_caps.has_external_max_speed = true;
    both_caps.external_max_speed = 7.5;
    Expect(std::fabs(Resolve(both_caps, 12.0) - 7.5) < 1e-9,
           "external max cap is combined with Behavior target by minimum");

    // There is intentionally no previous-target input anymore: identical raw
    // constraints must resolve identically regardless of earlier cycles.
    Expect(std::fabs(Resolve(behavior_target, 12.0) -
                     Resolve(behavior_target, 12.0)) < 1e-12,
           "raw cap resolver is pure and stateless");

    if (g_failures == 0) {
        std::cout << "[ALL PASS]\n";
        return 0;
    }
    return 1;
}
