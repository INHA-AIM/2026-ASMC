#include "Frenet/Speed/CurveSpeed.hpp"
#include "Frenet/Generator/LongitudinalGenerator.hpp"
#include "Frenet/Evaluation/KinematicFilter.hpp"
#include "Frenet/Generator/BoundaryConditionPolicy.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>

namespace {
int failures = 0;
void Expect(bool ok, const std::string& name) {
    std::cout << (ok ? "[PASS] " : "[FAIL] ") << name << '\n';
    if (!ok) ++failures;
}
CurveSpeedConfig Curve() { return {13.0, 27.0, 0.8, 0.5, 0.5}; }
KinematicLimits Limits() {
    KinematicLimits l{};
    l.max_longitudinal_accel = 3.0;
    l.max_lateral_accel = 2.6;
    l.max_longitudinal_jerk = 6.0;
    l.max_curvature = 0.23;
    l.boundary_condition_accel_fraction = 0.8;
    return l;
}
PathGeneratorConfig Grid() {
    PathGeneratorConfig c{};
    c.time_horizon = {1.5, 5.0, 0.5};
    c.delta_s_dot = {-2.0, 2.0, 1.0};
    c.dt = 0.05;
    return c;
}
RefLine Reference(double corner, bool closed = false) {
    RefLine ref;
    ref.length = 200.0;
    ref.closed_loop = closed;
    for (double s = 0.0; s <= ref.length; s += 0.25) {
        RefPoint p{};
        p.s = p.x = s;
        // Analytic reference curvature: local safe speed exactly 6 m/s.
        p.kappa = s >= corner && s < corner + 50.0 ? 2.08 / 36.0 : 0.0;
        ref.points.push_back(p);
    }
    return ref;
}
std::vector<FrenetPath> Generate(const RefLine& ref, const FrenetState& ego,
                                double cap, double initial_accel) {
    const auto limits = Limits();
    auto start = ego;
    start.s_dd = initial_accel;
    auto envelope = BuildKeepSpeedEnvelope(ref, ego.s, ego.s_d, initial_accel,
        Grid().time_horizon.max, Grid().dt, cap, Curve(),
        limits.max_longitudinal_accel, limits.max_lateral_accel,
        limits.max_longitudinal_jerk * limits.boundary_condition_accel_fraction);
    return GenerateEnvelopeVelocityCandidates(start, cap, envelope, Grid(), limits);
}
const FrenetPath* Nominal(const std::vector<FrenetPath>& paths, double duration) {
    for (const auto& path : paths) {
        if (std::fabs(path.t.back() - duration) < 1e-9) return &path;
    }
    return nullptr;
}
}

int main() {
    const auto ref = Reference(40.0);
    const auto limits = Limits();
    FrenetState ego{};
    ego.s_d = std::sqrt(36.0 + 2.0 * 1.5 * 40.0);
    const double cap = LookaheadTargetSpeed(ref, 0.0, ego.s_d, Curve(), 3.0, 2.6, 0.23);
    auto old = GenerateVelocityKeepingCandidates(ego, cap, Grid());
    const FrenetPath* old_nominal = nullptr;
    for (const auto& path : old) {
        if (std::fabs(path.delta_s_dot) < 1e-9 &&
            std::fabs(path.t.back() - 5.0) < 1e-9) old_nominal = &path;
    }
    auto paths = Generate(ref, ego, cap, 0.0);
    const FrenetPath* nominal = Nominal(paths, 5.0);
    Expect(old_nominal && nominal, "baseline and spatial candidates exist");
    if (!nominal || !old_nominal) return 1;
    std::cout << "boundary v=" << ego.s_d << " baseline v(0.5)=" << old_nominal->s_d[10]
              << " spatial v(0.5)=" << nominal->s_d[10] << '\n';
    Expect(std::fabs(old_nominal->s_d[10] - ego.s_d) < 1e-6,
           "reproduce: current ceiling as terminal speed produces no braking");
    Expect(nominal->s_d[10] < ego.s_d - 0.5,
           "boundary: braking begins in first half second");
    bool safe_corner = true, entered = false;
    for (std::size_t i = 0; i < nominal->s.size(); ++i) {
        if (nominal->s[i] >= 40.0 && nominal->s[i] < 90.0) {
            entered = true;
            safe_corner &= nominal->s_d[i] <= 6.05;
        }
    }
    Expect(entered && safe_corner, "reach curve at its local safe speed");
    FilterLongitudinalByKinematicLimits(paths, limits);
    Expect(std::all_of(paths.begin(), paths.end(), [](const FrenetPath& p) { return p.valid; }),
           "all curve candidates satisfy acceleration, jerk and forward speed limits");
    bool same_prefix = true;
    for (double duration = 1.5; duration <= 5.0; duration += 0.5) {
        const auto* p = Nominal(paths, duration);
        same_prefix &= p && std::fabs(p->s_d[10] - nominal->s_d[10]) < 1e-9;
    }
    Expect(same_prefix, "choosing a longer horizon cannot weaken initial braking");

    // Use the first 50ms of every freshly generated path as the next measured
    // state, exercising the actual receding-horizon failure mechanism.
    bool rolling_safe = true;
    ego.s = 0.0; ego.s_dd = 0.0;
    double entry_speed = 0.0;
    for (int cycle = 0; cycle < 200 && ego.s < 45.0; ++cycle) {
        const double raw = LookaheadTargetSpeed(ref, ego.s, ego.s_d, Curve(), 3.0, 2.6, 0.23);
        const auto envelope = BuildKeepSpeedEnvelope(ref, ego.s, ego.s_d, ego.s_dd,
            5.0, 0.05, raw, Curve(), 3.0, 2.6, 4.8);
        const double a0 = ResolveEnvelopeBoundaryAcceleration(ego.s_dd, ego.s_d, raw, envelope, 0.1);
        const auto replanned = Generate(ref, ego, raw, a0);
        const auto* next = Nominal(replanned, 1.5);
        if (!next || !next->valid) { rolling_safe = false; break; }
        ego.s = next->s[1]; ego.s_d = next->s_d[1]; ego.s_dd = next->s_dd[1];
        if (ego.s >= 40.0) { if (entry_speed == 0.0) std::cout << "first entry=" << ego.s_d << " at " << ego.s << "\n"; entry_speed = ego.s_d; rolling_safe &= ego.s_d <= 6.05; }
    }
    std::cout << "rolling entry speed=" << entry_speed << '\n';
    Expect(rolling_safe && ego.s >= 45.0, "20Hz replanning preserves early braking and curve limit");

    auto straight = Reference(300.0);
    ego = {}; ego.s_d = 11.16;
    auto overspeed = Generate(straight, ego, 10.59, 0.0);
    auto* decel = Nominal(overspeed, 1.5);
    Expect(decel && decel->s_d[10] < 10.9,
           "recorded 11.16 -> 10.59 intent creates near-term deceleration");
    auto acceleration = Generate(straight, ego, 13.0, 0.0);
    auto* accel = Nominal(acceleration, 1.5);
    Expect(accel && accel->s_d[10] > ego.s_d, "straight road can accelerate after cap clears");
    auto no_curve = BuildKeepSpeedEnvelope(straight, 195.0, 11.0, 0.0,
        5.0, 0.05, 13.0, Curve(), 3.0, 2.6, 6.0);
    Expect(std::all_of(no_curve.begin(), no_curve.end(), [](const auto& p) {
        return std::fabs(p.speed_squared - 169.0) < 1e-8;
    }), "open path does not wrap to its start");
    auto loop = Reference(10.0, true);
    auto wrapped = BuildKeepSpeedEnvelope(loop, 190.0, 11.0, 0.0,
        5.0, 0.05, 13.0, Curve(), 3.0, 2.6, 6.0);
    Expect(wrapped.front().speed_squared < 169.0 && wrapped.back().s > 200.0,
           "closed path anticipates curve across unwrapped seam");

    ego = {}; ego.s_d = 3.0;
    auto zero = Generate(straight, ego, 0.0, 0.0);
    FilterLongitudinalByKinematicLimits(zero, limits);
    Expect(std::any_of(zero.begin(), zero.end(), [](const auto& p) { return p.valid; }),
           "zero KEEP cap retains a forward braking candidate");
    return failures ? 1 : 0;
}
