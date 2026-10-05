#include "frenet_planner/Speed/SpeedProfiler.hpp"

#include "frenet_planner/Evaluation/KinematicFilter.hpp"
#include "frenet_planner/Generator/BoundaryConditionPolicy.hpp"
#include "frenet_planner/Generator/LongitudinalGenerator.hpp"
#include "frenet_planner/Speed/TargetSpeedResolver.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace {

constexpr double kDt = 0.05;
const SpeedProfileLimits kLimits{/*max_acceleration=*/2.0,
                                 /*max_deceleration=*/1.5,
                                 /*max_jerk=*/6.0};
int g_failures = 0;

void Expect(bool condition, const std::string& name) {
    if (condition) std::cout << "[PASS] " << name << '\n';
    else { std::cerr << "[FAIL] " << name << '\n'; ++g_failures; }
}

SpeedProfileOutput Step(SpeedProfiler& profiler, double cap,
                        double ego_speed, double ego_acceleration = 0.0) {
    return profiler.Update(
        SpeedProfileInput{cap, ego_speed, ego_acceleration, kDt}, kLimits);
}

struct RunResult {
    std::vector<double> speeds;
    std::vector<double> accelerations;
    bool bounds_ok = true;
};

RunResult RunTransition(double start, double target, int cycles) {
    SpeedProfiler profiler;
    SpeedProfileOutput previous = Step(profiler, start, start);
    RunResult result;
    for (int i = 0; i < cycles; ++i) {
        const SpeedProfileOutput current = Step(profiler, target, start);
        result.speeds.push_back(current.target_speed);
        result.accelerations.push_back(current.target_acceleration);
        result.bounds_ok = result.bounds_ok &&
            current.target_acceleration <= kLimits.max_acceleration + 1e-9 &&
            current.target_acceleration >= -kLimits.max_deceleration - 1e-9 &&
            std::fabs(current.target_acceleration -
                      previous.target_acceleration) <=
                kLimits.max_jerk * kDt + 1e-9 &&
            std::fabs(current.target_speed - previous.target_speed) <=
                std::max(kLimits.max_acceleration,
                         kLimits.max_deceleration) * kDt + 1e-9;
        previous = current;
    }
    return result;
}

void TestInitializationAndConstantCap() {
    SpeedProfiler profiler;
    const SpeedProfileOutput first = Step(profiler, 10.0, 10.0);
    Expect(std::fabs(first.target_speed - 10.0) < 1e-12,
           "A initialization starts at ego speed, not zero");
    bool stable = std::fabs(first.target_acceleration) < 1e-12;
    for (int i = 0; i < 100; ++i) {
        const SpeedProfileOutput output = Step(profiler, 10.0, 10.0);
        stable = stable && std::fabs(output.target_speed - 10.0) < 1e-12 &&
                 std::fabs(output.target_acceleration) < 1e-12;
    }
    Expect(stable, "B constant cap stays exactly stable");
}

void TestRoadSpeedTransitions() {
    SpeedProfiler downward;
    Step(downward, 13.0, 13.0);
    const SpeedProfileOutput braking = Step(downward, 8.0, 11.0);
    Expect(std::fabs(braking.target_speed - 8.0) < 1e-12,
           "C downward 13->8 cap is the scalar target in one cycle");
    Expect(std::fabs(braking.target_acceleration) < 1e-12 &&
               std::fabs(downward.speed() - 8.0) < 1e-12 &&
               std::fabs(downward.acceleration()) < 1e-12,
           "C downward bypass realigns stored speed and acceleration state");

    SpeedProfiler ego_below_cap;
    Step(ego_below_cap, 13.0, 13.0);
    const SpeedProfileOutput accelerating = Step(ego_below_cap, 8.0, 5.0);
    Expect(std::fabs(accelerating.target_speed - 8.0) < 1e-12,
           "D lowered cap remains 8 when ego is below it (acceleration intent)");

    const RunResult up = RunTransition(8.33, 13.89, 300);
    Expect(up.speeds.front() < 13.89 - 1.0,
           "E upward 30->50 recovery does not jump in one cycle");
    Expect(up.bounds_ok, "E upward recovery respects acceleration and jerk bounds");
    Expect(std::fabs(up.speeds.back() - 13.89) < 1e-9 &&
           std::fabs(up.accelerations.back()) < 1e-9,
           "E upward recovery converges exactly without residual acceleration");
}

void TestNearCapAndToggle() {
    const RunResult near = RunTransition(8.2, 8.5, 100);
    const double max_speed = *std::max_element(
        near.speeds.begin(), near.speeds.end());
    bool nondecreasing = true;
    for (std::size_t i = 1; i < near.speeds.size(); ++i) {
        nondecreasing = nondecreasing &&
            near.speeds[i] + 1e-10 >= near.speeds[i - 1];
    }
    Expect(max_speed <= 8.5 + 1e-9 && nondecreasing,
           "E cap-near profile reaches cap without overshoot or oscillation");

    SpeedProfiler profiler;
    Step(profiler, 13.0, 13.0);
    const SpeedProfileOutput low_1 = Step(profiler, 8.0, 11.0);
    const SpeedProfileOutput high = Step(profiler, 13.0, 11.0);
    const SpeedProfileOutput low_2 = Step(profiler, 8.0, 11.0);
    Expect(std::fabs(low_1.target_speed - 8.0) < 1e-12 &&
               high.target_speed > 8.0 && high.target_speed < 13.0 &&
               std::fabs(low_2.target_speed - 8.0) < 1e-12,
           "F 13->8 is immediate, 8->13 is limited, next 13->8 is immediate");
    Expect(std::fabs(low_2.target_acceleration) < 1e-12 &&
               std::fabs(profiler.acceleration()) < 1e-12,
           "F repeated low cap clears stale upward recovery acceleration");
}

void TestHighwayTransitions() {
    const RunResult up = RunTransition(13.89, 27.78, 500);
    Expect(up.bounds_ok && std::fabs(up.speeds.back() - 27.78) < 1e-9,
           "G generic 50->100 km/h profile converges within physical bounds");
    SpeedProfiler down;
    Step(down, 27.78, 27.78);
    const SpeedProfileOutput lowered = Step(down, 13.89, 27.78);
    Expect(std::fabs(lowered.target_speed - 13.89) < 1e-9,
           "H generic highway cap decrease is immediate at scalar target");
}

void TestCandidateCapAndLifecycle() {
    SpeedProfiler profiler;
    Step(profiler, 13.89, 13.89);
    const double base_cap = 13.89;
    const double candidate_cap = 8.9;
    const double final_raw_cap = std::min(base_cap, candidate_cap);
    const SpeedProfileOutput output = Step(profiler, final_raw_cap, 13.89);
    Expect(std::fabs(final_raw_cap - candidate_cap) < 1e-12 &&
           std::fabs(output.target_speed - candidate_cap) < 1e-12,
           "I candidate-aware lower cap becomes generator target immediately");

    profiler.Reset();
    const SpeedProfileOutput restarted = Step(profiler, 20.0, 7.0);
    Expect(restarted.target_speed < 7.1 && restarted.target_speed >= 7.0,
           "lifecycle reset realigns the next KEEP profile to ego speed");
}

void TestCurvePreviewFeedsImmediateBrakingTrajectory() {
    RefLine reference;
    for (int i = 0; i <= 100; ++i) {
        RefPoint point{};
        point.x = static_cast<double>(i);
        point.s = static_cast<double>(i);
        point.kappa = i >= 20 ? 0.08 : 0.0;
        reference.points.push_back(point);
    }
    reference.length = 100.0;

    CurveSpeedConfig curve{};
    curve.target_vel = 13.0;
    curve.curve_lookahead_m = 27.0;
    curve.lateral_accel_safety_factor = 0.8;
    curve.comfortable_decel_factor = 0.5;
    curve.min_usable_decel = 0.5;

    KinematicLimits limits{};
    limits.max_lateral_accel = 2.6;
    limits.max_longitudinal_accel = 3.0;
    limits.max_curvature = 0.23;
    limits.max_longitudinal_jerk = 6.0;
    limits.accel_intent_deadband_mps = 0.3;

    TargetSpeedInput input{};
    input.ego_s = 0.0;
    input.ego_speed = 11.0;
    const double raw_curve_cap = ResolveTargetSpeed(
        reference, input, curve, limits);
    Expect(raw_curve_cap < input.ego_speed - limits.accel_intent_deadband_mps,
           "J curve preview lowers the raw cap before curve entry");

    SpeedProfiler profiler;
    Step(profiler, 13.0, 13.0);
    const SpeedProfileOutput profiled = profiler.Update(
        SpeedProfileInput{raw_curve_cap, input.ego_speed, 0.45, kDt},
        SpeedProfileLimits{/*max_acceleration=*/2.0,
                           /*max_deceleration=*/1.5,
                           /*max_jerk=*/limits.max_longitudinal_jerk});
    Expect(std::fabs(profiled.target_speed - raw_curve_cap) < 1e-12,
           "J curve preview cap reaches the generator in the same cycle");

    FrenetState start{};
    start.s_d = input.ego_speed;
    start.s_dd = ResolveVelocityKeepingBoundaryAcceleration(
        /*raw_s_ddot=*/0.45, start.s_d, profiled.target_speed,
        limits.accel_intent_deadband_mps);
    Expect(std::fabs(start.s_dd) < 1e-12,
           "J positive boundary acceleration is removed for curve deceleration intent");

    PathGeneratorConfig generator{};
    generator.time_horizon = {3.0, 5.0, 0.5};
    generator.delta_s_dot = {0.0, 0.0, 1.0};
    generator.dt = 0.1;
    std::vector<FrenetPath> trajectories =
        GenerateVelocityKeepingCandidates(start, profiled.target_speed,
                                           generator);
    FilterLongitudinalByKinematicLimits(trajectories, limits);
    const FrenetPath* valid = nullptr;
    for (const FrenetPath& trajectory : trajectories) {
        if (trajectory.valid) {
            valid = &trajectory;
            break;
        }
    }
    Expect(valid != nullptr,
           "J immediate curve cap still produces a kinematically valid KEEP trajectory");
    if (valid) {
        const std::size_t half_second = 5;
        const std::size_t one_second = 10;
        Expect(valid->s_d.size() > one_second &&
                   std::fabs(valid->s_d.front() - input.ego_speed) < 1e-12 &&
                   valid->s_d[half_second] < valid->s_d.front() &&
                   valid->s_d[one_second] < valid->s_d[half_second],
               "J physical trajectory starts at ego speed and brakes before the curve");
    }
}

}  // namespace

int main() {
    TestInitializationAndConstantCap();
    TestRoadSpeedTransitions();
    TestNearCapAndToggle();
    TestHighwayTransitions();
    TestCandidateCapAndLifecycle();
    TestCurvePreviewFeedsImmediateBrakingTrajectory();
    if (g_failures == 0) {
        std::cout << "[ALL PASS]\n";
        return 0;
    }
    return 1;
}
