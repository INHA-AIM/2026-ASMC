// Longitudinal kinematics validity 회귀.
//
// 이 테스트가 지키는 계약:
//   A. |s_dd| <= max_longitudinal_accel인 candidate라도 중간 구간 s_d < 0
//      (reverse)가 나오면 invalid 처리되어야 한다.
//   B. |s_dd|가 accel limit 안에 있어도 jerk(|s_ddd|)가
//      max_longitudinal_jerk를 넘으면 invalid 처리되어야 한다.
//   C. 정상 velocity keeping candidate는 새 filter 이후에도 최소 하나
//      살아남아야 한다.
//   D. STOP의 정확한 s_d = 0은 reverse-velocity reject 때문에 죽으면 안
//      된다.
//
// 이 테스트는 실제 MakeQuintic / GenerateStoppingCandidates /
// GenerateVelocityKeepingCandidates 결과를 사용한다 (숫자 blind hard-code 금지).

#include "frenet_planner/frenet/path_generator.hpp"
#include "frenet_planner/math/polynomial.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <string>

namespace {

int g_failures = 0;

void Expect(bool condition, const std::string& name) {
    if (condition) std::cout << "[PASS] " << name << '\n';
    else { std::cerr << "[FAIL] " << name << '\n'; ++g_failures; }
}

KinematicLimits MakeLimits() {
    KinematicLimits limits;
    limits.max_lateral_accel = 3.0;
    limits.max_longitudinal_accel = 3.0;
    limits.max_curvature = 0.1704;
    limits.max_longitudinal_jerk = 5.0;
    return limits;
}

}  // namespace

// CASE 1 - negative velocity candidate
//
// 급감속 초기상태(s_dd=-2.4)에서 짧은 거리 stop target을 향해 quintic을
// 만들면, 종료조건(s_d=0)에 도달하기 전에 오버슈트로 s_d가 음수가 되는
// 구간이 생긴다. FilterLongitudinalByKinematicLimits는 이 candidate를
// invalid 처리해야 한다.
void TestNegativeVelocityReproducer() {
    FrenetState start{};
    start.s = 0.0;
    start.s_d = 5.0;
    start.s_dd = -2.4;
    start.d = 0.0;
    start.d_d = 0.0;
    start.d_dd = 0.0;

    PathGeneratorConfig cfg;
    cfg.delta_s = {-1.0, 0.0, 0.25};
    cfg.time_horizon = {5.0, 5.0, 0.0};
    cfg.dt = 0.1;

    std::vector<FrenetPath> candidates =
        GenerateStoppingCandidates(start, /*stop_s=*/5.0, cfg);
    Expect(!candidates.empty(), "negative-velocity reproducer: candidates generated");

    bool found_negative = false;
    double min_s_d = std::numeric_limits<double>::infinity();
    for (const auto& path : candidates) {
        for (double sd : path.s_d) {
            min_s_d = std::min(min_s_d, sd);
            if (sd < -1e-6) found_negative = true;
        }
    }
    std::cout << "  reproducer min s_d (before filter) = " << min_s_d << '\n';
    Expect(found_negative,
           "negative-velocity reproducer: at least one candidate has s_d < 0 "
           "before filtering");

    KinematicLimits limits = MakeLimits();
    FilterLongitudinalByKinematicLimits(candidates, limits);
    bool any_negative_still_valid = false;
    std::size_t valid_count = 0;
    for (const auto& path : candidates) {
        if (!path.valid) continue;
        ++valid_count;
        for (double sd : path.s_d) {
            if (sd < -1e-6) { any_negative_still_valid = true; break; }
        }
    }
    std::cout << "  negative-velocity case: valid candidates after filter = "
              << valid_count << '\n';
    Expect(!any_negative_still_valid,
           "negative-velocity reproducer: FilterLongitudinalByKinematicLimits "
           "rejects every negative-velocity candidate");
}

// CASE 2 - excessive jerk candidate
//
// 큰 초기 가속도에서 짧은 horizon 안에 종료 속도로 전환하는 quartic은
// |s_dd|는 accel limit 안에 있어도 jerk peak가 커질 수 있다.
// GenerateVelocityKeepingCandidates와 정확히 같은 MakeQuartic 호출로
// QuarticPolynomial을 재구성해(동일 다항식 계수) EvalJerk로 실제 피크를
// 계산하고, FilterLongitudinalByKinematicLimits가 이를 걸러내는지 확인한다.
void TestExcessiveJerkReproducer() {
    FrenetState start{};
    start.s = 0.0;
    start.s_d = 1.0;
    start.s_dd = -2.5;
    start.d = 0.0;
    start.d_d = 0.0;
    start.d_dd = 0.0;

    const double target_speed = 5.0;
    const double dsdot = -2.5;
    const double T = 1.5;  // 짧은 horizon일수록 jerk가 커짐

    PathGeneratorConfig cfg;
    cfg.delta_s_dot = {dsdot, dsdot, 0.0};
    cfg.time_horizon = {T, T, 0.0};
    cfg.dt = 0.1;

    std::vector<FrenetPath> candidates =
        GenerateVelocityKeepingCandidates(start, target_speed, cfg);
    Expect(!candidates.empty(), "excessive-jerk reproducer: candidates generated");

    // path_generator.cpp::GenerateVelocityKeepingCandidates와 동일한 호출
    // (v1 = target_speed + dsdot, MakeQuartic(start.s, start.s_d, start.s_dd,
    //  v1, 0.0, T)) 로 같은 다항식을 재구성한다.
    const double v1 = target_speed + dsdot;
    QuarticPolynomial poly = MakeQuartic(start.s, start.s_d, start.s_dd, v1, 0.0, T);

    double max_abs_jerk = 0.0;
    for (double t = 0.0; t <= T + 1e-9; t += 0.01) {
        max_abs_jerk = std::max(max_abs_jerk, std::fabs(EvalJerk(poly, t)));
    }
    std::cout << "  reproducer exact max |s_ddd| (EvalJerk) = " << max_abs_jerk << '\n';

    KinematicLimits limits = MakeLimits();
    Expect(max_abs_jerk > limits.max_longitudinal_jerk,
           "excessive-jerk reproducer: candidate's exact jerk peak exceeds "
           "the hard limit");

    double max_abs_accel = 0.0;
    for (const auto& path : candidates)
        for (double a : path.s_dd) max_abs_accel = std::max(max_abs_accel, std::fabs(a));
    std::cout << "  reproducer max |s_dd| = " << max_abs_accel << '\n';
    Expect(max_abs_accel <= limits.max_longitudinal_accel,
           "excessive-jerk reproducer: candidate's accel stays within the "
           "accel-only limit (isolates the jerk violation)");

    FilterLongitudinalByKinematicLimits(candidates, limits);
    bool candidate_still_valid = false;
    for (const auto& path : candidates) if (path.valid) candidate_still_valid = true;
    Expect(!candidate_still_valid,
           "excessive-jerk reproducer: FilterLongitudinalByKinematicLimits "
           "rejects the high-jerk candidate");
}

// CASE 3 - 정상 KEEP candidate
void TestNormalKeepCandidateSurvives() {
    FrenetState start{};
    start.s = 0.0;
    start.s_d = 8.0;
    start.s_dd = 0.0;
    start.d = 0.0;
    start.d_d = 0.0;
    start.d_dd = 0.0;

    PathGeneratorConfig cfg;
    cfg.delta_s_dot = {-2.0, 2.0, 1.0};
    cfg.time_horizon = {1.5, 5.0, 0.5};
    cfg.dt = 0.1;

    std::vector<FrenetPath> candidates =
        GenerateVelocityKeepingCandidates(start, /*target_speed=*/8.0, cfg);
    const std::size_t total_before = candidates.size();
    KinematicLimits limits = MakeLimits();
    FilterLongitudinalByKinematicLimits(candidates, limits);

    std::size_t valid_count = 0;
    for (const auto& path : candidates) if (path.valid) ++valid_count;
    std::cout << "  normal KEEP: candidates before filter = " << total_before
              << ", valid after filter = " << valid_count << '\n';
    Expect(valid_count > 0,
           "normal KEEP candidate: at least one candidate survives the new "
           "kinematic-limits filter");
}

// CASE 4 - zero velocity
void TestExactStopVelocityIsValid() {
    FrenetState start{};
    start.s = 0.0;
    start.s_d = 3.0;
    start.s_dd = 0.0;
    start.d = 0.0;
    start.d_d = 0.0;
    start.d_dd = 0.0;

    PathGeneratorConfig cfg;
    cfg.delta_s = {0.0, 0.0, 0.0};
    cfg.time_horizon = {3.0, 3.0, 0.0};
    cfg.dt = 0.1;

    std::vector<FrenetPath> candidates =
        GenerateStoppingCandidates(start, /*stop_s=*/4.0, cfg);
    Expect(!candidates.empty(), "exact-stop: candidate generated");
    Expect(!candidates.empty() && std::fabs(candidates.back().s_d.back()) < 1e-6,
           "exact-stop: terminal s_d is exactly 0");

    KinematicLimits limits = MakeLimits();
    FilterLongitudinalByKinematicLimits(candidates, limits);
    bool exact_stop_survives = false;
    for (const auto& path : candidates) if (path.valid) exact_stop_survives = true;
    Expect(exact_stop_survives,
           "exact-stop: candidate remains valid after the kinematic-limits "
           "filter (s_d == 0 is not rejected as reverse)");
}

int main() {
    TestNegativeVelocityReproducer();
    TestExcessiveJerkReproducer();
    TestNormalKeepCandidateSurvives();
    TestExactStopVelocityIsValid();

    if (g_failures == 0) {
        std::cout << "[ALL PASS]\n";
        return 0;
    }
    std::cerr << g_failures << " test(s) failed\n";
    return 1;
}
