// Cycle-to-cycle target speed recovery continuity 회귀
// (fix/yuntae-frenet-speed-continuity Phase 2).
//
// 이 테스트가 지키는 계약:
//   - safe raw target 감소는 즉시 허용된다 (curve braking envelope가 이미
//     안전하게 처리하므로 여기서 추가 제약을 두지 않는다).
//   - raw target 증가는 max_target_speed_accel * replanning_period_s로
//     rate-limit된다.
//   - 여러 cycle에 걸쳐 정상적으로 목표 속도까지 회복된다.
//   - 첫 cycle(has_history=false)에는 rate limit이 적용되지 않는다
//     (초기 ego 속도와 무관하게 0 근처로 묶이지 않아야 한다).
//
// R7-B: 이 rate limit은 실제 production Speed/TargetSpeedResolver.cpp의
// ResolveTargetSpeed 내부(과거 SpeedPolicy.hpp의 ApplyTargetSpeedRateLimit)로
// 옮겨졌다 - 더 이상 단독으로 링크되는 free function이 아니므로, 공개 API인
// ResolveTargetSpeed를 곧은 직선 RefLine(kappa=0)으로 호출해 curve envelope가
// 항상 target_vel(=raw target 상한 역할)을 그대로 반환하게 만들고, external
// target/max cap 없이 ego_speed로 raw target을 지정해 rate-limit 동작만
// 분리해서 검증한다. 로직을 복사하지 않는다 - 실제 production 코드를 그대로
// 호출한다.

#include "frenet_planner/Speed/TargetSpeedResolver.hpp"

#include <cmath>
#include <iostream>
#include <string>

namespace {

int g_failures = 0;

void Expect(bool condition, const std::string& name) {
    if (condition) std::cout << "[PASS] " << name << '\n';
    else { std::cerr << "[FAIL] " << name << '\n'; ++g_failures; }
}

RefLine MakeFlatRefLine(double target_vel) {
    RefLine ref;
    for (double s = 0.0; s <= 200.0; s += 1.0) {
        RefPoint p{};
        p.x = s;
        p.y = 0.0;
        p.s = s;
        p.kappa = 0.0;
        ref.points.push_back(p);
    }
    ref.length = ref.points.empty() ? 0.0 : ref.points.back().s;
    ref.closed_loop = false;
    return ref;
}

CurveSpeedConfig MakeCurveCfg(double target_vel) {
    CurveSpeedConfig cfg;
    cfg.target_vel = target_vel;
    cfg.curve_lookahead_m = 15.0;
    cfg.lateral_accel_safety_factor = 0.8;
    cfg.comfortable_decel_factor = 0.5;
    cfg.min_usable_decel = 0.5;
    return cfg;
}

KinematicLimits MakeLimits() {
    KinematicLimits limits{};
    limits.max_lateral_accel = 3.0;
    limits.max_longitudinal_accel = 3.0;
    limits.max_curvature = 0.3;
    limits.max_longitudinal_jerk = 5.0;
    limits.low_speed_fallback_v = 0.5;
    limits.low_speed_crawl_duration_s = 2.0;
    limits.near_low_speed_fallback_v = 1.0;
    return limits;
}

// raw_target(target_vel, straight road에서 curve envelope = target_vel 그대로)
// 에 대해 rate limit만 검증하는 helper.
double ResolveRaw(double raw_target, double previous_target_speed,
                   bool has_history, double max_target_speed_accel,
                   double replanning_period_s) {
    const RefLine ref = MakeFlatRefLine(raw_target);
    const CurveSpeedConfig curve_cfg = MakeCurveCfg(raw_target);
    const KinematicLimits limits = MakeLimits();

    TargetSpeedInput input;
    input.ego_s = 0.0;
    input.ego_speed = raw_target;  // straight road: envelope caps at target_vel
    input.has_external_target_speed = false;
    input.has_external_max_speed = false;
    input.has_history = has_history;
    input.previous_target_speed = previous_target_speed;

    return ResolveTargetSpeed(ref, input, curve_cfg, limits,
                              max_target_speed_accel, replanning_period_s);
}

}  // namespace

// 10 -> safe 5.x 감소는 braking envelope에 의해 안전하게 이루어짐(여기서는
// 이미 감소된 raw_target을 rate limiter가 그대로 통과시키는지 확인).
void TestDownwardTransitionIsImmediate() {
    const double result = ResolveRaw(
        /*raw_target=*/5.3, /*previous_target_speed=*/10.0, /*has_history=*/true,
        /*max_target_speed_accel=*/2.0, /*replanning_period_s=*/0.05);
    std::cout << "  10.0 -> raw 5.3: result = " << result << '\n';
    Expect(std::fabs(result - 5.3) < 1e-9,
           "downward raw target (10 -> 5.3) passes through immediately, "
           "unrate-limited");
}

// 5.x -> 10 복귀는 한 cycle에 jump하지 않는다.
void TestUpwardTransitionIsRateLimited() {
    const double max_accel = 2.0;
    const double dt = 0.05;
    const double result = ResolveRaw(
        /*raw_target=*/10.0, /*previous_target_speed=*/5.3, /*has_history=*/true,
        max_accel, dt);
    const double expected_cap = 5.3 + max_accel * dt;
    std::cout << "  5.3 -> raw 10.0: result = " << result
              << " (cap = " << expected_cap << ")\n";
    Expect(std::fabs(result - expected_cap) < 1e-9,
           "upward raw target (5.3 -> 10) is capped at "
           "last_target + max_target_speed_accel*replanning_period_s");
    Expect(result < 10.0 - 1e-6,
           "upward transition does not jump to raw_target in one cycle");
}

// multiple cycles에서 정상적으로 10까지 회복.
void TestMultiCycleRecoveryReachesTarget() {
    const double max_accel = 2.0;
    const double dt = 0.05;
    const double raw_target = 10.0;
    double last = 5.3;
    int cycles = 0;
    constexpr int kMaxCycles = 10000;  // 무한루프 방지 safety bound
    while (last < raw_target - 1e-9 && cycles < kMaxCycles) {
        last = ResolveRaw(raw_target, last, /*has_history=*/true, max_accel, dt);
        ++cycles;
    }
    std::cout << "  cycles to recover 5.3 -> 10.0 = " << cycles
              << ", final = " << last << '\n';
    Expect(cycles > 1 && cycles < kMaxCycles,
           "multi-cycle recovery reaches target speed within a bounded "
           "number of cycles (not one jump, not never)");
    Expect(std::fabs(last - raw_target) < 1e-6,
           "final recovered value equals raw_target exactly (no overshoot, "
           "no undershoot)");
}

// first cycle/uninitialized state 처리: has_history=false면 raw_target을
// 그대로 반환한다 (last_target의 기본값 0.0과 무관하게 묶이지 않음).
void TestFirstCycleIsNotRateLimited() {
    const double result = ResolveRaw(
        /*raw_target=*/8.0, /*previous_target_speed=*/0.0, /*has_history=*/false,
        /*max_target_speed_accel=*/2.0, /*replanning_period_s=*/0.05);
    std::cout << "  first cycle (has_history=false): result = " << result << '\n';
    Expect(std::fabs(result - 8.0) < 1e-9,
           "first cycle (has_history=false) is not rate-limited even though "
           "last_target defaults to 0.0");
}

// external max_speed/target_speed restriction을 초과하지 않는다: rate
// limiter는 raw_target(이미 외부 제한이 적용된 값) 이하로만 결과를 낸다.
void TestNeverExceedsRawTarget() {
    const double raw_target = 3.0;  // 예: 외부 max_speed=3.0으로 이미 clamp된 값
    const double result = ResolveRaw(
        raw_target, /*previous_target_speed=*/0.0, /*has_history=*/true,
        /*max_target_speed_accel=*/2.0, /*replanning_period_s=*/0.05);
    std::cout << "  raw_target(external-capped)=3.0, last=0.0: result = "
              << result << '\n';
    Expect(result <= raw_target + 1e-9,
           "rate limiter never returns a value above raw_target (external "
           "restriction is respected)");
}

int main() {
    TestDownwardTransitionIsImmediate();
    TestUpwardTransitionIsRateLimited();
    TestMultiCycleRecoveryReachesTarget();
    TestFirstCycleIsNotRateLimited();
    TestNeverExceedsRawTarget();

    if (g_failures == 0) {
        std::cout << "[ALL PASS]\n";
        return 0;
    }
    std::cerr << g_failures << " test(s) failed\n";
    return 1;
}
