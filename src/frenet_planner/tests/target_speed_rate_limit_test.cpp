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
// 실제 production ApplyTargetSpeedRateLimit(Speed/SpeedPolicy.hpp)를
// 그대로 호출한다 - 로직을 복사하지 않는다.

#include "frenet_planner/Speed/SpeedPolicy.hpp"

#include <cmath>
#include <iostream>
#include <string>

namespace {

int g_failures = 0;

void Expect(bool condition, const std::string& name) {
    if (condition) std::cout << "[PASS] " << name << '\n';
    else { std::cerr << "[FAIL] " << name << '\n'; ++g_failures; }
}

}  // namespace

// 10 -> safe 5.x 감소는 braking envelope에 의해 안전하게 이루어짐(여기서는
// 이미 감소된 raw_target을 rate limiter가 그대로 통과시키는지 확인).
void TestDownwardTransitionIsImmediate() {
    const double result = ApplyTargetSpeedRateLimit(
        /*raw_target=*/5.3, /*last_target=*/10.0, /*has_history=*/true,
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
    const double result = ApplyTargetSpeedRateLimit(
        /*raw_target=*/10.0, /*last_target=*/5.3, /*has_history=*/true,
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
        last = ApplyTargetSpeedRateLimit(raw_target, last, /*has_history=*/true,
                                         max_accel, dt);
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
    const double result = ApplyTargetSpeedRateLimit(
        /*raw_target=*/8.0, /*last_target=*/0.0, /*has_history=*/false,
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
    const double result = ApplyTargetSpeedRateLimit(
        raw_target, /*last_target=*/0.0, /*has_history=*/true,
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
