#include "frenet_planner/Speed/TargetSpeedResolver.hpp"

#include <algorithm>

#include "frenet_planner/Speed/CurveSpeed.hpp"

namespace {

// R7-B: SpeedPolicy.cpp에 있던 것을 그대로 옮겼다 - 수식/의미 변경 없음.
//
// Cycle-to-cycle target speed recovery continuity (fix/yuntae-frenet-speed-
// continuity Phase 2). 안전 감속 방향(raw_target < last_target)은 curve
// braking envelope(LookaheadTargetSpeed)가 이미 처리하므로 여기서는 즉시
// 허용한다. 반대로 target이 이전 cycle보다 올라가는 방향만
// max_target_speed_accel * replanning_period_s 만큼으로 제한해, 커브를
// 벗어난 직후 5.x -> 10 같은 한 cycle급 점프를 막는다.
//
// has_history가 false(첫 cycle, 또는 EMERGENCY 직후 리셋 이후 첫 정상
// cycle)이면 raw_target을 그대로 반환한다 - 그렇지 않으면 last_target의
// 초기값(0.0)과 무관한 실제 ego 속도가 있어도 0 근처로 묶인다.
double ApplyTargetSpeedRateLimit(double raw_target, double last_target,
                                  bool has_history,
                                  double max_target_speed_accel,
                                  double replanning_period_s) {
    if (!has_history) return raw_target;
    if (raw_target <= last_target) return raw_target;
    const double allowed_increase = max_target_speed_accel * replanning_period_s;
    return std::min(raw_target, last_target + allowed_increase);
}

}  // namespace

double ResolveTargetSpeed(const RefLine& ref, const TargetSpeedInput& input,
                          const CurveSpeedConfig& curve_cfg,
                          const KinematicLimits& limits,
                          double max_target_speed_accel,
                          double replanning_period_s) {
    double resolved = LookaheadTargetSpeed(
        ref, input.ego_s, input.ego_speed, curve_cfg,
        limits.max_longitudinal_accel, limits.max_lateral_accel,
        limits.max_curvature);
    if (input.has_external_target_speed) {
        resolved = std::min(resolved, input.external_target_speed);
    }
    if (input.has_external_max_speed) {
        resolved = std::min(resolved, input.external_max_speed);
    }
    resolved = ApplyTargetSpeedRateLimit(
        resolved, input.previous_target_speed, input.has_history,
        max_target_speed_accel, replanning_period_s);
    return resolved;
}
