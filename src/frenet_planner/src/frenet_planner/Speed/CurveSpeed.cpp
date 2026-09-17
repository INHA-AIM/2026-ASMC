#include "frenet_planner/Speed/CurveSpeed.hpp"

#include "frenet_planner/Math/Utils.hpp"

#include <algorithm>
#include <cmath>

double VelocityFromCurvature(double k, const CurveSpeedConfig& cfg, double max_lateral_accel) {
    constexpr double kMinKappaForCalc = 1e-4;      // 0 나누기 방지 (거의 직선)
    const double v_curve = std::sqrt(
        max_lateral_accel * cfg.lateral_accel_safety_factor / std::max(k, kMinKappaForCalc));
    return clip(v_curve, 0.0, cfg.target_vel);
}

double LookaheadTargetSpeed(const RefLine& ref, double s_start, double ego_speed,
                             const CurveSpeedConfig& cfg, double max_longitudinal_accel,
                             double max_lateral_accel, double max_curvature) {
    const double usable_decel = std::max(
        cfg.min_usable_decel, max_longitudinal_accel * cfg.comfortable_decel_factor);
    const double v_floor = VelocityFromCurvature(max_curvature, cfg, max_lateral_accel);
    const double speed_drop = std::max(0.0, ego_speed * ego_speed - v_floor * v_floor);
    const double required_dist = speed_drop / (2.0 * usable_decel);
    // curve_lookahead_m은 더 이상 동적 제동거리의 하한이 아니다. 동적
    // 제동거리 *앞쪽*을 얼마나 추가로 볼지 정하는 명시적인 preview다.
    // 따라서 15 -> 30 m 변경은 현재 속도와 관계없이 정확히 15 m만큼 더
    // 먼 곡률을 보게 한다. max(base, dynamic) 구조에서는 고속일수록 설정값이
    // 동적 거리에 가려져 튜닝해도 아무 변화가 없는 구간이 생겼다.
    const double lookahead_m =
        required_dist + std::max(0.0, cfg.curve_lookahead_m);

    // ref.closed_loop이 Reference 계층의 topology 계약이다(F-11) - 이 함수가
    // 별도로 폐루프 여부를 추정하지 않는다. 닫힌 트랙만 이음새 너머(실제로는
    // 바로 이어지는 트랙 시작 구간)까지 lookahead가 감아 넘어간다. 열린
    // 경로는 끝에서 lookahead를 멈춘다 - 반대쪽(트랙 시작)이 물리적으로
    // 이어지지 않기 때문이다.
    const double length = ref.length;
    const bool closed = ref.closed_loop;

    double s_start_w = s_start;
    if (closed && length > 0.0) {
        s_start_w = std::fmod(s_start, length);
        if (s_start_w < 0.0) s_start_w += length;
    }
    const double s_end_w = s_start_w + lookahead_m;
    // wrap 여부는 폐루프에서만 의미가 있다. 열린 경로는 s_end_w가 length를
    // 넘어도 그 너머 구간(존재하지 않음)을 보지 않는다.
    const bool wraps = closed && length > 0.0 && s_end_w > length;
    const double wrapped_end = wraps ? (s_end_w - length) : 0.0;

    // straight road(장애물 없음) 기본값 - lookahead 안에 커브가 전혀 없으면
    // 이 값이 그대로 남는다(cfg.target_vel과 동일, VelocityFromCurvature(0, ...)).
    double envelope_speed = cfg.target_vel;
    for (const auto& p : ref.points) {
        bool in_range;
        double forward_dist;
        if (wraps) {
            in_range = (p.s >= s_start_w || p.s <= wrapped_end);
            forward_dist = (p.s >= s_start_w) ? (p.s - s_start_w)
                                              : (length - s_start_w + p.s);
        } else {
            in_range = (p.s >= s_start_w && p.s <= s_end_w);
            forward_dist = p.s - s_start_w;
        }
        if (!in_range) continue;

        const double local_curve_speed =
            VelocityFromCurvature(std::fabs(p.kappa), cfg, max_lateral_accel);
        // "그 지점에서 local_curve_speed까지 comfortable_decel로 감속"을
        // 거꾸로 풀면 "지금 허용 가능한 최대 속도"가 나온다:
        //   current_allowed_speed^2 = local_curve_speed^2 + 2*decel*distance
        const double current_allowed_speed = std::sqrt(
            local_curve_speed * local_curve_speed +
            2.0 * usable_decel * forward_dist);
        envelope_speed = std::min(envelope_speed, current_allowed_speed);
    }
    return clip(envelope_speed, 0.0, cfg.target_vel);
}
