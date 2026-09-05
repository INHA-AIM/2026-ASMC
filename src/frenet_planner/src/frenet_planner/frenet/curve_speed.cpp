#include "frenet_planner/frenet/curve_speed.hpp"

#include "global/utils.hpp"

#include <algorithm>
#include <cmath>

double VelocityFromCurvature(double k, const CurveSpeedConfig& cfg, double max_lateral_accel) {
    constexpr double kSafetyMargin = 0.8;         // 한계의 80%만 사용 (여유폭 확보)
    constexpr double kMinKappaForCalc = 1e-4;      // 0 나누기 방지 (거의 직선)
    const double v_curve = std::sqrt(max_lateral_accel * kSafetyMargin / std::max(k, kMinKappaForCalc));
    return clip(v_curve, 0.0, cfg.target_vel);
}

double LookaheadTargetSpeed(const RefLine& ref, double s_start, double ego_speed,
                             const CurveSpeedConfig& cfg, double max_longitudinal_accel,
                             double max_lateral_accel, double max_curvature) {
    constexpr double kComfortableDecelFactor = 0.5;  // 실제 평균 감속도는 max의 절반 정도로 가정
    constexpr double kReactionMargin = 5.0;          // [m] 추가 여유거리
    const double usable_decel = std::max(0.5, max_longitudinal_accel * kComfortableDecelFactor);
    const double v_floor = VelocityFromCurvature(max_curvature, cfg, max_lateral_accel);
    const double speed_drop = std::max(0.0, ego_speed * ego_speed - v_floor * v_floor);
    const double required_dist = speed_drop / (2.0 * usable_decel);
    const double lookahead_m = std::max(cfg.curve_lookahead_m, required_dist + kReactionMargin);

    // 폐루프 wrap-around: s_start+lookahead_m이 s_max를 넘으면(피니시라인
    // 근처) 원래 코드는 뒤가 안 나와 break만 되고 이음새 너머(실제로는 바로
    // 이어지는 트랙 시작 구간)의 곡률을 놓쳤다 - 그 구간에 급커브가 있으면
    // 감속 없이 그대로 진입하게 된다. s_max로 감아서 [s_start, s_max]와
    // [0, 넘친 만큼] 두 구간을 모두 본다.
    const double s_max = ref.points.back().s;
    double s_start_w = std::fmod(s_start, s_max);
    if (s_start_w < 0.0) s_start_w += s_max;
    const double s_end_w = s_start_w + lookahead_m;
    const double wrapped_end = s_end_w - s_max;  // s_end_w > s_max일 때만 유효

    // straight road(장애물 없음) 기본값 - lookahead 안에 커브가 전혀 없으면
    // 이 값이 그대로 남는다(cfg.target_vel과 동일, VelocityFromCurvature(0, ...)).
    double envelope_speed = cfg.target_vel;
    for (const auto& p : ref.points) {
        const bool in_range = (s_end_w <= s_max)
            ? (p.s >= s_start_w && p.s <= s_end_w)
            : (p.s >= s_start_w || p.s <= wrapped_end);
        if (!in_range) continue;

        // 이 지점까지의 전진 호길이(wrap-around 포함).
        const double forward_dist = (p.s >= s_start_w)
            ? (p.s - s_start_w)
            : (s_max - s_start_w + p.s);

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
