#include "Frenet/Speed/CurveSpeed.hpp"

#include "Frenet/Math/Utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

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

std::vector<SpeedEnvelopePoint> BuildKeepSpeedEnvelope(
    const RefLine& ref, double s_start, double ego_speed, double initial_accel,
    double candidate_duration, double sample_dt, double raw_speed_cap,
    const CurveSpeedConfig& cfg, double max_longitudinal_accel,
    double max_lateral_accel, double max_longitudinal_jerk) {
    if (ref.points.empty() || !std::isfinite(s_start) ||
        !std::isfinite(ego_speed) || !std::isfinite(initial_accel) ||
        !std::isfinite(raw_speed_cap) || raw_speed_cap < 0.0 ||
        candidate_duration <= 0.0 || sample_dt <= 0.0 ||
        max_longitudinal_accel <= 0.0 || max_longitudinal_jerk <= 0.0) {
        return {};
    }
    const double decel = std::min(max_longitudinal_accel, std::max(
        cfg.min_usable_decel,
        max_longitudinal_accel * cfg.comfortable_decel_factor));
    const double cap = std::min(cfg.target_vel, raw_speed_cap);
    const double travel_speed = std::max(cfg.target_vel, std::max(0.0, ego_speed));
    // The braking equation assumes immediate constant deceleration. Reserve
    // distance for ramping from the start acceleration to -decel first, so a
    // jerk-limited profile can begin braking BEFORE that ideal boundary.
    const double response_time = std::max(sample_dt,
        (std::max(0.0, initial_accel) + decel) / max_longitudinal_jerk);
    const double response_distance = travel_speed * response_time +
        0.5 * max_longitudinal_accel * response_time * response_time;
    const double span = travel_speed * candidate_duration +
        travel_speed * travel_speed / (2.0 * decel) +
        std::max(0.0, cfg.curve_lookahead_m) + response_distance;
    const double end = s_start + span;
    std::vector<SpeedEnvelopePoint> samples{{s_start, cap * cap}};
    const bool closed = ref.closed_loop && ref.length > 0.0;
    const long first_lap = closed
        ? static_cast<long>(std::floor(s_start / ref.length)) : 0;
    const long last_lap = closed
        ? static_cast<long>(std::floor((end + response_distance) / ref.length)) : 0;
    for (long lap = first_lap; lap <= last_lap; ++lap) {
        const double shift = closed ? lap * ref.length : 0.0;
        for (const RefPoint& point : ref.points) {
            const double position = point.s + shift;
            if (position < s_start || position > end + response_distance) continue;
            const double local_cap = std::min(cap,
                VelocityFromCurvature(std::fabs(point.kappa), cfg,
                                      max_lateral_accel));
            samples.push_back({std::max(s_start, position - response_distance),
                               local_cap * local_cap});
        }
    }
    // Include the interpolated current curvature, not only the next waypoint.
    const double current_cap = std::min(cap, VelocityFromCurvature(
        std::fabs(Interpolate(ref, s_start).kappa), cfg, max_lateral_accel));
    samples.push_back({s_start, current_cap * current_cap});
    samples.push_back({end, cap * cap});
    std::sort(samples.begin(), samples.end(),
        [](const SpeedEnvelopePoint& a, const SpeedEnvelopePoint& b) {
            return a.s < b.s;
        });
    std::vector<SpeedEnvelopePoint> envelope;
    for (const auto& point : samples) {
        if (!envelope.empty() && point.s <= envelope.back().s + 1e-9) {
            envelope.back().speed_squared = std::min(
                envelope.back().speed_squared, point.speed_squared);
        } else {
            envelope.push_back(point);
        }
    }
    for (std::size_t i = envelope.size() - 1; i > 0; --i) {
        const double ds = envelope[i].s - envelope[i - 1].s;
        envelope[i - 1].speed_squared = std::min(
            envelope[i - 1].speed_squared,
            envelope[i].speed_squared + 2.0 * decel * ds);
    }
    return envelope;
}

CandidateCurveSpeedResult EvaluateCandidateCurveSpeed(
    const CartesianPath& candidate,
    double raw_speed_cap,
    double applied_target_speed,
    const CurveSpeedConfig& cfg,
    double max_longitudinal_accel,
    double max_lateral_accel) {
    CandidateCurveSpeedResult result;
    const std::size_t count = std::min(
        {candidate.x.size(), candidate.y.size(), candidate.kappa.size(),
         candidate.v.size()});
    if (count == 0) return result;

    const double usable_decel = std::max(
        cfg.min_usable_decel,
        max_longitudinal_accel * cfg.comfortable_decel_factor);
    if (!std::isfinite(usable_decel) || usable_decel <= 0.0 ||
        !std::isfinite(cfg.target_vel) || cfg.target_vel < 0.0 ||
        !std::isfinite(raw_speed_cap) || raw_speed_cap < 0.0 ||
        !std::isfinite(applied_target_speed) ||
        applied_target_speed < 0.0) {
        return result;
    }

    double distance = 0.0;
    // raw_speed_cap is the environmental/upstream ceiling.  It must remain
    // distinct from applied_target_speed: initializing this cap from the
    // lower profiled target would feed the profiler output back as its own cap
    // and permanently prevent recovery on a straight road.
    double speed_cap = std::min(cfg.target_vel, raw_speed_cap);
    const double initial_speed = std::fabs(candidate.v.front());
    for (std::size_t i = 0; i < count; ++i) {
        if (!std::isfinite(candidate.x[i]) ||
            !std::isfinite(candidate.y[i]) ||
            !std::isfinite(candidate.kappa[i]) ||
            !std::isfinite(candidate.v[i])) {
            return CandidateCurveSpeedResult{};
        }
        if (i > 0) {
            distance += std::hypot(candidate.x[i] - candidate.x[i - 1],
                                   candidate.y[i] - candidate.y[i - 1]);
        }

        const double abs_kappa = std::fabs(candidate.kappa[i]);
        const double local_curve_speed =
            VelocityFromCurvature(abs_kappa, cfg, max_lateral_accel);
        const double current_allowed_speed = std::sqrt(
            local_curve_speed * local_curve_speed +
            2.0 * usable_decel * distance);
        if (current_allowed_speed < speed_cap) {
            speed_cap = current_allowed_speed;
            result.limiting_sample_index = i;
        }

        // current_allowed_speed는 "지금 이 속도라면 감속을 시작해 미래
        // 곡률에 맞출 수 있는가"를 나타낸다. 하지만 velocity-keeping
        // quartic이 그 사이에 다시 가속하면 그 envelope를 실제로 따르지
        // 않을 수 있다. 각 거리에서 comfortable_decel로도 피할 수 없는
        // 최소속도와 local curve speed 중 큰 값을 sample 허용속도로 삼아,
        // 실제 candidate가 그보다 빠르면 terminal target을 같은 비율로
        // 낮춘다. 초기 sample은 minimum_reachable_speed=initial_speed라서
        // 이미 높은 현재속도의 불가피한 transient를 reject하지 않는다.
        const double minimum_reachable_speed = std::sqrt(std::max(
            0.0, initial_speed * initial_speed -
                     2.0 * usable_decel * distance));
        const double sample_allowed_speed =
            std::max(local_curve_speed, minimum_reachable_speed);
        const double candidate_speed = std::fabs(candidate.v[i]);
        if (candidate_speed > sample_allowed_speed &&
            candidate_speed > std::numeric_limits<double>::epsilon()) {
            const double profile_consistent_target =
                applied_target_speed * sample_allowed_speed /
                candidate_speed;
            if (profile_consistent_target < speed_cap) {
                speed_cap = profile_consistent_target;
                result.limiting_sample_index = i;
            }
        }
        result.max_abs_curvature =
            std::max(result.max_abs_curvature, abs_kappa);
        result.max_lateral_accel = std::max(
            result.max_lateral_accel,
            candidate.v[i] * candidate.v[i] * abs_kappa);
    }

    result.speed_cap = clip(speed_cap, 0.0, cfg.target_vel);
    result.valid = std::isfinite(result.speed_cap);
    return result;
}
