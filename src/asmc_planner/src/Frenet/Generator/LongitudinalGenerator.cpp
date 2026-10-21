#include "Frenet/Generator/LongitudinalGenerator.hpp"

#include <algorithm>
#include <cmath>

#include "Frenet/Generator/SamplingUtils.hpp"
#include "Frenet/Evaluation/KinematicFilter.hpp"
#include "Frenet/Math/Polynomial.hpp"

// 종방향 quintic 하나를 시간 샘플에 맞춰 FrenetPath로 변환 (s, s_d, s_dd만 채움)
FrenetPath SampleLongitudinalQuintic(const QuinticPolynomial& poly, const std::vector<double>& times) {
    FrenetPath path;
    path.t = times;
    path.valid = true;
    path.cost_lat = path.cost_lon = path.cost_total = 0.0;
    path.jerk_cost_lat = 0.0;
    path.jerk_cost_lon = JerkCost(poly, times.back());
    path.delta_s = 0.0;
    path.delta_s_dot = 0.0;
    path.s.reserve(times.size());
    path.s_d.reserve(times.size());
    path.s_dd.reserve(times.size());
    path.s_ddd.reserve(times.size());
    for (double t : times) {
        path.s.push_back(EvalPos(poly, t));
        path.s_d.push_back(EvalVel(poly, t));
        path.s_dd.push_back(EvalAcc(poly, t));
        path.s_ddd.push_back(EvalJerk(poly, t));
    }
    return path;
}

// 종방향 quartic 하나(velocity keeping)를 시간 샘플에 맞춰 FrenetPath로 변환
FrenetPath SampleLongitudinalQuartic(const QuarticPolynomial& poly, const std::vector<double>& times) {
    FrenetPath path;
    path.t = times;
    path.valid = true;
    path.cost_lat = path.cost_lon = path.cost_total = 0.0;
    path.jerk_cost_lat = 0.0;
    path.jerk_cost_lon = JerkCost(poly, times.back());
    path.delta_s = 0.0;
    path.delta_s_dot = 0.0;
    path.s.reserve(times.size());
    path.s_d.reserve(times.size());
    path.s_dd.reserve(times.size());
    path.s_ddd.reserve(times.size());
    for (double t : times) {
        path.s.push_back(EvalPos(poly, t));
        path.s_d.push_back(EvalVel(poly, t));
        path.s_dd.push_back(EvalAcc(poly, t));
        path.s_ddd.push_back(EvalJerk(poly, t));
    }
    return path;
}

// =========================================================
// [Sec.V-A] Following
// =========================================================

double FollowingApproachSpeed(double ego_s,double leader_s,double leader_speed,
                             double time_gap,double min_gap,double cruise_speed,
                             double gap_gain) {
    const double gap_error=(leader_s-ego_s)-(min_gap+time_gap*leader_speed);
    return std::max(0.0,std::min(cruise_speed,leader_speed+gap_gain*gap_error));
}

std::vector<FrenetPath> GenerateFollowingCandidates(const FrenetState& start,
                                                     double leader_s,
                                                     double leader_speed,
                                                     double leader_accel,
                                                     double time_gap,
                                                     double min_gap,
                                                     double cruise_target_speed,
                                                     double gap_gain,
                                                     const PathGeneratorConfig& cfg) {
    std::vector<FrenetPath> result;

    std::vector<double> delta_list = SampleRange(cfg.delta_s);
    std::vector<double> T_list     = SampleRange(cfg.time_horizon);

    // Constant-time-gap 오차를 접근 속도로 피드백한다. 목표 간격보다 멀면
    // leader보다 빠르게 접근하고, 목표 간격에서는 leader 속도와 같아진다.
    // 상한은 도로 곡률 등을 반영해 상위 planner가 정한 순항 목표속도이다.
    const double approach_speed=FollowingApproachSpeed(start.s,leader_s,leader_speed,
        time_gap,min_gap,cruise_target_speed,gap_gain);

    for (double T : T_list) {
        if (T <= 1e-6) continue;
        std::vector<double> times = SampleTimes(T, cfg.dt);

        // 선두 차량 등가속(leader_accel = const) 예측을 T 시점까지 시간적분
        const double s_lv_T     = leader_s + leader_speed * T + 0.5 * leader_accel * T * T;
        const double s_lv_dot_T = leader_speed + leader_accel * T;

        // s_follow(t) = s_lv(t) - [D0 + tau*s_lv_dot(t)] (D0=min_gap, tau=time_gap).
        // leader가 멀리 있을 때 following 지점을 한 horizon 안에 강제로 맞추면
        // 비현실적인 quintic이 된다. 그 경우에는 현재 속도를 고정하지 않고,
        // 위의 간격 피드백으로 계산한 접근 속도까지 부드럽게 천이한다.
        const double s_follow_T = s_lv_T - (min_gap + time_gap * s_lv_dot_T);
        const double free_s_T = start.s + 0.5 * (start.s_d + approach_speed) * T;
        // s_dot_target(t) = s_lv_dot(t) - tau*s_lv_ddot(t), s_lv_ddot=leader_accel(상수)
        const double s_follow_dot_T = s_lv_dot_T - time_gap * leader_accel;
        // s_ddot_target(t) = s_lv_ddot(t1) = leader_accel (상수, jerk=0 가정)
        const double s_follow_ddot_T = leader_accel;

        const bool gap_control_needed = s_follow_T <= free_s_T;
        const double s_target_T = gap_control_needed ? s_follow_T : free_s_T;
        const double s_target_dot_T = gap_control_needed ? s_follow_dot_T : approach_speed;
        const double s_target_ddot_T = gap_control_needed ? s_follow_ddot_T : 0.0;

        for (double delta : delta_list) {
            const double s1 = s_target_T + delta;
            QuinticPolynomial poly = MakeQuintic(start.s, start.s_d, start.s_dd,
                                                  s1, s_target_dot_T, s_target_ddot_T, T);
            FrenetPath path = SampleLongitudinalQuintic(poly, times);
            path.delta_s = delta;   // Ct의 [s1-sd]^2 = delta^2 항에 쓰임
            result.push_back(std::move(path));
        }
    }

    return result;
}

// =========================================================
// [Sec.V-A] Stopping
// =========================================================

std::vector<FrenetPath> GenerateStoppingCandidates(const FrenetState& start,
                                                    double stop_s,
                                                    const PathGeneratorConfig& cfg) {
    std::vector<FrenetPath> result;

    std::vector<double> delta_list = SampleRange(cfg.delta_s);
    std::vector<double> T_list     = SampleRange(cfg.time_horizon);

    for (double T : T_list) {
        if (T <= 1e-6) continue;
        std::vector<double> times = SampleTimes(T, cfg.dt);

        // s_target = s_stop (상수), s_dot_target = 0, s_ddot_target = 0  (논문 명시)
        for (double delta : delta_list) {
            const double s1 = stop_s + delta;
            QuinticPolynomial poly = MakeQuintic(start.s, start.s_d, start.s_dd,
                                                  s1, 0.0, 0.0, T);
            FrenetPath path = SampleLongitudinalQuintic(poly, times);
            path.delta_s = delta;
            result.push_back(std::move(path));
        }
    }

    return result;
}

// =========================================================
// [Sec.V-A] Merging
// =========================================================

std::vector<FrenetPath> GenerateMergingCandidates(const FrenetState& start,
                                                   double sa, double sa_speed, double sa_accel,
                                                   double sb, double sb_speed, double sb_accel,
                                                   const PathGeneratorConfig& cfg) {
    std::vector<FrenetPath> result;

    std::vector<double> delta_list = SampleRange(cfg.delta_s);
    std::vector<double> T_list     = SampleRange(cfg.time_horizon);

    for (double T : T_list) {
        if (T <= 1e-6) continue;
        std::vector<double> times = SampleTimes(T, cfg.dt);

        // sa(t), sb(t) 각각 등가속 예측 (Following과 동일한 적분, time-gap 항은 없음)
        const double sa_T     = sa + sa_speed * T + 0.5 * sa_accel * T * T;
        const double sa_dot_T = sa_speed + sa_accel * T;
        const double sb_T     = sb + sb_speed * T + 0.5 * sb_accel * T * T;
        const double sb_dot_T = sb_speed + sb_accel * T;

        // s_target(t) = 1/2 [sa(t)+sb(t)]  (논문 식 (3))
        const double s_target_T      = 0.5 * (sa_T + sb_T);
        const double s_target_dot_T  = 0.5 * (sa_dot_T + sb_dot_T);
        const double s_target_ddot_T = 0.5 * (sa_accel + sb_accel);

        for (double delta : delta_list) {
            const double s1 = s_target_T + delta;
            QuinticPolynomial poly = MakeQuintic(start.s, start.s_d, start.s_dd,
                                                  s1, s_target_dot_T, s_target_ddot_T, T);
            FrenetPath path = SampleLongitudinalQuintic(poly, times);
            path.delta_s = delta;
            result.push_back(std::move(path));
        }
    }

    return result;
}

std::vector<FrenetPath> GenerateMergeArrivalCandidates(const FrenetState& start,
                                                        double conflict_s,
                                                        double entry_time,
                                                        double entry_speed,
                                                        const PathGeneratorConfig& cfg,
                                                        double arrival_time_match_tolerance) {
    std::vector<FrenetPath> result;
    // lateral 후보와 정확히 같은 T 격자를 써야 Combine 단계에서 결합된다.
    // 선택 entry_time 주변의 격자만 남겨 시간 목표를 유지한다.
    for (double T = cfg.time_horizon.min;
         T <= cfg.time_horizon.max + 1e-9; T += cfg.time_horizon.step) {
        if (T <= 1e-6) continue;
        if (std::fabs(T - entry_time) >
            std::max(arrival_time_match_tolerance, cfg.time_horizon.step)) continue;
        const std::vector<double> times = SampleTimes(T, cfg.dt);
        // 위치 종단조건을 느슨한 평균속도로 대체하지 않고 conflict_s에 직접 고정한다.
        QuinticPolynomial poly = MakeQuintic(start.s, start.s_d, start.s_dd,
                                              conflict_s, entry_speed, 0.0, T);
        result.push_back(SampleLongitudinalQuintic(poly, times));
    }
    // entry_time이 격자 밖인 예외에는 가장 가까운 horizon 하나를 사용한다.
    if (result.empty()) {
        const double grid_index = std::round(
            (entry_time - cfg.time_horizon.min) / cfg.time_horizon.step);
        const double T = std::max(cfg.time_horizon.min,
            std::min(cfg.time_horizon.min + grid_index * cfg.time_horizon.step,
                     cfg.time_horizon.max));
        const std::vector<double> times = SampleTimes(T, cfg.dt);
        QuinticPolynomial poly = MakeQuintic(start.s, start.s_d, start.s_dd,
                                              conflict_s, entry_speed, 0.0, T);
        result.push_back(SampleLongitudinalQuintic(poly, times));
    }
    return result;
}

// =========================================================
// [Sec.V-B] Velocity Keeping
// =========================================================

std::vector<FrenetPath> GenerateVelocityKeepingCandidates(const FrenetState& start,
                                                           double target_speed,
                                                           const PathGeneratorConfig& cfg) {
    std::vector<FrenetPath> result;

    std::vector<double> dsdot_list = SampleRange(cfg.delta_s_dot);
    std::vector<double> T_list     = SampleRange(cfg.time_horizon);

    for (double T : T_list) {
        if (T <= 1e-6) continue;
        std::vector<double> times = SampleTimes(T, cfg.dt);

        for (double dsdot : dsdot_list) {
            // 종료조건 [s_dot_target, 0, T] — 위치 자유, 종료 가속도 항상 0 (논문 명시)
            const double v1 = target_speed + dsdot;
            QuarticPolynomial poly = MakeQuartic(start.s, start.s_d, start.s_dd,
                                                  v1, 0.0, T);
            FrenetPath path = SampleLongitudinalQuartic(poly, times);
            path.delta_s_dot = dsdot;   // Cv의 [s_dot1-s_dot_d]^2 = dsdot^2 항에 쓰임
            result.push_back(std::move(path));
        }
    }

    return result;
}


namespace {
struct EnvelopeSample { double speed; double acceleration; };

EnvelopeSample SampleSpeedEnvelope(
    const std::vector<SpeedEnvelopePoint>& envelope, double s, double offset) {
    auto upper = std::upper_bound(envelope.begin(), envelope.end(), s,
        [](double value, const SpeedEnvelopePoint& point) { return value < point.s; });
    if (upper == envelope.begin()) ++upper;
    if (upper == envelope.end()) upper = envelope.end() - 1;
    const auto& lo = *(upper - 1);
    const auto& hi = *upper;
    const double ds = hi.s - lo.s;
    const double ratio = std::clamp((s - lo.s) / ds, 0.0, 1.0);
    const double speed = std::sqrt(std::max(0.0,
        lo.speed_squared + ratio * (hi.speed_squared - lo.speed_squared)));
    const double shifted_speed = std::max(0.0, speed + offset);
    // a = (1/2) * d(v^2)/ds. Adjust for a negative speed offset.
    const double acceleration = speed > 1e-9
        ? 0.5 * (hi.speed_squared - lo.speed_squared) / ds * shifted_speed / speed
        : 0.0;
    return {shifted_speed, acceleration};
}
}  // namespace

std::vector<FrenetPath> GenerateEnvelopeVelocityCandidates(
    const FrenetState& start, double target_speed,
    const std::vector<SpeedEnvelopePoint>& envelope,
    const PathGeneratorConfig& cfg, const KinematicLimits& limits) {
    if (envelope.size() < 2 || cfg.dt <= 0.0 ||
        limits.max_longitudinal_accel <= 0.0 ||
        limits.max_longitudinal_jerk <= 0.0) return {};
    // Reuse configured projection headroom: Frenet derivatives at their exact
    // hard limits can exceed Cartesian limits after lateral/curvature coupling.
    const double headroom = std::clamp(limits.boundary_condition_accel_fraction,
                                      1e-6, 1.0);
    const double accel_limit = limits.max_longitudinal_accel * headroom;
    // Leave only numerical headroom for the existing strict hard-limit filter.
    const double jerk_limit = limits.max_longitudinal_jerk * headroom * (1.0 - 1e-12);
    // Tracking time is derived from existing physical limits, not horizon T.
    // Therefore replanning or selecting a longer T cannot postpone braking.
    const double response_time = std::max(cfg.dt, accel_limit / jerk_limit);
    std::vector<double> offsets{0.0};
    for (double value : SampleRange(cfg.delta_s_dot)) {
        if (value < -1e-9) offsets.push_back(value);
    }
    std::vector<FrenetPath> result;
    for (double duration : SampleRange(cfg.time_horizon)) {
        if (duration <= 1e-6) continue;
        for (double offset : offsets) {
            FrenetPath path{};
            path.valid = true;
            path.t = SampleTimes(duration, cfg.dt);
            double s = start.s;
            double v = std::max(0.0, start.s_d);
            double a = std::clamp(start.s_dd, -accel_limit, accel_limit);
            path.s.push_back(s);
            path.s_d.push_back(v);
            path.s_dd.push_back(a);
            path.s_ddd.push_back(0.0);
            for (std::size_t i = 1; i < path.t.size(); ++i) {
                const double h = path.t[i] - path.t[i - 1];
                const EnvelopeSample ceiling = SampleSpeedEnvelope(envelope, s, offset);
                double desired_a = std::clamp(ceiling.acceleration +
                    (ceiling.speed - v) / response_time, -accel_limit, accel_limit);
                // Preserve enough speed to release the next braking acceleration
                // at +jerk_limit: v_next >= a_next^2/(2*jerk_limit).
                // Solving this with trapezoidal acceleration integration gives
                // the discrete lower bound below (not an after-the-fact v clamp).
                const double discriminant = jerk_limit * jerk_limit * h * h +
                    8.0 * jerk_limit * std::max(0.0, v) +
                    4.0 * jerk_limit * a * h;
                const double stopping_accel_floor = 0.5 * (jerk_limit * h -
                    std::sqrt(std::max(0.0, discriminant)));
                desired_a = std::max(desired_a, stopping_accel_floor);
                const double next_a = std::clamp(desired_a,
                    a - jerk_limit * h, a + jerk_limit * h);
                const double jerk = (next_a - a) / h;
                const double next_v = v + a * h + 0.5 * jerk * h * h;
                const double next_s = s + v * h + 0.5 * a * h * h +
                    jerk * h * h * h / 6.0;
                // Never clamp v or s after integration: that would publish
                // derivatives inconsistent with the actual trajectory.
                if (next_v < -1e-8 || next_s < s - 1e-8) path.valid = false;
                s = next_s;
                v = next_v;
                a = next_a;
                path.s.push_back(s);
                path.s_d.push_back(v);
                path.s_dd.push_back(a);
                path.s_ddd.push_back(jerk);
                path.jerk_cost_lon += jerk * jerk * h;
            }
            path.delta_s_dot = path.s_d.back() - target_speed;
            result.push_back(std::move(path));
        }
    }
    return result;
}
