#include "frenet_planner/Generator/LateralGenerator.hpp"

#include <cmath>

#include "frenet_planner/Generator/SamplingUtils.hpp"
#include "frenet_planner/Math/Polynomial.hpp"

// 횡방향 quintic 하나를 시간 샘플에 맞춰 FrenetPath로 변환 (d, d_d, d_dd만 채움)
FrenetPath SampleLateralQuintic(const QuinticPolynomial& poly, const std::vector<double>& times) {
    FrenetPath path;
    path.t = times;
    path.valid = true;
    path.cost_lat = path.cost_lon = path.cost_total = 0.0;
    path.jerk_cost_lat = JerkCost(poly, times.back());
    path.jerk_cost_lon = 0.0;
    path.delta_s = 0.0;
    path.delta_s_dot = 0.0;
    path.d.reserve(times.size());
    path.d_d.reserve(times.size());
    path.d_dd.reserve(times.size());
    for (double t : times) {
        path.d.push_back(EvalPos(poly, t));
        path.d_d.push_back(EvalVel(poly, t));
        path.d_dd.push_back(EvalAcc(poly, t));
    }
    return path;
}

// =========================================================
// [Sec.IV-A] 횡방향 후보 집합 (고속 모드, d(t) quintic)
// =========================================================

std::vector<FrenetPath> GenerateLateralCandidates(const FrenetState& start,
                                                   const PathGeneratorConfig& cfg) {
    std::vector<FrenetPath> result;

    // 저속 임시 처리 (헤더 주석 참고): 이 속도 미만에서는
    // d(t) quintic이 물리적으로 불가능한(옆으로 미끄러지는) 후보만 만들어
    // 곡률 필터에서 전부 걸러지므로, d1 격자 대신 현재 d를 유지하는 후보
    // 하나만 생성해 최소한 직진 가속은 가능하게 한다.
    // 저속 단일 후보(start.d)만 두면, 그 하나가 어떤 이유로든 필터에 걸리는
    // 순간(예: 경계조건 오버슈트) 대안이 아예 없어 "정지 -> 후보 0개 -> 정지
    // 유지"로 영원히 갇히는 문제가 실측으로 확인됐다(속도가 임계값을 못 넘어
    // 저속 분기를 못 벗어남). start.d와 다르면(=0 근처가 아니면) 차선 중앙
    // (d1=0) 후보도 같이 만들어 최소한 하나는 살아남을 여지를 준다.
    constexpr double kLowSpeedThreshold = 0.5;  // [m/s]
    std::vector<double> d1_list;
    if (std::fabs(start.s_d) < kLowSpeedThreshold) {
        d1_list.push_back(start.d);
        if (std::fabs(start.d) > 1e-3) d1_list.push_back(0.0);
    } else {
        d1_list = SampleRange(cfg.lateral_d1);
    }
    std::vector<double> T_list  = SampleRange(cfg.time_horizon);

    for (double T : T_list) {
        if (T <= 1e-6) continue;  // MakeQuintic의 T 분모 보호
        std::vector<double> times = SampleTimes(T, cfg.dt);

        for (double d1 : d1_list) {
            // 종료조건 [d1, 0, 0, T] — 논문 Sec.IV-A, "차선과 평행하게(dot=ddot=0) 복귀"
            QuinticPolynomial poly = MakeQuintic(start.d, start.d_d, start.d_dd,
                                                  d1, 0.0, 0.0, T);
            result.push_back(SampleLateralQuintic(poly, times));
        }
    }

    return result;
}

std::vector<FrenetPath> GenerateLateralContinuityCandidates(const FrenetState& start,
                                                             const PathGeneratorConfig& cfg) {
    PathGeneratorConfig continuity_cfg = cfg;
    continuity_cfg.lateral_d1.min = start.d;
    continuity_cfg.lateral_d1.max = start.d;
    continuity_cfg.lateral_d1.step = 0.0;
    return GenerateLateralCandidates(start, continuity_cfg);
}

double ResolveLateralOffset(const PlannerCommand& cmd, double lane_width) {
    switch (cmd.mode) {
        case AVOID:
            return cmd.avoidance_d_offset;

        // D7: FSM이 target_pose_map으로 준 목표를 Frenet d로 투영한 값을
        // 그대로 쓴다(LaneChange.cpp가 lane_change_d_offset을 채운다). 예전의
        // 고정 ±lane_width는 FSM이 실제로 지정한 목표점을 무시하고 항상 한
        // 차로 폭만큼만 이동한다고 가정했다 - 실제 인접 차로 중심이
        // lane_width와 다르면(곡선 구간, 차로폭이 다른 도로) 틀린 목표로
        // 수렴한다.
        case LANE_CHANGE_LEFT:
        case LANE_CHANGE_RIGHT:
            return cmd.lane_change_d_offset;

        case MERGE:
            return 0.0;  // 회전교차로 진입 global path 자체를 횡방향으로 추종

        default:
            return 0.0;  // 차선 중앙 유지
    }
}
