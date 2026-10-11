#ifndef FRENET_CURVE_SPEED_HPP
#define FRENET_CURVE_SPEED_HPP

#include <cstddef>

#include "Frenet/Common/CartesianTypes.hpp"
#include "Frenet/Reference/Reference.hpp"
#include "Frenet/Config/FrenetConfig.hpp"

// frenet_planner.cpp의 익명 namespace에 있던 곡률 기반 목표속도 계산을 분리한
// 것이다 (fix/yuntae-frenet-speed-continuity Phase 1) - longitudinal_kinematics_test류
// 회귀 테스트가 실제 waypoint 파일의 smoothing/resampling을 거치지 않고, 정확히
// 통제된 RefLine으로 이 로직을 직접 검증할 수 있어야 하기 때문이다. 로직 자체는
// 옮기기 전과 동일하며 이 분리 작업에서 수식을 바꾸지 않는다.

// 횡가속도 한계에서 곡률별 연속 목표속도를 계산한다.
double VelocityFromCurvature(double k, const CurveSpeedConfig& cfg, double max_lateral_accel);

// 현재 속도의 제동거리에 설정된 추가 preview 거리를 더한 범위를 훑되, 그 범위 안의
// "가장 급한 커브 하나"의 목표속도를 바로 지금 요구하지 않는다. 각 지점마다
// "그 지점의 커브 안전속도까지, 지금부터 그 지점까지의 거리 동안
// comfortable_decel로 감속하면 도달 가능한 현재 속도"를 계산하고, 그 중
// 최솟값을 사용한다.
double LookaheadTargetSpeed(const RefLine& ref, double s_start, double ego_speed,
                             const CurveSpeedConfig& cfg, double max_longitudinal_accel,
                             double max_lateral_accel, double max_curvature);

// Backward braking envelope over the complete candidate travel distance plus
// braking preview. Includes distance for the initial jerk-limited response.
std::vector<SpeedEnvelopePoint> BuildKeepSpeedEnvelope(
    const RefLine& ref, double s_start, double ego_speed, double initial_accel,
    double candidate_duration, double sample_dt, double raw_speed_cap,
    const CurveSpeedConfig& cfg, double max_longitudinal_accel,
    double max_lateral_accel, double max_longitudinal_jerk);

// 실제 MPC에 전달되는 Cartesian candidate를 기준으로 한 곡률-속도 envelope.
// reference line의 kappa가 아니라 selected trajectory의 kappa를 사용하고,
// 시작점부터 각 sample까지의 실제 Cartesian 호길이로 감속 가능 거리를
// 계산한다. speed_cap은 "현재부터 감속을 시작할 때 각 미래 곡률 지점의
// local curve speed에 도달 가능한 최대 목표속도"다.
struct CandidateCurveSpeedResult {
    bool valid = false;
    double speed_cap = 0.0;
    double max_abs_curvature = 0.0;
    double max_lateral_accel = 0.0;
    std::size_t limiting_sample_index = 0;
};

CandidateCurveSpeedResult EvaluateCandidateCurveSpeed(
    const CartesianPath& candidate,
    double raw_speed_cap,
    double applied_target_speed,
    const CurveSpeedConfig& cfg,
    double max_longitudinal_accel,
    double max_lateral_accel);

#endif
