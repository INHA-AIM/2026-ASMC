#ifndef FRENET_CURVE_SPEED_HPP
#define FRENET_CURVE_SPEED_HPP

#include "frenet_planner/frenet/ref_line.hpp"
#include "frenet_planner/global/data_logger.hpp"

// frenet_planner.cpp의 익명 namespace에 있던 곡률 기반 목표속도 계산을 분리한
// 것이다 (fix/yuntae-frenet-speed-continuity Phase 1) - longitudinal_kinematics_test류
// 회귀 테스트가 실제 waypoint 파일의 smoothing/resampling을 거치지 않고, 정확히
// 통제된 RefLine으로 이 로직을 직접 검증할 수 있어야 하기 때문이다. 로직 자체는
// 옮기기 전과 동일하며 이 분리 작업에서 수식을 바꾸지 않는다.

// 횡가속도 한계에서 곡률별 연속 목표속도를 계산한다.
double VelocityFromCurvature(double k, const CurveSpeedConfig& cfg, double max_lateral_accel);

// 현재 속도의 제동거리와 기본 lookahead 중 큰 범위를 훑되, 그 범위 안의
// "가장 급한 커브 하나"의 목표속도를 바로 지금 요구하지 않는다. 각 지점마다
// "그 지점의 커브 안전속도까지, 지금부터 그 지점까지의 거리 동안
// comfortable_decel로 감속하면 도달 가능한 현재 속도"를 계산하고, 그 중
// 최솟값을 사용한다.
double LookaheadTargetSpeed(const RefLine& ref, double s_start, double ego_speed,
                             const CurveSpeedConfig& cfg, double max_longitudinal_accel,
                             double max_lateral_accel, double max_curvature);

#endif
