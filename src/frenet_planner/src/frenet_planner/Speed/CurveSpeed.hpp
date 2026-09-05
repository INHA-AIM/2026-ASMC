#ifndef FRENET_CURVE_SPEED_HPP
#define FRENET_CURVE_SPEED_HPP

#include "frenet_planner/Reference/Reference.hpp"
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

// Cycle-to-cycle target speed recovery continuity (fix/yuntae-frenet-speed-continuity
// Phase 2). 안전 감속 방향(raw_target < last_target)은 curve braking envelope
// (LookaheadTargetSpeed)가 이미 처리하므로 여기서는 즉시 허용한다. 반대로
// target이 이전 cycle보다 올라가는 방향만
// max_target_speed_accel * replanning_period_s 만큼으로 제한해, 커브를
// 벗어난 직후 5.x -> 10 같은 한 cycle급 점프를 막는다.
//
// has_history가 false(첫 cycle, 또는 EMERGENCY 직후 리셋 이후 첫 정상
// cycle)이면 raw_target을 그대로 반환한다 - 그렇지 않으면 last_target의
// 초기값(0.0)과 무관한 실제 ego 속도가 있어도 0 근처로 묶인다.
double ApplyTargetSpeedRateLimit(double raw_target, double last_target,
                                  bool has_history,
                                  double max_target_speed_accel,
                                  double replanning_period_s);

// external KEEP/LANE_KEEPING의 velocity-keeping 후보 생성에 쓸 "계획용" 초기
// 종방향 가속도를 결정한다 (fix/yuntae-frenet-speed-continuity Phase A).
//
// CartesianToFrenet이 넘겨주는 s_ddot은 실측 IMU 가속도의 역산값이라, 직전
// cycle에 강하게 제동했으면 이번 목표속도가 더 높아도 그 음(-) 가속도가 새
// quartic의 초기 곡률(a2=as/2)로 그대로 들어간다. target_speed가 계속
// 올라가는데도 매 cycle 다시 브레이킹 경계조건을 강제해 v_ref가 조금씩 계속
// 깎이는 것("braking-memory")은 이 boundary가 재주입되기 때문이다.
//
// 완화 조건은 세 가지 모두를 만족할 때만 성립한다:
//   1) 이 후보 생성이 external KEEP/LANE_KEEPING용일 때만 (호출부에서 보장)
//   2) resolved_target_speed가 현재 속도보다 deadband 이상 높을 때만(가속 의도)
//   3) 실측 raw_s_ddot이 음수일 때만(제동 경계조건이 실제로 남아있을 때만)
// 위 조건을 만족하지 않으면(감속 의도, 또는 이미 양의 가속도) raw_s_ddot을
// 그대로 반환한다 - 진짜 감속 요구의 boundary는 절대 건드리지 않는다.
//
// 반환값은 "이 velocity-keeping 후보 생성 1회에 한정된 계획용 경계조건"이며,
// 진단/피드백용 raw_s_ddot 자체를 덮어쓰지 않는다(호출부가 별도 local
// variable로 유지).
double ResolveVelocityKeepingBoundaryAcceleration(double raw_s_ddot,
                                                   double current_s_dot,
                                                   double resolved_target_speed,
                                                   double deadband);

#endif
