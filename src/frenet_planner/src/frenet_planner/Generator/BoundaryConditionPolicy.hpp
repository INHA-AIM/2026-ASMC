#ifndef FRENET_GENERATOR_BOUNDARY_CONDITION_POLICY_HPP
#define FRENET_GENERATOR_BOUNDARY_CONDITION_POLICY_HPP

#include "frenet_planner/Common/FrenetTypes.hpp"
#include "frenet_planner/Evaluation/KinematicFilter.hpp"

// R7-C: "지금 센서/ego 상태에서 얻은 Frenet state 중, trajectory polynomial의
// 초기조건으로 어떤 값을 사용할 것인가"만 담당한다. frenet_planner.cpp의
// PlanInternal에 있던 물리적 clamp(투영 오차로 인한 비현실적 초기 가속도/
// 횡속도 제한)를 그대로 옮긴 것이며, 이 분리 작업에서 수식/파라미터 의미를
// 바꾸지 않는다.
//
// raw(클램프 전) 값은 진단/피드백용(last_d_dot_ 등)으로 호출부가 별도로
// 보존한다 - 이 함수는 clamp *이후* planning에 쓸 초기조건만 계산한다.

// s_ddot: 투영 오차가 초기 종방향 가속도 경계조건을 비현실적으로 키우지
//         않도록 max_longitudinal_accel * boundary_condition_accel_fraction로
//         제한한다.
// d_ddot: 같은 이유로 max_lateral_accel * boundary_condition_accel_fraction로
//         제한한다.
// d_dot:  큰 heading 오차로 과대 추정될 수 있는 초기 횡속도를
//         max_initial_d_dot으로 제한한다.
FrenetState ClampInitialBoundaryState(double s, double s_dot, double raw_s_ddot,
                                       double d, double raw_d_dot,
                                       double raw_d_ddot,
                                       const KinematicLimits& limits);

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

#endif  // FRENET_GENERATOR_BOUNDARY_CONDITION_POLICY_HPP
