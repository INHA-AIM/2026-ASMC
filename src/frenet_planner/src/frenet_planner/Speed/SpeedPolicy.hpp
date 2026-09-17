#ifndef FRENET_SPEED_POLICY_HPP
#define FRENET_SPEED_POLICY_HPP

// R7-B: target-speed policy(rate limit 등)는 Speed/TargetSpeedResolver.*로
// 옮겼다. 이 파일은 initial boundary-condition 관련 함수만 남아 있으며,
// R7-C에서 Generator/BoundaryConditionPolicy로 이동 예정이다(§9: 서로 다른
// responsibility를 같은 commit에서 섞지 않는다).

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
