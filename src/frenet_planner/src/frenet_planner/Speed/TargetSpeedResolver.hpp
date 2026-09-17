#ifndef FRENET_SPEED_TARGET_SPEED_RESOLVER_HPP
#define FRENET_SPEED_TARGET_SPEED_RESOLVER_HPP

#include "frenet_planner/Config/FrenetConfig.hpp"
#include "frenet_planner/Evaluation/KinematicFilter.hpp"
#include "frenet_planner/Reference/Reference.hpp"

// R7-B: "이번 planning cycle에서 planner가 사용할 최종 target speed는 얼마인가"
// 만 담당한다. frenet_planner.cpp의 PreparePlanningContext에 있던 조립 순서
// (curve envelope -> external target cap -> external max cap -> rate limit)를
// 그대로 옮긴 것이며, 이 분리 작업에서 순서/수식을 바꾸지 않는다.
//
// BehaviorState, PlannerCommand 전체 객체, ROS message, StopContext,
// FollowingContext, AvoidContext는 여기 들어오지 않는다 - 호출부(FrenetPlanner)가
// 필요한 scalar/flag만 해석해서 넘긴다. history(last_target_speed_/
// has_target_speed_history_)는 계속 FrenetPlanner가 소유하며, 이 함수는 그 값을
// 입력으로만 받는 pure 계산이다(불필요한 stateful service object를 만들지 않는다).
struct TargetSpeedInput {
    double ego_s = 0.0;
    double ego_speed = 0.0;

    bool has_external_target_speed = false;
    double external_target_speed = 0.0;

    bool has_external_max_speed = false;
    double external_max_speed = 0.0;

    bool has_history = false;
    double previous_target_speed = 0.0;
};

// 순서 계약(바꾸지 않음): curve envelope(LookaheadTargetSpeed) -> external
// target cap -> external max cap -> cycle-to-cycle rate limit.
double ResolveTargetSpeed(const RefLine& ref, const TargetSpeedInput& input,
                          const CurveSpeedConfig& curve_cfg,
                          const KinematicLimits& limits,
                          double max_target_speed_accel,
                          double replanning_period_s);

#endif  // FRENET_SPEED_TARGET_SPEED_RESOLVER_HPP
