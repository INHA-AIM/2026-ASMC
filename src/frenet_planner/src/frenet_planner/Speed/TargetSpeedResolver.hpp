#ifndef FRENET_SPEED_TARGET_SPEED_RESOLVER_HPP
#define FRENET_SPEED_TARGET_SPEED_RESOLVER_HPP

#include "frenet_planner/Config/FrenetConfig.hpp"
#include "frenet_planner/Evaluation/KinematicFilter.hpp"
#include "frenet_planner/Reference/Reference.hpp"

// 이번 planning cycle의 raw/base speed cap만 계산한다. SpeedProfiler는
// 상승 방향 scalar recovery만 소유하고, 실제 v/a/jerk 연속성은 종방향
// trajectory가 소유한다. 따라서 이 함수는 과거 target이나 replanning
// 주기를 입력받지 않으며, 두 단계 smoothing을 만들지 않는다.
//
// BehaviorState, PlannerCommand 전체 객체, ROS message, StopContext,
// FollowingContext, AvoidContext는 여기 들어오지 않는다 - 호출부(FrenetPlanner)가
// 필요한 scalar/flag만 해석해서 넘긴다. state가 없는 pure cap resolver다.
struct TargetSpeedInput {
    double ego_s = 0.0;
    double ego_speed = 0.0;

    bool has_external_target_speed = false;
    double external_target_speed = 0.0;

    bool has_external_max_speed = false;
    double external_max_speed = 0.0;

};

// 순서 계약: curve envelope(LookaheadTargetSpeed) -> external target cap ->
// external max cap. 반환값은 applied target이 아니라 raw speed cap이다.
double ResolveTargetSpeed(const RefLine& ref, const TargetSpeedInput& input,
                          const CurveSpeedConfig& curve_cfg,
                          const KinematicLimits& limits);

#endif  // FRENET_SPEED_TARGET_SPEED_RESOLVER_HPP
