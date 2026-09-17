#ifndef FRENET_PIPELINE_TRAJECTORY_PIPELINE_HPP
#define FRENET_PIPELINE_TRAJECTORY_PIPELINE_HPP

#include <vector>

#include "frenet_planner/Common/BehaviorTypes.hpp"
#include "frenet_planner/Common/FrenetTypes.hpp"
#include "frenet_planner/Common/ObjectTypes.hpp"
#include "frenet_planner/Common/PlannerCommand.hpp"
#include "frenet_planner/Generator/CandidateCombiner.hpp"
#include "frenet_planner/Pipeline/PlannerConfigBundle.hpp"
#include "frenet_planner/Pipeline/TrajectoryRequest.hpp"
#include "frenet_planner/Reference/Reference.hpp"

// KEEP/FOLLOW/STOP이 공유하는 candidate 파이프라인 중, FrenetPlanner의
// mutable 상태(stop_/v2_following_/last_*/prev_mode_/debug_writer_)에
// 전혀 의존하지 않는 부분만 뽑아낸 컴포넌트(R3 2/4단계).
//
// ExecuteManeuverPipeline은 여전히 FrenetPlanner 멤버 함수로 남아 STOP
// lifecycle(stop_)과 진단 상태(last_*/prev_mode_/debug_writer_) 갱신을
// 담당하고, 이 클래스는 그 안에서 "생성 -> 곡률/충돌 필터 -> continuity
// candidate 추가"만 담당한다 - 이 구간은 config/RefLine/입력 인자만으로
// 완전히 자급자족하므로 순수 이동이 안전하다.
class TrajectoryPipeline {
public:
    TrajectoryPipeline(const PlannerConfigBundle& config, const RefLine& ref);

    // legacy behavior-aware path: ResolveManeuver(PlannerCommand, switch(cmd.mode))
    // -> (external KEEP && lane 안일 때) continuity candidate 추가까지.
    // AVOID/LANE_CHANGE/MERGE 전용(R5 범위 밖) - KEEP/FOLLOW/STOP은 더 이상
    // 이 오버로드를 거치지 않는다. maneuver_limits는 STOP 서비스 감속 허용 등
    // 호출자가 이미 조정한 값을 그대로 받는다(이 클래스는 limits를 변형하지
    // 않는다).
    std::vector<FrenetPath> GenerateCandidates(
        const FrenetState& maneuver_start, const PlannerCommand& cmd,
        const KinematicLimits& maneuver_limits,
        const std::vector<TrackedObject>& planning_collision_obstacles,
        bool append_keep_continuity_candidates,
        PlannerDebugStats* stats) const;

    // request-native path(R5): KEEP/FOLLOW/STOP 전용. GenerateRequestedCandidates
    // (switch(request.longitudinal_mode), BehaviorState/PlannerCommand 없음)를
    // 직접 호출한다 - 위 PlannerCommand 오버로드로 역변환하지 않는다(두
    // 오버로드는 서로를 호출하지 않는다).
    std::vector<FrenetPath> GenerateCandidates(
        const FrenetState& maneuver_start, const TrajectoryRequest& request,
        const KinematicLimits& maneuver_limits,
        const std::vector<TrackedObject>& planning_collision_obstacles,
        bool append_keep_continuity_candidates,
        PlannerDebugStats* stats) const;

private:
    // moderate-speed curvature starvation 완화(Phase B) - Keep.cpp의
    // AppendExternalKeepContinuityCandidates를 그대로 옮긴 것. 현재
    // 오프셋을 유지하는 continuity candidate를 candidates에 추가로
    // 얹는다(대체가 아니라 추가). target_speed만 필요로 하므로(PlannerCommand/
    // TrajectoryRequest 어느 쪽도 아닌 스칼라) 두 GenerateCandidates
    // 오버로드가 이 헬퍼를 공유해도 서로를 호출하지 않는다(§10/§11).
    void AppendKeepContinuityCandidates(
        const FrenetState& maneuver_start, double target_speed,
        const std::vector<TrackedObject>& planning_collision_obstacles,
        std::vector<FrenetPath>& candidates) const;

    PlannerConfigBundle config_;
    const RefLine& ref_;
};

#endif
