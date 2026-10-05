#ifndef FRENET_PIPELINE_TRAJECTORY_PIPELINE_HPP
#define FRENET_PIPELINE_TRAJECTORY_PIPELINE_HPP

#include <vector>

#include "frenet_planner/Common/FrenetTypes.hpp"
#include "frenet_planner/Common/ObjectTypes.hpp"
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
//
// R6-E: PlannerCommand를 받는 legacy 오버로드는 제거했다 - production에서
// 이 클래스를 호출하는 유일한 경로(ExecuteManeuverPipeline, Keep.cpp)가
// 이제 항상 TrajectoryRequest를 만들어 아래 오버로드만 부른다(frenet_planner.cpp
// PlanInternal switch가 AVOID/LANE_CHANGE를 먼저 가로채고 MERGE는
// IsSupportedExternalMode에서 거부되므로, 이 클래스에는 애초에 그 세 모드가
// PlannerCommand로 도달한 적이 없었다). AVOID/LANE_CHANGE는 각자
// GenerateRequestedCandidates를 직접 호출하고(Avoid.cpp/LaneChange.cpp),
// MERGE는 Generator/CandidateCombiner.cpp의 ResolveManeuver(레거시,
// PlannerCommand 그대로 유지) 안에만 남아 있다 - 이 클래스와는 무관하다.
class TrajectoryPipeline {
public:
    TrajectoryPipeline(const PlannerConfigBundle& config, const RefLine& ref);

    // request-native path(R5/R6): KEEP/FOLLOW/STOP 전용. GenerateRequestedCandidates
    // (switch(request.longitudinal_mode), BehaviorState/PlannerCommand 없음)를
    // 직접 호출한다. maneuver_limits는 STOP 서비스 감속 허용 등 호출자가 이미
    // 조정한 값을 그대로 받는다(이 클래스는 limits를 변형하지 않는다).
    std::vector<FrenetPath> GenerateCandidates(
        const FrenetState& maneuver_start, const TrajectoryRequest& request,
        const KinematicLimits& maneuver_limits,
        const std::vector<TrackedObject>& planning_collision_obstacles,
        bool append_keep_continuity_candidates,
        PlannerDebugStats* stats,
        CartesianGeometryMode geometry_mode =
            CartesianGeometryMode::HYBRID) const;

private:
    // moderate-speed curvature starvation 완화(Phase B) - Keep.cpp의
    // AppendExternalKeepContinuityCandidates를 그대로 옮긴 것. 현재
    // 오프셋을 유지하는 continuity candidate를 candidates에 추가로
    // 얹는다(대체가 아니라 추가). 종방향 상한은 정상 후보와 동일하다.
    void AppendKeepContinuityCandidates(
        const FrenetState& maneuver_start, const TrajectoryRequest& request,
        const KinematicLimits& maneuver_limits,
        const std::vector<TrackedObject>& planning_collision_obstacles,
        std::vector<FrenetPath>& candidates,
        CartesianGeometryMode geometry_mode) const;

    PlannerConfigBundle config_;
    const RefLine& ref_;
};

#endif
