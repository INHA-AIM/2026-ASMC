#include "frenet_planner/Pipeline/TrajectoryPipeline.hpp"

#include <cmath>

#include "frenet_planner/Evaluation/KinematicFilter.hpp"
#include "frenet_planner/Generator/LateralGenerator.hpp"
#include "frenet_planner/Generator/LongitudinalGenerator.hpp"

TrajectoryPipeline::TrajectoryPipeline(const PlannerConfigBundle& config,
                                       const RefLine& ref)
    : config_(config), ref_(ref) {}

void TrajectoryPipeline::AppendKeepContinuityCandidates(
    const FrenetState& maneuver_start, const PlannerCommand& cmd,
    const std::vector<TrackedObject>& planning_collision_obstacles,
    std::vector<FrenetPath>& candidates) const {
    std::vector<FrenetPath> continuity_lateral =
        GenerateLateralContinuityCandidates(maneuver_start, config_.path,
                                            config_.limits.low_speed_fallback_v);
    FilterLateralByAcceleration(continuity_lateral, config_.limits);

    std::vector<FrenetPath> longitudinal_for_continuity =
        GenerateVelocityKeepingCandidates(maneuver_start, cmd.target_speed,
                                          config_.path);
    FilterLongitudinalByKinematicLimits(longitudinal_for_continuity, config_.limits);

    std::vector<FrenetPath> continuity_combined = CombineLateralLongitudinal(
        continuity_lateral, longitudinal_for_continuity);
    FilterByCurvature(continuity_combined, ref_, config_.limits);
    if (!planning_collision_obstacles.empty()) {
        FilterByCollision(continuity_combined, ref_,
                          planning_collision_obstacles, config_.vehicle_shape,
                          config_.collision, /*coast_exempt_id=*/-1);
    }
    candidates.insert(candidates.end(),
                      std::make_move_iterator(continuity_combined.begin()),
                      std::make_move_iterator(continuity_combined.end()));
}

std::vector<FrenetPath> TrajectoryPipeline::GenerateCandidates(
    const FrenetState& maneuver_start, const PlannerCommand& cmd,
    const KinematicLimits& maneuver_limits,
    const std::vector<TrackedObject>& planning_collision_obstacles,
    bool append_keep_continuity_candidates,
    PlannerDebugStats* stats) const {
    std::vector<FrenetPath> candidates = ResolveManeuver(
        maneuver_start, cmd, ref_, config_.path, maneuver_limits,
        config_.lane_width, planning_collision_obstacles,
        config_.vehicle_shape, config_.collision, config_.stop, config_.merge,
        stats);

    // moderate-speed curvature starvation 완화(Phase B): 실측 MORAI에서
    // 급커브 근접 중 유효한 결합 후보가 있는데도 전부 중앙(d1=0) 복귀였던
    // 탓에 사후 곡률 필터에서 통째로 탈락하는 사례가 있었다. 현재 오프셋을
    // 유지하는 continuity candidate를 추가한다 - d=0 강제 클램프가 아니라
    // 대안 후보 하나를 더 얹는 것뿐이다(호출자가 lane 안인지 이미 확인).
    if (append_keep_continuity_candidates) {
        AppendKeepContinuityCandidates(maneuver_start, cmd,
                                       planning_collision_obstacles,
                                       candidates);
    }
    return candidates;
}
