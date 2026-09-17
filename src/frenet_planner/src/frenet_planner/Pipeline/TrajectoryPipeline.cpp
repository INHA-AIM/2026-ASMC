#include "frenet_planner/Pipeline/TrajectoryPipeline.hpp"

#include <cmath>

#include "frenet_planner/Evaluation/KinematicFilter.hpp"
#include "frenet_planner/Generator/LateralGenerator.hpp"
#include "frenet_planner/Generator/LongitudinalGenerator.hpp"

namespace {

// TrajectoryRequest -> ResolveManeuver가 이해하는 PlannerCommand scratch 값
// 변환. ResolveManeuver(CandidateCombiner.cpp)의 switch(cmd.mode)는 R5에서
// switch(request.longitudinal_mode)로 교체될 때까지 이 변환을 거친다 -
// 이 함수 밖에서는 어떤 코드도 이 PlannerCommand를 다시 들여다보지 않는다
// (target_object_id/merge_*/avoidance_d_offset 등 KEEP/FOLLOW/STOP과 무관한
// 필드는 채우지 않는다 - aggregate 기본값 그대로 둔다).
PlannerCommand MakeResolveManeuverCommand(const TrajectoryRequest& request) {
    PlannerCommand cmd{};
    cmd.target_speed = request.target_speed;
    // ResolveManeuver의 lateral 목표/tolerance 계산은 cmd.mode와
    // cmd.has_stop_lateral_hold를 함께 본다(CandidateCombiner.cpp 참고):
    //   - has_stop_lateral_hold=true  -> target_d 고정, tolerance=0
    //   - mode==LANE_KEEPING/FOLLOWING (hold 없음) -> ResolveLateralOffset의
    //     default: 0.0, FOLLOWING만 tolerance=cfg.lateral_target_tolerance
    //     (KEEP은 tolerance=0 - CandidateCombiner의 별도 LANE_KEEPING 분기)
    // request.lateral.search_tolerance != 0(=STOP lateral hold가 아닌
    // 넓은 탐색이 필요한 경우)은 R4 범위의 KEEP/FOLLOW/STOP 어디에서도
    // 아직 쓰이지 않으므로, 0이 아닌 target_d는 곧 "STOP lateral hold"로
    // 해석한다 - AVOID/LANE_CHANGE가 이 오버로드를 쓰게 되면(R5+) 이 가정을
    // 다시 검토해야 한다.
    const bool has_lateral_hold = request.lateral.target_d != 0.0;
    cmd.has_stop_lateral_hold = has_lateral_hold;
    cmd.stop_lateral_hold_d = request.lateral.target_d;
    switch (request.longitudinal_mode) {
        case LongitudinalMode::FOLLOWING:
            cmd.mode = FOLLOWING;
            cmd.target_object_id = request.following.leader_object_id;
            cmd.leader_s = request.following.leader_s;
            cmd.leader_speed = request.following.leader_speed;
            cmd.leader_accel = request.following.leader_accel;
            cmd.time_gap = request.following.time_gap;
            cmd.min_gap = request.following.min_gap;
            cmd.gap_gain = request.following.gap_gain;
            break;
        case LongitudinalMode::STOPPING:
            cmd.mode = STOP;
            cmd.stop_position = request.stop.stop_s;
            break;
        case LongitudinalMode::VELOCITY_KEEPING:
        default:
            cmd.mode = LANE_KEEPING;
            break;
    }
    return cmd;
}

}  // namespace

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

std::vector<FrenetPath> TrajectoryPipeline::GenerateCandidates(
    const FrenetState& maneuver_start, const TrajectoryRequest& request,
    const KinematicLimits& maneuver_limits,
    const std::vector<TrackedObject>& planning_collision_obstacles,
    bool append_keep_continuity_candidates,
    PlannerDebugStats* stats) const {
    const PlannerCommand cmd = MakeResolveManeuverCommand(request);
    return GenerateCandidates(maneuver_start, cmd, maneuver_limits,
                              planning_collision_obstacles,
                              append_keep_continuity_candidates, stats);
}
