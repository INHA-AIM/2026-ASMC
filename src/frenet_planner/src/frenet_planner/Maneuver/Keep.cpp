/**
 * @file Keep.cpp
 * @brief external KEEP(LANE_KEEPING) 전용 candidate 보강.
 *
 * PlanInternal의 공용 candidate 파이프라인(ResolveManeuver/EvaluateCosts)은
 * KEEP/STOP-delegated/MERGE가 함께 거치므로 이 파일로 옮기지 않는다. 이
 * 파일은 그 공용 경로 위에 external KEEP에서만 추가되는 continuity
 * candidate 보강 하나만 담당한다 - moderate-speed curvature starvation
 * 완화(fix/yuntae-frenet-speed-continuity Phase B).
 */

#include "frenet_planner/frenet_planner.hpp"

#include <cmath>
#include <cstdio>

#include "frenet_planner/Generator/LateralGenerator.hpp"
#include "frenet_planner/Generator/LongitudinalGenerator.hpp"

void FrenetPlanner::AppendExternalKeepContinuityCandidates(
    const FrenetState& maneuver_start, const PlannerCommand& cmd,
    const std::vector<TrackedObstacle>& planning_collision_obstacles,
    std::vector<FrenetPath>& candidates) const {
    std::vector<FrenetPath> continuity_lateral =
        GenerateLateralContinuityCandidates(maneuver_start, path_cfg_);
    FilterLateralByAcceleration(continuity_lateral, limits_);

    std::vector<FrenetPath> longitudinal_for_continuity =
        GenerateVelocityKeepingCandidates(maneuver_start, cmd.target_speed,
                                          path_cfg_);
    FilterLongitudinalByKinematicLimits(longitudinal_for_continuity, limits_);

    std::vector<FrenetPath> continuity_combined = CombineLateralLongitudinal(
        continuity_lateral, longitudinal_for_continuity);
    FilterByCurvature(continuity_combined, ref_, limits_);
    if (!planning_collision_obstacles.empty()) {
        FilterByCollision(continuity_combined, ref_,
                          planning_collision_obstacles, vehicle_shape_,
                          collision_cfg_, /*coast_exempt_id=*/-1);
    }
    candidates.insert(candidates.end(),
                      std::make_move_iterator(continuity_combined.begin()),
                      std::make_move_iterator(continuity_combined.end()));
}

bool FrenetPlanner::PlanExternalKeepLowSpeedFallback(const CartesianState& ego,
                                                      double s, double d,
                                                      double resolved_target_speed,
                                                      CartesianPath& out_path) {
    const bool fallback_ok = PlanLowSpeedFallback(
        ego, s, d, /*target_d=*/0.0, out_path, /*duration_s=*/2.0,
        resolved_target_speed);
    std::printf("[FrenetPlanner-KEEP] near-low-speed fail-safe: no "
                "candidate (v=%.2f), emitting crawl fallback path\n",
                ego.v);
    last_failure_reason_ = PlanFailureReason::NONE;
    return fallback_ok;
}
