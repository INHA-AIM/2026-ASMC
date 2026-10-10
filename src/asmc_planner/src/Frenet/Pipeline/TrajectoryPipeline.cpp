#include "Frenet/Pipeline/TrajectoryPipeline.hpp"

#include <cmath>

#include "Frenet/Evaluation/KinematicFilter.hpp"
#include "Frenet/Generator/LateralGenerator.hpp"
#include "Frenet/Generator/LongitudinalGenerator.hpp"

TrajectoryPipeline::TrajectoryPipeline(const PlannerConfigBundle& config,
                                       const RefLine& ref)
    : config_(config), ref_(ref) {}

void TrajectoryPipeline::AppendKeepContinuityCandidates(
    const FrenetState& maneuver_start, const TrajectoryRequest& request,
    const KinematicLimits& maneuver_limits,
    const std::vector<TrackedObject>& planning_collision_obstacles,
    std::vector<FrenetPath>& candidates,
    CartesianGeometryMode geometry_mode) const {
    std::vector<FrenetPath> continuity_lateral =
        GenerateLateralContinuityCandidates(maneuver_start, config_.path,
                                            config_.limits.low_speed_fallback_v);
    FilterLateralByAcceleration(continuity_lateral, config_.limits);

    std::vector<FrenetPath> longitudinal_for_continuity =
        request.speed_envelope.empty()
            ? GenerateVelocityKeepingCandidates(maneuver_start, request.target_speed,
                                                config_.path)
            : GenerateEnvelopeVelocityCandidates(
                maneuver_start, request.target_speed, request.speed_envelope,
                config_.path, maneuver_limits);
    FilterLongitudinalByKinematicLimits(longitudinal_for_continuity, maneuver_limits);

    std::vector<FrenetPath> continuity_combined = CombineLateralLongitudinal(
        continuity_lateral, longitudinal_for_continuity);
    FilterByCurvature(continuity_combined, ref_, config_.limits, geometry_mode);
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
    const FrenetState& maneuver_start, const TrajectoryRequest& request,
    const KinematicLimits& maneuver_limits,
    const std::vector<TrackedObject>& planning_collision_obstacles,
    bool append_keep_continuity_candidates,
    PlannerDebugStats* stats,
    CartesianGeometryMode geometry_mode) const {
    // Request-native path(R5/R6) - PlannerCommand로 역변환하지 않는다.
    std::vector<FrenetPath> candidates = GenerateRequestedCandidates(
        maneuver_start, request, ref_, config_.path, maneuver_limits,
        planning_collision_obstacles, config_.vehicle_shape, config_.collision,
        stats, geometry_mode);

    if (append_keep_continuity_candidates) {
        AppendKeepContinuityCandidates(maneuver_start, request, maneuver_limits,
                                       planning_collision_obstacles,
                                       candidates, geometry_mode);
    }
    return candidates;
}
