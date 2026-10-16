#pragma once

#include <cmath>
#include <algorithm>

#include "Frenet/Common/PlannerCommand.hpp"
#include "Frenet/Config/FrenetConfig.hpp"
#include "Frenet/Evaluation/Cost.hpp"
#include "Frenet/Evaluation/KinematicFilter.hpp"

// Inspect the original external command, never STOP's internal KEEP delegate.
// Missing/low targets, special maneuvers and an explicit lower cap retain the
// base configuration. Route eligibility uses configured command metadata;
// this layer does not reinterpret perception or change the source map.
inline bool UsesKeepCruiseTuning(const PlannerCommand& command,
                                 const KeepCruiseConfig& config) {
    return config.enabled && command.mode == LANE_KEEPING &&
        std::find(config.source_links.begin(), config.source_links.end(),
                  command.source_link_id) != config.source_links.end() &&
        command.merge_type == MergeType::NONE &&
        command.has_target_speed && std::isfinite(command.target_speed) &&
        command.target_speed >= config.min_requested_speed &&
        (!command.has_max_speed ||
         (std::isfinite(command.max_speed) &&
          command.max_speed >= config.min_requested_speed));
}

inline KinematicLimits ResolveKeepCruiseLimits(
    const PlannerCommand& command, const KeepCruiseConfig& config,
    const KinematicLimits& base) {
    KinematicLimits result = base;
    if (UsesKeepCruiseTuning(command, config)) {
        result.max_lateral_accel = config.max_lateral_accel;
    }
    return result;
}

inline CostWeights ResolveKeepCruiseCosts(
    const PlannerCommand& command, const KeepCruiseConfig& config,
    const CostWeights& base) {
    CostWeights result = base;
    if (UsesKeepCruiseTuning(command, config)) {
        result.ks_dot = config.speed_error_weight;
    }
    return result;
}
