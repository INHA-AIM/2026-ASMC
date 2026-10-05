#include "frenet_planner/Speed/TargetSpeedResolver.hpp"

#include <algorithm>

#include "frenet_planner/Speed/CurveSpeed.hpp"

double ResolveTargetSpeed(const RefLine& ref, const TargetSpeedInput& input,
                          const CurveSpeedConfig& curve_cfg,
                          const KinematicLimits& limits) {
    double resolved = LookaheadTargetSpeed(
        ref, input.ego_s, input.ego_speed, curve_cfg,
        limits.max_longitudinal_accel, limits.max_lateral_accel,
        limits.max_curvature);
    if (input.has_external_target_speed) {
        resolved = std::min(resolved, input.external_target_speed);
    }
    if (input.has_external_max_speed) {
        resolved = std::min(resolved, input.external_max_speed);
    }
    return resolved;
}
