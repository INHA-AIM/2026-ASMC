#pragma once

#include "Frenet/Math/Polynomial.hpp"
#include "Frenet/Evaluation/KinematicFilter.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

struct StoppingLateralGeometry {
    QuinticPolynomial polynomial;
    double distance;
    double target_d;
};

// Shared geometry only: callers retain their stop frontier, phase and safety
// checks. At most two terminal offsets and two alignment distances are tried.
inline std::vector<StoppingLateralGeometry> MakeStoppingLateralGeometries(
    double d, double slope, double second, double target_d, double span,
    double speed, const KinematicLimits& limits) {
    std::vector<StoppingLateralGeometry> result;
    if (!std::isfinite(span) || span <= 1e-3 || !std::isfinite(d) ||
        !std::isfinite(slope) || !std::isfinite(second)) return result;
    const std::array<double,2> targets{target_d, d};
    for (std::size_t target_index=0; target_index<targets.size(); ++target_index) {
        const double target = targets[target_index];
        if (target_index && std::fabs(target-targets[0]) < 1e-6) continue;
        const double usable_curvature = std::max(1e-3, std::min(
            limits.max_curvature, limits.max_lateral_accel /
            std::max(1.0, speed * speed)));
        const double alignment = std::min(span, std::max({1.0,
            2.0 * std::sqrt(std::fabs(d-target) / usable_curvature),
            2.0 * std::fabs(slope) / usable_curvature}));
        const std::array<double,2> distances{span, alignment};
        for (std::size_t attempt=0; attempt<distances.size(); ++attempt) {
            if (attempt && alignment == span) continue;
            const double distance = distances[attempt];
            result.push_back({MakeQuintic(d, slope, second, target, 0, 0, distance),
                              distance, target});
        }
    }
    return result;
}

inline bool StopLateralHoldAllowed(double speed, double d,
                                   double hold_speed, double tolerance) {
    return std::fabs(speed) <= hold_speed && std::fabs(d) <= tolerance;
}
