#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

// Coordinates are the pitch-corrected LiDAR frame used by RANSAC and tracking.
// This is observation evidence, not a replacement for the authoritative OBB.
struct GroundPlaneSnapshot {
    bool valid = false;
    std::array<double, 4> coefficients{{0.0, 0.0, 1.0, 0.0}};
    double near_threshold_m = 0.30;
    double far_threshold_m = 0.40;
    double far_range_m = 50.0;
};

struct ObjectGroundEvidence {
    bool valid = false;
    int sample_count = 0;
    double median_clearance_m = 0.0;
    double rejection_band_m = 0.0;
};

template <class Points>
ObjectGroundEvidence MeasureObjectGroundEvidence(
    const Points& points, const GroundPlaneSnapshot& plane) {
    ObjectGroundEvidence result;
    if (!plane.valid) return result;
    for (double coefficient : plane.coefficients) {
        if (!std::isfinite(coefficient)) return result;
    }
    const auto& p = plane.coefficients;
    const double norm = std::sqrt(p[0]*p[0] + p[1]*p[1] + p[2]*p[2]);
    if (!(norm > 1e-6) || p[2] <= 0.0 ||
        !std::isfinite(plane.near_threshold_m) ||
        !std::isfinite(plane.far_threshold_m) ||
        !std::isfinite(plane.far_range_m) || plane.far_range_m <= 0.0 ||
        plane.near_threshold_m < 0.0 ||
        plane.far_threshold_m < plane.near_threshold_m) return result;
    std::vector<double> clearances;
    std::vector<double> bands;
    clearances.reserve(points.size());
    bands.reserve(points.size());
    for (const auto& point : points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
            !std::isfinite(point.z)) continue;
        // Signed distance distinguishes elevated returns from below-plane noise.
        clearances.push_back((p[0]*point.x + p[1]*point.y +
                              p[2]*point.z + p[3]) / norm);
        const double ratio = std::min(1.0, std::hypot(point.x, point.y) /
                                             plane.far_range_m);
        bands.push_back(plane.near_threshold_m + ratio *
                         (plane.far_threshold_m - plane.near_threshold_m));
    }
    result.sample_count = static_cast<int>(clearances.size());
    // A single stray high return must not rescue a flat ground fragment.
    if (clearances.size() < 3) return result;
    const auto median = [](std::vector<double>& values) {
        std::sort(values.begin(), values.end());
        const std::size_t middle = values.size() / 2;
        return values.size() % 2 ? values[middle] :
               0.5 * (values[middle - 1] + values[middle]);
    };
    result.median_clearance_m = median(clearances);
    result.rejection_band_m = median(bands);
    result.valid = true;
    return result;
}
