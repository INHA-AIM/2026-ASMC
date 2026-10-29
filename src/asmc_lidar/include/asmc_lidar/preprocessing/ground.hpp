#pragma once
// RANSAC ground removal and per-object ground evidence.
// Ported from Main Ransac.cpp (common-geometry branch) and GroundEvidence.hpp at 8f93c22e.

#include <asmc_lidar/core/types.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace asmc {
namespace lidar {
namespace pipeline {

// Main ~ground_* private parameters (main.launch values = code defaults).
struct GroundConfig {
  double near_threshold_m = 0.30;
  double far_threshold_m = 0.40;
  double far_range_m = 50.0;
  double maximum_tilt_deg = 25.0;
  double seed_min_z_m = -2.0;
  double seed_max_z_m = -0.65;
  int maximum_iterations = 80;
  int minimum_seed_points = 30;
  // GPS-independent signed half-space removal. False preserves legacy replay behavior.
  bool remove_below_plane = false;
  // 0 preserves the deployed far-band fit. LOCAL-2C audit separates fitting
  // consensus from removal tolerance; no length/age-based object rejection.
  double fit_threshold_m = 0.;
};

// Main NormalVectorFiltering() common branch. The constructor applies Main's
// ConfigureOnce clamps. A failed fit keeps every finite point and leaves the
// plane invalid (no previous-plane reuse: that is legacy-only in Main).
class GroundFilter {
 public:
  explicit GroundFilter(const GroundConfig& config = {});
  // Clamped values; maximum_tilt_deg keeps the requested value (Main clamps only
  // the radian copy, see maximum_tilt_rad()).
  const GroundConfig& config() const { return config_; }
  // max(1, min(45, maximum_tilt_deg)) * pi / 180, the RANSAC eps angle and tilt gate.
  double maximum_tilt_rad() const { return maximum_tilt_rad_; }

  // input: leveled cloud. output: non-ground points in input order.
  // kept_indices (optional): input index of every output point (cleared first).
  // Fewer than 3 input points: output cleared (points lost), plane invalid.
  void Filter(const CloudPtr& input, const CloudPtr& output, std::vector<int>* kept_indices);

  // Plane used for the last Filter() call (valid=false when the fit failed).
  const GroundPlaneSnapshot& CurrentPlane() const { return current_plane_; }

 private:
  GroundConfig config_;
  double maximum_tilt_rad_ = 0.;
  GroundPlaneSnapshot current_plane_;
};

// Main MeasureObjectGroundEvidence (header-only in Main, copied verbatim).
template <class Points>
ObjectGroundEvidence MeasureObjectGroundEvidence(const Points& points,
                                                 const GroundPlaneSnapshot& plane) {
  ObjectGroundEvidence result;
  if (!plane.valid) return result;
  for (double coefficient : plane.coefficients) {
    if (!std::isfinite(coefficient)) return result;
  }
  const auto& p = plane.coefficients;
  const double norm = std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
  if (!(norm > 1e-6) || p[2] <= 0.0 || !std::isfinite(plane.near_threshold_m) ||
      !std::isfinite(plane.far_threshold_m) || !std::isfinite(plane.far_range_m) ||
      plane.far_range_m <= 0.0 || plane.near_threshold_m < 0.0 ||
      plane.far_threshold_m < plane.near_threshold_m)
    return result;
  std::vector<double> clearances;
  std::vector<double> bands;
  clearances.reserve(points.size());
  bands.reserve(points.size());
  for (const auto& point : points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
    // Signed distance distinguishes elevated returns from below-plane noise.
    clearances.push_back((p[0] * point.x + p[1] * point.y + p[2] * point.z + p[3]) / norm);
    const double ratio = std::min(1.0, std::hypot(point.x, point.y) / plane.far_range_m);
    bands.push_back(plane.near_threshold_m +
                    ratio * (plane.far_threshold_m - plane.near_threshold_m));
  }
  result.sample_count = static_cast<int>(clearances.size());
  // A single stray high return must not rescue a flat ground fragment.
  if (clearances.size() < 3) return result;
  const auto median = [](std::vector<double>& values) {
    std::sort(values.begin(), values.end());
    const std::size_t middle = values.size() / 2;
    return values.size() % 2 ? values[middle] : 0.5 * (values[middle - 1] + values[middle]);
  };
  result.median_clearance_m = median(clearances);
  result.rejection_band_m = median(bands);
  result.valid = true;
  return result;
}

}  // namespace pipeline
}  // namespace lidar
}  // namespace asmc
