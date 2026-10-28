// RANSAC ground removal on the leveled cloud.
// Ported from Main src/Lidar/ImuRansac/Ransac.cpp at 8f93c22e, common-geometry branch.
// Main keeps a process-wide GroundFilterState singleton configured lazily from ~ground_*
// parameters; here one GroundFilter instance owns the (clamped) config and the plane.

#include <asmc_lidar/preprocessing/ground.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <utility>

namespace asmc {
namespace lidar {
namespace pipeline {
namespace {

constexpr double kPi = 3.14159265358979323846;

// Ported from Main src/Lidar/ImuRansac/Ransac.cpp:100-116 (8f93c22e)
bool NormalizePlane(const pcl::ModelCoefficients& coefficients, std::array<double, 4>* plane) {
  if (!plane || coefficients.values.size() < 4) return false;
  const double a = coefficients.values[0];
  const double b = coefficients.values[1];
  const double c = coefficients.values[2];
  const double d = coefficients.values[3];
  const double norm = std::sqrt(a * a + b * b + c * c);
  if (!(norm > 1e-6) || !std::isfinite(norm)) return false;
  *plane = {{a / norm, b / norm, c / norm, d / norm}};
  if ((*plane)[2] < 0.0) {
    for (double& value : *plane) value = -value;
  }
  return std::all_of(plane->begin(), plane->end(),
                     [](double value) { return std::isfinite(value); });
}

// Ported from Main src/Lidar/ImuRansac/Ransac.cpp:118-163 (8f93c22e)
bool FitGroundPlane(const Cloud::ConstPtr& input, const GroundConfig& config,
                    double maximum_tilt_rad, std::array<double, 4>* plane) {
  Cloud::Ptr seeds(new Cloud);
  seeds->points.reserve(input->points.size());
  for (const auto& point : input->points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }
    if (point.z >= config.seed_min_z_m && point.z <= config.seed_max_z_m) {
      seeds->points.push_back(point);
    }
  }
  seeds->width = static_cast<std::uint32_t>(seeds->points.size());
  seeds->height = 1;
  seeds->is_dense = false;
  if (static_cast<int>(seeds->points.size()) < config.minimum_seed_points) {
    return false;
  }

  pcl::SACSegmentation<pcl::PointXYZI> segmentation;
  segmentation.setOptimizeCoefficients(true);
  segmentation.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  segmentation.setMethodType(pcl::SAC_RANSAC);
  segmentation.setAxis(Eigen::Vector3f::UnitZ());
  segmentation.setEpsAngle(maximum_tilt_rad);
  segmentation.setMaxIterations(config.maximum_iterations);
  // Main fits with the far threshold over the whole cloud (removal is adaptive).
  segmentation.setDistanceThreshold(config.fit_threshold_m > 0.
      ? config.fit_threshold_m : config.far_threshold_m);
  segmentation.setInputCloud(seeds);

  pcl::PointIndices inliers;
  pcl::ModelCoefficients coefficients;
  segmentation.segment(inliers, coefficients);
  if (static_cast<int>(inliers.indices.size()) < config.minimum_seed_points) {
    return false;
  }
  if (!NormalizePlane(coefficients, plane)) return false;
  const double tilt = std::acos(std::min(1.0, std::abs((*plane)[2])));
  return std::isfinite(tilt) && tilt <= maximum_tilt_rad;
}

// Ported from Main src/Lidar/ImuRansac/Ransac.cpp:165-171 (8f93c22e)
// x/y arrive as float and are widened to double before hypot, as in Main.
double RangeAdaptiveThreshold(double x, double y, const GroundConfig& config) {
  const double ratio = std::min(1.0, std::hypot(x, y) / config.far_range_m);
  return config.near_threshold_m + ratio * (config.far_threshold_m - config.near_threshold_m);
}

// Ported from Main src/Lidar/ImuRansac/Ransac.cpp:173-184 (8f93c22e)
// kept_indices (optional) records the input index of every copied point.
void CopyFinitePoints(const Cloud::ConstPtr& input, const CloudPtr& output,
                      std::vector<int>* kept_indices) {
  output->clear();
  output->points.reserve(input->points.size());
  for (std::size_t i = 0; i < input->points.size(); ++i) {
    const auto& point = input->points[i];
    if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
      output->points.push_back(point);
      if (kept_indices) kept_indices->push_back(static_cast<int>(i));
    }
  }
}

}  // namespace

// Ported from Main src/Lidar/ImuRansac/Ransac.cpp:41-98 (8f93c22e), ConfigureOnce clamps.
// Applied at construction instead of on the first eligible frame (the parameters
// are constant for the process, so the result is the same). Main's ROS_INFO
// (which prints the unclamped tilt) is not reproduced. fallback_frames is legacy-only.
GroundFilter::GroundFilter(const GroundConfig& config) : config_(config) {
  config_.near_threshold_m = std::max(0.02, config_.near_threshold_m);
  config_.far_threshold_m = std::max(config_.near_threshold_m, config_.far_threshold_m);
  config_.far_range_m = std::max(1.0, config_.far_range_m);
  maximum_tilt_rad_ = std::max(1.0, std::min(45.0, config_.maximum_tilt_deg)) * kPi / 180.0;
  if (config_.seed_min_z_m > config_.seed_max_z_m) {
    std::swap(config_.seed_min_z_m, config_.seed_max_z_m);
  }
  config_.maximum_iterations = std::max(20, config_.maximum_iterations);
  config_.minimum_seed_points = std::max(3, config_.minimum_seed_points);
}

// Ported from Main src/Lidar/ImuRansac/Ransac.cpp:192-256 (8f93c22e)
void GroundFilter::Filter(const CloudPtr& input, const CloudPtr& output,
                          std::vector<int>* kept_indices) {
  if (kept_indices) kept_indices->clear();
  // Common geometry cannot justify transporting an old ground plane without the
  // intervening translation and attitude: a failed fit preserves finite points.
  current_plane_ = GroundPlaneSnapshot{};
  if (!input || !output || input->points.size() < 3) {
    // Main clears the output here (the points are lost, not preserved).
    if (output) output->clear();
    return;
  }

  std::array<double, 4> plane;
  const bool current_plane_valid = FitGroundPlane(input, config_, maximum_tilt_rad_, &plane);
  if (!current_plane_valid) {
    // Legacy previous-plane reuse (enable_common_geometry=false) is not ported.
    CopyFinitePoints(input, output, kept_indices);
    output->width = static_cast<std::uint32_t>(output->points.size());
    output->height = 1;
    output->is_dense = false;
    return;
  }

  current_plane_.valid = true;
  current_plane_.coefficients = plane;
  current_plane_.near_threshold_m = config_.near_threshold_m;
  current_plane_.far_threshold_m = config_.far_threshold_m;
  current_plane_.far_range_m = config_.far_range_m;
  output->clear();
  output->points.reserve(input->points.size());
  for (std::size_t i = 0; i < input->points.size(); ++i) {
    const auto& point = input->points[i];
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }
    // The normal points upward. In signed mode only returns above the upper
    // ground band survive; do not widen that band and erase elevated obstacles.
    const double signed_distance =
        plane[0] * point.x + plane[1] * point.y + plane[2] * point.z + plane[3];
    const double distance = config_.remove_below_plane ? signed_distance : std::abs(signed_distance);
    if (distance > RangeAdaptiveThreshold(point.x, point.y, config_)) {
      output->points.push_back(point);
      if (kept_indices) kept_indices->push_back(static_cast<int>(i));
    }
  }
  output->width = static_cast<std::uint32_t>(output->points.size());
  output->height = 1;
  output->is_dense = false;
}

}  // namespace pipeline
}  // namespace lidar
}  // namespace asmc
