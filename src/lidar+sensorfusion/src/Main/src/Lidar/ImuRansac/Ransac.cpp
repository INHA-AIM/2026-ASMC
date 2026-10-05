#include "ImuRansac.hpp"

#include <pcl/segmentation/sac_segmentation.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace {

constexpr double kPi = 3.14159265358979323846;

struct GroundFilterConfig {
    double near_threshold_m = 0.30;
    double far_threshold_m = 0.40;
    double far_range_m = 50.0;
    double maximum_tilt_rad = 25.0 * kPi / 180.0;
    double seed_min_z_m = -2.0;
    double seed_max_z_m = -0.65;
    int maximum_iterations = 80;
    int minimum_seed_points = 30;
    int fallback_frames = 2;
};

struct GroundFilterState {
    GroundFilterConfig config;
    bool configured = false;
    bool has_previous_plane = false;
    std::array<double, 4> previous_plane{{0.0, 0.0, 1.0, 0.0}};
    int consecutive_failures = 0;
    GroundPlaneSnapshot current_plane;
};

GroundFilterState& State() {
    static GroundFilterState state;
    return state;
}

void ConfigureOnce() {
    GroundFilterState& state = State();
    if (state.configured) return;

    ros::NodeHandle private_node("~");
    private_node.param(
        "ground_distance_threshold_near_m",
        state.config.near_threshold_m, state.config.near_threshold_m);
    private_node.param(
        "ground_distance_threshold_far_m",
        state.config.far_threshold_m, state.config.far_threshold_m);
    private_node.param(
        "ground_threshold_far_range_m",
        state.config.far_range_m, state.config.far_range_m);
    double maximum_tilt_deg = 25.0;
    private_node.param(
        "ground_max_tilt_deg", maximum_tilt_deg, maximum_tilt_deg);
    private_node.param(
        "ground_seed_min_z_m",
        state.config.seed_min_z_m, state.config.seed_min_z_m);
    private_node.param(
        "ground_seed_max_z_m",
        state.config.seed_max_z_m, state.config.seed_max_z_m);
    private_node.param(
        "ground_ransac_maximum_iterations",
        state.config.maximum_iterations, state.config.maximum_iterations);
    private_node.param(
        "ground_minimum_seed_points",
        state.config.minimum_seed_points, state.config.minimum_seed_points);
    private_node.param(
        "ground_fallback_frames",
        state.config.fallback_frames, state.config.fallback_frames);

    state.config.near_threshold_m = std::max(
        0.02, state.config.near_threshold_m);
    state.config.far_threshold_m = std::max(
        state.config.near_threshold_m, state.config.far_threshold_m);
    state.config.far_range_m = std::max(1.0, state.config.far_range_m);
    state.config.maximum_tilt_rad = std::max(
        1.0, std::min(45.0, maximum_tilt_deg)) * kPi / 180.0;
    if (state.config.seed_min_z_m > state.config.seed_max_z_m) {
        std::swap(state.config.seed_min_z_m, state.config.seed_max_z_m);
    }
    state.config.maximum_iterations = std::max(
        20, state.config.maximum_iterations);
    state.config.minimum_seed_points = std::max(
        3, state.config.minimum_seed_points);
    state.config.fallback_frames = std::max(
        0, state.config.fallback_frames);
    state.configured = true;

    ROS_INFO(
        "Ground filter configured: threshold %.2f->%.2fm by %.1fm, tilt<=%.1fdeg, seed_z=[%.2f,%.2f], fallback=%d frames",
        state.config.near_threshold_m, state.config.far_threshold_m,
        state.config.far_range_m, maximum_tilt_deg,
        state.config.seed_min_z_m, state.config.seed_max_z_m,
        state.config.fallback_frames);
}

bool NormalizePlane(const pcl::ModelCoefficients& coefficients,
                    std::array<double, 4>* plane) {
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
    return std::all_of(
        plane->begin(), plane->end(),
        [](double value) { return std::isfinite(value); });
}

bool FitGroundPlane(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& input,
    std::array<double, 4>* plane) {
    const GroundFilterConfig& config = State().config;
    pcl::PointCloud<pcl::PointXYZI>::Ptr seeds(
        new pcl::PointCloud<pcl::PointXYZI>);
    seeds->points.reserve(input->points.size());
    for (const auto& point : input->points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
            !std::isfinite(point.z)) {
            continue;
        }
        if (point.z >= config.seed_min_z_m &&
            point.z <= config.seed_max_z_m) {
            seeds->points.push_back(point);
        }
    }
    seeds->width = static_cast<std::uint32_t>(seeds->points.size());
    seeds->height = 1;
    seeds->is_dense = false;
    if (static_cast<int>(seeds->points.size()) <
        config.minimum_seed_points) {
        return false;
    }

    pcl::SACSegmentation<pcl::PointXYZI> segmentation;
    segmentation.setOptimizeCoefficients(true);
    segmentation.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setAxis(Eigen::Vector3f::UnitZ());
    segmentation.setEpsAngle(config.maximum_tilt_rad);
    segmentation.setMaxIterations(config.maximum_iterations);
    segmentation.setDistanceThreshold(config.far_threshold_m);
    segmentation.setInputCloud(seeds);

    pcl::PointIndices inliers;
    pcl::ModelCoefficients coefficients;
    segmentation.segment(inliers, coefficients);
    if (static_cast<int>(inliers.indices.size()) <
        config.minimum_seed_points) {
        return false;
    }
    if (!NormalizePlane(coefficients, plane)) return false;
    const double tilt = std::acos(std::min(1.0, std::abs((*plane)[2])));
    return std::isfinite(tilt) && tilt <= config.maximum_tilt_rad;
}

double RangeAdaptiveThreshold(double x, double y) {
    const GroundFilterConfig& config = State().config;
    const double ratio = std::min(
        1.0, std::hypot(x, y) / config.far_range_m);
    return config.near_threshold_m +
        ratio * (config.far_threshold_m - config.near_threshold_m);
}

void CopyFinitePoints(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& input,
    pcl::PointCloud<pcl::PointXYZI>::Ptr output) {
    output->clear();
    output->points.reserve(input->points.size());
    for (const auto& point : input->points) {
        if (std::isfinite(point.x) && std::isfinite(point.y) &&
            std::isfinite(point.z)) {
            output->points.push_back(point);
        }
    }
}

}  // namespace

GroundPlaneSnapshot CurrentGroundPlaneSnapshot() {
    return State().current_plane;
}

void NormalVectorFiltering(
    pcl::PointCloud<pcl::PointXYZI>::Ptr input,
    pcl::PointCloud<pcl::PointXYZI>::Ptr output,
    const GPS_IMU&) {
    // Never attach an arbitrarily old plane to a new observation.
    State().current_plane = GroundPlaneSnapshot{};
    if (!input || !output || input->points.size() < 3) {
        if (output) output->clear();
        return;
    }
    ConfigureOnce();

    GroundFilterState& state = State();
    std::array<double, 4> plane;
    const bool current_plane_valid = FitGroundPlane(input, &plane);
    if (current_plane_valid) {
        state.previous_plane = plane;
        state.has_previous_plane = true;
        state.consecutive_failures = 0;
    } else {
        ++state.consecutive_failures;
        if (state.has_previous_plane &&
            state.consecutive_failures <= state.config.fallback_frames) {
            plane = state.previous_plane;
            ROS_WARN_THROTTLE(
                2.0,
                "Ground fit missed once; reusing the last valid plane (%d/%d)",
                state.consecutive_failures, state.config.fallback_frames);
        } else {
            CopyFinitePoints(input, output);
            output->width = static_cast<std::uint32_t>(output->points.size());
            output->height = 1;
            output->is_dense = false;
            ROS_WARN_THROTTLE(
                2.0,
                "Ground plane unavailable; preserving finite points for fail-safe processing");
            return;
        }
    }

    state.current_plane.valid = true;
    state.current_plane.coefficients = plane;
    state.current_plane.near_threshold_m = state.config.near_threshold_m;
    state.current_plane.far_threshold_m = state.config.far_threshold_m;
    state.current_plane.far_range_m = state.config.far_range_m;
    output->clear();
    output->points.reserve(input->points.size());
    for (const auto& point : input->points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
            !std::isfinite(point.z)) {
            continue;
        }
        const double distance = std::abs(
            plane[0] * point.x + plane[1] * point.y +
            plane[2] * point.z + plane[3]);
        if (distance > RangeAdaptiveThreshold(point.x, point.y)) {
            output->points.push_back(point);
        }
    }
    output->width = static_cast<std::uint32_t>(output->points.size());
    output->height = 1;
    output->is_dense = false;
}
