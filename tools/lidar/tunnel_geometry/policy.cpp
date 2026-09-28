#include "policy.hpp"
#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <pcl/search/kdtree.h>

namespace l1b4 {
using namespace tunnel_filter;
namespace {
double Median(std::vector<double> values) {
    if (values.empty()) return 0;
    std::sort(values.begin(), values.end());
    const auto middle = values.size() / 2;
    return values.size() % 2 ? values[middle] : .5 * (values[middle - 1] + values[middle]);
}
double Ground(const FrameResult& frame, double x, double y) {
    return frame.ground.source == PlaneSource::kUnavailable
        ? frame.ground_reference_height : frame.ground.plane.HeightAt(x, y);
}
bool ValidPoint(const pcl::PointXYZI& p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}
}  // namespace

std::vector<std::uint8_t> PreWallLabels(const FrameResult& reference) {
    auto labels = reference.labels;
    for (auto& value : labels) if (value == kWall) value = kKept;
    return labels;
}

Decision Apply(const FrameResult& frame, const std::vector<std::uint8_t>& camera,
               const tunnel_filter::Params& legacy, const Params& params, bool enabled) {
    if (!frame.leveled || camera.size() != frame.labels.size() ||
        frame.leveled->size() != frame.labels.size()) throw std::runtime_error("invalid frozen frame");
    if (params.band_min <= 0 || params.band_max < params.band_min ||
        params.connection_radius <= 0 || params.expansion_max < params.connection_radius)
        throw std::runtime_error("invalid geometry params");
    Decision result;
    result.labels = PreWallLabels(frame);
    const auto n = result.labels.size();
    result.reasons.assign(n, OutsideProposal); result.geometry_protected.assign(n, 0);
    result.uncertain.assign(n, 0); result.component_id.assign(n, -1);
    if (!enabled) { result.labels = frame.labels; return result; }
    for (const auto& wall : frame.walls) {
        if (!wall.removal_allowed) continue;
        for (auto segment : wall.segments) {
            if (!segment.valid || !segment.plane.Finite()) continue;
            auto& plane = segment.plane;
            const double norm = std::sqrt(plane.a * plane.a + plane.b * plane.b + plane.c * plane.c);
            if (norm < 1e-8 || std::abs(plane.d) < 1e-8) continue;
            const double scale = (plane.d < 0 ? -1. : 1.) / norm;
            plane.a *= scale; plane.b *= scale; plane.c *= scale; plane.d *= scale;
            Model model; model.segment = segment;
            std::vector<double> residuals;
            for (std::size_t i = 0; i < n; ++i) {
                if (result.labels[i] != kKept || camera[i] != kProtectNone) continue;
                const auto& p = frame.leveled->points[i];
                if (!ValidPoint(p) || !segment.InIntervals(segment.AlongWall(p.x, p.y), 0)) continue;
                const double z = p.z - Ground(frame, p.x, p.y);
                const double distance = std::abs(plane.SignedDistance(p.x, p.y, p.z));
                if (distance <= legacy.wall_fit_distance_m && z >= legacy.wall_candidate_min_height_m &&
                    z <= legacy.wall_candidate_max_height_m) residuals.push_back(distance);
            }
            model.residual_samples = residuals.size();
            model.median = Median(residuals);
            for (auto& value : residuals) value = std::abs(value - model.median);
            model.mad = Median(residuals);
            model.band = std::clamp(model.median + params.mad_scale * params.mad_normal_scale * model.mad,
                                    params.band_min, params.band_max);
            result.models.push_back(model);
        }
    }
    std::vector<int> best(n, -1);
    std::vector<double> signed_distance(n, 0);
    std::vector<std::uint8_t> proposal(n, 0);
    auto seeds = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    std::vector<int> seed_original;
    for (std::size_t i = 0; i < n; ++i) {
        if (result.labels[i] != kKept) { result.reasons[i] = Upstream; continue; }
        const auto& p = frame.leveled->points[i];
        if (!ValidPoint(p)) continue;
        double nearest = std::numeric_limits<double>::infinity();
        bool has_supported = false;
        for (std::size_t mi = 0; mi < result.models.size(); ++mi) {
            const auto& model = result.models[mi]; const auto& segment = model.segment;
            const double distance = segment.plane.SignedDistance(p.x, p.y, p.z);
            const double s = segment.AlongWall(p.x, p.y);
            if (p.z - Ground(frame, p.x, p.y) < legacy.wall_remove_min_height_m) continue;
            if (std::abs(distance) <= legacy.wall_remove_distance_m &&
                segment.InIntervals(s, legacy.wall_remove_margin_m)) proposal[i] = 1;
            if (!segment.InIntervals(s, 0) || model.residual_samples == 0) continue;
            if (std::abs(distance) <= legacy.wall_remove_distance_m + params.expansion_max &&
                std::abs(distance) < nearest) {
                nearest = std::abs(distance); best[i] = mi; signed_distance[i] = distance; has_supported = true;
            }
        }
        if (camera[i] != kProtectNone) { result.reasons[i] = CameraProtected; continue; }
        if (!has_supported) {
            if (proposal[i]) { result.uncertain[i] = 1; result.reasons[i] = Unsupported; }
            continue;
        }
        const auto& model = result.models[best[i]];
        if (signed_distance[i] > model.band) {
            seeds->push_back(p); seed_original.push_back(i); result.reasons[i] = Foreground;
            result.geometry_protected[i] = 1;
        } else if (proposal[i] && signed_distance[i] < -model.band) {
            result.reasons[i] = BehindWall; result.uncertain[i] = 1;
        } else if (proposal[i]) {
            result.labels[i] = kWall; result.reasons[i] = WallConforming;
        }
    }
    if (seeds->empty()) return result;
    pcl::search::KdTree<pcl::PointXYZI> tree;
    tree.setInputCloud(seeds);
    std::vector<int> ids; std::vector<float> distances;
    std::vector<int> component(seeds->size(), -1);
    for (std::size_t start = 0; start < seeds->size(); ++start) {
        if (component[start] != -1) continue;
        const int ci = result.components++;
        std::deque<int> queue{static_cast<int>(start)}; component[start] = ci;
        while (!queue.empty()) {
            const int index = queue.front(); queue.pop_front();
            tree.radiusSearch(seeds->points[index], params.connection_radius + 1e-6, ids, distances);
            std::sort(ids.begin(), ids.end());
            for (int neighbor : ids) if (component[neighbor] < 0) {
                component[neighbor] = ci; queue.push_back(neighbor);
            }
        }
    }
    for (std::size_t i = 0; i < seed_original.size(); ++i)
        result.component_id[seed_original[i]] = component[i];
    // One hop from original foreground seeds only. Never flood through wall skin.
    for (std::size_t i = 0; i < n; ++i) {
        if (best[i] < 0 || !proposal[i] || result.labels[i] != kWall || camera[i] != kProtectNone) continue;
        const auto& model = result.models[best[i]];
        if (signed_distance[i] < .5 * model.band) continue;
        tree.radiusSearch(frame.leveled->points[i], params.expansion_max + 1e-6, ids, distances);
        if (ids.empty()) continue;
        auto chosen = std::min_element(ids.begin(), ids.end(), [&](int a, int b) {
            const Eigen::Vector3f da = seeds->points[a].getVector3fMap() - frame.leveled->points[i].getVector3fMap();
            const Eigen::Vector3f db = seeds->points[b].getVector3fMap() - frame.leveled->points[i].getVector3fMap();
            return da.squaredNorm() == db.squaredNorm() ? seed_original[a] < seed_original[b] : da.squaredNorm() < db.squaredNorm();
        });
        result.labels[i] = kKept; result.geometry_protected[i] = 1;
        result.reasons[i] = LocalExpansion; result.component_id[i] = component[*chosen];
    }
    return result;
}
}  // namespace l1b4
