// Range-adaptive Euclidean clustering and float cluster centroids.
// Ported from Main src/Lidar/Clustering/Clustering.cpp at 8f93c22e (stateless; a new
// KdTree is built for every scan).

#include <asmc_lidar/detection/clustering.hpp>

#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <utility>
#include <vector>

namespace asmc {
namespace lidar {
namespace pipeline {
namespace {

constexpr float kDefaultClusterTolerance = 0.5F;
constexpr int kMaximumClusterPoints = 10000;

// Ported from Main src/Lidar/Clustering/Clustering.cpp:17-19 (8f93c22e)
float PositiveOr(float value, float fallback) {
  return std::isfinite(value) && value > 0.0F ? value : fallback;
}

// Ported from Main src/Lidar/Clustering/Clustering.cpp:21-93 (8f93c22e)
// Breadth-first growth from seeds in index order. An edge needs d^2 <= min(tol(p),
// tol(q))^2, so each cluster is a connected component (independent of BFS order).
// Clusters outside [min_points, 10000] are dropped whole; their points stay processed.
std::vector<pcl::PointIndices> ExtractAdaptiveClusters(
    const CloudPtr& cloud, const pcl::search::KdTree<pcl::PointXYZI>::Ptr& tree,
    const ClusteringConfig& config, int min_points) {
  std::vector<pcl::PointIndices> clusters;
  if (!cloud || cloud->empty()) return clusters;

  std::vector<std::uint8_t> processed(cloud->size(), 0U);
  std::vector<int> queue;
  std::vector<int> neighbors;
  std::vector<float> squared_distances;
  queue.reserve(cloud->size());

  for (std::size_t seed = 0; seed < cloud->size(); ++seed) {
    if (processed[seed] != 0U) continue;
    const auto& seed_point = cloud->points[seed];
    if (!std::isfinite(seed_point.x) || !std::isfinite(seed_point.y) ||
        !std::isfinite(seed_point.z)) {
      processed[seed] = 1U;
      continue;
    }

    queue.clear();
    queue.push_back(static_cast<int>(seed));
    processed[seed] = 1U;

    for (std::size_t cursor = 0; cursor < queue.size(); ++cursor) {
      const int point_index = queue[cursor];
      const auto& point = cloud->points[point_index];
      const float point_range = std::hypot(point.x, point.y);
      const float point_tolerance = AdaptiveClusterTolerance(point_range, config);

      neighbors.clear();
      squared_distances.clear();
      if (tree->radiusSearch(point_index, point_tolerance, neighbors, squared_distances) <= 0) {
        continue;
      }
      for (std::size_t n = 0; n < neighbors.size(); ++n) {
        const int neighbor_index = neighbors[n];
        if (neighbor_index < 0 || static_cast<std::size_t>(neighbor_index) >= cloud->size() ||
            processed[neighbor_index] != 0U) {
          continue;
        }
        const auto& neighbor = cloud->points[neighbor_index];
        const float neighbor_tolerance =
            AdaptiveClusterTolerance(std::hypot(neighbor.x, neighbor.y), config);
        const float symmetric_tolerance = std::min(point_tolerance, neighbor_tolerance);
        if (squared_distances[n] > symmetric_tolerance * symmetric_tolerance) {
          continue;
        }
        processed[neighbor_index] = 1U;
        queue.push_back(neighbor_index);
      }
    }

    if (static_cast<int>(queue.size()) < min_points ||
        static_cast<int>(queue.size()) > kMaximumClusterPoints) {
      continue;
    }
    std::sort(queue.begin(), queue.end());
    pcl::PointIndices indices;
    indices.indices = queue;
    clusters.push_back(std::move(indices));
  }
  return clusters;
}

}  // namespace

// Ported from Main src/Lidar/Clustering/Clustering.cpp:97-118 (8f93c22e)
float AdaptiveClusterTolerance(float horizontal_range_m, const ClusteringConfig& config) {
  const float fixed = PositiveOr(config.f32_Eps, kDefaultClusterTolerance);
  if (!config.b_Adaptive) return fixed;

  // A configured NearRange of 0 is not positive and falls back to 8 m (Main behaviour).
  const float near_range = std::max(0.0F, PositiveOr(config.f32_AdaptiveNearRange, 8.0F));
  const float far_range =
      std::max(near_range + 1.0F, PositiveOr(config.f32_AdaptiveFarRange, 50.0F));
  const float near_tolerance = PositiveOr(config.f32_AdaptiveNearEps, fixed);
  const float far_tolerance =
      std::max(near_tolerance, PositiveOr(config.f32_AdaptiveFarEps, near_tolerance));
  const float range =
      std::isfinite(horizontal_range_m) ? std::max(0.0F, horizontal_range_m) : near_range;
  const float alpha =
      std::clamp((range - near_range) / (far_range - near_range), 0.0F, 1.0F);
  return near_tolerance + alpha * (far_tolerance - near_tolerance);
}

// Ported from Main src/Lidar/Clustering/Clustering.cpp:121-233 (8f93c22e)
// Main's GPS_IMU parameter and the extern st_Planning/st_VehicleSpec are unused
// and dropped; config.b_Enabled is never consulted (as in Main).
void Clustering(const CloudPtr& input, const CloudPtr& centroids,
                std::vector<pcl::PointIndices>& clusters, const ClusteringConfig& config) {
  if (input->empty()) {
    // Main pushes the dummy into the INPUT cloud (points only; width/height stay).
    input->points.clear();
    pcl::PointXYZI pcl_EmptyPoint;
    pcl_EmptyPoint.x = 0.0;
    pcl_EmptyPoint.y = 0.0;
    pcl_EmptyPoint.z = 0.0;
    pcl_EmptyPoint.intensity = 0.0;
    input->points.push_back(pcl_EmptyPoint);
    clusters.clear();
    return;
  }
  pcl::search::KdTree<pcl::PointXYZI>::Ptr pcl_Tree(new pcl::search::KdTree<pcl::PointXYZI>);
  pcl_Tree->setInputCloud(input);
  std::vector<pcl::PointIndices> vp_ClusterIndices;
  const float tolerance = std::isfinite(config.f32_Eps) && config.f32_Eps > 0.0F
                              ? config.f32_Eps
                              : kDefaultClusterTolerance;
  const int min_points = std::max(1, config.s32_MinPoints);
  if (config.b_Adaptive) {
    vp_ClusterIndices = ExtractAdaptiveClusters(input, pcl_Tree, config, min_points);
  } else {
    // PCL path: clusters come out sorted by size, largest first (non-stable sort).
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> pcl_Ec;
    pcl_Ec.setInputCloud(input);
    pcl_Ec.setClusterTolerance(tolerance);
    pcl_Ec.setMinClusterSize(min_points);
    pcl_Ec.setMaxClusterSize(kMaximumClusterPoints);
    pcl_Ec.setSearchMethod(pcl_Tree);
    pcl_Ec.extract(vp_ClusterIndices);
  }
  // Main's per-frame min/max cluster-size summary only fed a commented-out log; dropped.

  clusters = vp_ClusterIndices;
  if (vp_ClusterIndices.size() == 0) {
    // Dummy (0,0,0) appended to the INPUT cloud; centroids is left untouched.
    pcl::PointXYZI pcl_EmptyPoint;
    pcl_EmptyPoint.x = 0.0;
    pcl_EmptyPoint.y = 0.0;
    pcl_EmptyPoint.z = 0.0;
    pcl_EmptyPoint.intensity = 0.0;
    input->points.push_back(pcl_EmptyPoint);
    return;
  }

  std::vector<pcl::PointIndices> vp_FilteredClusterIndices;

  for (auto vp_ClusterIt = vp_ClusterIndices.begin(); vp_ClusterIt != vp_ClusterIndices.end();
       ++vp_ClusterIt) {
    pcl::PointXYZI pcl_Centroid;
    pcl_Centroid.x = 0.0;
    pcl_Centroid.y = 0.0;
    pcl_Centroid.z = 0.0;
    pcl_Centroid.intensity = 0.0f;

    // Float accumulation in cluster index order, then division by float(size).
    for (auto point_it = vp_ClusterIt->indices.begin(); point_it != vp_ClusterIt->indices.end();
         ++point_it) {
      pcl_Centroid.x += input->points[*point_it].x;
      pcl_Centroid.y += input->points[*point_it].y;
      pcl_Centroid.z += input->points[*point_it].z;
    }
    pcl_Centroid.x /= vp_ClusterIt->indices.size();
    pcl_Centroid.y /= vp_ClusterIt->indices.size();
    pcl_Centroid.z /= vp_ClusterIt->indices.size();
    // Main's commented-out planning-area InOrOut filter is not ported.
    centroids->points.push_back(pcl_Centroid);
    vp_FilteredClusterIndices.push_back(*vp_ClusterIt);
  }

  clusters = vp_FilteredClusterIndices;

  centroids->width = centroids->points.size();
  centroids->height = 1;
  centroids->is_dense = true;
}

}  // namespace pipeline
}  // namespace lidar
}  // namespace asmc
