#pragma once
// Range-adaptive Euclidean clustering and cluster centroids.
// Ported from Main Clustering.cpp at 8f93c22e.

#include <asmc_lidar/core/types.hpp>

#include <pcl/PointIndices.h>

#include <vector>

namespace asmc {
namespace lidar {
namespace pipeline {

// Main LiDARClustering (setting.yaml LiDAR.Clustering). b_Enabled is read but
// never consulted by Main; it is kept only to load the same file.
struct ClusteringConfig {
  bool b_Enabled = true;
  float32 f32_Eps = 0.5f;
  int32 s32_MinPoints = 5;
  bool b_Adaptive = true;
  float32 f32_AdaptiveNearRange = 8.f;
  float32 f32_AdaptiveFarRange = 50.f;
  float32 f32_AdaptiveNearEps = 0.55f;
  float32 f32_AdaptiveFarEps = 0.85f;
  // x < 0 in the leveled LiDAR frame. Zero keeps legacy symmetric tolerance.
  float32 f32_AdaptiveRearNearEps = 0.f;
  float32 f32_AdaptiveRearFarEps = 0.f;
};

// x == 0 belongs to the front profile. No sensor->body translation is applied.
float AdaptiveClusterTolerance(float horizontal_range_m, const ClusteringConfig& config,
                               float local_x_m = 0.f);

// Main Clustering(). input is the non-ground leveled cloud. Like Main it MUTATES
// input: a single (0,0,0) point is pushed when input is empty or no cluster is
// found. centroids receives one float-mean point per cluster in cluster order;
// like Main it is appended to, never cleared (pass a fresh cloud per scan).
// Cluster order: smallest seed index (adaptive) or size descending (PCL path).
void Clustering(const CloudPtr& input, const CloudPtr& centroids,
                std::vector<pcl::PointIndices>& clusters, const ClusteringConfig& config);

}  // namespace pipeline
}  // namespace lidar
}  // namespace asmc
