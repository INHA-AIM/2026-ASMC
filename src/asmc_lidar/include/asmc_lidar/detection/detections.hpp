#pragma once
// Formerly declared in pipeline/tracker.hpp. Produces unassociated records;
// tracking fields stay at defaults and no second point-cloud owner is created.
#include <asmc_lidar/tracking/track.hpp>
#include <pcl/PointIndices.h>
#include <vector>

namespace asmc::lidar::pipeline {
// Main Objectinfo_for_km(): one detection per cluster (points from `points`,
// centroid from `centroids`, AABB and raw L-shape OBB).
void BuildDetections(std::vector<TrackedObject>& detections, const CloudPtr& points,
                     const CloudPtr& centroids, const std::vector<pcl::PointIndices>& clusters);

}  // namespace asmc::lidar::pipeline
