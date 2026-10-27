#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <vector>

namespace asmc::lidar::pipeline {
// Exact radius connectivity, not voxel/downsampling or an approximate radius.
// Input indices must be unique, finite and in cloud bounds (as tunnel labels are).
// Same float XYZ distance, size limits and output order as PCL Euclidean extraction.
// Only unvisited points remain in spatial buckets: dense walls do not repeatedly
// allocate/sort the already visited radius-neighbour list for every point.
std::vector<pcl::PointIndices> EuclideanComponents(
    const pcl::PointCloud<pcl::PointXYZI>& cloud, const std::vector<int>& indices,
    float tolerance, int min_points, int max_points);
}  // namespace asmc::lidar::pipeline
