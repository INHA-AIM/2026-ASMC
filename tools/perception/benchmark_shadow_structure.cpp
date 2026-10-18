// Observation-only PCL surface evidence. No production ground/cluster changes.
// Input records are float x/y/z + uint32 original point index (16 bytes).
// Normals/planes are in RAW sensor coordinates, not gravity-aligned semantic GT.
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/search/kdtree.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

using Clock = std::chrono::steady_clock;
double ms(Clock::time_point start) {
  return std::chrono::duration<double, std::milli>(Clock::now()-start).count();
}

int main(int argc, char** argv) {
  if (argc != 4) {
    std::cerr << "usage: benchmark_shadow_structure INPUT_LIST OUTPUT_JSONL INDEX_DIRECTORY\n";
    return 2;
  }
  std::ifstream list(argv[1]);
  std::ofstream out(argv[2]);
  if (!list || !out) return 3;
  out << std::setprecision(12);
  std::string stamp, file;
  while (list >> stamp >> file) {
    auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<std::uint32_t> lineage;
    std::ifstream input(file, std::ios::binary);
    float xyz[3]; std::uint32_t raw;
    while (input.read(reinterpret_cast<char*>(xyz), 12)) {
      if (!input.read(reinterpret_cast<char*>(&raw), 4)) return 4;
      cloud->push_back(pcl::PointXYZ(xyz[0], xyz[1], xyz[2]));
      lineage.push_back(raw);
    }
    if (cloud->empty()) return 5;
    auto started=Clock::now();
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setInputCloud(cloud);
    normal_estimator.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
    normal_estimator.setKSearch(12);
    pcl::PointCloud<pcl::Normal> normals;
    normal_estimator.compute(normals);
    const double normal_ms=ms(started);
    std::size_t finite_normals=0;
    for (const auto& n:normals)
      finite_normals += std::isfinite(n.normal_x)&&std::isfinite(n.normal_y)&&std::isfinite(n.normal_z);
    started=Clock::now();
    std::vector<int> remaining(cloud->size());
    for (std::size_t i=0;i<remaining.size();++i) remaining[i]=i;
    out << "{\"stamp_ns\":" << stamp << ",\"input_points\":" << cloud->size()
        << ",\"finite_normals\":" << finite_normals << ",\"normal_ms\":" << normal_ms << ",\"planes\":[";
    bool comma=false;
    std::ofstream indices(std::string(argv[3])+"/"+stamp+".tsv");
    for (int plane=0;plane<4&&remaining.size()>=80;++plane) {
      pcl::SACSegmentation<pcl::PointXYZ> segmentation;
      segmentation.setOptimizeCoefficients(true);
      segmentation.setModelType(pcl::SACMODEL_PLANE);
      segmentation.setMethodType(pcl::SAC_RANSAC);
      segmentation.setMaxIterations(150);
      segmentation.setDistanceThreshold(.12);
      segmentation.setInputCloud(cloud);
      segmentation.setIndices(pcl::IndicesPtr(new std::vector<int>(remaining)));
      pcl::PointIndices inliers; pcl::ModelCoefficients coefficients;
      segmentation.segment(inliers,coefficients);
      if (inliers.indices.size()<80) break;
      float minimum[3]={INFINITY,INFINITY,INFINITY}, maximum[3]={-INFINITY,-INFINITY,-INFINITY};
      double residual=0;
      std::vector<bool> selected(cloud->size(),false);
      for (int i:inliers.indices) {
        selected[i]=true;
        const auto& p=cloud->points[i];
        const float value[3]={p.x,p.y,p.z};
        for (int j=0;j<3;++j) {
          minimum[j]=std::min(minimum[j],value[j]); maximum[j]=std::max(maximum[j],value[j]);
        }
        residual += std::abs(coefficients.values[0]*p.x+coefficients.values[1]*p.y+
                             coefficients.values[2]*p.z+coefficients.values[3]);
        indices << lineage[i] << '\t' << plane << '\n';
      }
      if(comma) out << ','; comma=true;
      out << "{\"support_points\":" << inliers.indices.size() << ",\"coefficients\":[";
      for(int j=0;j<4;++j) {if(j)out<<',';out<<coefficients.values[j];}
      out << "],\"observed_min_xyz\":["<<minimum[0]<<','<<minimum[1]<<','<<minimum[2]
          << "],\"observed_max_xyz\":["<<maximum[0]<<','<<maximum[1]<<','<<maximum[2]
          << "],\"mean_absolute_residual_m\":"<<residual/inliers.indices.size()<<'}';
      remaining.erase(std::remove_if(remaining.begin(),remaining.end(),[&](int i){return selected[i];}),remaining.end());
    }
    const double plane_ms=ms(started);
    out << "],\"unassigned_points\":"<<remaining.size()<<",\"plane_ms\":"<<plane_ms
        << ",\"deletion_enabled\":false,\"gravity_aligned\":false}\n";
  }
}
