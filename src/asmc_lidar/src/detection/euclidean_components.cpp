#include <asmc_lidar/detection/euclidean_components.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <unordered_map>

namespace asmc::lidar::pipeline {
namespace {
using Key = std::array<std::int64_t,3>;
struct Hash {
  std::size_t operator()(const Key& k) const {
    std::size_t h=0;
    for (auto v:k) h^=std::hash<std::int64_t>{}(v)+0x9e3779b9+(h<<6)+(h>>2);
    return h;
  }
};
// PCL/FLANN L2_Simple<float>: preserve accumulation order and strict radius.
float DistanceSquared(const pcl::PointXYZI& a,const pcl::PointXYZI& b) {
  float d=a.x-b.x, result=d*d;
  d=a.y-b.y;result+=d*d;d=a.z-b.z;result+=d*d;return result;
}
}
std::vector<pcl::PointIndices> EuclideanComponents(
    const pcl::PointCloud<pcl::PointXYZI>& cloud,const std::vector<int>& indices,
    float tolerance,int min_points,int max_points) {
  if (!(tolerance>0) || !std::isfinite(tolerance) || min_points<1 || max_points<1)
    throw std::invalid_argument("invalid Euclidean component limits");
  // PCL promotes tolerance to double for squaring, then narrows to float.
  const float radius2=static_cast<float>(double(tolerance)*double(tolerance));
  // A cell wider than the radius avoids missing float-boundary neighbours.
  // The exact float squared-distance check below still defines connectivity.
  const double width=2.*double(tolerance);
  std::unordered_map<Key,std::vector<int>,Hash> bins;
  bins.reserve(indices.size());
  std::vector<Key> keys(cloud.size());
  std::vector<std::uint8_t> visited(cloud.size(),0),present(cloud.size(),0);
  bool safe=std::isfinite(radius2) && radius2>=std::numeric_limits<float>::min();
  for (int i:indices) {
    if (i<0 || static_cast<std::size_t>(i)>=cloud.size() || present[i])
      throw std::invalid_argument("invalid or duplicate Euclidean component index");
    present[i]=1;const auto& p=cloud[i];
    if (!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))
      throw std::invalid_argument("non-finite Euclidean component point");
    const double xyz[]={p.x,p.y,p.z};
    for (int a=0;a<3;++a) {
      const double value=std::floor(xyz[a]/width);
      if (!std::isfinite(value)||std::abs(value)>1e12) {safe=false;break;}
      keys[i][a]=static_cast<std::int64_t>(value);
    }
  }
  // Unusual numeric scales use the preserved PCL search, never truncated cells.
  if (!safe) {
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> extraction;
    extraction.setClusterTolerance(tolerance);extraction.setMinClusterSize(min_points);
    extraction.setMaxClusterSize(max_points);extraction.setInputCloud(cloud.makeShared());
    extraction.setIndices(pcl::IndicesPtr(new std::vector<int>(indices)));
    extraction.setSearchMethod(pcl::search::KdTree<pcl::PointXYZI>::Ptr(new pcl::search::KdTree<pcl::PointXYZI>()));
    std::vector<pcl::PointIndices> result;extraction.extract(result);return result;
  }
  for (int i:indices) bins[keys[i]].push_back(i);
  std::vector<pcl::PointIndices> result;
  std::vector<int> queue;queue.reserve(indices.size());
  for (int seed:indices) {
    if (visited[seed])continue;
    queue.clear();queue.push_back(seed);visited[seed]=1;
    for (std::size_t next=0;next<queue.size();++next) {
      const int i=queue[next];const auto key=keys[i];
      for(int x=-1;x<=1;++x)for(int y=-1;y<=1;++y)for(int z=-1;z<=1;++z) {
        const auto bucket=bins.find({key[0]+x,key[1]+y,key[2]+z});
        if(bucket==bins.end())continue;
        auto& candidates=bucket->second;
        for(std::size_t n=0;n<candidates.size();) {
          const int j=candidates[n];
          if(visited[j] || DistanceSquared(cloud[i],cloud[j])<radius2) {
            if(!visited[j]) {visited[j]=1;queue.push_back(j);}
            candidates[n]=candidates.back();candidates.pop_back();
          } else ++n;
        }
      }
    }
    if(queue.size()<static_cast<std::size_t>(min_points) || queue.size()>static_cast<std::size_t>(max_points))continue;
    pcl::PointIndices component;component.header=cloud.header;component.indices=queue;
    std::sort(component.indices.begin(),component.indices.end());result.push_back(std::move(component));
  }
  // Preserve PCL's ordering, including ties, from the same input seed order.
  std::sort(result.rbegin(),result.rend(),pcl::comparePointClusters);
  return result;
}
}  // namespace asmc::lidar::pipeline
