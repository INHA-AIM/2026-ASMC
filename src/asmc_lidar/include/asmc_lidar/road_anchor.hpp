#pragma once
#include <asmc_lidar/core.hpp>
#include <array>
namespace asmc_lidar {
// Research-only near-field plane. No GT, history, or map input.
struct RoadAnchor {
  bool valid=false;
  std::array<double,4> plane{};
  double min_x=0,max_x=0,min_y=0,max_y=0;
  std::size_t input_points=0,inliers=0;
  double height=0,median_residual=0;
  bool Supports(const pcl::PointXYZI& p)const {
    return valid&&p.x>=min_x-2&&p.x<=max_x+2&&p.y>=min_y-2&&p.y<=max_y+2
      &&p.x*p.x+p.y*p.y<=17*17;
  }
  double HeightAt(double x,double y)const {
    return -(plane[0]*x+plane[1]*y+plane[3])/plane[2];
  }
};
RoadAnchor FitRoadAnchor(const Cloud&);
}
