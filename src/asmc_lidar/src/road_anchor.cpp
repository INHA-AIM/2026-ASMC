#include <asmc_lidar/road_anchor.hpp>
#include <pcl/segmentation/sac_segmentation.h>
#include <algorithm>
#include <cmath>
#include <limits>
namespace asmc_lidar {
RoadAnchor FitRoadAnchor(const Cloud& cloud) {
  RoadAnchor out;Cloud::Ptr seed(new Cloud);
  for(const auto& p:cloud) {
    const double r2=p.x*p.x+p.y*p.y;
    if(r2>=25&&r2<=225&&p.z>=-2.9&&p.z<=-.7)seed->push_back(p);
  }
  out.input_points=seed->size();if(seed->size()<30)return out;
  pcl::SACSegmentation<pcl::PointXYZI> fit;
  fit.setOptimizeCoefficients(true);fit.setModelType(pcl::SACMODEL_PLANE);
  fit.setMethodType(pcl::SAC_RANSAC);fit.setMaxIterations(60);fit.setDistanceThreshold(.12);
  fit.setInputCloud(seed);pcl::PointIndices ids;pcl::ModelCoefficients c;fit.segment(ids,c);
  out.inliers=ids.indices.size();
  if(c.values.size()!=4||out.inliers<30||double(out.inliers)/seed->size()<.35)return out;
  double norm=std::sqrt(c.values[0]*c.values[0]+c.values[1]*c.values[1]+c.values[2]*c.values[2]);
  if(!std::isfinite(norm)||norm<1e-6)return out;
  for(int i=0;i<4;++i)out.plane[i]=c.values[i]*(c.values[2]<0?-1.:1.)/norm;
  if(out.plane[2]<std::cos(15.*3.141592653589793/180))return out;
  out.height=out.HeightAt(0,0);
  if(out.height< -2.10||out.height> -1.50)return out;
  out.min_x=out.min_y=std::numeric_limits<double>::infinity();
  out.max_x=out.max_y=-std::numeric_limits<double>::infinity();
  std::vector<double> residuals;
  for(int i:ids.indices) {
    const auto& p=seed->at(i);
    out.min_x=std::min(out.min_x,double(p.x));out.max_x=std::max(out.max_x,double(p.x));
    out.min_y=std::min(out.min_y,double(p.y));out.max_y=std::max(out.max_y,double(p.y));
    residuals.push_back(std::abs(out.plane[0]*p.x+out.plane[1]*p.y+out.plane[2]*p.z+out.plane[3]));
  }
  std::sort(residuals.begin(),residuals.end());out.median_residual=residuals[residuals.size()/2];
  out.valid=out.max_x-out.min_x>=4&&out.max_y-out.min_y>=2&&out.median_residual<=.08;
  return out;
}
}
