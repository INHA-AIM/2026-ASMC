#include <asmc_lidar/core.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

// Isolated, untracked-object form of the L-shape search in Son 6435af1
// Tracking.cpp::obb_util::computeOBB3D_LShape. The 1-degree search,
// closeness score, low-point fallback, corner order and float OBB conversion
// are retained; tracker smoothing is intentionally outside this stage.
namespace asmc_lidar {
namespace {
constexpr double pi = 3.14159265358979323846;
constexpr double half_pi = pi / 2.0;
struct P2 { double x, y; };

double AxisYaw(double yaw) {
  while (yaw >= half_pi) yaw -= pi;
  while (yaw < -half_pi) yaw += pi;
  return yaw;
}

tunnel_filter::Obb AtYaw(const Cloud& cloud, double yaw) {
  const double c = std::cos(yaw), s = std::sin(yaw);
  double umin = std::numeric_limits<double>::infinity(), umax = -umin;
  double vmin = umin, vmax = -umin, zmin = umin, zmax = -umin;
  for (const auto& p : cloud.points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
    const double u = p.x*c + p.y*s, v = -p.x*s + p.y*c;
    umin = std::min(umin,u); umax = std::max(umax,u);
    vmin = std::min(vmin,v); vmax = std::max(vmax,v);
    zmin = std::min(zmin,double(p.z)); zmax = std::max(zmax,double(p.z));
  }
  tunnel_filter::Obb obb;
  if (!std::isfinite(umin)) return obb;
  const double cu=(umin+umax)/2, cv=(vmin+vmax)/2;
  const double cx=cu*c-cv*s, cy=cu*s+cv*c;
  obb.cx=static_cast<float>(cx); obb.cy=static_cast<float>(cy);
  obb.cz=static_cast<float>((zmin+zmax)/2);
  obb.length=static_cast<float>(umax-umin);
  obb.width=static_cast<float>(vmax-vmin);
  obb.z_min=static_cast<float>(zmin); obb.z_max=static_cast<float>(zmax);
  obb.yaw=static_cast<float>(AxisYaw(yaw));
  const std::array<std::pair<double,double>,4> signs{{{-1,-1},{1,-1},{1,1},{-1,1}}};
  for (std::size_t i=0;i<4;++i) {
    const double x=cx+signs[i].first*obb.length*.5*c-signs[i].second*obb.width*.5*s;
    const double y=cy+signs[i].first*obb.length*.5*s+signs[i].second*obb.width*.5*c;
    obb.corners[i]=pcl::PointXYZ(float(x),float(y),float(zmin));
    obb.corners[i+4]=pcl::PointXYZ(float(x),float(y),float(zmax));
  }
  return obb;
}

tunnel_filter::Obb Fit(const Cloud& cloud) {
  std::vector<P2> xy;
  xy.reserve(cloud.size());
  for (const auto& p:cloud.points)
    if (std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z))
      xy.push_back({p.x,p.y});
  if (xy.size()<10) {
    auto obb=AtYaw(cloud,0);
    return obb.length<obb.width?AtYaw(cloud,half_pi):obb;
  }
  double best=-std::numeric_limits<double>::infinity(), best_yaw=0;
  for (double theta=0;theta<half_pi;theta+=pi/180.0) {
    const double c=std::cos(theta),s=std::sin(theta);
    double umin=std::numeric_limits<double>::infinity(),umax=-umin;
    double vmin=umin,vmax=-umin;
    for (const auto& p:xy) {
      const double u=p.x*c+p.y*s,v=-p.x*s+p.y*c;
      umin=std::min(umin,u);umax=std::max(umax,u);
      vmin=std::min(vmin,v);vmax=std::max(vmax,v);
    }
    double score=0;
    for (const auto& p:xy) {
      const double u=p.x*c+p.y*s,v=-p.x*s+p.y*c;
      score+=1.0/std::max(std::min(std::min(u-umin,umax-u),
                                    std::min(v-vmin,vmax-v)),0.05);
    }
    if (score>best) {best=score;best_yaw=theta;}
  }
  auto obb=AtYaw(cloud,best_yaw);
  return obb.length<obb.width?AtYaw(cloud,best_yaw+half_pi):obb;
}
}  // namespace

tunnel_filter::ObbFunction PinnedLShapeObb() {
  return [](const Cloud::Ptr& cloud,const std::vector<pcl::PointIndices>& clusters,
            std::vector<tunnel_filter::Obb>* obbs) {
    obbs->clear(); obbs->reserve(clusters.size());
    for (const auto& cluster:clusters) {
      Cloud points; points.reserve(cluster.indices.size());
      for (int i:cluster.indices) points.push_back(cloud->at(std::size_t(i)));
      obbs->push_back(Fit(points));
    }
  };
}
}  // namespace asmc_lidar
