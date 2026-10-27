#pragma once
// Short-horizon experiment: camera ground-contact bands, not obstacle OBBs.
// A band occludes the observed road corridor downstream so a point contact cannot
// authorize crossing back through an unobserved object body. All geometry uses
// the image-time base_link frame; no map pose or motion classification is used.
#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace lane_boundary_adapter::clearance {
struct Point { double x=0, y=0; };
struct Bounds { double xmin=0, xmax=0, ymin=0, ymax=0; };
struct Vehicle { double front=3.845,rear=.790,half_width=.946,lateral_margin=.30,longitudinal_margin=.5; };
// Hermite quintic basis for unit START slope, zero endpoint position/slope/
// curvature. Multiplying the position blend by x is not this slope basis and
// causes an unnecessary lateral hump when ego already turns toward the lane.
inline double startSlopeBasis(double u) {
  u=std::clamp(u,0.,1.);
  return u-6*u*u*u+8*u*u*u*u-3*u*u*u*u*u;
}
inline bool finite(const Point& p) { return std::isfinite(p.x)&&std::isfinite(p.y); }
// Separating-axis test, including boundary contact. Invalid input is occupied.
inline bool overlaps(const std::array<Point,4>& a,const std::array<Point,4>& b) {
  for(const auto& p:a) if(!finite(p)) return true;
  for(const auto& p:b) if(!finite(p)) return true;
  for(const auto* polygon:{&a,&b}) for(unsigned i=0;i<4;++i) {
    const auto& p=(*polygon)[i];const auto& q=(*polygon)[(i+1)%4];
    const Point axis{q.y-p.y,p.x-q.x};
    if(std::hypot(axis.x,axis.y)<1e-9) continue;
    double amin=1e100,amax=-1e100,bmin=1e100,bmax=-1e100;
    for(const auto& v:a){double d=v.x*axis.x+v.y*axis.y;amin=std::min(amin,d);amax=std::max(amax,d);}
    for(const auto& v:b){double d=v.x*axis.x+v.y*axis.y;bmin=std::min(bmin,d);bmax=std::max(bmax,d);}
    if(amax<bmin-1e-9||bmax<amin-1e-9)return false;
  }
  return true;
}
using Polygon = std::array<Point,4>;
inline bool referenceY(const std::vector<Point>& reference,double x,double& y) {
  if(reference.size()<2)return false;
  for(std::size_t i=1;i<reference.size();++i) {
    const auto& a=reference[i-1];const auto& b=reference[i];
    if(!finite(a)||!finite(b)||b.x<=a.x)return false;
    if(a.x<=x && x<=b.x){y=a.y+(b.y-a.y)*(x-a.x)/(b.x-a.x);return true;}
  }
  return false;
}
// Preserve the whole visible contact rectangle. Downstream occlusion follows
// the SAME observed road reference, not the vehicle's temporary heading during
// a turn. It is unknown corridor occupancy, NOT an estimated object length.
inline bool contactOccupancy(const std::vector<Bounds>& contacts,const std::vector<Point>& reference,
                             double horizon,std::vector<Polygon>& occupied) {
  occupied.clear();
  if(!std::isfinite(horizon)||horizon<=0)return false;
  for(const auto& r:contacts) {
    if(!std::isfinite(r.xmin)||!std::isfinite(r.xmax)||!std::isfinite(r.ymin)||!std::isfinite(r.ymax)||
       r.xmax<=r.xmin||r.ymax<=r.ymin)return false;
    if(r.xmin>horizon)continue;
    const double end=std::min(r.xmax,horizon);
    occupied.push_back({{{r.xmin,r.ymin},{end,r.ymin},{end,r.ymax},{r.xmin,r.ymax}}});
    if(end>=horizon)continue;
    double origin;if(!referenceY(reference,end,origin))return false;
    double x=end,dy=0;
    while(x<horizon-1e-8) {
      const double next=std::min(x+.25,horizon);double y;
      if(!referenceY(reference,next,y))return false;
      const double next_dy=y-origin;
      occupied.push_back({{{x,r.ymin+dy},{next,r.ymin+next_dy},
                            {next,r.ymax+next_dy},{x,r.ymax+dy}}});
      x=next;dy=next_dy;
    }
  }
  return true;
}
inline bool pathClearOfPolygons(const std::vector<Point>& path,const std::vector<Polygon>& occupied,
                      const Vehicle& v,double supported_horizon,double sample_padding=.25) {
  if(path.size()<2||!std::isfinite(supported_horizon)||supported_horizon<=0)return false;
  for(std::size_t i=0;i<path.size();++i){
    if(!finite(path[i]))return false;
    const auto& a=path[i?i-1:i]; const auto& b=path[std::min(i+1,path.size()-1)];
    const double yaw=std::atan2(b.y-a.y,b.x-a.x),c=std::cos(yaw),s=std::sin(yaw);
    Polygon ego;unsigned k=0;
    for(const Point p:Polygon{{{v.front+v.longitudinal_margin+sample_padding,v.half_width+v.lateral_margin},
       {-v.rear-v.longitudinal_margin-sample_padding,v.half_width+v.lateral_margin},
       {-v.rear-v.longitudinal_margin-sample_padding,-v.half_width-v.lateral_margin},
       {v.front+v.longitudinal_margin+sample_padding,-v.half_width-v.lateral_margin}}}) {
      ego[k++]={path[i].x+p.x*c-p.y*s,path[i].y+p.x*s+p.y*c};
    }
    double xmin=1e100,xmax=-1e100;
    for(const auto& p:ego){
      if(p.x>supported_horizon+1e-6)return false;
      xmin=std::min(xmin,p.x);xmax=std::max(xmax,p.x);
    }
    for(const auto& obstacle:occupied){
      double omin=1e100,omax=-1e100;
      for(const auto& p:obstacle){omin=std::min(omin,p.x);omax=std::max(omax,p.x);}
      if(omax<xmin||omin>xmax)continue;
      if(overlaps(ego,obstacle))return false;
    }
  }
  return true;
}
// Straight-reference fixture helper. Runtime passes its exact paired geometry.
inline bool pathClear(const std::vector<Point>& path,const std::vector<Bounds>& contacts,
                      const Vehicle& v,double horizon,double sample_padding=.25) {
  std::vector<Polygon> occupied;
  return contactOccupancy(contacts,{{0.,0.},{horizon,0.}},horizon,occupied) &&
         pathClearOfPolygons(path,occupied,v,horizon,sample_padding);
}
} // namespace lane_boundary_adapter::clearance
