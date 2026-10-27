#pragma once
// Short-horizon experiment: camera ground-contact bands, not obstacle OBBs.
// A band is extended forward to the observed horizon so a point contact cannot
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
inline bool pathClear(const std::vector<Point>& path,const std::vector<Bounds>& contacts,
                      const Vehicle& v,double supported_horizon,double sample_padding=.25) {
  if(path.size()<2||!std::isfinite(supported_horizon)||supported_horizon<=0)return false;
  for(std::size_t i=0;i<path.size();++i){
    if(!finite(path[i]))return false;
    const auto& a=path[i?i-1:i]; const auto& b=path[std::min(i+1,path.size()-1)];
    const double yaw=std::atan2(b.y-a.y,b.x-a.x),c=std::cos(yaw),s=std::sin(yaw);
    std::array<Point,4> ego;unsigned k=0;
    for(const Point p:std::array<Point,4>{{{v.front+v.longitudinal_margin+sample_padding,v.half_width+v.lateral_margin},
       {-v.rear-v.longitudinal_margin-sample_padding,v.half_width+v.lateral_margin},
       {-v.rear-v.longitudinal_margin-sample_padding,-v.half_width-v.lateral_margin},
       {v.front+v.longitudinal_margin+sample_padding,-v.half_width-v.lateral_margin}}}) {
      ego[k++]={path[i].x+p.x*c-p.y*s,path[i].y+p.x*s+p.y*c};
    }
    // The full swept vehicle, not merely its rear axle, must be inside support.
    for(const auto& p:ego) if(p.x>supported_horizon+1e-6)return false;
    for(const auto& r:contacts){
      if(!std::isfinite(r.xmin)||!std::isfinite(r.xmax)||!std::isfinite(r.ymin)||!std::isfinite(r.ymax)||
         r.xmax<=r.xmin||r.ymax<=r.ymin)return false;
      if(r.xmin>supported_horizon)continue; // not declared clear beyond the horizon
      const double far=std::max(r.xmax,supported_horizon);
      const std::array<Point,4> occupied={{{r.xmin,r.ymin},{far,r.ymin},{far,r.ymax},{r.xmin,r.ymax}}};
      if(overlaps(ego,occupied))return false;
    }
  }
  return true;
}
} // namespace lane_boundary_adapter::clearance
