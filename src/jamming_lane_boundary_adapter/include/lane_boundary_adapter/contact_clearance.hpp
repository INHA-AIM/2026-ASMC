#pragma once
// Short-horizon experiment: camera ground-contact bands, not obstacle OBBs.
// A band occludes the observed road corridor downstream so a point contact cannot
// authorize crossing back through an unobserved object body. All geometry uses
// the image-time base_link frame; no map pose or motion classification is used.
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace lane_boundary_adapter::clearance {
// IMU-corrected shared-normal samples need not start at exactly x=1m.
// Allow at most one existing0.25m sample interval for the NEAR tangent anchor;
// this never joins an interior INVALID gap or extends forward support.
constexpr double kNearSampleAllowanceM=.25;
inline double nearReferenceX(double first_x,double nominal=1.) {
  return std::isfinite(first_x) && first_x<=nominal+kNearSampleAllowanceM ?
      std::max(first_x,nominal):std::numeric_limits<double>::quiet_NaN();
}
inline double contiguousSupport(std::vector<std::pair<double,double>> intervals) {
  std::sort(intervals.begin(),intervals.end());double end=0.;bool started=false;
  for(const auto& span:intervals){
    if(!std::isfinite(span.first)||!std::isfinite(span.second)||span.second<=span.first)return 0.;
    if(!started){if(!std::isfinite(nearReferenceX(span.first)))break;end=span.second;started=true;}
    else if(span.first<=end+1e-4)end=std::max(end,span.second);
    else break; // interior INVALID remains a hard split.
  }return end;
}
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
// Bounded convex footprint, ordered counter-clockwise. Preserve the projection
// of all eight OBB corners; roll/pitch can produce more than four XY vertices.
// The Minkowski sum with the same axis-aligned padding square retains the old
// 0.25m uncertainty in BOTH axes without inventing the OBB's entire AABB.
inline double cross(const Point& a,const Point& b,const Point& c) {
  return (b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x);
}
inline bool convexValid(const std::vector<Point>& p) {
  if(p.size()<3 || p.size()>32)return false;
  double area=0.;for(std::size_t i=0;i<p.size();++i){
    if(!finite(p[i]))return false;
    area+=p[i].x*p[(i+1)%p.size()].y-p[i].y*p[(i+1)%p.size()].x;
  }
  if(!std::isfinite(area)||std::abs(area)<1e-10)return false;
  for(std::size_t i=0;i<p.size();++i){
    if(std::abs(cross(p[i],p[(i+1)%p.size()],p[(i+2)%p.size()]))<1e-10)return false;
    for(const auto& v:p)if(cross(p[i],p[(i+1)%p.size()],v)*area < -1e-10)return false;
  }return true;
}
inline std::vector<Point> paddedHull(const std::vector<Point>& points,double padding) {
  if(points.size()<3 || points.size()>32 || !std::isfinite(padding)||padding<0.)return {};
  std::vector<Point> p;p.reserve(points.size()*4);
  for(const auto& v:points){if(!finite(v))return {};
    for(double dx:{-padding,padding})for(double dy:{-padding,padding})p.push_back({v.x+dx,v.y+dy});}
  std::sort(p.begin(),p.end(),[](const Point& a,const Point& b){return a.x<b.x || (a.x==b.x && a.y<b.y);});
  p.erase(std::unique(p.begin(),p.end(),[](const Point& a,const Point& b){return a.x==b.x&&a.y==b.y;}),p.end());
  if(p.size()<3)return {};
  std::vector<Point> hull;hull.reserve(p.size()*2);
  for(const auto& v:p){while(hull.size()>1 && cross(hull[hull.size()-2],hull.back(),v)<=1e-10)hull.pop_back();hull.push_back(v);}
  const auto lower=hull.size();
  for(auto i=p.rbegin()+1;i!=p.rend();++i){while(hull.size()>lower && cross(hull[hull.size()-2],hull.back(),*i)<=1e-10)hull.pop_back();hull.push_back(*i);}
  hull.pop_back();return convexValid(hull)?hull:std::vector<Point>{};
}
// Separating-axis test, including boundary contact. Invalid input is occupied.
inline bool overlapsConvex(const std::vector<Point>& a,const std::vector<Point>& b) {
  for(const auto& p:a) if(!finite(p)) return true;
  for(const auto& p:b) if(!finite(p)) return true;
  for(const auto* polygon:{&a,&b}) for(std::size_t i=0;i<polygon->size();++i) {
    const auto& p=(*polygon)[i];const auto& q=(*polygon)[(i+1)%polygon->size()];
    const Point axis{q.y-p.y,p.x-q.x};
    if(std::hypot(axis.x,axis.y)<1e-9) continue;
    double amin=1e100,amax=-1e100,bmin=1e100,bmax=-1e100;
    for(const auto& v:a){double d=v.x*axis.x+v.y*axis.y;amin=std::min(amin,d);amax=std::max(amax,d);}
    for(const auto& v:b){double d=v.x*axis.x+v.y*axis.y;bmin=std::min(bmin,d);bmax=std::max(bmax,d);}
    if(amax<bmin-1e-9||bmax<amin-1e-9)return false;
  }
  return true;
}
inline bool overlaps(const std::vector<Point>& a,const std::vector<Point>& b) {
  return !convexValid(a)||!convexValid(b)||overlapsConvex(a,b);
}
using Polygon = std::vector<Point>;
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
    occupied.push_back({{r.xmin,r.ymin},{end,r.ymin},{end,r.ymax},{r.xmin,r.ymax}});
    if(end>=horizon)continue;
    double origin;if(!referenceY(reference,end,origin))return false;
    double x=end,dy=0;
    while(x<horizon-1e-8) {
      const double next=std::min(x+.25,horizon);double y;
      if(!referenceY(reference,next,y))return false;
      const double next_dy=y-origin;
      occupied.push_back({{x,r.ymin+dy},{next,r.ymin+next_dy},
                            {next,r.ymax+next_dy},{x,r.ymax+dy}});
      x=next;dy=next_dy;
    }
  }
  return true;
}
// Image contact uncertainty is a projected convex quadrilateral. Its AABB
// mixes a far corner's lateral extent with the near edge (r36), needlessly
// blocking the adjacent lane. Preserve all four measured rays. Unseen depth
// AFTER the farthest corner remains the same conservative downstream band.
inline bool projectedContactOccupancy(const std::vector<Polygon>& contacts,const std::vector<Point>& reference,
                                     double horizon,std::vector<Polygon>& occupied) {
  occupied.clear();
  if(!std::isfinite(horizon)||horizon<=0.)return false;
  for(const auto& polygon:contacts){
    if(polygon.size()!=4)return false;
    Bounds b{1e100,-1e100,1e100,-1e100};double orientation=0.;
    for(unsigned i=0;i<4;++i){
      const auto& p=polygon[i];const auto& q=polygon[(i+1)%4];const auto& r=polygon[(i+2)%4];
      if(!finite(p)||!finite(q)||!finite(r))return false;
      const double turn=(q.x-p.x)*(r.y-q.y)-(q.y-p.y)*(r.x-q.x);
      if(std::abs(turn)<1e-9 || (orientation!=0. && turn*orientation<0.))return false;
      orientation=turn;
      b.xmin=std::min(b.xmin,p.x);b.xmax=std::max(b.xmax,p.x);
      b.ymin=std::min(b.ymin,p.y);b.ymax=std::max(b.ymax,p.y);
    }
    if(b.xmin>horizon)continue;
    // Keeping the full polygon does not extend vehicle support: the swept
    // vehicle test independently rejects any corner beyond the real horizon.
    occupied.push_back(polygon);
    std::vector<Polygon> conservative;
    if(!contactOccupancy({b},reference,horizon,conservative))return false;
    occupied.insert(occupied.end(),conservative.begin()+1,conservative.end());
  }
  return true;
}
inline bool pathClearOfPolygons(const std::vector<Point>& path,const std::vector<Polygon>& occupied,
                      const Vehicle& v,double supported_horizon,double sample_padding=.25) {
  for(const auto& p:occupied)if(!convexValid(p))return false;
  for(const auto& p:path)if(!finite(p))return false;
  for(double x:{v.front,v.rear,v.half_width,v.lateral_margin,v.longitudinal_margin,sample_padding})
    if(!std::isfinite(x)||x<0.)return false;
  if(path.size()<2||!std::isfinite(supported_horizon)||supported_horizon<=0)return false;
  for(std::size_t i=0;i<path.size();++i){
    if(!finite(path[i]))return false;
    const auto& a=path[i?i-1:i]; const auto& b=path[std::min(i+1,path.size()-1)];
    const double yaw=std::atan2(b.y-a.y,b.x-a.x),c=std::cos(yaw),s=std::sin(yaw);
    Polygon ego(4);unsigned k=0;
    for(const Point p:Polygon{{v.front+v.longitudinal_margin+sample_padding,v.half_width+v.lateral_margin},
       {-v.rear-v.longitudinal_margin-sample_padding,v.half_width+v.lateral_margin},
       {-v.rear-v.longitudinal_margin-sample_padding,-v.half_width-v.lateral_margin},
       {v.front+v.longitudinal_margin+sample_padding,-v.half_width-v.lateral_margin}}) {
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
      if(overlapsConvex(ego,obstacle))return false;
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
