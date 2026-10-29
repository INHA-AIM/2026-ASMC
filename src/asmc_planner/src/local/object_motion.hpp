#pragma once
#include "local/planner.hpp"
#include <algorithm>
#include <cmath>
#include <deque>
#include <map>
#include <set>

namespace local_planning {
// Planning-only, short relative displacement evidence. Does not certify an
// absolute velocity, classify a semantic group as moving/static, or publish
// synthetic detections. Duplicated scans cannot accumulate confirmations.
class ObjectMotion {
  struct Sample { double t,x,y,w,h; };
  std::map<int,std::deque<Sample>> history_;
public:
  void clear() { history_.clear(); }
  size_t size() const { return history_.size(); }
  void apply(std::vector<Obstacle>& objects,const Pose& origin,double stamp,
             double horizon) {
    if(!std::isfinite(stamp+origin.x+origin.y+origin.yaw+horizon) || horizon<0 || horizon>3.) {
      clear();return;
    }
    const double c=std::cos(origin.yaw),s=std::sin(origin.yaw);
    for(auto it=history_.begin();it!=history_.end();) {
      if(stamp-it->second.back().t>.65 || stamp<it->second.back().t)it=history_.erase(it);
      else ++it;
    }
    for(auto &o:objects) {
      o.motion_sweep.clear();o.local_vx=o.local_vy=0.;
      if(o.retained || o.polygon.size()<3)continue;
      double x0=1e9,y0=1e9,x1=-1e9,y1=-1e9;
      for(auto p:o.polygon) {
        double x=origin.x+c*p.x-s*p.y,y=origin.y+s*p.x+c*p.y;
        x0=std::min(x0,x);x1=std::max(x1,x);y0=std::min(y0,y);y1=std::max(y1,y);
      }
      Sample a{stamp,(x0+x1)*.5,(y0+y1)*.5,x1-x0,y1-y0};
      auto &q=history_[o.id];
      if(!q.empty() && (stamp-q.back().t>.30 || std::hypot(a.x-q.back().x,a.y-q.back().y)>1.5))q.clear();
      if(q.empty() || stamp>q.back().t)q.push_back(a);
      while(q.size()>8 || (!q.empty() && stamp-q.front().t>.60))q.pop_front();
      if(q.size()<4 || q.back().t-q.front().t<.20)continue;
      // Shape/face jumps are not rigid motion. Do not let OBB growth turn a
      // guardrail/partial cluster into a confident moving obstacle.
      bool shape=true,stopped=false;
      for(auto p:q) {
        shape &= std::abs(p.w-a.w)<=.30 && std::abs(p.h-a.h)<=.30;
        if(stamp-p.t>=.18 && std::hypot(p.x-a.x,p.y-a.y)<.05)stopped=true;
      }
      if(!shape || stopped)continue;
      double mt=0,mx=0,my=0;
      for(auto p:q) {mt+=p.t-stamp;mx+=p.x;my+=p.y;}
      mt/=q.size();mx/=q.size();my/=q.size();
      double denom=0,vx=0,vy=0;
      for(auto p:q) {double t=p.t-stamp-mt;denom+=t*t;vx+=t*(p.x-mx);vy+=t*(p.y-my);}
      if(denom<1e-8)continue;
      vx/=denom;vy/=denom;double speed=std::hypot(vx,vy);
      if(speed<.5 || speed>8. || std::hypot(a.x-q.front().x,a.y-q.front().y)<.15)continue;
      double residual=0,travel=0;
      for(size_t i=0;i<q.size();++i) {
        auto p=q[i];double t=p.t-stamp-mt;
        residual=std::max(residual,std::hypot(p.x-mx-vx*t,p.y-my-vy*t));
        if(i)travel+=std::hypot(p.x-q[i-1].x,p.y-q[i-1].y);
      }
      if(residual>.12 || travel>1.35*std::hypot(a.x-q.front().x,a.y-q.front().y))continue;
      o.local_vx=c*vx+s*vy;o.local_vy=-s*vx+c*vy;
      auto points=o.polygon;
      for(auto p:o.polygon)points.push_back({p.x+o.local_vx*horizon,p.y+o.local_vy*horizon});
      o.motion_sweep=geometry::ComputeConvexHull(points);
    }
    while(history_.size()>64)history_.erase(history_.begin());
  }
};
} // namespace local_planning
