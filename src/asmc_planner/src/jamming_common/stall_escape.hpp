#pragma once
#include "jamming_common/speed_profile.hpp"
#include "Frenet/Evaluation/CollisionChecker.hpp"
#include <chrono>

namespace local_planning {
// Two5s windows (including a continuous10s hold) request a different search.
// Source time must advance. Dynamic yielding, epochs, stale input and2m of
// measured progress retire the escalation; no command or validity is cached.
class StallEscapeWatch {
  std::string frame_;
  double stamp_=-1.,seen_=-1.,since_=-1.;
  Pose anchor_;
  unsigned windows_=0;
public:
  void reset(){frame_.clear();stamp_=seen_=since_=-1.;windows_=0;}
  unsigned windows()const{return windows_;}
  bool update(const std::string& frame,double stamp,double steady,const Pose& pose,double speed,bool dynamic_wait){
    if(!std::isfinite(stamp+steady+pose.x+pose.y+speed)||dynamic_wait){reset();return false;}
    if(frame!=frame_ || stamp<stamp_ || steady<seen_ || (seen_>=0&&steady-seen_>.5) ||
       (seen_>=0&&std::hypot(pose.x-anchor_.x,pose.y-anchor_.y)>=2.)){
      reset();frame_=frame;anchor_=pose;
    }
    if(stamp<=stamp_)return windows_>=2;
    stamp_=stamp;seen_=steady;
    if(std::abs(speed)>=.3){since_=-1.;return windows_>=2;}
    if(since_<0.)since_=steady;
    if(steady-since_>=5.){windows_=std::min(2u,windows_+1);since_=steady;}
    return windows_>=2;
  }
};

inline bool MovingForwardConflict(const Scene& scene,const Config& cfg){
  Pose p;const double k=InitialCurvature(scene,cfg);
  for(double d=0.;d<=8.;d+=.25){
    const auto body=MakeEgoFootprint(CartesianState{p.x,p.y,p.yaw,k,0,0},VehicleShape{cfg.width,cfg.length,cfg.rear},0.);
    for(const auto& o:scene.obstacles)if(!o.motion_sweep.empty() &&
      geometry::ConvexPolygonDistance(body,o.motion_sweep)<=cfg.margin+o.uncertainty+o.extra_clearance)return true;
    p.x+=.25*std::cos(p.yaw+.125*k);p.y+=.25*std::sin(p.yaw+.125*k);p.yaw+=.25*k;
  }
  return false;
}

// Same bounded fan, different evidence callbacks: LOCAL raster/OBB sweep or
// Localization map/OBB sweep. Failed routes are never re-labelled as drivable.
template<class Sweep>
Result FindStallEscape(const Scene& scene,const Config& cfg,Sweep sweep){
  Result best;best.reason="stall_escape_no_observed_opening";
  best.escape_requested=true;
  if(MovingForwardConflict(scene,cfg)){best.reason="stall_escape_dynamic_yield";return best;}
  const auto start=std::chrono::steady_clock::now();double best_score=-1.;
  const double k0=InitialCurvature(scene,cfg);
  for(double target:{0.,-.06,.06,-.12,.12,-.18,.18,-.23,.23}){
    if(std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count()>cfg.budget_ms)break;
    Result r;r.recovery=true;r.escape_requested=true;
    Pose p{0,0,0,k0,0};r.path.push_back(p);
    if(!sweep(p,p,r))continue;
    for(double d=.15;d<=8.01;d+=.15){
      Pose q=p;q.s=d;q.k=k0+std::min(1.,d/2.)*(target-k0);
      q.yaw=p.yaw+.15*.5*(p.k+q.k);
      q.x+=.15*std::cos(.5*(p.yaw+q.yaw));q.y+=.15*std::sin(.5*(p.yaw+q.yaw));
      if(std::abs(q.yaw)>.9 || std::abs(q.k)>cfg.max_k || !sweep(p,q,r))break;
      r.path.push_back(q);p=q;
    }
    if(r.path.back().s<.75 || !BuildSpeedProfile(scene,cfg,r))continue;
    const double score=r.path.back().s+.5*r.path.back().x-.2*std::abs(r.path.back().yaw);
    if(score>best_score){r.valid=true;r.reason="verified_stall_escape";r.path_source="stall_escape_fan";best_score=score;best=std::move(r);}
  }
  best.elapsed_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
  return best;
}
} // namespace local_planning
