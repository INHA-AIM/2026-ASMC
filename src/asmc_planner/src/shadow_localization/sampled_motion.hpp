#pragma once
#include "local/motion.hpp"
namespace shadow_localization {
// Source-stamped external poses. Interpolation only: never integrates speed
// or IMU a second time. The upstream session owns odometry and its epoch.
class SampledMotion {
 public:
  void clear(){rows_.clear();}
  bool push(local_planning::Motion m){
    if(!std::isfinite(m.t+m.x+m.y+m.yaw+m.v+m.w)||m.t<=0)return false;
    if(!rows_.empty()){
      const auto& p=rows_.back();if(m.t<=p.t)return false;
      m.yaw=p.yaw+std::remainder(m.yaw-p.yaw,2*M_PI);
      m.a=.7*p.a+.3*std::clamp((m.v-p.v)/(m.t-p.t),-5.,2.);
    }
    rows_.push_back(m);
    while(rows_.size()>256||rows_.back().t-rows_.front().t>2.)rows_.pop_front();
    return true;
  }
  std::optional<local_planning::Motion> at(double t)const{
    if(rows_.empty()||t<rows_.front().t||t>rows_.back().t+.05)return {};
    if(t>=rows_.back().t){auto q=rows_.back();const double dt=t-q.t;
      q.x+=q.v*dt*std::cos(q.yaw+.5*q.w*dt);q.y+=q.v*dt*std::sin(q.yaw+.5*q.w*dt);
      q.yaw+=q.w*dt;q.t=t;return q;}
    for(size_t i=1;i<rows_.size();++i)if(rows_[i].t>=t){
      auto a=rows_[i-1],b=rows_[i];
      if(t==a.t)return a;
      if(t==b.t)return b;
      if(b.t-a.t>.15)return {};
      double q=(t-a.t)/(b.t-a.t);
      return local_planning::Motion{t,a.x+q*(b.x-a.x),a.y+q*(b.y-a.y),
        a.yaw+q*(b.yaw-a.yaw),a.v+q*(b.v-a.v),a.w+q*(b.w-a.w),
        a.roll+q*(b.roll-a.roll),a.pitch+q*(b.pitch-a.pitch),a.a+q*(b.a-a.a)};
    }
    return {};
  }
 private:std::deque<local_planning::Motion> rows_;
};
} // namespace shadow_localization
