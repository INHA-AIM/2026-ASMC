#pragma once
#include <asmc_lidar/experimental/tunnel_filter_core.hpp>
#include <asmc_lidar/core/frame_time.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <mutex>
#include <set>

namespace asmc::lidar::pipeline {
// A separate causal history for tunnel wall motion. Legacy speed magnitude,
// local-input validity and public ego velocity retain their existing contracts.
class TunnelMotionHistory {
  struct Sample { std::int64_t stamp; double value; bool trusted; double lateral=0; };
  std::deque<Sample> yaw_,speed_;
  mutable std::mutex mutex_;
  static void Append(std::deque<Sample>& q,Sample s) {
    if(s.stamp<=0 || (!q.empty() && s.stamp<q.back().stamp))return;
    if(!q.empty() && s.stamp==q.back().stamp)q.back()=s;else q.push_back(s);
    while(q.size()>2 && (q.front().stamp+5000000000LL<q.back().stamp || q.size()>4096))q.pop_front();
  }
  static const Sample* At(const std::deque<Sample>& q,std::int64_t stamp) {
    auto it=std::upper_bound(q.begin(),q.end(),stamp,[](auto t,const auto& s){return t<s.stamp;});
    return it==q.begin()?nullptr:&*--it;
  }
 public:
  void AddYaw(std::int64_t stamp,double yaw,bool trusted) {
    std::lock_guard<std::mutex> lock(mutex_);Append(yaw_,{stamp,yaw,trusted});
  }
  void AddSpeed(std::int64_t stamp,double signed_longitudinal_mps,bool trusted,double lateral_mps=0) {
    std::lock_guard<std::mutex> lock(mutex_);Append(speed_,{stamp,signed_longitudinal_mps,trusted,lateral_mps});
  }
  tunnel_filter::EgoMotion Between(std::int64_t begin,std::int64_t end,double begin_yaw,double end_yaw,
                                   const asmc::lidar::Vec& sensor_in_base) const {
    std::lock_guard<std::mutex> lock(mutex_);
    tunnel_filter::EgoMotion out;out.integrated=true;out.reliable=false;
    if(begin<=0 || end<=begin || end-begin>1000000000LL || !std::isfinite(begin_yaw) || !std::isfinite(end_yaw))return out;
    out.dt_s=(end-begin)*1e-9;
    out.dyaw_rad=std::atan2(std::sin(end_yaw-begin_yaw),std::cos(end_yaw-begin_yaw));
    std::set<std::int64_t> knots{begin,end};
    for(const auto& s:yaw_)if(s.stamp>begin && s.stamp<end)knots.insert(s.stamp);
    for(const auto& s:speed_)if(s.stamp>begin && s.stamp<end)knots.insert(s.stamp);
    bool trusted=true;double dx=0,dy=0;
    auto check=[&](std::int64_t stamp) {
      auto y=At(yaw_,stamp),v=At(speed_,stamp);
      return y&&v&&std::isfinite(y->value)&&std::isfinite(v->value)&&std::isfinite(v->lateral)&&std::hypot(v->value,v->lateral)<=35 &&
          stamp-y->stamp<=120000000LL&&stamp-v->stamp<=120000000LL;
    };
    if(!check(begin)||!check(end))return out;
    trusted=At(yaw_,begin)->trusted&&At(speed_,begin)->trusted&&At(yaw_,end)->trusted&&At(speed_,end)->trusted;
    auto a=begin;
    for(auto b:knots) {
      if(b<=a)continue;
      const auto mid=a+(b-a)/2;
      if(!check(mid))return out;
      const auto y=At(yaw_,mid),v=At(speed_,mid);
      trusted=trusted&&y->trusted&&v->trusted;
      const double dt=(b-a)*1e-9,angle=y->value-begin_yaw;
      dx+=(v->value*std::cos(angle)-v->lateral*std::sin(angle))*dt;
      dy+=(v->value*std::sin(angle)+v->lateral*std::cos(angle))*dt;
      ++out.integration_steps;a=b;
    }
    // Translation of the LiDAR origin, not just the rear-axle origin.
    const double c=std::cos(out.dyaw_rad),s=std::sin(out.dyaw_rad);
    dx+=(c-1)*sensor_in_base.x()-s*sensor_in_base.y();
    dy+=s*sensor_in_base.x()+(c-1)*sensor_in_base.y();
    out.dx_m=dx;out.dy_m=dy;out.valid=std::isfinite(dx)&&std::isfinite(dy);
    out.reliable=out.valid&&trusted;return out;
  }
};
}  // namespace asmc::lidar::pipeline
