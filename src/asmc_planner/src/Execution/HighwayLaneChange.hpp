#pragma once
#include "Frenet/Reference/LaneReference.hpp"
#include <memory>
#include <vector>
#include <algorithm>
#include <cmath>

namespace asmc_planner::lane_reference {
// Immutable preparation shared with Behavior. It contains road facts and
// policy, never a second planner or an independently selected timed gap.
struct HighwayLaneChangePolicy {
  bool enabled=false;
  double deficit_mps=0,confirmation_s=0,cooldown_s=0,min_gain_s=0;
  double settle_reserve_s=0,observation_front_m=0,observation_rear_m=0;
  double unseen_speed_mps=0;
};
struct HighwayLaneChangeNetwork {
  HighwayLaneChangePolicy policy;
  double forward_accel=0,comfortable_decel=0,source_max_age=0;
  std::vector<std::shared_ptr<const LaneReference>> lanes;
  std::shared_ptr<const LaneReference> common;
  std::size_t indexForLink(const std::string& link) const {
    for(std::size_t i=0;i<lanes.size();++i)
      for(const auto& id:lanes[i]->lanes()) {
        if(id==common->lanes().front())break;
        if(id=="lane:"+link)return i;
      }
    return lanes.size();
  }
};
struct HighwayObservedBox { double x=0,y=0,yaw=0,length=0,width=0; };
inline bool HighwayLeftChangeSupportsMerge(const HighwayLaneChangeNetwork& network,
    const std::string& source,const std::string& target) {
  if(!network.common)return false;
  const auto a=network.indexForLink(source),b=network.indexForLink(target);
  if(a>=network.lanes.size()||b>=network.lanes.size()||a==b)return false;
  const auto* road=network.common->map()->lane("lane:"+source);
  return road&&road->left_adjacent=="lane:"+target&&
      network.lanes[a]->lanes().back()==network.common->lanes().front()&&
      network.lanes[b]->lanes().back()==network.common->lanes().front();
}
inline double HighwayLaneChangeReserve(const HighwayLaneChangeNetwork& network,
    double speed,double vehicle_length,bool supports_merge) {
  // A compatible left change is already progress toward the shared merge.
  // It needs settling room, not an additional full stop before that merge.
  return speed*network.policy.settle_reserve_s+vehicle_length+
      (supports_merge?0:speed*speed/(2*network.comfortable_decel));
}
// This checks that a fresh observation covers the bounded target window.
// Observed vehicles may occupy it. The common transition generator, not
// Behavior's rectangular window, owns time-dependent collision/gap admission.
// Reported-box recall/occlusion is not certified by a synthetic ray cast.
// The candidate-level unseen-rear reach bound remains mandatory.
inline bool HighwayTargetWindowObserved(const LaneReference& lane,const CartesianState& ego,
    const std::vector<HighwayObservedBox>& boxes,const HighwayLaneChangePolicy& policy) {
  const auto origin=ProjectOnLaneReference(lane,ego.x,ego.y,ego.yaw,ego.v);
  if(!origin.valid||origin.s-policy.observation_rear_m<lane.usableBegin()||
     origin.s+policy.observation_front_m>lane.usableEnd())return false;
  for(const auto& box:boxes)
    if(!std::isfinite(box.x)||!std::isfinite(box.y)||!std::isfinite(box.yaw)||
       !std::isfinite(box.length)||!std::isfinite(box.width)||box.length<=0||box.width<=0)return false;
  for(double ds=-policy.observation_rear_m;ds<=policy.observation_front_m;ds+=5) {
    const auto point=Interpolate(lane.line(),origin.s+ds);
    const double dx=point.x-ego.x,dy=point.y-ego.y;
    const double longitudinal=std::cos(ego.yaw)*dx+std::sin(ego.yaw)*dy;
    // Actual Main ROI: x[-30,50], y[-20,20]. Policy stations are on the
    // target curve and need not equal sensor-frame x (45 m is not the ROI end).
    if(longitudinal < -30.1||longitudinal>50.1||
       std::fabs(-std::sin(ego.yaw)*dx+std::cos(ego.yaw)*dy)>20)return false;
  }
  return true;
}

}
