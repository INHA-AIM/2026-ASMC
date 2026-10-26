#pragma once
#include "Frenet/Geometry/ObservedMotion.hpp"
#include "Frenet/Reference/LaneReference.hpp"
#include <memory>
#include <vector>

namespace highway_lane_motion_detail {
// A small, read-only ambiguity query on the already validated lane polygon.
// Boundary counts as ambiguous; it never grants lane ownership or clearance.
inline bool Contains(const std::vector<asmc_planner::Point2D>& polygon,double x,double y) {
  if(polygon.size()<3)return true;
  bool inside=false;
  for(std::size_t i=0,j=polygon.size()-1;i<polygon.size();j=i++) {
    const auto a=polygon[j],b=polygon[i];const double dx=b.x-a.x,dy=b.y-a.y;
    const double length2=dx*dx+dy*dy;
    if(length2>1e-12) {
      const double u=std::clamp(((x-a.x)*dx+(y-a.y)*dy)/length2,0.0,1.0);
      if(std::hypot(x-a.x-u*dx,y-a.y-u*dy)<1e-7)return true;
    }
    if((a.y>y)!=(b.y>y)&&x<(b.x-a.x)*(y-a.y)/(b.y-a.y)+a.x)inside=!inside;
  }
  return inside;
}
}

// Competition-specific highway assumption: surrounding traffic keeps its
// current mapped lane. This is never enabled for pedestrians, static actors,
// unresolved lane membership, ordinary urban driving or roundabout traffic.
// References are built once by HighwayReferenceExecution and shared by cores.
inline TrackedObject PrepareHighwayLaneMotion(const TrackedObject& object,
    const ObservedObjectMotion* track,
    const std::vector<std::shared_ptr<const asmc_planner::lane_reference::LaneReference>>& lanes,
    double minimum_span,double minimum_alignment) {
  using namespace asmc_planner::lane_reference;
  auto result=object;
  const auto vehicle=static_cast<int>(asmc_planner::ObjectType::VEHICLE);
  const auto unknown=static_cast<int>(asmc_planner::ObjectType::UNKNOWN);
  if((object.type!=vehicle&&object.type!=unknown)||object.motion_state==ObjectMotionState::LOW_THREAT||
     object.motion_state==ObjectMotionState::UNKNOWN||!track||track->samples.size()<3||
     track->stamp-track->samples.front().time<minimum_span)return result;
  const double speed=std::hypot(object.vx,object.vy);
  if(!std::isfinite(speed)||speed<1)return result;
  std::shared_ptr<const LaneReference> best;
  ReferenceProjection projection;double distance=INFINITY;
  std::vector<std::pair<std::shared_ptr<const LaneReference>,ReferenceProjection>> matches;
  for(const auto& lane:lanes) {
    if(!lane||!lane->coversCenter(object.x,object.y))continue;
    const auto p=ProjectOnLaneReference(*lane,object.x,object.y,0,object.vx,object.vy);
    if(!p.valid||p.s<0||p.s>lane->line().length)continue;
    const auto at=Interpolate(lane->line(),p.s);
    if((object.vx*std::cos(at.theta)+object.vy*std::sin(at.theta))/speed<minimum_alignment)continue;
    // Unique source frames confirm membership, including geometry fragments
    // classified UNKNOWN. A crossing actor or an ID discontinuity abstains.
    bool stable=true;std::size_t checked=0;
    for(auto it=track->samples.rbegin();it!=track->samples.rend()&&checked<3;++it,++checked)
      if(!lane->coversCenter(it->x,it->y)){stable=false;break;}
    if(!stable)continue;
    matches.push_back({lane,p});
    if(p.distance_m<distance){distance=p.distance_m;best=lane;projection=p;}
  }
  if(!best)return result;
  for(const auto& match:matches) {
    if(match.first==best||match.second.distance_m>distance+.3)continue;
    // Overlapping/taper polygons do not identify a route at a fork. An
    // ambiguous match is acceptable only while both prepared continuations
    // agree geometrically; otherwise preserve the original motion hypothesis.
    for(double ds:{10.0,20.0,40.0}) {
      const auto a=Interpolate(best->line(),projection.s+ds);
      const auto b=Interpolate(match.first->line(),match.second.s+ds);
      if(std::hypot(a.x-b.x,a.y-b.y)>.5)return result;
    }
  }
  const auto at=Interpolate(best->line(),projection.s);
  const double along=object.vx*std::cos(at.theta)+object.vy*std::sin(at.theta);
  // UNKNOWN roadside clutter with a fictitious velocity is not certified by
  // a class guess. Its actual source-time displacement must support motion.
  if(object.type==unknown) {
    const auto& first=track->samples.front();const double dt=track->stamp-first.time;
    const double displacement=(object.x-first.x)*std::cos(at.theta)+(object.y-first.y)*std::sin(at.theta);
    if(displacement<=.1||displacement/dt<.5*along)return result;
  }
  result.prediction_lane=std::shared_ptr<const RefLine>(best,&best->line());
  result.prediction_lane_s=projection.s;result.prediction_lane_d=projection.d;
  result.prediction_lane_anchor_x=at.x-projection.d*std::sin(at.theta);
  result.prediction_lane_anchor_y=at.y+projection.d*std::cos(at.theta);
  result.prediction_lane_heading=at.theta;
  result.prediction_lane_curvature=best->maximumCurvature();
  // An ego route is not the NPC's route. At the first unresolved successor
  // stop using the prepared ego continuation; retain the incoming tangent.
  result.prediction_lane_end_s=best->line().length;
  for(const auto& id:best->lanes()) {
    const auto* road=best->map()->lane(id);
    if(!road||road->successors.size()<=1)continue;
    const auto& p=road->centerline.back();
    const double end=FindClosestS(best->line(),p.x,p.y,nullptr,100);
    if(end>projection.s+.01){result.prediction_lane_end_s=std::min(result.prediction_lane_end_s,end);break;}
    for(const auto& successor:road->successors) {
      if(std::find(best->lanes().begin(),best->lanes().end(),successor)!=best->lanes().end())continue;
      const auto* alternative=best->map()->lane(successor);
      if(alternative&&highway_lane_motion_detail::Contains(alternative->polygon,object.x,object.y))
        return object;
    }
  }
  result.vx=along*std::cos(at.theta);result.vy=along*std::sin(at.theta);result.speed=along;
  if(std::isfinite(object.prediction_min_speed)&&std::isfinite(object.prediction_max_speed)) {
    result.prediction_min_speed=std::max(0.0,along-std::max(0.0,speed-object.prediction_min_speed));
    result.prediction_max_speed=along+std::max(0.0,object.prediction_max_speed-speed);
  }
  result.yaw_rate=0;result.turn_prediction_duration=INFINITY;
  result.continuation_vx=result.continuation_vy=NAN;
  return result;
}
