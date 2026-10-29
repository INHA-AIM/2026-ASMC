#pragma once
#include "Frenet/Maneuver/Merge/HighwayRearResponse.hpp"
#include <chrono>
namespace merge_response {
// Response is a conditional NPC braking hypothesis, not a measured state.
// Start at the original observation and retain its elapsed interval uncertainty.
inline bool Clear(const CartesianPath& path,const TrackedObject& object,
    const VehicleShape& shape,const CollisionCheckConfig& collision,double dt,
    double reaction,double maximum_decel,std::size_t samples,
    std::chrono::steady_clock::time_point deadline=std::chrono::steady_clock::time_point::max(), double age=0) {
    if(samples<2||samples>path.x.size()||path.y.size()!=path.x.size()||path.yaw.size()!=path.x.size()||
       !object.prediction_lane||object.prediction_origin||!std::isfinite(age)||age<0||!std::isfinite(dt)||dt<=0||
       !std::isfinite(reaction)||reaction<0||!std::isfinite(maximum_decel)||maximum_decel<=0)return false;
    if(!object.geometry_valid||InspectPredictionVelocity(object)!=PredictionInputStatus::MEASURED||
       object.prediction_lane->points.size()<3||!std::isfinite(object.prediction_lane_curvature)||
       object.prediction_lane_curvature<0)return false;
    for(std::size_t i=0;i<samples;++i)
        if(!std::isfinite(path.x[i])||!std::isfinite(path.y[i])||!std::isfinite(path.yaw[i]))return false;
    const double speed=std::hypot(object.vx,object.vy);
    highway_rear_response::State response;response.enabled=true;response.start=age+reaction;
    response.low=std::isfinite(object.prediction_min_speed)?object.prediction_min_speed:speed;
    response.high=std::isfinite(object.prediction_max_speed)?object.prediction_max_speed:speed;
    if(response.low<0||response.low>speed||response.high<speed)return false;
    response.floor=0;response.required_decel=response.maximum_decel=maximum_decel;
    const double duration=(samples-1)*dt;
    if(object.prediction_lane_s+response.distances(age+duration).second>
       std::min(object.prediction_lane->length,object.prediction_lane_end_s))return false;
    double radius=0,ego_motion=0;const double er=std::hypot(shape.length,shape.width*.5);
    for(const auto& p:object.corners)radius=std::max(radius,std::hypot(p.x-object.x,p.y-object.y));
    for(std::size_t i=1;i<samples;++i)ego_motion=std::max(ego_motion,
        (std::hypot(path.x[i]-path.x[i-1],path.y[i]-path.y[i-1])+
         er*std::fabs(std::remainder(path.yaw[i]-path.yaw[i-1],2*M_PI)))/dt);
    const double bound=std::max(1.0,ego_motion+response.high*
        (1+(std::fabs(object.prediction_lane_d)+radius)*object.prediction_lane_curvature));
    const double step=std::min(dt,.2/bound),count=std::ceil(duration/step);
    if(!std::isfinite(count)||count>20000)return false;
    for(std::size_t k=0;k<=static_cast<std::size_t>(count);++k) {
        if(k%16==0&&std::chrono::steady_clock::now()>=deadline)return false;
        const double t=std::min(duration,k*step),index=t/dt;
        const auto i=std::min(samples-2,static_cast<std::size_t>(index));const double w=std::clamp(index-i,0.0,1.0);
        CartesianState ego{path.x[i]+w*(path.x[i+1]-path.x[i]),path.y[i]+w*(path.y[i+1]-path.y[i]),
            path.yaw[i]+w*std::remainder(path.yaw[i+1]-path.yaw[i],2*M_PI),0,0,0};
        const auto footprint=response.footprint(object,age+t);
        const double margin=collision.safety_margin+collision.margin_growth_rate*t+bound*step*.5;
        if(footprint.size()<3)return false;
        const double clearance=geometry::ConvexPolygonDistance(MakeEgoFootprint(ego,shape,0),footprint);
        if(!std::isfinite(clearance)||clearance<=margin)return false;
    }
    return true;
}
inline bool Rear(const TrackedObject& o,const CartesianPath& path,const VehicleShape& shape,const CollisionCheckConfig& collision) {
    if(!o.prediction_lane||o.velocity_unavailable||o.motion_state==ObjectMotionState::LOW_THREAT||
       (o.type!=0&&o.type!=1)||path.x.empty())return false;
    CartesianState e{path.x[0],path.y[0],path.yaw[0],0,0,0};const auto body=PredictObstacleFootprint(o,0);
    if(body.size()<3||geometry::ConvexPolygonDistance(MakeEgoFootprint(e,shape,0),body)<=collision.safety_margin)return false;
    const auto s=FindClosestS(*o.prediction_lane,e.x,e.y,nullptr,100);const auto p=Interpolate(*o.prediction_lane,s);
    const double tx=std::cos(p.theta),ty=std::sin(p.theta),rear=shape.rear_overhang>=0?shape.rear_overhang:shape.length*.5;
    return std::cos(e.yaw-p.theta)>.7&&o.vx*tx+o.vy*ty>0&&
        std::all_of(body.begin(),body.end(),[&](auto q){return (q.x-e.x)*tx+(q.y-e.y)*ty < -rear-collision.safety_margin;});
}
}
