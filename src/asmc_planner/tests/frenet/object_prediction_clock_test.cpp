#include "Frenet/Evaluation/CollisionChecker.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
int checks=0,failures=0;
void Expect(bool ok,const char* what){++checks;if(!ok){++failures;std::cerr<<"FAIL: "<<what<<'\n';}}
bool Near(double a,double b){return std::fabs(a-b)<1e-8;}
TrackedObject Object(){
    TrackedObject o;o.id=11;o.geometry_valid=o.z_valid=true;
    o.motion_state=ObjectMotionState::DYNAMIC;o.x=4;o.y=2;o.heading=.17;
    o.vx=10;o.vy=1;o.length=4.5;o.width=1.9;o.height=1.5;
    const double xy[4][2]={{-2.25,-.95},{2.25,-.95},{2.25,.95},{-2.25,.95}};
    for(int i=0;i<8;++i){const auto& p=xy[i%4];
        o.corners[i]={o.x+std::cos(o.heading)*p[0]-std::sin(o.heading)*p[1],
            o.y+std::sin(o.heading)*p[0]+std::cos(o.heading)*p[1],i<4?0.:1.5};}
    return o;
}
bool SamePrediction(const TrackedObject& a,double ta,const TrackedObject& b,double tb){
    const auto p=PredictTrackedObjectPose(a,ta),q=PredictTrackedObjectPose(b,tb);
    const auto v=PredictTrackedObjectVelocity(a,ta),w=PredictTrackedObjectVelocity(b,tb);
    bool same=Near(p.x,q.x)&&Near(p.y,q.y)&&Near(p.heading,q.heading)&&Near(v.first,w.first)&&Near(v.second,w.second);
    const auto x=PredictTrackedObjectCorners(a,ta),y=PredictTrackedObjectCorners(b,tb);
    for(int i=0;i<8;++i)same&=Near(x[i].x,y[i].x)&&Near(x[i].y,y[i].y)&&Near(x[i].z,y[i].z);
    const auto f=PredictObstacleFootprint(a,ta),g=PredictObstacleFootprint(b,tb);
    same&=f.size()==g.size();
    for(std::size_t i=0;i<std::min(f.size(),g.size());++i)same&=Near(f[i].x,g[i].x)&&Near(f[i].y,g[i].y);
    return same;
}
void Check(const TrackedObject& raw,const char* label){
    const auto original=raw;
    for(double delay:{0.,.05,.1,.2}){
        const auto aligned=AdvanceObjectObservation(raw,delay);
        for(double future:{0.,.1,1.,3.})Expect(SamePrediction(raw,delay+future,aligned,future),label);
        const auto twice=AdvanceObjectObservation(AdvanceObjectObservation(raw,.04),delay);
        Expect(SamePrediction(raw,.04+delay+.3,twice,.3),"alignment composition counts each elapsed interval once");
        if(delay)Expect(aligned.prediction_origin&&!aligned.prediction_origin->prediction_origin,
            "derived geometry has one flat immutable observation origin");
    }
    Expect(!raw.prediction_origin&&raw.x==original.x&&raw.y==original.y&&raw.vx==original.vx&&raw.vy==original.vy,
        "alignment leaves the observed state and motion-history input untouched");
    for(int i=0;i<8;++i)Expect(raw.corners[i].x==original.corners[i].x&&raw.corners[i].y==original.corners[i].y,
        "authoritative observed corners remain unchanged");
}
}
int main(){
    auto raw=Object();Check(raw,"straight model retains source-age plus future-time prediction");
    raw.yaw_rate=.4;Check(raw,"turn-rate model retains source-age plus future-time prediction");
    raw.turn_prediction_duration=.15;raw.continuation_vx=8;raw.continuation_vy=-.3;
    Check(raw,"bounded-turn continuation does not restart after alignment");
    raw=Object();raw.prediction_min_speed=8;raw.prediction_max_speed=13;
    Check(raw,"speed-range occupancy retains uncertainty accumulated before the ego timestamp");
    auto lane=std::make_shared<RefLine>();lane->length=100;
    for(int i=0;i<=100;++i){RefPoint p{};const double angle=i/40.;
        p.s=i;p.x=40*std::sin(angle);p.y=40*(1-std::cos(angle));p.theta=angle;p.kappa=1./40;
        lane->points.push_back(p);}
    raw=Object();raw.prediction_lane=lane;raw.prediction_lane_s=5;raw.prediction_lane_d=.3;
    raw.prediction_lane_end_s=96;
    auto p=Interpolate(*lane,5);raw.prediction_lane_anchor_x=p.x-.3*std::sin(p.theta);
    raw.prediction_lane_anchor_y=p.y+.3*std::cos(p.theta);raw.prediction_lane_heading=p.theta;
    Check(raw,"lane model preserves measured offset, heading and rigid corners after alignment");
    raw.prediction_lane_s=95.5;p=Interpolate(*lane,95.5);
    raw.prediction_lane_anchor_x=p.x-.3*std::sin(p.theta);raw.prediction_lane_anchor_y=p.y+.3*std::cos(p.theta);
    raw.prediction_lane_heading=p.theta;
    Check(raw,"finite lane successor/tangent behavior is unchanged by alignment");
    raw=Object();raw.motion_state=ObjectMotionState::UNKNOWN;raw.velocity_unavailable=true;
    raw.continuation_vx=30;raw.continuation_vy=4;raw.yaw_rate=.4;
    const auto occupancy=AdvanceObjectObservation(raw,.2);
    Expect(occupancy.x==raw.x&&occupancy.y==raw.y&&occupancy.velocity_unavailable,
        "unmeasured velocity remains observed occupancy, never a measured zero-speed claim");
    bool rejected=false;try{AdvanceObjectObservation(raw,-.1);}catch(const std::invalid_argument&){rejected=true;}
    Expect(rejected,"negative elapsed alignment is rejected");
    std::cout<<checks<<" checks, "<<failures<<" failures\n";return failures?1:0;
}
