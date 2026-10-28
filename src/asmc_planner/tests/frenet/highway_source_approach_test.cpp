#include "Frenet/frenet_planner.hpp"
#include "Frenet/Maneuver/Merge/HighwayPrediction.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
int checks=0,failures=0;
void expect(bool x,const char* label) {++checks;if(!x){++failures;std::cerr<<"FAIL: "<<label<<'\n';}}
RefLine line(double yaw) {
    RefLine r;r.length=500;
    for(double s=0;s<=500;s+=.5)r.points.push_back({s*std::cos(yaw),s*std::sin(yaw),s,yaw,0,0});
    return r;
}
void prediction() {
    HighwayMergeConfig cfg{};cfg.speed_error_mps=5;cfg.speed_error_floor_mps=1;
    cfg.motion_fit_min_span_s=.2;cfg.min_heading_alignment=.5;
    for(double yaw:{0.0,-M_PI/2,0.7}) {
        auto lane=line(yaw);const double tx=std::cos(yaw),ty=std::sin(yaw);
        TrackedObject object;object.motion_state=ObjectMotionState::DYNAMIC;
        object.vx=12*tx+.03*(-ty);object.vy=12*ty+.03*tx;
        ObservedObjectMotion track;
        // Same four-frame jitter pattern as the recorded straight actor:
        // a short fit invents about -1 m/s transverse motion.
        const double times[]={-.264,-.167,-.067,0};
        const double offsets[]={3.8864,3.7261,3.5806,3.6523};
        for(int i=0;i<4;++i) {
            object.x=(100+13*times[i])*tx-offsets[i]*ty;
            object.y=(100+13*times[i])*ty+offsets[i]*tx;
            track.Observe(object,100+times[i],1);
        }
        const auto fitted=PrepareHighwayObjectPrediction(object,&track,cfg,&lane);
        expect(std::abs(fitted.vx*tx+fitted.vy*ty-13)<1e-5,"position history refines longitudinal speed");
        expect(std::abs(-fitted.vx*ty+fitted.vy*tx-.03)<1e-6,"center jitter cannot overwrite observed transverse motion");
        expect(std::abs(fitted.prediction_max_speed-fitted.speed-1)<1e-6,"speed residual floor is retained");
        const auto future=PredictTrackedObjectPose(fitted,2.1);
        expect(std::abs((-future.x*ty+future.y*tx)-(offsets[3]+.03*cfg.speed_error_correlation_s))<1e-5,
            "straight actor is not extrapolated two metres into ego lane");
        auto cutin=object;cutin.vx=12*tx+1.2*ty;cutin.vy=12*ty-1.2*tx;
        const auto crossing=PrepareHighwayObjectPrediction(cutin,&track,cfg,&lane);
        expect(std::abs(-crossing.vx*ty+crossing.vy*tx+1.2)<1e-6,"real transverse intrusion remains predicted");
        auto stationary=cutin;stationary.motion_state=ObjectMotionState::LOW_THREAT;
        const auto fixed=PrepareHighwayObjectPrediction(stationary,&track,cfg,&lane);
        const auto at=PredictTrackedObjectPose(fixed,10);
        expect(at.x==stationary.x&&at.y==stationary.y,"confirmed static occupancy does not move");
    }
}
void source(std::size_t zi) {
    FrenetPlanner planner;std::string error;
    if(!planner.Init(FRENET_TEST_CONFIG)||!planner.EnableHighwayMerge(error))throw std::runtime_error(error);
    const auto route=planner.highway_route();const auto& z=route->zones[zi];
    expect(z.source_reference.points.size()>3&&bool(z.source_region),"source lane chain is prepared before driving");
    const auto end=Interpolate(z.source_reference,z.source_stop_local_s);
    const asmc_planner::lane_reference::FootprintShape body{route->vehicle.length-route->vehicle.rear_overhang,
        route->vehicle.rear_overhang,route->vehicle.width*.5};
    expect(z.source_region->coversFootprint(end.x,end.y,end.theta,body,0),"last stop contains the complete car in source lane");
    expect(planner.RuntimeRoadFootprintClear(end.x,end.y,end.theta),"last stop passes production full-body road geometry");
    expect(z.source_stop_s>z.config.start_s+10,"source frontier leaves a usable rolling approach");
    if(zi==1)expect(z.source_stop_s>z.entry_s+50,"long source taper remains usable beyond original global-path entry");
    // Never approve a merge: follow PREPARE source outputs to the actual end.
    const auto begin=Interpolate(route->reference,z.config.start_s+5);
    double station=FindClosestS(z.source_reference,begin.x,begin.y,nullptr,100);
    const auto p=Interpolate(z.source_reference,station);
    CartesianState ego{p.x,p.y,p.theta,p.kappa,10,0};
    PlannerCommand c;c.mode=MERGE;c.merge_type=MergeType::HIGHWAY;c.has_target_speed=true;
    c.target_speed=route->config.speed_cap_mps;c.source_link_id=z.config.source_link;
    c.target_link_id=route->map->lane(z.config.target_lanes.back())->link_id;
    double now=100,progress=0;bool safe=true,stopped=false;double max_decel=0;
    for(int tick=0;tick<400;++tick) {
        CartesianPath path;FrenetPlanner::HighwayMergeResult result;
        now+=.1;
        if(!planner.PlanHighwayMerge(ego,{},c,now,now,path,result,now,nullptr,&z.source_reference)||path.x.size()<2){safe=false;std::cerr<<"source "<<zi<<" t="<<tick<<" "<<result.detail<<'\n';break;}
        if(tick==0)expect(*std::max_element(path.v.begin(),path.v.end())>ego.v+.5,
            "empty source approach prefers useful acceleration rather than a fixed low cruise");
        for(std::size_t k=0;k<path.x.size();++k) {
            const double s=FindClosestS(z.source_reference,path.x[k],path.y[k],nullptr,100);
            safe&=s<=z.source_stop_local_s+1e-3;
            safe&=planner.RuntimeRoadFootprintClear(path.x[k],path.y[k],path.yaw[k]);
        }
        ego={path.x[1],path.y[1],path.yaw[1],path.kappa[1],path.v[1],path.a[1]};
        max_decel=std::max(max_decel,-ego.a);
        const double next=FindClosestS(z.source_reference,ego.x,ego.y,nullptr,100);
        progress=next-station;
        if(ego.v<.05&&z.source_stop_local_s-next<.3){stopped=true;break;}
    }
    expect(safe,"PREPARE forward paths retain full-body road checks and never cross source end");
    expect(progress>20,"no merge approval still permits rolling source-lane progress");
    expect(stopped,"permanently unapproved merge stops at final source boundary");
    expect(!planner.highway_merge_committed(),"approach progress cannot grant its own merge commit");
    std::cout<<"SOURCE zone="<<zi<<" stop_mission_s="<<z.source_stop_s<<" progress="<<progress<<" goal_projection_error="<<FindClosestS(z.source_reference,end.x,end.y,nullptr,100)-z.source_stop_local_s<<" remaining="<<z.source_stop_local_s-FindClosestS(z.source_reference,ego.x,ego.y,nullptr,100)<<" v="<<ego.v<<" a="<<ego.a<<" max_decel="<<max_decel<<'\n';
    planner.ResetHighwayMerge();
    planner.ResetProjectionHistory();
    ego={p.x,p.y,p.theta,p.kappa,0,0};bool restarted=true;double forward_accel=0;
    for(int tick=0;tick<20;++tick) {
        CartesianPath path;FrenetPlanner::HighwayMergeResult result;now+=.1;
        if(!planner.PlanHighwayMerge(ego,{},c,now,now,path,result,now,nullptr,&z.source_reference)||path.x.size()<2) {
            restarted=false;break;
        }
        for(double a:path.a)forward_accel=std::max(forward_accel,a);
        ego={path.x[1],path.y[1],path.yaw[1],path.kappa[1],path.v[1],path.a[1]};
    }
    expect(restarted&&ego.v>1,"stopped PREPARE regains useful speed without a merge approval");
    expect(forward_accel<=route->config.forward_accel_mps2+1e-6,
        "approach restart uses measured positive response independently of service braking");
    expect(!planner.highway_merge_committed(),"speed recovery does not grant its own COMMIT");
}
void robust_prediction() {
    HighwayMergeConfig cfg{};cfg.motion_fit_min_span_s=.2;cfg.min_heading_alignment=.5;
    auto lane=line(0);TrackedObject object;object.motion_state=ObjectMotionState::DYNAMIC;
    object.vx=15;object.vy=.02;
    auto initial=PrepareHighwayObjectPrediction(object,nullptr,cfg,&lane);
    expect(std::abs(initial.prediction_max_speed-std::hypot(15,.02)-1)<1e-6,
        "new-track prior is explicit and no longer an unmeasured five m/s bound");
    ObservedObjectMotion track;
    for(int i=0;i<6;++i) {
        object.x=100+15*i*.08+(i==1?1.0:0);object.y=0;
        object.vx=15+(i%2?-.05:.05);
        track.Observe(object,100+i*.08,1);
    }
    auto predicted=PrepareHighwayObjectPrediction(object,&track,cfg,&lane);
    expect(std::abs(predicted.vx-15)<1e-6,"robust longitudinal fit rejects an isolated OBB-center jump");
    expect(predicted.prediction_max_speed-predicted.speed<.5,
        "consistent coflow observations replace the initial prior with observed error");
    expect(predicted.vy==object.vy,"robust speed refinement preserves lateral intrusion evidence");
    for(auto& sample:track.samples)sample.vx+=(int(std::round(sample.time*100))%3-1)*3;
    predicted=PrepareHighwayObjectPrediction(object,&track,cfg,&lane);
    expect(predicted.prediction_max_speed-predicted.speed>=cfg.speed_error_floor_mps,
        "observed velocity variation retains a nonzero uncertainty");
    track={};object.vx=15;object.vy=-.2;
    for(int i=0;i<6;++i) {
        object.x=100+15*i*.08;object.y=2+.3*i*.08;
        track.Observe(object,110+i*.08,1);
    }
    predicted=PrepareHighwayObjectPrediction(object,&track,cfg,&lane);
    expect(std::isfinite(predicted.turn_prediction_duration),
        "small transverse velocity contradicted by positions is not an indefinite cut-in");
    const auto bounded=PredictTrackedObjectPose(predicted,3);
    expect(std::abs(bounded.y-object.y+.2*cfg.speed_error_correlation_s)<1e-6,
        "provisional lateral observation is retained over its noise window");
    for(int i=0;i<4;++i) {
        const double dx=i==0||i==3?-2:2,dy=i<2?-.9:.9;
        predicted.corners[i]={object.x+dx,object.y+dy,0};
        predicted.corners[i+4]={object.x+dx,object.y+dy,1};
    }
    predicted.geometry_valid=true;
    const auto hull=PredictHighwayObstacleFootprint(predicted,3,cfg.speed_error_floor_mps,cfg);
    double lo=INFINITY,hi=-INFINITY;
    for(const auto& p:hull){lo=std::min(lo,p.x);hi=std::max(hi,p.x);}
    expect(hi-lo>5.49,"longitudinal uncertainty persists over the complete continued trajectory");
    track={};object.vy=.2;
    for(int i=0;i<10;++i) {
        object.x=100+15*i*.08;object.y=2+.2*i*.08;
        track.Observe(object,120+i*.08,1,.05,12,.8);
    }
    predicted=PrepareHighwayObjectPrediction(object,&track,cfg,&lane);
    expect(!std::isfinite(predicted.turn_prediction_duration)&&
        std::abs(PredictTrackedObjectPose(predicted,3).y-object.y-.6)<1e-6,
        "even a slow position-supported transverse intrusion remains predicted");
    track={};const double heading=.1;
    object.vx=12*std::cos(heading);object.vy=12*std::sin(heading);
    for(int i=0;i<6;++i) {
        object.x=(100+13*i*.08)*std::cos(heading);
        object.y=(100+13*i*.08)*std::sin(heading);
        track.Observe(object,130+i*.08,1);
    }
    predicted=PrepareHighwayObjectPrediction(object,&track,cfg,nullptr,heading);
    expect(std::abs(-predicted.vx*std::sin(heading)+predicted.vy*std::cos(heading))<1e-6,
        "a parallel actor follows its own mapped lane rather than ego's differently angled target lane");
}
}
int main(){prediction();robust_prediction();try{source(0);source(1);}catch(const std::exception& e){++failures;std::cerr<<e.what()<<'\n';}
std::cout<<checks<<" checks, "<<failures<<" failures\n";return failures?1:0;}
