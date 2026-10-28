#include "Frenet/Generator/BrakingProfile.hpp"
#include "Frenet/frenet_planner.hpp"
#include "Execution/HighwayReferenceExecution.hpp"
#include <cmath>
#include <iostream>
#include <limits>

int main() {
    int failures=0;
    const auto expect=[&](bool ok,const char* label) {
        std::cout<<(ok?"PASS ":"FAIL ")<<label<<'\n';failures+=!ok;
    };
    for(double acceleration:{1.7072151763538201,-1.3,-5.0}) {
        const auto p=MakeBrakingProfile(14.020497025549023,acceleration,3,6,true);
        expect(p.valid,"achieved highway boundary has a complete jerk-limited brake");
        const auto start=p.sample(0),end=p.sample(p.duration+.01);
        expect(start[0]==0&&std::abs(start[1]-14.020497025549023)<1e-10&&
            std::abs(start[2]-acceleration)<1e-10,"brake preserves measured distance/velocity/acceleration");
        bool continuous=true;
        auto previous=start;
        const double dt=.001;
        for(double t=dt;t<p.duration;t+=dt) {
            const auto x=p.sample(t);
            continuous&=x[0]>=previous[0]-1e-9&&x[1]>=-1e-9&&
                std::abs(x[2]-previous[2])<=6*dt+1e-8;
            if(t>=p.release)continuous&=std::abs(x[2])<=3+1e-8;
            previous=x;
        }
        expect(continuous&&end[1]==0&&end[2]==0,"release and braking obey jerk, normal bounds and nonreverse stop");
    }
    expect(!MakeBrakingProfile(14,-5,3,6).valid,"ordinary callers keep the original boundary contract");
    expect(!MakeBrakingProfile(1,-5,3,6,true).valid,"unreleasable low-speed boundary is rejected");
    expect(!MakeBrakingProfile(14,4,3,6,true).valid,"recovery cannot invent an excessive acceleration maneuver");
    expect(!MakeBrakingProfile(14,std::numeric_limits<double>::quiet_NaN(),3,6,true).valid,
        "nonfinite boundary is rejected");

    FrenetPlanner planner;std::string error;
    expect(planner.Init(FRENET_TEST_CONFIG)&&planner.EnableHighwayMerge(error),"official highway route prepared");
    if(!planner.highway_route())return 1;
    asmc_planner::execution::ManeuverState execution;
    asmc_planner::lane_reference::HighwayReferenceExecution references(planner,FRENET_TEST_CONFIG,execution);
    // Recorded onset of KEEP -> MERGE, before any lane change is allowed.
    CartesianState ego{54.01711466186527,307.5814640204329,-1.1762545648397582,
        -.20389042794704437/14.020497025549023,14.020497025549023,1.7072151763538201};
    const auto& zone=planner.highway_route()->zones[0];
    PlannerCommand command;command.mode=MERGE;command.merge_type=MergeType::HIGHWAY;
    command.source_link_id=zone.config.source_link;
    command.target_link_id=planner.highway_route()->map->lane(zone.config.target_lanes.back())->link_id;
    command.has_target_speed=true;command.target_speed=planner.highway_route()->config.speed_cap_mps;
    CartesianPath path;FrenetPlanner::HighwayMergeResult result;
    bool ok=planner.PlanHighwayMerge(ego,{},command,100,100,path,result,100,
        &references.target(0).line(),&planner.reference());
    expect(ok&&path.x.size()>2,"recorded overspeed curve retains a steerable source-corridor brake");
    expect(!result.ready&&!result.candidate_valid&&!planner.highway_merge_committed(),
        "recovery never grants a lane-change or merge approval");
    if(ok) {
        expect(path.v.front()==ego.v&&path.a.front()==ego.a&&path.kappa.front()==ego.kappa,
            "recovery path starts at the actual measured state");
        double max_curvature=0;
        for(double k:path.kappa)max_curvature=std::max(max_curvature,std::abs(k));
        const double safe_speed=std::sqrt(planner.kinematic_limits().max_lateral_accel/max_curvature);
        bool recovered=false,no_new_excursion=true;
        for(std::size_t i=0;i<path.v.size();++i) {
            const double lateral=path.v[i]*path.v[i]*std::abs(path.kappa[i]);
            if(path.v[i]<=safe_speed&&path.a[i]<=0)recovered=true;
            if(recovered&&lateral>planner.kinematic_limits().max_lateral_accel+1e-9)
                no_new_excursion=false;
        }
        expect(recovered&&no_new_excursion&&path.v.back()==0,
            "full braking reaches the corridor lateral bound and stops without reversing");
    }
    TrackedObject object{};object.id=900;object.x=ego.x;object.y=ego.y;
    object.type=3;object.motion_state=ObjectMotionState::LOW_THREAT;
    object.geometry_valid=object.z_valid=true;object.length=object.width=6;object.height=2;
    for(unsigned i=0;i<8;++i)object.corners[i]={ego.x+(i&1?3.0:-3.0),ego.y+(i&2?3.0:-3.0),i&4?2.0:0.0};
    planner.ResetHighwayMerge();planner.ResetProjectionHistory();
    ok=planner.PlanHighwayMerge(ego,{object},command,101,101,path,result,101,
        &references.target(0).line(),&planner.reference());
    expect(!ok,"a physical collision still rejects the recovery brake");
    std::cout<<"failures="<<failures<<'\n';return failures?1:0;
}
