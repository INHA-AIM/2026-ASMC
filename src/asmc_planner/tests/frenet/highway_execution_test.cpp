#include "Frenet/frenet_planner.hpp"
#include <iostream>
#include <stdexcept>
#include <chrono>
#include <cmath>
#include "Frenet/Geometry/PolygonUtils.hpp"

namespace {
int checks=0,failures=0;
void expect(bool condition,const std::string& message) {
    ++checks;if(!condition){++failures;std::cerr<<"FAIL: "<<message<<'\n';}
}
CartesianState pose(const FrenetPlanner& planner,double s,double speed) {
    const auto p=Interpolate(planner.reference(),s);return {p.x,p.y,p.theta,p.kappa,speed,0};
}
PlannerCommand command(const HighwayMergeRoute& route,std::size_t index) {
    const auto& z=route.zones[index];PlannerCommand c;
    c.mode=MERGE;c.merge_type=MergeType::HIGHWAY;c.has_target_speed=true;
    c.target_speed=route.config.speed_cap_mps;c.source_link_id=z.config.source_link;
    c.target_link_id=route.map->lane(z.config.target_lanes.back())->link_id;return c;
}
void runZone(std::size_t index,double initial_speed) {
    FrenetPlanner planner;std::string error;
    if(!planner.Init(FRENET_TEST_CONFIG))throw std::runtime_error("core init failed");
    expect(!planner.highway_merge_enabled(),"ordinary Init never enables highway policy");
    if(!planner.EnableHighwayMerge(error))throw std::runtime_error(error);
    const auto route=planner.highway_route();const auto& z=route->zones[index];
    // Begin in a physically stoppable SOURCE state. The former fixture put
    // a 12 m/s car only a few metres from its unapproved source frontier.
    const double local=initial_speed>0?z.source_stop_local_s-
        initial_speed*initial_speed/(2*route->approach_decel_mps2)-initial_speed*2-10:
        z.source_stop_local_s-1;
    const auto origin=Interpolate(z.source_reference,local);
    const double s=FindClosestS(planner.reference(),origin.x,origin.y,nullptr,100);
    CartesianState ego{origin.x,origin.y,origin.theta,origin.kappa,initial_speed,0};auto c=command(*route,index);
    FrenetPlanner::HighwayMergeResult result;CartesianPath path;
    double now=100;
    for(int i=0;i<3;++i) {
        expect(planner.PlanHighwayMerge(ego,{},c,now,100,path,result),"fresh empty-road request has source maintenance");
        expect(!result.ready,"one sensor observation cannot become three confirmations");
        now+=.05;
    }
    // Wall-clock search exhaustion must fail closed. It is allowed under CI
    // contention, so await three consecutive fresh safe observations rather
    // than assuming every requested tick completed its certificate in time.
    for(int i=0;i<20&&!result.ready;++i) {
        now=100.2+.05*i;
        expect(planner.PlanHighwayMerge(ego,{},c,now,now,path,result),"independent source observation can plan");
    }
    expect(result.candidate_valid&&result.ready,"rolling and stopped departures both produce a full-traversal candidate");
    if(!result.ready){std::cerr<<"zone="<<index<<" speed="<<initial_speed<<" detail="<<result.detail<<'\n';return;}
    expect(planner.CanCommitHighwayMerge(result),"current READY can be committed");
    const auto old=result;
    now+=.05;planner.PlanHighwayMerge(ego,{},c,now,now,path,result);
    expect(!planner.CanCommitHighwayMerge(old),"previous candidate epoch cannot commit after replanning");
    for(int i=0;i<20&&!result.ready;++i) {
        now+=.05;planner.PlanHighwayMerge(ego,{},c,now,now,path,result);
    }
    expect(planner.CanCommitHighwayMerge(result),"latest candidate remains commit eligible");
    planner.ApplyHighwayMergeCommit(result);
    expect(planner.highway_merge_committed(),"commit keeps highway execution ownership");
    // A physical blocker makes both forward motion and braking unsafe.
    // This must not become INVALID_COMMAND/terminal and restart a new entry.
    TrackedObject blocker;blocker.id=2058;blocker.type=3;
    blocker.motion_state=ObjectMotionState::LOW_THREAT;
    blocker.x=ego.x;blocker.y=ego.y;blocker.width=4;blocker.length=8;blocker.height=2;
    for(int k=0;k<8;++k) {
        const double u=(k%4==0||k%4==3)?-4:4;
        const double v=(k%4<2)?-2:2;
        blocker.corners[k]={ego.x+u*std::cos(ego.yaw)-v*std::sin(ego.yaw),
                           ego.y+u*std::sin(ego.yaw)+v*std::cos(ego.yaw),k<4?0.0:2.0};
    }
    blocker.geometry_valid=geometry::ValidateCorners8(blocker.corners).valid;
    expect(blocker.geometry_valid,"blocked-execution fixture has authoritative valid OBB");
    now+=.05;
    expect(!planner.PlanHighwayMerge(ego,{blocker},c,now,now,path,result),"physical overlap never produces an unchecked driving path");
    expect(planner.last_failure_reason()!=FrenetPlanner::PlanFailureReason::INVALID_COMMAND,
           "post-commit collision is recoverable, never an invalid command");
    expect(planner.highway_merge_committed(),"blocked execution retains the committed merge owner");
    bool resumed=false;
    for(int i=0;i<20&&!resumed;++i) {
        now+=.05;
        resumed=planner.PlanHighwayMerge(ego,{},c,now,now,path,result)&&result.candidate_valid;
    }
    expect(resumed&&planner.highway_merge_committed(),"cleared blocker resumes the same merge without another PREPARE/commit");
    auto invalid=c;invalid.source_link_id="wrong-source";now+=.05;
    expect(!planner.PlanHighwayMerge(ego,{},invalid,now,now,path,result)&&
           planner.last_failure_reason()==FrenetPlanner::PlanFailureReason::INVALID_COMMAND,
           "a genuinely invalid source identity remains rejected");
    const auto reference_before=planner.reference().length;
    bool completed=false;int ticks=0;double maximum_ms=0,maximum_speed=ego.v;
    for(;ticks<450;++ticks) {
        now+=planner.sample_dt();
        const auto start=std::chrono::steady_clock::now();
        if(!planner.PlanHighwayMerge(ego,{},c,now,now,path,result))break;
        maximum_ms=std::max(maximum_ms,std::chrono::duration<double,std::milli>(
            std::chrono::steady_clock::now()-start).count());
        if(result.maneuver_completed){completed=true;break;}
        if(path.x.size()<2)break;
        ego={path.x[1],path.y[1],path.yaw[1],path.kappa[1],path.v[1],path.a[1]};
        maximum_speed=std::max(maximum_speed,ego.v);
    }
    expect(completed,"ideal candidate follower reaches aligned, rear-clear completion");
    expect(planner.reference().length==reference_before,"existing target reference and mission stations are retained");
    const auto after=HighwayApproachAt(*route,ego);
    expect(!after.approach,"completed station cannot immediately request the same merge again");
    std::cout<<"zone="<<index<<" initial_speed="<<initial_speed<<" ticks="<<ticks
        <<" maximum_speed_kph="<<maximum_speed*3.6<<" maximum_ms="<<maximum_ms<<'\n';
    planner.ResetHighwayMerge();
    PlannerCommand keep;keep.mode=LANE_KEEPING;keep.has_target_speed=true;keep.target_speed=10;
    expect(planner.Plan(ego,{},keep,path),"normal KEEP continuation works after completion");
    ego={origin.x,origin.y,origin.theta,origin.kappa,initial_speed,0};now+=1;
    planner.PlanHighwayMerge(ego,{},c,now,now,path,result);
    const auto pending=result;
    expect(!planner.PlanHighwayMerge(ego,{},c,now+.5,now,path,result),"stale perception cannot certify even an empty road");
    expect(!planner.CanCommitHighwayMerge(pending),"stale rejection invalidates pending token");
    expect(!planner.PlanHighwayMerge(ego,{},c,now,now+1,path,result),"future source stamps fail closed");
}
}
int main() {
    CurveSpeedConfig ordinary{};ordinary.target_vel=58.0/3.6;
    ordinary.lateral_accel_safety_factor=.95;ordinary.comfortable_decel_factor=.65;
    HighwayMergeConfig highway{};highway.speed_cap_mps=70.0/3.6;
    PlannerCommand c;c.mode=MERGE;c.merge_type=MergeType::HIGHWAY;
    const auto speed=ResolveHighwayCurveSpeed(ordinary,highway,true,c);
    expect(std::fabs(speed.target_vel-70.0/3.6)<1e-9,"enabled highway uses its own 70 km/h ceiling");
    expect(speed.lateral_accel_safety_factor==.8&&speed.comfortable_decel_factor==.5,
           "highway preserves its pre-rebase validated envelope");
    expect(ordinary.lateral_accel_safety_factor==.95&&ordinary.comfortable_decel_factor==.65,
           "highway policy cannot mutate main ordinary curve tuning");
    expect(ResolveHighwayCurveSpeed(ordinary,highway,false,c).target_vel==ordinary.target_vel,
           "disabled highway retains main ordinary ceiling");
    c.merge_type=MergeType::ROUNDABOUT;
    expect(ResolveHighwayCurveSpeed(ordinary,highway,true,c).target_vel==ordinary.target_vel,
           "roundabout retains main ordinary ceiling");
    for(auto mode:{LANE_KEEPING,FOLLOWING,AVOID,STOP}) {
        c.mode=mode;c.merge_type=MergeType::HIGHWAY;
        expect(ResolveHighwayCurveSpeed(ordinary,highway,true,c).target_vel==ordinary.target_vel,
               "ordinary maneuvers cannot inherit highway ceiling");
    }
    try {runZone(0,12);runZone(0,0);runZone(1,12);runZone(1,0);}
    catch(const std::exception& e){++failures;std::cerr<<e.what()<<'\n';}
    std::cout<<checks<<" checks, "<<failures<<" failures\n";return failures?1:0;
}
