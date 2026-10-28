#include "Execution/HighwayReferenceExecution.hpp"
#include <iostream>
#include <stdexcept>
#include <cmath>
using namespace asmc_planner::lane_reference;
namespace {
int checks=0,failures=0;
void expect(bool ok,const char* detail) {++checks;if(!ok){++failures;std::cerr<<"FAIL "<<detail<<'\n';}}
TrackedObject blocker(const CartesianState& ego) {
    TrackedObject o;o.id=8101;o.type=3;o.motion_state=ObjectMotionState::LOW_THREAT;
    o.x=ego.x;o.y=ego.y;o.length=8;o.width=4;o.height=2;o.z=1;o.z_valid=true;
    int i=0;for(double z:{0.,2.})for(auto p:std::vector<std::pair<double,double>>{{-4,-2},{4,-2},{4,2},{-4,2}})
        o.corners[i++]={ego.x+p.first*std::cos(ego.yaw)-p.second*std::sin(ego.yaw),
            ego.y+p.first*std::sin(ego.yaw)+p.second*std::cos(ego.yaw),z};
    o.geometry_valid=geometry::ValidateCorners8(o.corners).valid;return o;
}
void recovery() {
    FrenetPlanner planner;std::string error;
    if(!planner.Init(FRENET_TEST_CONFIG)||!planner.EnableHighwayMerge(error))throw std::runtime_error(error);
    asmc_planner::execution::ManeuverState state;
    HighwayReferenceExecution references(planner,FRENET_TEST_CONFIG,state);
    const auto route=planner.highway_route();const auto& zone=route->zones[0];
    PlannerCommand c;c.mode=MERGE;c.merge_type=MergeType::HIGHWAY;c.has_target_speed=true;
    c.target_speed=route->config.speed_cap_mps;c.source_link_id=zone.config.source_link;
    c.target_link_id=route->map->lane(zone.config.target_lanes.back())->link_id;
    const auto& target=references.target(0).line();
    // First failed live run: the car is still finishing the approach curve at
    // commitment. These are measured map-frame states, not perfect followers.
    CartesianState ego{62.0719120215,253.841615994,-1.55894979524,-.000982481015,15.0557255785,.481016385829};
    CartesianPath path;FrenetPlanner::HighwayMergeResult result;double now=100;
    for(int n=0;n<15&&!result.ready;++n){now+=.05;planner.PlanHighwayMerge(ego,{},c,now,now,path,result,now,&target,&planner.reference());}
    expect(result.ready&&result.candidate_valid,"recorded rolling departure has a current certificate");
    if(!result.ready)return;
    planner.ApplyHighwayMergeCommit(result);const double original_terminal=result.transition_terminal_s;
    const std::vector<CartesianState> observed{
        {62.081524,252.991915,-1.560012,-.017629/15.09618,15.09618,.834933},
        {62.088583,252.316209,-1.560681,-.015728/15.135756,15.135756,.910109},
        {62.079227,250.864925,-1.562037,-.011266/15.116891,15.116891,1.068689},
        {62.085814,250.110500,-1.562094,.006781/15.163602,15.163602,.872372}};
    for(const auto& actual:observed){
        const double before=result.transition_terminal_s;
        now+=.05;const bool ok=planner.PlanHighwayMerge(actual,{},c,now,now,path,result,now,&target,&planner.reference());
        expect(ok&&result.candidate_valid,"measured early tracking lag retains a certified driving path");
        expect(!result.candidate_valid||result.check.covers_completion,"retained/replanned path includes full target arrival and tail");
        if(result.transition_reused)expect(result.transition_terminal_s==before,"retained geometry never advances its endpoint");
    }
    // If braking has already carried the source-lane car past the old arrival,
    // recover with a reachable connector rather than forcing a 2 m lane change.
    ego={62.526,184.784,-1.5688,0,0,0};bool recovered=false;
    for(int n=0;n<10&&!recovered;++n){
        now+=.05;planner.PlanHighwayMerge(ego,{},c,now,now,path,result,now,&target,&planner.reference());
        recovered=result.candidate_valid&&result.transition_recovered;
    }
    expect(recovered,"stopped car beyond old endpoint can recover under the same committed transaction");
    expect(planner.highway_merge_committed(),"recovery never restarts PREPARE or relinquishes ownership");
    if(!recovered)return;
    expect(result.transition_terminal_s>original_terminal&&result.transition_terminal_s<=zone.config.completion_s,
        "recovery endpoint is reachable ahead and inside the mapped merge horizon");
    double largest=0;for(double k:path.kappa)largest=std::max(largest,std::abs(k));
    expect(largest<=planner.kinematic_limits().max_curvature,"recovery does not relax steering curvature");
    const double terminal=result.transition_terminal_s;
    now+=.05;planner.PlanHighwayMerge(ego,{},c,now,now,path,result,now,&target,&planner.reference());
    expect(result.candidate_valid&&result.transition_reused&&result.transition_terminal_s==terminal,
        "accepted recovery is retained instead of moving its finish every tick");
    now+=.05;
    expect(!planner.PlanHighwayMerge(ego,{blocker(ego)},c,now,now,path,result,now,&target,&planner.reference()),
        "new physical overlap blocks retained geometry and every recovery candidate");
    expect(!result.candidate_valid,"blocked curve is never replayed as a stale fallback");
    now+=.05;planner.PlanHighwayMerge(ego,{},c,now,now,path,result,now,&target,&planner.reference());
    expect(result.candidate_valid,"cleared blocker allows the same committed maneuver to resume");
    expect(!planner.PlanHighwayMerge(ego,{},c,now+.5,now,path,result,now,&target,&planner.reference())&&
        !result.candidate_valid,"stale input cannot reuse or recover a connector");
    expect(references.begin(12,0),"recovery uses the same reference-arrival gate");
    now+=.6;
    planner.PlanHighwayMerge(ego,{},c,now,now,path,result,now,&target,&planner.reference());
    bool adopted=false;
    for(int n=0;n<200&&!adopted&&path.x.size()>1;++n) {
        ego={path.x[1],path.y[1],path.yaw[1],path.kappa[1],path.v[1],path.a[1]};
        now+=planner.sample_dt();
        if(!planner.PlanHighwayMerge(ego,{},c,now,now,path,result,now,&target,&planner.reference())||!result.candidate_valid)break;
        ReferenceExecutionInput i;i.map=route->map->identity();i.sequence=n+1;
        i.observation={now,now,now,true,true,ego};
        if(references.prepare(i,path,c.target_speed,error))adopted=references.commit(i,[]{return true;},error);
    }
    expect(adopted&&references.status().completed[0],
        "ideal tracking of a recovered connector reaches real arrival and reference adoption");
    planner.ResetHighwayMerge();
    expect(!planner.highway_merge_committed(),"reset clears retained geometry ownership");
}
void kinematics() {
    // Stored curves receive the complete existing kinematic checks again.
    CartesianPath p;p.x={0,1,2};p.y={0,0,0};p.yaw={0,0,0};p.kappa={0,0,1};
    p.v={5,5,5};p.a={0,0,0};FrenetPath m;m.s_ddd={0,0,0};
    KinematicLimits limits{};limits.max_curvature=.23;limits.max_lateral_accel=2.6;
    limits.max_longitudinal_accel=3;limits.max_longitudinal_jerk=6;
    p.kappa[2]=0;
    expect(HighwayTransitionKinematicsValid(p,m,limits,3),"retimed straight geometry passes full kinematic checks");
    p.kappa[2]=1;
    expect(!HighwayTransitionKinematicsValid(p,m,limits,3),"a curvature violation anywhere in the path rejects reuse");
    p.kappa[1]=.2;expect(!HighwayTransitionKinematicsValid(p,m,limits,2),"published prefix retains lateral acceleration limit");
}
}
int main(){try{kinematics();recovery();}catch(const std::exception& e){++failures;std::cerr<<e.what()<<'\n';}
    std::cout<<checks<<" checks, "<<failures<<" failures\n";return failures?1:0;}
