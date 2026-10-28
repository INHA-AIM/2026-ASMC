#include "Execution/HighwayReferenceExecution.hpp"
#include <iostream>
#include <stdexcept>
#include <cmath>
using namespace asmc_planner::lane_reference;
namespace {
int checks=0,failures=0;
void expect(bool ok,const char* what) {++checks;if(!ok){++failures;std::cerr<<"FAIL "<<what<<'\n';}}
CartesianState at(const RefLine& line,double s,double speed) {
    const auto p=Interpolate(line,s);return {p.x,p.y,p.theta,p.kappa,speed,0};
}
void follow(CartesianState& ego,const CartesianPath& p) {
    if(p.x.size()<2)throw std::runtime_error("no next trajectory sample");
    ego={p.x[1],p.y[1],p.yaw[1],p.kappa[1],p.v[1],p.a[1]};
}
ReferenceExecutionInput input(const HighwayMergeRoute& r,const CartesianState& ego,int tick,double now) {
    ReferenceExecutionInput i;i.map=r.map->identity();i.sequence=tick+1;
    i.observation={now,now,now,true,true,ego};return i;
}
void exercise(double initial_speed) {
    FrenetPlanner mission;asmc_planner::execution::ManeuverState state;
    mission.BindExecutionState(state);std::string error;
    if(!mission.Init(FRENET_TEST_CONFIG)||!mission.EnableHighwayMerge(error))throw std::runtime_error(error);
    HighwayReferenceExecution execution(mission,FRENET_TEST_CONFIG,state);
    const auto route=mission.highway_route();double now=100;int tick=0;
    CartesianState last_ego;CartesianPath last_path;
    for(std::size_t zone=0;zone<route->zones.size();++zone) {
        mission.ResetHighwayMerge();const auto& z=route->zones[zone];
        PlannerCommand c;c.mode=MERGE;c.merge_type=MergeType::HIGHWAY;
        c.source_link_id=z.config.source_link;
        c.target_link_id=route->map->lane(z.config.target_lanes.back())->link_id;
        c.has_target_speed=true;c.target_speed=route->config.speed_cap_mps;
        // Zone 2 begins on the lane already adopted in zone 1. No automatic
        // jump back to the original reference is allowed between transitions.
        auto ego=at(execution.planner().reference(),FindClosestS(execution.planner().reference(),
            Interpolate(route->reference,z.config.start_s+5).x,
            Interpolate(route->reference,z.config.start_s+5).y,nullptr,100),initial_speed);
        expect(execution.begin(10+zone,zone),"bind Behavior transaction to its prebuilt target");
        FrenetPlanner::HighwayMergeResult result;CartesianPath path;
        for(int n=0;n<30&&!result.ready;++n) {
            now+=.05;++tick;
            mission.PlanHighwayMerge(ego,{},c,now,now,path,result,now,
                &execution.target(zone).line(),&execution.planner().reference());
        }
        std::cerr<<"zone="<<zone<<" ready="<<result.ready<<" detail="<<result.detail<<'\n';
        expect(result.ready&&result.candidate_valid,"actual LC has full-body arrival and checked continuation before READY");
        if(!result.ready)continue;
        expect(result.check.covers_completion&&result.check.clear_time_s>=0,"gap certificate includes actual target arrival");
        expect(result.check.clear_time_s<route->config.max_horizon_s-route->config.post_merge_horizon_s,
            "lane arrival is certified without waiting for the mission-zone endpoint");
        expect(mission.CanCommitHighwayMerge(result),"only current candidate certificate commits");
        mission.ApplyHighwayMergeCommit(result);path=result.candidate_path;
        bool adopted=false;int callbacks=0;
        for(int n=0;n<200&&!adopted;++n) {
            follow(ego,path);now+=mission.sample_dt();++tick;
            if(!mission.PlanHighwayMerge(ego,{},c,now,now,path,result,now,
                &execution.target(zone).line(),&execution.planner().reference())) {
                std::cerr<<"execution failure "<<result.detail<<'\n';break;
            }
            auto i=input(*route,ego,tick,now);
            const auto prior_state=state;
            state.follow={true,777,0};
            const bool prepared=execution.prepare(i,path,c.target_speed,error);
            if(prepared)expect(state.follow.active&&state.follow.leader_id==777,
                "preparing a dormant core cannot mutate the active execution state");
            state=prior_state;
            if(n%20==0)std::cerr<<"zone="<<zone<<" tick="<<n<<" handover="<<error<<" path="<<result.detail<<'\n';
            if(prepared) {
                auto changed=i;changed.sequence++;
                expect(!execution.commit(changed,[&]{++callbacks;return true;},error),
                    "changed snapshot cannot commit an otherwise prepared reference");
                // A new source pose must produce a new preparation certificate.
                follow(ego,path);now+=mission.sample_dt();++tick;
                mission.PlanHighwayMerge(ego,{},c,now,now,path,result,now,
                    &execution.target(zone).line(),&execution.planner().reference());
                i=input(*route,ego,tick,now);
                if(!execution.prepare(i,path,c.target_speed,error))continue;
                const auto next=execution.preparedFrame()->path;
                expect(!execution.commit(i,[]{return false;},error),"rejected manager completion retains source reference");
                // Same source pose cannot advance settle; next cycle retries.
                follow(ego,path);now+=mission.sample_dt();++tick;
                mission.PlanHighwayMerge(ego,{},c,now,now,path,result,now,
                    &execution.target(zone).line(),&execution.planner().reference());
                i=input(*route,ego,tick,now);
                if(!execution.prepare(i,path,c.target_speed,error))continue;
                path=execution.preparedFrame()->path;
                adopted=execution.commit(i,[&]{++callbacks;return true;},error);
            }
        }
        std::cerr<<"zone="<<zone<<" adopted="<<adopted<<" detail="<<error<<'\n';
        expect(adopted&&callbacks==1,"one completion atomically adopts target core and checked continuation");
        if(!adopted)continue;
        expect(execution.status().completed[zone]&&execution.status().lane,
            "Behavior receives adopted reference and completed-zone suppression together");
        expect(&execution.planner()!=&mission,"KEEP/FOLLOW use adopted core after completion");
        PlannerCommand keep;keep.mode=LANE_KEEPING;keep.has_target_speed=true;keep.target_speed=c.target_speed;
        CartesianPath continuation;
        execution.planner().SetObjectObservationTime(now);
        expect(execution.planner().Plan(ego,{},keep,continuation),"ordinary KEEP plans on the adopted lane");
        if(!continuation.x.empty()) {
            const auto p=ProjectOnLaneReference(execution.target(zone),continuation.x.back(),
                continuation.y.back(),continuation.yaw.back(),continuation.v.back());
            expect(p.valid&&std::abs(p.d)<.5,"KEEP never steers back to the former mission lane");
        }
        auto guard=keep;const auto& lane=execution.target(zone);
        execution.guardFiniteEnd(at(lane.line(),lane.usableEnd()-10,10),guard);
        expect(guard.mode==STOP&&std::abs(guard.stop_position-lane.usableEnd())<1e-6,
            "unavailable mission return cannot extrapolate beyond a finite lane reference");
        last_ego=ego;last_path=continuation;
    }
    PlannerCommand keep;keep.mode=LANE_KEEPING;keep.has_target_speed=true;keep.target_speed=route->config.speed_cap_mps;
    bool restored=false;
    for(int n=0;n<500&&!restored&&!last_path.x.empty();++n) {
        follow(last_ego,last_path);now+=mission.sample_dt();++tick;
        execution.planner().SetObjectObservationTime(now);
        if(!execution.planner().Plan(last_ego,{},keep,last_path))break;
        auto i=input(*route,last_ego,tick,now);
        if(execution.prepareMissionReturn(i,last_path,keep.target_speed,error)) {
            expect(execution.preparedFrame()->ego.s>1500,
                "mission-return ego/object projections use the actual restored reference, not common-lane local s");
            const auto next=execution.preparedFrame()->path;
            restored=execution.commitMissionReturn(i,error);if(restored)last_path=next;
        }
    }
    std::cerr<<"mission restored="<<restored<<" detail="<<error<<'\n';
    expect(restored && &execution.planner()==&mission,"common downstream lane permits a continuous return to the immutable mission core");
    expect(!execution.status().lane&&mission.reference().closed_loop,"finite reference never traps subsequent ordinary driving");
    expect(execution.status().completed[0]&&execution.status().completed[1],"mission return preserves completed-zone suppression");
    execution.observeMissionStation(route->reference.length*.9);
    execution.observeMissionStation(route->reference.length*.1);
    expect(!execution.status().completed[0]&&!execution.status().completed[1],"next lap reopens zone eligibility only after mission wrap");
    expect(execution.begin(10,0),"dormant execution can bind even a reused command ID in the next lap");
}
}
int main() {try{exercise(12);exercise(0);}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
    std::cout<<checks<<" checks, "<<failures<<" failures\n";return failures?1:0;}
