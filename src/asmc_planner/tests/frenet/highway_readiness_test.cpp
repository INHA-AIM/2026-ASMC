#include "Execution/HighwayReferenceExecution.hpp"
#include "Frenet/Geometry/PolygonUtils.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
int checks=0,failures=0;
void expect(bool ok,const char* message) {
    ++checks;if(!ok){++failures;std::cerr<<"FAIL: "<<message<<'\n';}
}
void policy() {
    HighwayReadiness evidence;
    expect(!evidence.ObserveSafe(10,10,3,.3),"first observation cannot authorize READY");
    expect(!evidence.ObserveSafe(10.01,10,3,.3),"duplicate observation is not a confirmation");
    expect(!evidence.ObserveSafe(10.05,10.05,3,.3),"two observations are insufficient");
    expect(evidence.ObserveSafe(10.1,10.1,3,.3),"three independent observations confirm");
    expect(!evidence.WasIssued(10.1,.3),"core READY alone is not a published handshake");
    evidence.MarkIssued(10.11);
    evidence.Expire(10.15,.3); // A computational timeout: no new safe observation.
    expect(evidence.WasIssued(10.15,.3),"short unknown interval preserves handshake evidence");
    expect(evidence.confirmations==3,"unknown does not add or destroy fresh confirmations");
    expect(!evidence.WasIssued(10.45,.3),"old READY cannot authorize immediate execution");
    evidence.Expire(10.45,.3);
    expect(!evidence.ObserveSafe(10.45,10.45,3,.3),"expired evidence starts again");
    evidence.Reset();
    expect(!evidence.WasIssued(10.46,.3)&&evidence.confirmations==0,"unsafe/reset revokes both histories");
    evidence.ObserveSafe(11,11,3,.3);evidence.Expire(10,.3);
    expect(evidence.confirmations==0,"clock reversal revokes evidence");
}
void integration() {
    FrenetPlanner planner;std::string error;
    if(!planner.Init(FRENET_TEST_CONFIG)||!planner.EnableHighwayMerge(error))throw std::runtime_error(error);
    asmc_planner::execution::ManeuverState state;
    asmc_planner::lane_reference::HighwayReferenceExecution execution(planner,FRENET_TEST_CONFIG,state);
    const auto route=planner.highway_route();
    // Deterministic safety semantics; production 30 ms is exercised by bag replay.
    auto mutable_route=std::const_pointer_cast<HighwayMergeRoute>(route);
    mutable_route->config.planning_budget_ms=500;
    const auto& zone=route->zones.front();
    const double station=zone.config.start_s+5;
    auto p=Interpolate(planner.reference(),station);
    CartesianState ego{p.x,p.y,p.theta,p.kappa,16,0};
    PlannerCommand command;command.mode=MERGE;command.merge_type=MergeType::HIGHWAY;
    command.has_target_speed=true;command.target_speed=route->config.speed_cap_mps;
    command.source_link_id=zone.config.source_link;
    command.target_link_id=route->map->lane(zone.config.target_lanes.back())->link_id;
    FrenetPlanner::HighwayMergeResult result;CartesianPath path;double now=100;
    auto plan=[&](const std::vector<TrackedObject>& objects,bool execute=false,double age=0) {
        now+=.05;
        return planner.PlanHighwayMerge(ego,objects,command,now,now-age,path,result,now,
            &execution.target(0).line(),&planner.reference(),execute);
    };
    for(int i=0;i<3;++i)expect(plan({}),"fresh preparation maintains source driving");
    expect(result.ready&&result.candidate_valid&&!result.output_requires_commit,"PREPARE does not issue lateral motion");
    planner.MarkHighwayReadyIssued(now+.001);
    const auto old=result;
    expect(planner.HighwayReadyWasIssued(now+.01),"published READY is remembered");
    mutable_route->config.planning_budget_ms=.000001;
    plan({},true);
    expect(!result.ready&&!result.candidate_valid&&!result.output_requires_commit,"timeout grants no current execution path");
    expect(!planner.CanCommitHighwayMerge(old)&&!planner.CanCommitHighwayMerge(result),"timeout invalidates all candidate tokens");
    expect(planner.HighwayReadyWasIssued(now),"short timeout retains only historical handshake");
    mutable_route->config.planning_budget_ms=500;
    p=Interpolate(planner.reference(),station+.5);ego={p.x,p.y,p.theta,p.kappa,16,0};
    expect(plan({},true),"fresh revalidation resumes immediately");
    expect(result.ready&&result.candidate_valid&&result.output_requires_commit,"fresh certificate plus prior READY returns same-cycle commit output");
    expect(path.x==result.candidate_path.x&&!planner.highway_merge_committed(),"runtime must commit before publishing candidate output");
    expect(std::hypot(path.x.front()-ego.x,path.y.front()-ego.y)<.01,"cached speed intent is rebuilt at CURRENT pose");
    expect(planner.CanCommitHighwayMerge(result),"current candidate alone is commit eligible");
    planner.RevokeHighwayMergeReadiness();
    expect(!planner.HighwayReadyWasIssued(now)&&!planner.CanCommitHighwayMerge(result),"cancellation invalidates candidate and handshake");
    for(int i=0;i<3;++i)plan({});
    planner.MarkHighwayReadyIssued(now+.001);
    TrackedObject blocker;blocker.id=42;blocker.type=3;blocker.x=ego.x;blocker.y=ego.y;
    blocker.width=4;blocker.length=8;blocker.height=2;blocker.motion_state=ObjectMotionState::LOW_THREAT;
    const double c=std::cos(ego.yaw),s=std::sin(ego.yaw);
    for(int i=0;i<8;++i) {
        const double x=(i%4==0||i%4==3)?-4:4,y=i%4<2?-2:2;
        blocker.corners[i]={ego.x+c*x-s*y,ego.y+s*x+c*y,i<4?0.0:2.0};
    }
    blocker.geometry_valid=geometry::ValidateCorners8(blocker.corners).valid;
    plan({blocker},true);
    expect(!result.ready&&!result.output_requires_commit&&!planner.HighwayReadyWasIssued(now),"actual new collision revokes evidence immediately");
    expect(plan({})&&!result.ready,"cleared collision must collect confirmations again");
    plan({},false,.5);
    expect(!result.ready&&!planner.HighwayReadyWasIssued(now),"stale inputs revoke historical approval");
    planner.ResetHighwayMerge();
    expect(!planner.HighwayReadyWasIssued(now),"new binding/reset cannot inherit another command's READY");
}
}
int main() {
    policy();try{integration();}catch(const std::exception& error){++failures;std::cerr<<error.what()<<'\n';}
    std::cout<<checks<<" checks, "<<failures<<" failures\n";return failures?1:0;
}
