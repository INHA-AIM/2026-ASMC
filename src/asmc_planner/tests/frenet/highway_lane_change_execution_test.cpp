#include "Execution/HighwayReferenceExecution.hpp"
#include "Frenet/Geometry/PolygonUtils.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace asmc_planner::lane_reference;
namespace {
int checks=0,failures=0;
void expect(bool ok,const char* message){++checks;if(!ok){++failures;std::cerr<<"FAIL "<<message<<'\n';}}
CartesianState at(const RefLine& r,double s,double v){const auto p=Interpolate(r,s);return {p.x,p.y,p.theta,p.kappa,v,0};}
void follow(CartesianState& e,const CartesianPath& p){if(p.x.size()<2)throw std::runtime_error("empty issued path");e={p.x[1],p.y[1],p.yaw[1],p.kappa[1],p.v[1],p.a[1]};}
std::string link(const LaneReference& lane,const CartesianState& e){
  double best=INFINITY;std::string result;
  for(const auto& id:lane.lanes())for(const auto& p:lane.map()->lane(id)->centerline){
    const double d=std::hypot(e.x-p.x,e.y-p.y);if(d<best){best=d;result=id.substr(5);}}
  return result;
}
TrackedObject object(const CartesianState& e,int id,double speed){
  TrackedObject o;o.id=id;o.type=int(asmc_planner::ObjectType::VEHICLE);o.x=e.x;o.y=e.y;
  o.heading=e.yaw;o.vx=speed*std::cos(e.yaw);o.vy=speed*std::sin(e.yaw);
  o.length=4.5;o.width=1.8;o.height=1.5;o.motion_state=ObjectMotionState::DYNAMIC;
  o.geometry_valid=o.z_valid=true;
  const double c=std::cos(e.yaw),s=std::sin(e.yaw);
  const double x[]={2.25,-2.25,-2.25,2.25},y[]={.9,.9,-.9,-.9};
  for(int i=0;i<8;++i)o.corners[i]={e.x+c*x[i%4]-s*y[i%4],e.y+s*x[i%4]+c*y[i%4],i<4?0.0:1.5};
  return o;
}
void exercise(){
  FrenetPlanner mission;asmc_planner::execution::ManeuverState state;mission.BindExecutionState(state);
  std::string detail;if(!mission.Init(FRENET_TEST_CONFIG)||!mission.EnableHighwayMerge(detail))throw std::runtime_error(detail);
  HighwayReferenceExecution execution(mission,FRENET_TEST_CONFIG,state);const auto route=mission.highway_route();
  const auto network=execution.status().network;
  expect(network&&network->lanes.size()==3,"all three explicit highway chains preflight before driving");
  if(!network)return;
  expect(network->indexForLink("A2256W000432")==3,"dead-end parallel successor is not a driving route");
  double now=100;std::uint64_t sequence=0;
  auto input=[&](const CartesianState& e){ReferenceExecutionInput i;i.sequence=++sequence;i.map=route->map->identity();i.observation={now,now,now,true,true,e};return i;};
  auto ego=at(mission.reference(),route->zones[0].config.start_s+5,16);
  PlannerCommand command;command.mode=MERGE;command.merge_type=MergeType::HIGHWAY;
  command.source_link_id=route->zones[0].config.source_link;
  command.target_link_id=route->zones[0].config.target_lanes.back().substr(5);
  command.has_target_speed=true;command.target_speed=route->config.speed_cap_mps;
  expect(execution.begin(1,0),"ordinary entry binds existing MERGE");
  CartesianPath path;FrenetPlanner::HighwayMergeResult result;
  for(int i=0;i<20&&!result.ready;++i){now+=.05;mission.PlanHighwayMerge(ego,{},command,now,now,path,result,now,&execution.target(0).line(),&execution.planner().reference());}
  if(!result.ready)throw std::runtime_error("entry not ready: "+result.detail);
  mission.ApplyHighwayMergeCommit(result);path=result.candidate_path;
  bool adopted=false;
  for(int i=0;i<200&&!adopted;++i){follow(ego,path);now+=.1;
    if(!mission.PlanHighwayMerge(ego,{},command,now,now,path,result,now,&execution.target(0).line(),&execution.planner().reference()))throw std::runtime_error(result.detail);
    const auto in=input(ego);if(execution.prepare(in,path,command.target_speed,detail)){path=execution.preparedFrame()->path;adopted=execution.commit(in,[]{return true;},detail);}}
  expect(adopted,"existing MERGE adopts source driving lane before optional change");if(!adopted)throw std::runtime_error(detail);
  for(int target_index:{1,2,1,0,1,2}) {
    const auto& source=*execution.status().lane;
    ego=at(source.line(),target_index==1?170:235,16);
    if(target_index==2) {
      const auto& merge=network->common->line().points.front();
      ego=at(source.line(),FindClosestS(source.line(),merge.x,merge.y,nullptr,100)-110,16);
    }
    const int source_index=int(network->indexForLink(link(source,ego)));
    const bool left=target_index>source_index;
    command={};command.mode=left?LANE_CHANGE_LEFT:LANE_CHANGE_RIGHT;command.has_target_speed=true;command.target_speed=route->config.speed_cap_mps;
    command.source_link_id=link(source,ego);
    const auto* road=route->map->lane("lane:"+command.source_link_id);
    command.target_link_id=(left?road->left_adjacent:road->right_adjacent).substr(5);
    std::cerr<<"LC "<<command.source_link_id<<" -> "<<command.target_link_id<<'\n';
    auto invalid=command;invalid.target_link_id=target_index==1?"A2256W000418":"A2256W000432";
    expect(!execution.beginLaneTransition(10+target_index,invalid,ego,true,detail),"nonadjacent/unprepared requests cannot bind");
    expect(execution.beginLaneTransition(10+target_index,command,ego,true,detail),"adjacent request binds common transition engine");
    if(!execution.hasLaneTransition(10+target_index))throw std::runtime_error(detail);
    expect(execution.transitionZone().following_lanes.size()==2,
        "optional left/right following scope contains only source and requested target");
    expect(bool(execution.transitionZone().following_road_footprint),
        "joining body uses the same runtime road certificate as prepared lane references");
    const auto& target=execution.transitionTarget();
    // A prepared next request can be canceled by KEEP. Its future return
    // context must not replace/destroy the current active lane's return owner.
    const auto canceled=input(ego);
    expect(!execution.prepareMissionReturn(canceled,path,command.target_speed,detail),
        "canceling a prepared change keeps a live return context for the CURRENT lane");
    auto& core=execution.transitionPlanner();
    const double target_s=FindClosestS(target.line(),ego.x,ego.y,nullptr,100);
    const auto blocker=object(at(target.line(),target_s+2,16),40,16);
    result={};now+=.05;
    const bool maintained=core.PlanHighwayTransition(*route,execution.transitionZone(),true,ego,{blocker},command,now,now,path,result,now,target.line());
    expect(maintained&&!result.candidate_valid&&!result.ready,"occupied target rejects LC while source KEEP still drives");
    if(maintained){const auto last=ProjectOnLaneReference(source,path.x.back(),path.y.back(),path.yaw.back(),path.v.back());expect(std::fabs(last.d)<.5,"rejection does not issue lateral motion");}
    auto unobserved=execution.transitionZone();unobserved.admission_rear_observation_m=1;
    now+=.05;result={};
    const bool held=core.PlanHighwayTransition(*route,unobserved,true,ego,{},command,now,now,path,result,now,target.line());
    expect(held&&!result.candidate_valid&&!result.ready,
        "insufficient rear observation filters transition candidates while preserving source driving");
    result={};for(int i=0;i<20&&!result.ready;++i){now+=.05;core.PlanHighwayTransition(*route,execution.transitionZone(),true,ego,{},command,now,now,path,result,now,target.line());}
    std::cerr<<"ready="<<result.ready<<" detail="<<result.detail<<'\n';
    expect(result.ready&&result.check.covers_completion,"new observations certify actual LC arrival and continuation");
    if(!result.ready)throw std::runtime_error(result.detail);
    expect(core.CanCommitHighwayMerge(result),"same candidate certificate permits commit");
    core.MarkHighwayReadyIssued(now+.001);now+=.05;
    expect(core.PlanHighwayTransition(*route,execution.transitionZone(),true,ego,{},command,
        now,now,path,result,now,target.line(),true)&&result.ready&&result.output_requires_commit,
        "optional EXECUTE freshly certifies and returns the transition in the same cycle");
    auto stale=result;stale.epoch--;expect(!core.CanCommitHighwayMerge(stale),"stale candidate cannot commit");
    core.ApplyHighwayMergeCommit(result);path=result.candidate_path;adopted=false;int complete=0;
    expect(execution.preserveInterruptedTransition(100+target_index),
        "new emergency transaction preserves physical optional-crossing geometry");
    expect(execution.recoveringTransition()&&core.highway_merge_committed(),
        "interruption retains recovery ownership independently of the wire command");
    expect(!execution.beginLaneTransition(100+target_index,command,ego,true,detail),
        "a fresh command cannot rebind the old source while recovery is pending");
    now+=.05;
    const bool continued=core.PlanHighwayTransition(*route,unobserved,true,ego,{},command,
        now,now,path,result,now,target.line());
    expect(continued&&result.candidate_valid,
        "COMMIT does not reapply the entry-only unseen-rear observation veto");
    for(int i=0;i<160&&!adopted;++i){follow(ego,path);now+=core.sample_dt();
      if(!core.PlanHighwayTransition(*route,execution.transitionZone(),true,ego,{},command,now,now,path,result,now,target.line()))throw std::runtime_error(result.detail);
      const auto in=input(ego);if(execution.prepare(in,path,command.target_speed,detail)){
        path=execution.preparedFrame()->path;adopted=execution.commit(in,[&]{++complete;return true;},detail);}}
    std::cerr<<"adopted="<<adopted<<" "<<detail<<'\n';
    expect(adopted&&complete==1,"LC completion atomically adopts new reference once");
    expect(execution.status().lane.get()==network->lanes[target_index].get(),"target lane remains active without original-lane return");
    expect(!execution.recoveringTransition(),"actual target adoption releases interrupted recovery ownership");
    if(!adopted)throw std::runtime_error(detail);
    expect(!execution.status().completed.back(),"optional LC does not falsely complete downstream mandatory merge");
    PlannerCommand keep;keep.mode=LANE_KEEPING;keep.has_target_speed=true;keep.target_speed=command.target_speed;keep.source_link_id=link(target,ego);
    expect(execution.planner().Plan(ego,{},keep,path),"normal KEEP works immediately after LC");
    if(!path.x.empty())expect(std::fabs(ProjectOnLaneReference(target,path.x.back(),path.y.back(),path.yaw.back(),path.v.back()).d)<.5,"KEEP tracks adopted lane");
  }
  const auto lane=execution.status().lane;const auto& q=network->common->line().points.front();
  const double common_s=FindClosestS(lane->line(),q.x,q.y,nullptr,100);
  ego=at(lane->line(),common_s-75,14);
  command={};command.mode=MERGE;command.merge_type=MergeType::HIGHWAY;command.has_target_speed=true;command.target_speed=route->config.speed_cap_mps;
  command.source_link_id=link(*lane,ego);command.target_link_id=network->common->lanes().front().substr(5);
  expect(execution.beginLaneTransition(30,command,ego,false,detail),"downstream merge binds the ACTUAL left lane, not old 445 source");
  if(!execution.hasLaneTransition(30))throw std::runtime_error(detail);
  expect(execution.transitionZone().following_lanes.size()==2,
      "mandatory join on the left lane includes its adjacent predecessor, not the far parallel lane");
  expect(!execution.transitionZone().target_region->coversPoint(ego.x,ego.y),
      "traffic membership never moves the common-lane handover frontier upstream");
  auto& core=execution.transitionPlanner();result={};
  for(int i=0;i<30&&!result.ready;++i){now+=.05;core.PlanHighwayTransition(*route,execution.transitionZone(),false,ego,{},command,now,now,path,result,now,execution.transitionTarget().line());}
  std::cerr<<"mandatory ready="<<result.ready<<" "<<result.detail<<'\n';
  expect(result.ready,"adopted lane has a safe common-lane continuation");if(!result.ready)throw std::runtime_error(result.detail);
  core.ApplyHighwayMergeCommit(result);path=result.candidate_path;adopted=false;
  for(int i=0;i<230&&!adopted;++i){follow(ego,path);now+=core.sample_dt();
    if(!core.PlanHighwayTransition(*route,execution.transitionZone(),false,ego,{},command,now,now,path,result,now,execution.transitionTarget().line()))throw std::runtime_error(result.detail);
    const auto in=input(ego);if(execution.prepare(in,path,command.target_speed,detail)){path=execution.preparedFrame()->path;adopted=execution.commit(in,[]{return true;},detail);}}
  std::cerr<<"mandatory adopted="<<adopted<<" "<<detail<<'\n';
  expect(adopted&&execution.status().completed.back(),"mandatory completion is tied to common-lane footprint and continuation");
  expect(execution.status().lane.get()==network->common.get(),"common lane owns next driving station");
}
}
int main(){try{exercise();}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}std::cout<<checks<<" checks, "<<failures<<" failures\n";return failures?1:0;}
