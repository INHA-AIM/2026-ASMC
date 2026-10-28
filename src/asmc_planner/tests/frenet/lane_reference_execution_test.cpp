#include "Execution/LaneReferenceExecution.hpp"
#include "Execution/ReferenceTrialObjects.hpp"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <unistd.h>

using namespace asmc_planner::lane_reference;
namespace {
int checks=0,failed=0;
void expect(bool yes,const char* what){++checks;if(!yes){++failed;std::cerr<<"FAIL "<<what<<'\n';}}
struct Fixture {
  std::filesystem::path root;std::string config;
  std::shared_ptr<const HdLaneMap> map;std::shared_ptr<const LaneReference> target;
  PlannerCommand lc;
  Fixture(){
    root=std::filesystem::temp_directory_path()/("asmc_reference_execution_"+std::to_string(getpid()));
    std::filesystem::create_directories(root);auto waypoint=root/"path.txt";
    {std::ofstream f(waypoint);for(int n=0;n<=300;++n)f<<n<<" 0\n";}
    auto yaml=YAML::LoadFile(FRENET_TEST_CONFIG);yaml["waypoint_file"]=waypoint.string();
    yaml["avoid_hd_map"]["enabled"]=false;yaml["roundabout_merge"]["enabled"]=false;
    yaml["planner"]["visualization"]["enabled"]=false;
    config=(root/"params.yaml").string();{std::ofstream f(config);f<<yaml;}
    LaneMapData data;data.identity={"map","fixture","test",std::string(64,'1')};data.complete=true;
    LaneData lane;lane.id="B";lane.link_id="B";lane.traversable=true;
    for(int n=20;n<=300;++n)lane.centerline.push_back({double(n),3.5});
    lane.polygon={{20,1.75},{300,1.75},{300,5.25},{20,5.25}};data.lanes={lane};
    data.areas={{"road",{{-10,-3},{310,-3},{310,7},{-10,7}},true}};
    std::string error;map=HdLaneMap::create(data,error);
    LaneReferenceOptions o;o.reference={.05,10,.5,300,.3,30};o.vehicle={3.845,.79,.95};
    o.max_curvature=.23;o.max_lateral_accel=2.6;o.max_join_gap_m=.05;o.sample_distance_m=.2;
    o.sample_yaw_rad=.02;o.max_samples=30000;
    auto result=BuildLaneReference(map,map->identity(),{"B"},o);
    if(!result)throw std::runtime_error(result.detail);target=result.candidate;
    lc.mode=LANE_CHANGE_LEFT;lc.has_target_pose_projection=true;lc.target_pose_projection_s=80;
    lc.target_pose_projection_d=3.5;lc.has_target_speed=true;lc.target_speed=5;
  }
  ~Fixture(){std::filesystem::remove_all(root);}
  std::unique_ptr<LaneReferenceExecution> session(){
    auto source=std::make_unique<FrenetPlanner>(),next=std::make_unique<FrenetPlanner>();
    if(!source->Init(config)||!next->Init(config))throw std::runtime_error("fixture planner init");
    auto run=std::make_unique<LaneReferenceExecution>(std::move(source),std::move(next),target,
      HandoverOptions{.4,.05,.2,.5,.3,.3,.21},HandoverContinuity{1,.35,.05,.03,.5,1},8,20);
    CartesianPath maintenance;FrenetPlanner::LaneChangePrepareResult p;
    const CartesianState ego{50,0,0,0,5,0};
    if(!run->planner().PrepareLaneChange(ego,{},lc,maintenance,p)||!p.candidate_valid)
      throw std::runtime_error("fixture LC candidate");
    run->planner().ApplyLaneChangeCommit(p);
    if(!run->begin(1))throw std::runtime_error("fixture binding");return run;
  }
};
ReferenceExecutionInput input(const Fixture& f,int n,double y=3.5){
  ReferenceExecutionInput i;i.map=f.map->identity();i.sequence=n+1;
  auto& o=i.observation;o.now_s=o.ego_stamp_s=o.objects_stamp_s=100+n*.1;
  o.ego_valid=o.objects_valid=true;o.ego={75+n*.5,y,0,0,5,0};return i;
}
CartesianPath executing(LaneReferenceExecution& s,const Fixture& f,const ReferenceExecutionInput& i){
  CartesianPath path;FrenetPlanner::LaneChangePrepareResult p;
  s.planner().SetObjectObservationTime(i.observation.objects_stamp_s);
  if(!s.planner().PlanCommittedLaneChange(i.observation.ego,i.objects,f.lc,path,p))
    throw std::runtime_error("fixture committed path");return path;
}
TrackedObject object(int id,double x,double y){
  TrackedObject o;o.id=id;o.type=int(asmc_planner::ObjectType::VEHICLE);o.x=x;o.y=y;o.z=.8;
  o.vx=o.speed=5;o.length=4;o.width=1.8;o.height=1.6;o.geometry_valid=o.z_valid=true;
  int index=0;for(double z:{0.,1.6})for(const auto& p:std::vector<asmc_planner::Point2D>{{x-2,y-.9},{x+2,y-.9},{x+2,y+.9},{x-2,y+.9}})
    o.corners[index++]={p.x,p.y,z};return o;
}
void trialObjectTests(){
  LaneMapData data;data.identity={"map","trial","test",std::string(64,'1')};data.complete=true;
  LaneData a;a.id=a.link_id="A";a.centerline={{0,0},{100,0}};
  a.polygon={{0,-1.75},{100,-1.75},{100,1.75},{0,1.75}};
  auto b=a;b.id=b.link_id="B";b.centerline={{0,3.5},{100,3.5}};
  b.polygon={{0,1.75},{100,1.75},{100,5.25},{0,5.25}};
  data.lanes={a,b};data.areas={{"road",{{0,-2},{100,-2},{100,6},{0,6}},true}};
  std::string detail;const auto map=HdLaneMap::create(data,detail);
  const auto corridor=map->corridor({"A","B"},detail);
  expect(TrialObjectsAllowStart(*corridor,{},detail),"empty trial still admits start");
  auto outside=object(1104,50,12);outside.type=int(asmc_planner::ObjectType::UNKNOWN);
  outside.motion_state=asmc_planner::MotionState::LOW_THREAT;
  // The classifier can settle static geometry while retaining noisy raw velocity.
  const std::vector<TrackedObject> input_objects{outside};
  expect(TrialObjectsAllowStart(*corridor,input_objects,detail),"settled off-corridor UNKNOWN does not block trial");
  expect(input_objects.size()==1&&input_objects[0].id==1104&&input_objects[0].vx==5,
    "admission preserves the complete object input and raw velocity");
  outside.type=int(asmc_planner::ObjectType::STATIC_OBSTACLE);
  expect(TrialObjectsAllowStart(*corridor,{outside},detail),"settled off-corridor static class also admitted");
  for(const auto state:{asmc_planner::MotionState::UNKNOWN,asmc_planner::MotionState::DYNAMIC_PENDING,asmc_planner::MotionState::DYNAMIC}){
    auto moving=outside;moving.motion_state=state;
    expect(!TrialObjectsAllowStart(*corridor,{moving},detail)&&detail.find("id=1104")!=std::string::npos,
      "uncertain or dynamic object blocks even outside the trial lanes");
  }
  for(const auto type:{asmc_planner::ObjectType::VEHICLE,asmc_planner::ObjectType::PEDESTRIAN}){
    auto actor=outside;actor.type=int(type);
    expect(!TrialObjectsAllowStart(*corridor,{actor},detail),"traffic actors require later gap policy even when LOW_THREAT");
  }
  auto malformed=outside;malformed.type=99;
  expect(!TrialObjectsAllowStart(*corridor,{malformed},detail),"foreign class cannot bypass trial gate");
  for(double y:{0.,3.5,5.25+.9}){
    auto inside=object(1105,50,y);inside.type=int(asmc_planner::ObjectType::STATIC_OBSTACLE);
    inside.motion_state=asmc_planner::MotionState::LOW_THREAT;
    expect(!TrialObjectsAllowStart(*corridor,{outside,inside},detail)&&detail.find("id=1105")!=std::string::npos,
      "source, target and exact OBB boundary contact block after an allowed object");
  }
  malformed=outside;malformed.geometry_valid=false;
  expect(!TrialObjectsAllowStart(*corridor,{malformed},detail),"invalid geometry flag cannot clear trial");
  malformed=outside;malformed.corners[0].x=std::numeric_limits<double>::quiet_NaN();
  expect(!TrialObjectsAllowStart(*corridor,{malformed},detail),"non-finite authoritative corner cannot clear trial");
  malformed=outside;malformed.corners={};
  expect(!TrialObjectsAllowStart(*corridor,{malformed},detail),"degenerate authoritative OBB cannot clear trial");
  malformed=outside;malformed.vx=std::numeric_limits<double>::infinity();
  expect(!TrialObjectsAllowStart(*corridor,{malformed},detail),"non-finite velocity cannot clear trial");
  expect(corridor->intersectsPolygon({{40,2},{60,2},{60,3},{40,3}}),"polygon entirely inside target is occupied");
  expect(corridor->intersectsPolygon({{-1,-3},{101,-3},{101,7},{-1,7}}),"enclosing polygon overlaps without an object vertex inside a lane");
  expect(corridor->intersectsPolygon({{50,2},{51,2}}),"invalid polygon conservatively overlaps");
  auto concave=a;concave.id=concave.link_id="C";
  concave.polygon={{0,0},{10,0},{10,2},{2,2},{2,10},{0,10}};
  data.lanes={concave};const auto bent=HdLaneMap::create(data,detail)->corridor({"C"},detail);
  expect(!bent->intersectsPolygon({{4,4},{5,4},{5,5},{4,5}}),"concave lane bounds alone do not block an unrelated object");
  expect(bent->intersectsPolygon({{.5,8},{1.5,8},{1.5,9},{.5,9}}),"curved/concave lane still detects occupied leg");
}
bool settle(LaneReferenceExecution& s,const Fixture& f,ReferenceExecutionInput& last,std::string& detail){
  bool ready=false;for(int n=0;n<8;++n){last=input(f,n);ready=s.prepare(last,executing(s,f,last),5,detail);if(ready)break;}return ready;
}
void run(){
  trialObjectTests();
 {
   CartesianPath cruise,follow;
   for(int n=0;n<=10;++n){const double t=n*.1;
     cruise.x.push_back(5*t+.5*t*t);cruise.y.push_back(3.5);cruise.yaw.push_back(0);cruise.kappa.push_back(0);
     cruise.v.push_back(5+t);cruise.a.push_back(1);
     follow.x.push_back(5*t+.5*t*t-t*t*t);follow.y.push_back(3.5);follow.yaw.push_back(0);follow.kappa.push_back(0);
     follow.v.push_back(5+t-3*t*t);follow.a.push_back(1-6*t);
   }
   const HandoverContinuity limits{1,.35,.05,.03,.5,1};
   expect(!HandoverPathsContinuous(cruise,follow,.1,limits),"distinct future FOLLOW timing differs from cruise timing");
   expect(HandoverPathsGeometricallyContinuous(cruise,follow,.1,limits),
       "settled same-lane FOLLOW can change future speed with identical boundary and steering geometry");
   auto bad=follow;bad.v[0]+=1;
   expect(!HandoverPathsGeometricallyContinuous(cruise,bad,.1,limits),"handover never permits an initial speed jump");
   bad=follow;bad.a[0]-=2;
   expect(!HandoverPathsGeometricallyContinuous(cruise,bad,.1,limits),"handover never permits an initial acceleration jump");
   bad=follow;for(auto& y:bad.y)y+=.6;
   expect(!HandoverPathsGeometricallyContinuous(cruise,bad,.1,limits),"equal-distance comparison rejects another lateral path");
   bad=follow;bad.kappa[3]=.1;
   expect(!HandoverPathsGeometricallyContinuous(cruise,bad,.1,limits),"steering curvature discontinuity still rejects adoption");
 }
  Fixture f;std::string detail;ReferenceExecutionInput last;
  auto s=f.session();FrenetPlanner* source=&s->planner();
  expect(settle(*s,f,last,detail),"fresh target alignment prepares real KEEP continuation");
  expect(!s->adopted()&&s->epoch()==1&&&s->planner()==source,"preparation never changes active core/ref epoch");
  expect(s->preparedFrame()&&std::abs(s->preparedFrame()->ego.d)<1e-8,"new ego projection ready before commit");
  expect(!s->commit(last,[]{return false;},detail),"failed completion transition rejects adoption");
  expect(!s->adopted()&&s->epoch()==1&&&s->planner()==source,"failed completion leaves old owner intact");
  int sample=int(last.sequence);
  auto i=input(f,sample++);expect(s->prepare(i,executing(*s,f,i),5,detail),"failed completion can safely replan");
  auto changed=i;++changed.sequence;
  expect(!s->commit(changed,[]{return true;},detail),"input callback after preparation invalidates token");
  expect(s->epoch()==1&&!s->adopted(),"input race cannot partially adopt");
  i=input(f,sample++);expect(s->prepare(i,executing(*s,f,i),5,detail),"fresh retry after changed input");
  int completions=0;
  expect(s->commit(i,[&]{++completions;return true;},detail),"successful completion exchanges ownership");
  if(!s->adopted())throw std::runtime_error("adoption failed: "+detail);
  expect(completions==1&&s->epoch()==2&&&s->planner()!=source,"one completion and one ref epoch increment");
  expect(!s->commit(i,[&]{++completions;return true;},detail)&&completions==1,"repeated commit cannot complete twice");
  expect(!s->begin(2),"one-shot trial cannot silently retarget an adopted reference");
  for(int n=10;n<20;++n){auto current=input(f,n);expect(s->continueLane(current,5,detail),"KEEP replans on adopted target");}
  expect(std::abs(s->frame()->path.y.back()-3.5)<.05,"KEEP does not return to mission centerline");
  expect(s->frame()->ref_epoch==2&&std::abs(s->frame()->route_s-s->frame()->ego.s-20)<.01,"mission progress never becomes the new lane station");
  i=input(f,20);i.objects={object(10,105,0),object(11,115,3.5)};
  expect(s->continueLane(i,5,detail),"FOLLOW continuation uses fresh world objects");
  expect(s->frame()->leader_id==11&&s->frame()->mode==FOLLOWING,"target-lane leader selected instead of nearer old-lane car");
  expect(s->frame()->objects.size()==2&&std::abs(s->frame()->objects[0].projection.d+3.5)<1e-6&&
    std::abs(s->frame()->objects[1].projection.d)<1e-6,"all objects reprojected with same active ref");
  i=input(f,21);i.observation.objects_stamp_s-=1;
  expect(!s->continueLane(i,5,detail)&&s->epoch()==2,"stale objects never become clear on active lane");
  i=input(f,22);i.map.build_id="changed";
  expect(!s->continueLane(i,5,detail),"map epoch change rejects active continuation");
  i=input(f,23);i.objects={object(12,i.observation.ego.x+4,3.5)};
  expect(!s->continueLane(i,5,detail),"blocking target obstacle cannot become a valid continuation");
  i=input(f,24);i.observation.ego.x=260;
  expect(s->continueLane(i,5,detail),"finite target reference has a planned end stop");
  expect(s->frame()->mode==STOP,"end horizon uses existing STOP instead of extrapolating KEEP");
  i=input(f,0);
  expect(!s->continueLane(i,5,detail),"a replay/source clock reset cannot reuse an adopted reference session");
  auto rejected=f.session();
  for(int n=0;n<9;++n){auto current=input(f,n,0);auto path=executing(*rejected,f,current);
    expect(!rejected->prepare(current,path,5,detail),"source-lane driving never satisfies arrival");}
  auto discontinuous=f.session();
  bool got_ready=false;for(int n=0;n<8;++n){auto current=input(f,n);auto p=executing(*discontinuous,f,current);
    for(auto& speed:p.v)speed+=2;
    got_ready=discontinuous->prepare(current,p,5,detail)||got_ready;}
  expect(!got_ready&&!discontinuous->adopted(),"speed discontinuity rejects handover after geometry alignment");
  auto blocked=f.session();bool can=false;
  for(int n=0;n<8;++n){auto current=input(f,n);auto p=executing(*blocked,f,current);
    current.objects={object(13,current.observation.ego.x+4,3.5)};
    can=blocked->prepare(current,p,5,detail)||can;}
  expect(!can&&blocked->epoch()==1,"collision rejects prepared continuation without switching reference");
  auto freshness=f.session();
  for(int n=0;n<10;++n){auto current=input(f,n);auto path=executing(*freshness,f,current);
    if(n==4)current.observation.objects_stamp_s-=1;
    expect(!freshness->prepare(current,path,5,detail),"one invalid input breaks consecutive alignment");}
  i=input(f,10);
  expect(freshness->prepare(i,executing(*freshness,f,i),5,detail),"alignment can settle again after invalid input");
  // The ROS manager owns this storage. Target preparation must use its own
  // context until completion, then the adopted core explicitly binds to it.
  asmc_planner::execution::ManeuverState external;
  auto bound=f.session();bound->planner().BindExecutionState(external);
  CartesianPath maintenance;FrenetPlanner::LaneChangePrepareResult candidate;
  const CartesianState start{50,0,0,0,5,0};
  if(!bound->planner().PrepareLaneChange(start,{},f.lc,maintenance,candidate)||!candidate.candidate_valid)
    throw std::runtime_error("bound LC preparation");
  bound->planner().ApplyLaneChangeCommit(candidate);
  expect(settle(*bound,f,last,detail),"manager-bound LC can prepare a target continuation");
  expect(external.lane_change.committed,"isolated target planning preserves the executing owner's LC lock");
  expect(bound->commit(last,[]{return true;},detail),"manager-bound owner adopts after completion approval");
  bound->planner().BindExecutionState(external);bound->planner().ResetExternalLaneChange();
  auto current=input(f,int(last.sequence));
  expect(!external.lane_change.active&&bound->continueLane(current,5,detail),"new owner rebinds execution storage and continues KEEP");
}
}
int main(){try{run();}catch(const std::exception& e){++failed;std::cerr<<e.what()<<'\n';}
 std::cout<<checks<<" checks, "<<failed<<" failures\n";return failed?1:0;}
