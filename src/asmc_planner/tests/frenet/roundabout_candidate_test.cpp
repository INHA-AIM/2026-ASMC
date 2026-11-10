#include "Frenet/Maneuver/Merge/RoundaboutMerge.hpp"
#include <iostream>
#include "Frenet/Maneuver/Merge/RoundaboutPrediction.hpp"
#include <cmath>
#include "Frenet/Maneuver/Merge/RoundaboutDeparture.hpp"
int main(){
 int failures=0;auto expect=[&](bool ok,const char* name){std::cout<<(ok?"[PASS] ":"[FAIL] ")<<name<<'\n';failures+=!ok;};
 RoundaboutMergeZone zone;zone.valid=true;zone.conflict_areas={{{20,-2},{40,-2},{40,2},{20,2}}};
 VehicleShape shape{1.9,4.635,.79};CollisionCheckConfig collision{.5,0,0};
 CartesianPath path;for(int i=0;i<=120;++i){path.x.push_back(i*.4);path.y.push_back(0);path.yaw.push_back(0);path.v.push_back(4);path.a.push_back(0);path.kappa.push_back(0);}
 auto check=[&](const CartesianPath& p,const std::vector<TrackedObject>& os,bool entering=true){return CheckRoundaboutCandidate(p,zone,os,shape,collision,.1,1.5,2,entering);};
 auto clear=check(path,{});expect(clear.safe&&clear.covers_exit&&clear.entry_time<clear.exit_time,"empty complete traversal is feasible");
 CartesianState rear{};rear.x=10;expect(!zone.Overlaps(rear,shape,.5),"rear axle stop can leave entire footprint outside corridor");
 rear.x=17;expect(zone.Overlaps(rear,shape,.5),"front enters before rear axle crosses corridor boundary");
 auto short_path=path;short_path.x.resize(75);short_path.y.resize(75);short_path.yaw.resize(75);
 expect(!check(short_path,{}).safe,"candidate ending inside corridor cannot authorize merge");
 auto box=[](int id,double x,double y,double vy){TrackedObject o{};o.id=id;o.x=x;o.y=y;o.z=0;o.vy=vy;o.vx=0;o.heading=0;o.length=2;o.width=2;o.height=1;o.geometry_valid=o.z_valid=true;o.motion_state=vy==0?ObjectMotionState::LOW_THREAT:ObjectMotionState::DYNAMIC;
  for(int i=0;i<8;++i){double dx[4]={-1,1,1,-1},dy[4]={-1,-1,1,1};o.corners[i]={x+dx[i%4],y+dy[i%4],double(i/4)};}return o;};
 auto stopped=box(1,25,0,0);expect(!check(path,{stopped}).safe,"stationary occupying vehicle blocks entry");
 stopped.id=99;expect(!check(path,{stopped}).safe,"tracker ID replacement cannot erase occupancy");
 auto invalid=stopped;invalid.corners={};expect(!check(path,{invalid}).safe,"invalid object geometry fails conservatively");
 auto transverse=box(2,25,18,-2);auto buffered=check(path,{transverse});
 auto unbuffered=CheckRoundaboutCandidate(path,zone,{transverse},shape,collision,.1,0,0,true);
 expect(unbuffered.safe&&!buffered.safe,"rear time buffer detects later crossing vehicle");
 expect(!check(path,{box(3,25,0,0)},false).safe,"committed replanning still checks physical collision");
 CartesianPath beyond=path;for(auto& x:beyond.x)x+=50;
 expect(check(beyond,{},false).safe,"post-commit path beyond corridor may hand off");
 auto beside=box(9,0,2.2,0);
 expect(check(path,{beside},false).safe,"committed path may leave an existing margin while physical clearance improves");
 expect(!check(path,{beside},true).safe,"precommit margin overlap cannot authorize entry");
 expect(!check(path,{beside,box(10,25,0,0)},false).safe,"margin recovery still rejects another physical obstacle");
 expect(!check(path,{box(11,0,1.8,0)},false).safe,"initial physical overlap cannot be excused as margin recovery");
 expect(!check(beyond,{},true).safe,"entry candidate must actually traverse the corridor");
 expect(!CheckRoundaboutCandidate(path,zone,{},shape,collision,NAN,1.5,2,true).safe,"invalid sample interval cannot produce safety");
 auto unknown=box(80,25,0,0);unknown.motion_state=ObjectMotionState::UNKNOWN;
 expect(UnclassifiedCirculatingObject({unknown},zone,20,0)==80,"new zero-velocity vehicle fragment on circulating lane blocks certification");
 auto fixture=unknown;fixture.length=.3;fixture.width=.3;
 expect(UnclassifiedCirculatingObject({fixture},zone,20,0)<0,"small stationary fixtures do not become merge vehicles");
 auto roadside=unknown;roadside.y=8;for(auto& corner:roadside.corners)corner.y+=8;
 expect(UnclassifiedCirculatingObject({roadside},zone,20,0)<0,"unknown roadside observations do not gate circulating traffic");
 auto lane_edge=box(81,25,4.5,0);lane_edge.motion_state=ObjectMotionState::UNKNOWN;
 expect(UnclassifiedCirculatingObject({lane_edge},zone,20,0)==81,"vehicle fragment just outside lane polygon still needs classification");
 lane_edge.y+=1;for(auto& corner:lane_edge.corners)corner.y+=1;
 expect(UnclassifiedCirculatingObject({lane_edge},zone,20,0)<0,"lane observation guard has a bounded two-metre extent");
 unknown.motion_state=ObjectMotionState::LOW_THREAT;
 expect(UnclassifiedCirculatingObject({unknown},zone,20,0)<0,"established static classification uses normal collision checks");
 RoundaboutPrediction remembered;
 auto body=box(90,25,2.5,0);body.vx=1;body.motion_state=ObjectMotionState::DYNAMIC;
 auto observe_body=[&](double t,double length,double width,double y) {
  body.x=25+(t-700);body.y=y;body.length=length;body.width=width;
  for(int i=0;i<8;++i){double dx=(i%4==0||i%4==3)?-length/2:length/2;
   double dy=i%4<2?-width/2:width/2;body.corners[i]={body.x+dx,body.y+dy,double(i/4)};}
  remembered.Observe({body},t,.6);
 };
 for(int i=0;i<5;++i)observe_body(700+i*.1,4.8,2.1,2.5);
 observe_body(700.5,1,.4,2.5);
 auto envelope=remembered.ConservativeOccupancies({body});
 expect(envelope.size()==1&&remembered.UncertainNear(25,0)<0,
  "fresh whole-body history can certify confidence during partial visibility");
 expect(check(path,{body}).safe,"a narrow visible fragment alone misses the body collision");
 auto bounded=std::vector<TrackedObject>{body};bounded.insert(bounded.end(),envelope.begin(),envelope.end());
 expect(!check(path,bounded).safe,"retained full-body occupancy still rejects that candidate through real collision checks");
 expect(body.length==1&&body.width==.4,"supplemental occupancy preserves authoritative fragment geometry");
 remembered.Reset();for(int i=0;i<5;++i)observe_body(701+i*.1,4.8,2.1,8);
 observe_body(701.5,1,.4,8);envelope=remembered.ConservativeOccupancies({body});
 bounded={body};bounded.insert(bounded.end(),envelope.begin(),envelope.end());
 expect(envelope.size()==1&&check(path,bounded).safe,
  "clear remembered body can certify departure without requiring a new full-size view");
 {
  RoundaboutPrediction missed;auto approaching=box(98,30,32.8,0);
  approaching.length=4.5;approaching.width=1.9;approaching.vx=0;approaching.vy=-4;
  approaching.motion_state=ObjectMotionState::DYNAMIC;
  for(int step=0;step<8;++step){approaching.y=32.8-.4*step;
   for(int k=0;k<8;++k)approaching.corners[k]={30+(k%4<2?-.95:.95),
       approaching.y+(k%2?-2.25:2.25),double(k/4)};
   missed.Observe({approaching},1100+.1*step,1,5);
  }
  missed.Observe({},1100.8,1,5);
  expect(missed.UncertainNear(20,0)<0,"approaching vehicle lies outside the old fixed proximity gate");
  auto evidence=missed.MissingDepartureOccupancies(1100.8,[](double,double){return true;},6,3,1);
  std::vector<TrackedObject> models;for(const auto& e:evidence)models.insert(models.end(),e.models.begin(),e.models.end());
  const auto rejected=check(path,models);
  expect(!models.empty()&&!rejected.safe&&rejected.blocking_object_id==98,
      "missing vehicle outside the old gate still blocks a future crossing on the complete path");
  for(auto& model:models){model.y+=100;for(auto& c:model.corners)c.y+=100;
      if(model.prediction_origin){auto origin=std::make_shared<TrackedObject>(*model.prediction_origin);origin->y+=100;for(auto& c:origin->corners)c.y+=100;model.prediction_origin=origin;}}
  expect(check(path,models).safe,"a separated missing occupancy does not veto a safe traversal");
 }

 {
  RoundaboutPrediction history;
  auto fragment=box(110,100,0,0);fragment.length=1;fragment.width=.3;
  fragment.vx=5;fragment.vy=0;fragment.motion_state=ObjectMotionState::DYNAMIC;
  for(int step=0;step<8;++step) {
   fragment.x=100+.5*step;
   for(int k=0;k<8;++k)fragment.corners[k]={fragment.x+(k%4<2?-.5:.5),
       k%2?-.15:.15,.02*(k/4)};
   history.Observe({fragment},1300+.1*step,1,5);
  }
  history.Observe({},1300.8,1,5);
  auto missing=history.MissingDepartureOccupancies(1300.85,
      [](double,double){return false;},6,3,1,20,0);
  expect(missing.size()==1&&missing[0].motion_supported&&!missing[0].body_supported&&
      missing[0].models.empty()&&!missing[0].relevance_models.empty(),
      "verified departing motion can supply relevance bounds without forging mapped or whole-body support");
  auto check_missing=[&](const CartesianPath& candidate) {
   return CheckMissingRoundaboutDeparture(candidate,zone,missing,shape,collision,.1,1.5,2);
  };
  const auto safe=check_missing(path);
  expect(safe.safe&&safe.non_interference_proven==1&&safe.blocking_id<0,
      "all conservative departing-fragment hypotheses clear a separated complete candidate");
  bool envelope=false,aligned=false;
  for(const auto& model:missing[0].relevance_models) {
   envelope|=model.length>=12.5&&model.width>=6.5;
   if(std::fabs(model.yaw_rate)<1e-9)aligned|=std::fabs(model.x-104.25)<1e-6&&
       std::fabs(model.prediction_origin_offset_s-.15)<1e-6;
  }
  expect(envelope&&aligned,"relevance body bounds fragment-centre uncertainty and aligns only once");
  auto other=path;
  const auto collision_pose=PredictTrackedObjectPose(missing[0].relevance_models.front(),2);
  other.x[20]=collision_pose.x;other.y[20]=collision_pose.y;
  const auto rejected=check_missing(other);
  expect(!rejected.safe&&rejected.blocking_id==110&&rejected.time>=0&&
      std::string(rejected.status)=="potential_conflict",
      "a safe candidate does not exempt the same missing vehicle on another colliding candidate");
  expect(check_missing(path).safe,"a rejected candidate leaves no stale blocker on a later safe candidate");
  const auto toward=history.MissingDepartureOccupancies(1300.85,
      [](double,double){return false;},6,3,1,200,0);
  expect(toward.size()==1&&toward[0].relevance_models.empty()&&
      !CheckMissingRoundaboutDeparture(path,zone,toward,shape,collision,.1,1.5,2).safe,
      "body-less approaching motion cannot use a departing-vehicle relevance exemption");
  history.Reset();fragment.id=10;history.Observe({fragment},1400,1,5);
  history.Observe({},1400.05,1,5);
  auto single=history.MissingDepartureOccupancies(1400.05,
      [](double,double){return false;},6,3,1,20,0);
  const auto unknown=CheckMissingRoundaboutDeparture(path,zone,single,shape,collision,.1,1.5,2);
  expect(single.size()==1&&single[0].observations==1&&single[0].relevance_models.empty()&&
      !unknown.safe&&unknown.blocking_id==10&&std::string(unknown.reason)=="motion_insufficient",
      "single low partial ID 10 cannot acquire confidence or a size-based exemption");
  history.Reset();fragment.velocity_unavailable=true;history.Observe({fragment},1500,1,5);
  history.Observe({},1500.05,1,5);
  const auto unavailable=history.MissingDepartureOccupancies(1500.05,
      [](double,double){return false;},6,3,1,20,0);
  expect(unavailable.size()==1&&unavailable[0].relevance_models.empty()&&
      std::string(unavailable[0].reason)=="velocity_missing",
      "unavailable velocity is distinct from missing body evidence and cannot be extrapolated");
 }
 return failures?1:0;
}
