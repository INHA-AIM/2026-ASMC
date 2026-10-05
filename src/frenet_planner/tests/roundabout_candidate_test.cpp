#include "frenet_planner/Maneuver/Merge/RoundaboutMerge.hpp"
#include <iostream>
#include <cmath>
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
 return failures?1:0;
}
