#include "shadow_localization/local_recovery.hpp"
#include <iostream>
using namespace shadow_localization;
int failures=0;
void check(bool ok,const char* name){std::cout<<(ok?"PASS ":"FAIL ")<<name<<'\n';failures+=!ok;}
int main(){
 LocalRecoveryPolicy p;
 check(!p.update(1,true,false,true,true,true,false),"does not switch on first stopped scan");
 check(!p.update(1.49,true,false,true,true,true,false),"requires continuous stationary dwell");
 check(p.update(1.51,true,false,true,true,true,false),"fresh validated LOCAL recovers static blockage");
 check(p.update(1.6,true,false,true,true,false,false),"does not abandon recovery when vehicle starts moving");
 check(p.update(2,true,true,false,true,false,false),"native recovery debounced");
 check(p.update(2.9,true,true,false,true,false,false),"keeps LOCAL through transient native recovery");
 check(!p.update(3.01,true,true,false,true,false,false),"returns after one second healthy native paths");
 p.update(4,true,false,true,true,true,false);p.update(4.6,true,false,true,true,true,false);
 check(!p.update(4.7,true,false,true,true,true,true),"moving or unresolved motion cancels recovery immediately");
 p.update(5,true,false,true,true,true,false);p.update(5.6,true,false,true,true,true,false);
 check(!p.update(5.7,true,false,true,false,true,false),"expired LOCAL never persists as a valid recovery");
 check(!p.update(6,true,false,false,true,true,false),"unknown contract failure is not recoverable");
 check(!p.update(5,true,false,true,true,true,false),"retrograde policy clock resets");
 check(RecoverableLocalFailure("localization_obb")&&!RecoverableLocalFailure("shadow_dynamic_yield")&&!RecoverableLocalFailure("position_imu_stale"),"explicit failure whitelist");
 asmc_msgs::LocalTrajectory in;in.valid=true;in.run_id="test";in.header.frame_id="local/test/1";
 in.header.stamp.fromSec(10);in.scene_stamp.fromSec(9.9);in.ego_at_plan.header=in.header;
 in.ego_at_plan.child_frame_id="base_link";in.ego_at_plan.pose.pose.position.x=100;
 in.ego_at_plan.pose.pose.position.y=-50;in.ego_at_plan.pose.pose.orientation.w=1;in.dt=.05;
 for(int i=0;i<41;++i){asmc_msgs::TrajectoryPoint q;q.x=100+i*.05;q.y=-50;q.velocity=1;in.points.push_back(q);}
 local_planning::Motion target;target.t=10;target.x=20;target.y=30;target.yaw=M_PI/2;
 auto out=ReframeLocalRecovery(in,target,"local/native/1","test",10.1,.1);
 check(out&&std::abs(out->points.back().x-20)<1e-9&&std::abs(out->points.back().y-32)<1e-9,"different odometry origins rotate and translate at matched time");
 check(out&&out->header.stamp==in.header.stamp&&out->scene_stamp==in.scene_stamp&&out->points.back().velocity==1,"conversion preserves measurement age and speed profile");
 check(!ReframeLocalRecovery(in,target,"local/native/1","other",10.1,.1),"reject other run");
 check(!ReframeLocalRecovery(in,target,"local/native/1","test",10.26,.1),"reject old plan");
 check(!ReframeLocalRecovery(in,target,"local/native/1","test",9.99,.1),"reject future plan");
 check(!ReframeLocalRecovery(in,target,"local/native/1","test",10.1,.26),"reject stale receipt");
 target.t=10.01;check(!ReframeLocalRecovery(in,target,"local/native/1","test",10.1,.1),"never align with a later ego sample");target.t=10;
 in.scene_stamp.fromSec(9.7);check(!ReframeLocalRecovery(in,target,"local/native/1","test",10.1,.1),"reject old scene even when plan is new");in.scene_stamp.fromSec(9.9);
 in.points.back().x=NAN;check(!ReframeLocalRecovery(in,target,"local/native/1","test",10.1,.1),"reject nonfinite path");in.points.back().x=102;
 in.ego_at_plan.header.frame_id="bad";check(!ReframeLocalRecovery(in,target,"local/native/1","test",10.1,.1),"reject conflicting ego frame");in.ego_at_plan.header=in.header;
 for(auto& q:in.points)q.velocity=0;check(!ReframeLocalRecovery(in,target,"local/native/1","test",10.1,.1),"zero-speed valid wait is not recovery");
 return failures?1:0;
}
