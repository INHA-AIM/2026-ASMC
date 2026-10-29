#include "shadow_localization/local_recovery.hpp"
#include "shadow_localization/anchor.hpp"
#include <iostream>
using namespace shadow_localization;
int failures=0;
void check(bool ok,const char* name){std::cout<<(ok?"PASS ":"FAIL ")<<name<<'\n';failures+=!ok;}
int main(){
 Anchor a;local_planning::Motion motion;
 for(int i=0;i<=20;++i){motion.t=1+i*.05;a.motion(motion);a.observe(motion,10,20,0);}
 std::string failure;
 check(!a.at(123,&failure).valid&&failure=="anchor_time_limit","expired anchor does not report stale gps_tracking success");
 check(!a.at(.5,&failure).valid&&failure=="anchor_no_past_sample","query before available anchor is explicit");
 motion.t=3;motion.x=251;a.motion(motion);
 check(!a.at(3,&failure).valid&&failure=="anchor_distance_limit","distance guard remains in force");
 LocalRecoveryPolicy p;
 check(p.update(1,true,false,true,true,true,false),"first native failure immediately selects validated LOCAL");
 check(p.update(1.05,true,false,true,true,true,false),"no additional repeat-failure waiting interval");
 check(p.update(1.51,true,false,true,true,true,false),"fresh validated LOCAL recovers static blockage");
 check(p.update(1.6,true,false,true,true,false,false),"does not abandon recovery when vehicle starts moving");
 check(p.update(2,true,true,false,true,false,false),"native recovery debounced");
 check(p.update(2.9,true,true,false,true,false,false),"keeps LOCAL through transient native recovery");
 check(!p.update(3.01,true,true,false,true,false,false),"returns after one second healthy native paths");
 p.update(4,true,false,true,true,true,false);p.update(4.6,true,false,true,true,true,false);
 check(!p.update(4.7,true,false,true,true,true,true),"moving or unresolved motion cancels recovery immediately");
 p.update(5,true,false,true,true,true,false);p.update(5.6,true,false,true,true,true,false);
 check(!p.update(5.7,true,false,true,false,true,false),"expired LOCAL never persists as a valid recovery");
 check(p.update(5.8,true,false,true,true,false,false),"fresh recovery resumes after one missing path without stationary dwell");
 check(!p.update(6,true,false,false,true,true,false),"unknown contract failure is not recoverable");
 check(!p.update(5,true,false,true,true,true,false),"retrograde policy clock resets");
 check(RecoverableLocalFailure("localization_obb")&&!RecoverableLocalFailure("shadow_dynamic_yield")&&!RecoverableLocalFailure("position_imu_stale"),"explicit failure whitelist");
 check(RecoverableAnchorFailure("motion_epoch_changed"),"map epoch loss can use independent LOCAL evidence");
 check(!RecoverableAnchorFailure("position_imu_stale")&&!RecoverableAnchorFailure("position_objects_stale"),"map recovery never permits stale sensor inputs");
 p.reset();p.update(10,true,false,RecoverableAnchorFailure("motion_epoch_changed"),true,true,false);
 check(p.update(10.51,true,false,true,true,true,false),"fresh local path can recover after map anchor loss");
 check(!p.update(10.6,true,false,true,true,true,true),"map loss does not waive observed dynamic hazard");
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
 for(size_t i=0;i<in.points.size();++i){in.points[i].x=100+i*.02;in.points[i].velocity=.12;}
 check(bool(ReframeLocalRecovery(in,target,"local/native/1","test",10.1,.1)),"validated 0.8m slow recovery preserved without speed inflation");
 check(RecoverableAnchorFailure("anchor_time_limit")&&RecoverableAnchorFailure("anchor_distance_limit"),"map expiry permits independent fresh local recovery");
 p.reset();check(p.update(20,true,false,true,true,false,false),"moving vehicle switches on first failure before braking to stop");
 check(p.update(20.1,true,false,true,true,false,false),"recovery starts before complete standstill");
 for(auto& q:in.points)q.velocity=0;check(!ReframeLocalRecovery(in,target,"local/native/1","test",10.1,.1),"zero-speed valid wait is not recovery");
 return failures?1:0;
}
