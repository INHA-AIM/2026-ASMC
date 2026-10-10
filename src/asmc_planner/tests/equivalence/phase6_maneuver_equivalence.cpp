#include "Execution/ManeuverState.hpp"
#include <random>
#include <stdexcept>
#include <iostream>
using namespace asmc_planner::execution;
using asmc_planner::LateralDirection;
void require(bool x){if(!x)throw std::runtime_error("maneuver transition mismatch");}
void same(const ManeuverState& a,const ManeuverState& b){
 require(a.avoid.active==b.avoid.active&&a.avoid.command_obstacle_id==b.avoid.command_obstacle_id&&
 a.avoid.obstacle_id==b.avoid.obstacle_id&&a.avoid.phase==b.avoid.phase&&a.avoid.missing_cycles==b.avoid.missing_cycles&&
 a.avoid.requested_direction==b.avoid.requested_direction&&a.avoid.committed==b.avoid.committed&&a.avoid.completed==b.avoid.completed&&
 a.lane_change.active==b.lane_change.active&&a.lane_change.committed==b.lane_change.committed&&a.lane_change.direction==b.lane_change.direction&&
 a.lane_change.phase==b.lane_change.phase&&a.follow.active==b.follow.active&&a.follow.leader_id==b.follow.leader_id&&
 a.follow.missing_cycles==b.follow.missing_cycles&&a.merge.committed==b.merge.committed&&a.merge.waiting_latched==b.merge.waiting_latched&&a.stop_phase==b.stop_phase);
}
// Direct statements below mirror Phase 5's original Frenet transition blocks.
void legacyAvoid(ManeuverState& a,bool entry,bool clear,bool lateral,bool center){
 if(!a.avoid.active||!a.avoid.committed)return;
 bool entered=false;
 if(!a.avoid.completed&&a.avoid.phase==AvoidPhase::TRACKING&&entry)a.avoid.phase=AvoidPhase::SHIFT;
 if(!a.avoid.completed&&a.avoid.phase==AvoidPhase::SHIFT&&clear)a.avoid.phase=AvoidPhase::RETURN;
 else if(!a.avoid.completed&&a.avoid.phase==AvoidPhase::SHIFT&&lateral){a.avoid.phase=AvoidPhase::PASS;entered=true;}
 if(!a.avoid.completed&&!entered&&a.avoid.phase==AvoidPhase::PASS&&clear)a.avoid.phase=AvoidPhase::RETURN;
 if(!a.avoid.completed&&a.avoid.phase==AvoidPhase::RETURN&&center)a.avoid.completed=true;
}
void currentAvoid(ManeuverExecution& e,bool entry,bool clear,bool lateral,bool center){
 if(!e->avoid.active||!e->avoid.committed)return;
 e.observeAvoidEntry({entry});bool entered=e.observeAvoidShift({clear,lateral});e.observeAvoidPass(clear,entered);e.observeAvoidReturn({center});
}
int main(){size_t exhaustive=0;
 for(int phase=0;phase<4;++phase)for(int flags=0;flags<8;++flags)for(int facts=0;facts<16;++facts){
 ManeuverState a,b;a.avoid.phase=AvoidPhase(phase);a.avoid.active=flags&1;a.avoid.committed=flags&2;a.avoid.completed=flags&4;b=a;
 ManeuverExecution e;e.bind(b);legacyAvoid(a,facts&1,facts&2,facts&4,facts&8);currentAvoid(e,facts&1,facts&2,facts&4,facts&8);same(a,b);++exhaustive;
 }
 for(int phase=0;phase<3;++phase)for(int facts=0;facts<8;++facts){ManeuverState a,b;a.stop_phase=StopExecutionPhase(phase);b=a;ManeuverExecution e;e.bind(b);
 if(a.stop_phase==StopExecutionPhase::APPROACH&&(facts&1))a.stop_phase=StopExecutionPhase::BRAKING;
 if((facts&2)&&(facts&4))a.stop_phase=StopExecutionPhase::HOLD;
 e.observeStop({bool(facts&1),bool(facts&2),bool(facts&4)});same(a,b);++exhaustive;
 a.lane_change.phase=LaneChangePhase(phase);b=a;
 if(a.lane_change.phase==LaneChangePhase::SHIFT&&(facts&1)&&(facts&2)&&(facts&4))a.lane_change.phase=LaneChangePhase::HOLD_TARGET_LANE;
 e.observeLaneChange({bool(facts&1),bool(facts&2),bool(facts&4)});same(a,b);++exhaustive;
 }
 std::mt19937 rng(20261010);size_t cycles=0;
 for(int scenario=0;scenario<200;++scenario){ManeuverState a,b;ManeuverExecution e;e.bind(b);
 for(int i=0;i<1000;++i,++cycles){auto event=rng()%24;bool x=rng()%2,y=rng()%2,z=rng()%2,w=rng()%2;int id=rng()%128,grace=rng()%8;
 switch(event){
 case 0:a.avoid.active=true;a.avoid.obstacle_id=id;e.observeAvoidObject(id);break;
 case 1:a.avoid.command_obstacle_id=id;e.bindAvoidCommandTarget(id);break;
 case 2:a.avoid.committed=true;a.avoid.completed=false;e.beginAvoidCommit();break;
 case 3:a.avoid.phase=x?AvoidPhase::SHIFT:AvoidPhase::TRACKING;e.initializeAvoidEntry({x});break;
 case 4:legacyAvoid(a,x,y,z,w);currentAvoid(e,x,y,z,w);break;
 case 5:a.avoid.phase=x?AvoidPhase::TRACKING:AvoidPhase::SHIFT;e.retimeAvoidEntry(x);break;
 case 6:if(a.avoid.phase==AvoidPhase::PASS&&x)a.avoid.phase=AvoidPhase::SHIFT;e.revalidateAvoidOffset(x);break;
 case 7:if(x)a.avoid.phase=AvoidPhase::RETURN;require(e.observeEarlyReturnCandidate(x)==x);break;
 case 8:{auto old=a.avoid,now=b.avoid;a.avoid.phase=AvoidPhase::SHIFT;e.beginAvoidShiftProbe();same(a,b);if(x){a.avoid=old;e.restoreAvoid(now);}break;}
 case 9:a.avoid.active=false;a.avoid.command_obstacle_id=-1;a.avoid.obstacle_id=-1;a.avoid.missing_cycles=0;e.clearAvoidBinding();break;
 case 10:require((++a.avoid.missing_cycles>grace)==e.observeAvoidMissing(grace));break;
 case 11:{bool ok=true;if(a.avoid.phase==AvoidPhase::TRACKING){if(x)a.avoid.missing_cycles=0;else if(++a.avoid.missing_cycles>grace)ok=false;}else a.avoid.missing_cycles=0;require(e.observeCommittedAvoidTarget(x,grace)==ok);break;}
 case 12:a.avoid=AvoidState{};e.resetAvoid();break;
 case 13:a.follow.active=true;a.follow.leader_id=id;e.observeFollowTarget(id);break;
 case 14:require((++a.follow.missing_cycles>grace)==e.observeFollowMissing(grace));break;
 case 15:a.follow=FollowState{};e.resetFollow();break;
 case 16:a.lane_change.phase=LaneChangePhase::SHIFT;a.lane_change.active=true;e.beginLaneChange();break;
 case 17:a.lane_change.committed=true;e.beginLaneChangeCommit();break;
 case 18:if(a.lane_change.phase==LaneChangePhase::SHIFT&&x&&y&&z)a.lane_change.phase=LaneChangePhase::HOLD_TARGET_LANE;e.observeLaneChange({x,y,z});break;
 case 19:a.lane_change=LaneChangeState{};e.resetLaneChange();break;
 case 20:a.merge.committed=true;a.merge.waiting_latched=false;e.beginMergeCommit();break;
 case 21:if(x)a.merge.waiting_latched=true;e.observeMergeWaiting(x);break;
 case 22:a.merge=MergeState{};e.resetMerge();break;
 case 23:if(a.stop_phase==StopExecutionPhase::APPROACH&&x)a.stop_phase=StopExecutionPhase::BRAKING;if(y&&z)a.stop_phase=StopExecutionPhase::HOLD;e.observeStop({x,y,z});break;
 }same(a,b);
 require(e.canCommitAvoid({x,y})==(x&&a.avoid.active&&!a.avoid.committed&&y));
 require(e.canCommitLaneChange({x,y})==(x&&a.lane_change.active&&!a.lane_change.committed&&y));
 require(e.canCommitMerge(x,y,z,w,true,true)==(x&&!a.merge.committed&&y&&z&&w));
 }
 // Independent copies must not retain the runtime's external storage binding.
 ManeuverExecution copied(e);auto prior=b.avoid.phase;copied.beginAvoidShiftProbe();require(b.avoid.phase==prior);
 }
 std::cout<<cycles<<" maneuver transition cycles + "<<exhaustive<<" exhaustive combinations identical\n";
}
