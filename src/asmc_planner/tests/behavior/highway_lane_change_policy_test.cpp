#include "Behavior/Feature/feature_builder.hpp"
#include "Behavior/Decision/behavior_decision.hpp"
#include "Behavior/Map/mgeo_loader.hpp"
#include "Output/Behavior/utils.hpp"
#include "Execution/HighwayReferenceExecution.hpp"
#include <iostream>
#include <cmath>
#include <stdexcept>
using namespace behavior_planner;
using namespace asmc_planner::lane_reference;
namespace {
struct Green:TrafficLightProvider{TrafficLightState getState(const std::string&,SignalMovement)const override{return TrafficLightState::GREEN;}bool hasRecentDetection(const std::string&)const override{return true;}};
int checks=0,failures=0;
void expect(bool ok,const char* m){++checks;if(!ok){++failures;std::cerr<<"FAIL "<<m<<'\n';}}
behavior_planner::TrackedObject actor(const RefLine& ref,double s,double speed,int id){const auto p=Interpolate(ref,s);behavior_planner::TrackedObject o;o.id=id;o.type=ObjectType::VEHICLE;o.motion_state=MotionState::DYNAMIC;o.x=p.x;o.y=p.y;o.yaw=p.theta;o.speed=speed;o.vx=speed*std::cos(p.theta);o.vy=speed*std::sin(p.theta);o.width=1.8;o.length=4.5;return o;}
void run(){
 const std::string root=BEHAVIOR_PLANNER_SOURCE_DIR;MGeoLoader map;RouteManager route;
 if(!map.load(root+"/config/behavior/link_set.json",root+"/config/behavior/node_set.json")||!route.initialize(map,loadRouteLinks(root+"/config/behavior/route_links_1.yaml")))throw std::runtime_error("map unavailable");
 FrenetPlanner mission;std::string detail;asmc_planner::execution::ManeuverState state;
 if(!mission.Init(root+"/config/frenet/params.yaml")||!mission.EnableHighwayMerge(detail))throw std::runtime_error(detail);
 HighwayReferenceExecution execution(mission,root+"/config/frenet/params.yaml",state);auto status=execution.status();
 status.lane=std::shared_ptr<const LaneReference>(status.network->lanes[0]);status.completed[0]=true;status.epoch=10;
 FeatureBuilder builder;builder.setHighwayRoute(mission.highway_route());builder.setHighwayReference(status);Green green;
 auto pose=Interpolate(status.lane->line(),170);EgoState ego{pose.x,pose.y,pose.theta,16.7,100,1,true};
 PerceptionObjects objects;objects.vehicles.push_back(actor(status.lane->line(),200,13,9));
 objects.source_stamp=100;
 auto feature=builder.build(ego,objects,route,green);
 std::cerr<<"source="<<feature.route.current_link_id<<" leader="<<feature.surrounding.has_front_vehicle<<" left="<<feature.lane_change.left.available<<" right="<<feature.lane_change.right.available<<" remaining="<<feature.lane_change.remaining_m<<" free="<<feature.lane_change.free_speed<<" merge="<<feature.highway.approach<<'\n';
 expect(feature.lane_change.left.available,"actual HD-map adjacency with empty visible target produces offer");
 expect(!feature.lane_change.right.available,"outside road / entry ramp is not an LC target");
 expect(feature.surrounding.has_front_vehicle&&feature.surrounding.front_vehicle.id==9,"slow leader is from current lane");
 BehaviorDecision decision;BehaviorContextData context;
 // Repeated timers on one object frame cannot mature the one-second witness.
 for(int n=0;n<30;++n){feature.timestamp=100+n*.05;context=decision.decide(feature,PlannerFeedbackData{},0);}
 expect(context.selected_behavior==BehaviorType::FOLLOW,"duplicate observations never approve LC");
 decision=BehaviorDecision{};
 for(int n=0;n<14;++n){ego.stamp=100+n*.1;objects.source_stamp=ego.stamp;feature=builder.build(ego,objects,route,green);context=decision.decide(feature,PlannerFeedbackData{},n>=11?200:0);}
 std::cerr<<"selected="<<int(context.selected_behavior)<<" phase="<<int(context.maneuver_phase)<<'\n';
 expect(context.selected_behavior==BehaviorType::LEFT_CHANGE&&context.target.has_target_pose,"persistent actual speed deficit requests one adjacent LC");
 const auto prepared=decision;
 // One missing lead frame preserves intent, never a stale READY or new LC.
 const auto with_lead=feature;
 auto missing=feature;missing.surrounding.has_front_vehicle=false;missing.surrounding.front_vehicle={};
 missing.timestamp+=.1;missing.lane_change.evaluation_stamp+=.1;missing.lane_change.observation_stamp+=.1;
 auto retained=prepared;
 auto grace_context=retained.decide(missing,PlannerFeedbackData{},200);
 expect(grace_context.selected_behavior==BehaviorType::LEFT_CHANGE&&
     grace_context.extra_flags["lane_change_lead_observation_grace"],"brief lead loss preserves the existing PREPARE only");
 PlannerFeedbackData grace_ready;grace_ready.valid=true;grace_ready.command_id=200;
 grace_ready.execution_state=PlannerExecutionState::READY;
 grace_context=retained.decide(missing,grace_ready,200);
 expect(grace_context.selected_behavior==BehaviorType::LEFT_CHANGE&&grace_context.maneuver_phase==ManeuverPhase::EXECUTE,
     "latest matching Frenet READY can authorize the retained intent");
 auto wrong_ready=prepared;grace_ready.command_id=201;
 auto wrong_context=wrong_ready.decide(missing,grace_ready,200);
 expect(wrong_context.maneuver_phase==ManeuverPhase::PREPARE,
     "intent grace never accepts READY belonging to another command");
 auto reversed=missing;reversed.lane_change.evaluation_stamp=with_lead.lane_change.observation_stamp-.01;
 reversed.timestamp=reversed.lane_change.evaluation_stamp;
 auto reversed_decision=prepared;
 expect(reversed_decision.decide(reversed,PlannerFeedbackData{},200).selected_behavior!=BehaviorType::LEFT_CHANGE,
     "clock reversal cannot extend the lead-observation grace");
 auto expired=missing;expired.timestamp+=.3;expired.lane_change.evaluation_stamp+=.3;expired.lane_change.observation_stamp+=.3;
 auto expired_decision=prepared;auto expired_context=expired_decision.decide(expired,PlannerFeedbackData{},200);
 expect(expired_context.selected_behavior!=BehaviorType::LEFT_CHANGE,"lead absence beyond 300ms cancels uncommitted preparation");
 BehaviorDecision empty;expect(empty.decide(missing,PlannerFeedbackData{},0).selected_behavior!=BehaviorType::LEFT_CHANGE,
     "missing lead never creates a new optional request");
 for(int veto=0;veto<6;++veto) {
   auto input=missing;auto isolated=prepared;
   if(veto==0)input.lane_change.fresh=false;
   if(veto==1)input.lane_change.left.available=false;
   if(veto==2)input.lane_change.reference_epoch++;
   if(veto==3)input.risk.pedestrian_risk=true;
   if(veto==4)input.risk.static_obstacle_ahead=true;
   if(veto==5)input.lane_change.left.target_link="different";
   expect(isolated.decide(input,PlannerFeedbackData{},200).selected_behavior!=BehaviorType::LEFT_CHANGE,
       "grace never overrides freshness, target, epoch or obstacle vetoes");
 }
 auto interrupted=prepared;auto emergency=missing;
 PlannerFeedbackData crossing;crossing.valid=true;crossing.command_id=200;crossing.execution_state=PlannerExecutionState::EXECUTING;
 interrupted.decide(with_lead,crossing,200);
 emergency.risk.emergency_risk=true;emergency.risk.min_ttc=.1;
 expect(interrupted.decide(emergency,crossing,200).selected_behavior==BehaviorType::EMERGENCY_STOP,
     "emergency preempts the crossing and its frozen optional offer");
 emergency.risk.emergency_risk=false;emergency.risk.min_ttc=1e9;
 crossing.command_id=201;
  BehaviorContextData emergency_context;
 for(int n=0;n<20;++n){emergency.timestamp+=.1;emergency.lane_change.evaluation_stamp+=.1;
   emergency.lane_change.observation_stamp+=.1;emergency_context=interrupted.decide(emergency,crossing,201);}
 expect(emergency_context.selected_behavior==BehaviorType::KEEP,
     "emergency release cannot reissue the interrupted source/target as a new LEFT_CHANGE");
 auto fast=with_lead;fast.timestamp+=.1;fast.lane_change.evaluation_stamp+=.1;
 fast.lane_change.observation_stamp+=.1;
 fast.surrounding.front_vehicle.vx=fast.lane_change.free_speed;
 fast.surrounding.front_vehicle.vy=0;
 auto benefit_lost=prepared;
 expect(benefit_lost.decide(fast,PlannerFeedbackData{},200).selected_behavior!=BehaviorType::LEFT_CHANGE,
     "an actual fast-lead observation withdraws the old benefit immediately");
 const auto frozen=context.target;
 PlannerFeedbackData feedback;feedback.valid=true;feedback.command_id=200;feedback.execution_state=PlannerExecutionState::READY;
 ego.stamp+=.1;objects.source_stamp=ego.stamp;feature=builder.build(ego,objects,route,green);context=decision.decide(feature,feedback,200);
 expect(context.maneuver_phase==ManeuverPhase::EXECUTE,"matching READY authorizes EXECUTE through existing FSM");
 feedback.execution_state=PlannerExecutionState::EXECUTING;
 const auto target=status.network->lanes[1];
 const double target_s=FindClosestS(target->line(),ego.x,ego.y,nullptr,100);
 objects.vehicles.push_back(actor(target->line(),target_s-12,19,20));
 ego.stamp+=.1;objects.source_stamp=ego.stamp;feature=builder.build(ego,objects,route,green);context=decision.decide(feature,feedback,200);
 expect(context.selected_behavior==BehaviorType::LEFT_CHANGE&&context.maneuver_phase==ManeuverPhase::COMMITTED,"post-commit target traffic does not cancel lateral ownership");
 expect(context.target.target_link_id==frozen.target_link_id&&context.target.target_x==frozen.target_x,"same request keeps immutable target geometry");
 status.lane=target;status.epoch++;builder.setHighwayReference(status);pose=Interpolate(target->line(),target_s);
 ego.x=pose.x;ego.y=pose.y;ego.yaw=pose.theta;ego.stamp+=.1;objects.vehicles.clear();objects.source_stamp=ego.stamp;
 feedback.execution_state=PlannerExecutionState::COMPLETED;feature=builder.build(ego,objects,route,green);context=decision.decide(feature,feedback,200);
 expect(context.selected_behavior==BehaviorType::KEEP,"completion returns to normal driving on adopted lane");
 expect(std::fabs(feature.ego.d)<.1,"Behavior ego and leader basis follows adopted reference");
 // Fresh inputs, late road, occlusion and target occupancy have independent vetoes.
 status.lane=status.network->lanes[0];status.epoch++;builder.setHighwayReference(status);pose=Interpolate(status.lane->line(),170);ego={pose.x,pose.y,pose.theta,16.7,200,1,true};
 objects.vehicles={actor(status.lane->line(),200,13,9)};objects.source_stamp=0;
 feature=builder.build(ego,objects,route,green);expect(!feature.lane_change.fresh&&!feature.lane_change.left.available,"missing object header is not an observed empty lane");
 objects.source_stamp=200;objects.vehicles.push_back(actor(target->line(),target_s+10,13,30));
 feature=builder.build(ego,objects,route,green);expect(feature.lane_change.left.available,"observed target traffic is offered to Frenet's timed gap check");
 objects.vehicles={actor(status.lane->line(),180,13,9)};
 feature=builder.build(ego,objects,route,green);expect(feature.lane_change.left.available,"near source leader does not falsely certify or veto an empty target window");
 objects.vehicles={actor(status.lane->line(),200,13,9)};
 const auto& start=status.network->common->line().points.front();const double end=FindClosestS(status.lane->line(),start.x,start.y,nullptr,100);
 pose=Interpolate(status.lane->line(),end-65);ego={pose.x,pose.y,pose.theta,16,300,1,true};objects.source_stamp=300;
 feature=builder.build(ego,objects,route,green);
 expect(feature.highway.approach&&feature.highway.source_link==feature.route.current_link_id,"mandatory taper uses adopted current lane, not original 445 source");
 BehaviorDecision late;context=late.decide(feature,PlannerFeedbackData{},0);
 expect(context.selected_behavior==BehaviorType::MERGE,"mandatory downstream merge preempts optional request");
 // There is enough room to finish a compatible left change and settle before
 // the shared successor, even though mandatory-merge observation has started.
 pose=Interpolate(status.lane->line(),end-110);ego={pose.x,pose.y,pose.theta,16,350,1,true};
 objects.vehicles={actor(status.lane->line(),end-85,6,99)};objects.source_stamp=350;
 BehaviorDecision merge_left;
 for(int n=0;n<15;++n){ego.stamp=350+n*.1;objects.source_stamp=ego.stamp;
   feature=builder.build(ego,objects,route,green);context=merge_left.decide(feature,PlannerFeedbackData{},300);}
 expect(feature.highway.approach&&feature.lane_change.left.available,
     "late compatible adjacent lane is observed inside merge approach");
 expect(context.selected_behavior==BehaviorType::LEFT_CHANGE,
     "safe beneficial left preparation can replace an uncommitted downstream merge");
 expect(HighwayLeftChangeSupportsMerge(*status.network,feature.lane_change.left.source_link,feature.lane_change.left.target_link),
     "allowed left offer shares the prepared downstream successor");
 expect(!HighwayLeftChangeSupportsMerge(*status.network,feature.lane_change.left.target_link,feature.lane_change.left.source_link),
     "opposite direction never receives the merge-progress reserve");
 auto too_late=feature;too_late.lane_change.remaining_m=10;BehaviorDecision closed;
 expect(closed.decide(too_late,PlannerFeedbackData{},0).selected_behavior==BehaviorType::MERGE,
     "insufficient remaining change and settling room keeps mandatory merge");
 BehaviorDecision committed_merge;auto no_benefit=feature;no_benefit.surrounding.has_front_vehicle=false;
 committed_merge.decide(no_benefit,PlannerFeedbackData{},400);
 PlannerFeedbackData locked;locked.valid=true;locked.command_id=400;locked.execution_state=PlannerExecutionState::EXECUTING;
 for(int n=0;n<15;++n){feature.timestamp+=.1;feature.lane_change.evaluation_stamp+=.1;
   feature.lane_change.observation_stamp+=.1;context=committed_merge.decide(feature,locked,400);}
 expect(context.selected_behavior==BehaviorType::MERGE,
     "committed mandatory merge cannot be replaced by a new optional left request");
 // Independently delivered inputs are fresh against the shared decision clock.
 pose=Interpolate(status.lane->line(),170);ego={pose.x,pose.y,pose.theta,16.7,400,1,true};
 objects.vehicles={actor(status.lane->line(),200,13,9)};
 BehaviorDecision asynchronous;
 for(int i=0;i<14;++i) {
   ego.stamp=400+i*.1;objects.source_stamp=ego.stamp+.002;
   feature=builder.build(ego,objects,route,green,ego.stamp+.03);
   context=asynchronous.decide(feature,PlannerFeedbackData{},0);
 }
 expect(feature.lane_change.fresh&&context.selected_behavior==BehaviorType::LEFT_CHANGE,
     "two milliseconds newer object frames do not reset the confirmation witness");
 objects.source_stamp=ego.stamp-.5;
 feature=builder.build(ego,objects,route,green,ego.stamp+.03);
 expect(!feature.lane_change.fresh,"actual stale objects remain blocked");
 context=asynchronous.decide(feature,PlannerFeedbackData{},0);
 for(int i=0;i<14;++i) {
   ego.stamp=402+i*.1;objects.source_stamp=ego.stamp+.002;
   feature=builder.build(ego,objects,route,green,ego.stamp+.03);
   context=asynchronous.decide(feature,PlannerFeedbackData{},0);
 }
 expect(context.selected_behavior==BehaviorType::LEFT_CHANGE,
     "canceled uncommitted preparation reconfirms without an eight-second executed-change cooldown");
 objects.source_stamp=ego.stamp+.2;
 feature=builder.build(ego,objects,route,green,ego.stamp+.03);
 expect(!feature.lane_change.fresh,"an object timestamp truly ahead of decision time is rejected");
 objects.source_stamp=ego.stamp+.4;
 feature=builder.build(ego,objects,route,green,ego.stamp+.43);
 expect(!feature.lane_change.fresh,"stale ego is independently rejected even with fresh objects");
}
}
int main(){try{run();}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}std::cout<<checks<<" checks, "<<failures<<" failures\n";return failures?1:0;}
