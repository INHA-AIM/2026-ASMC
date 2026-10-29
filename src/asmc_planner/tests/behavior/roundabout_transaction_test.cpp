#include "Behavior/Decision/fsm.hpp"
#include "Execution/IntentEncoder.hpp"
#include "Execution/RoundaboutReadyNotice.hpp"
#include <iostream>
#include <filesystem>
#include <fstream>
#include <chrono>
#include "Behavior/Feature/feature_builder.hpp"
#include "Behavior/Decision/behavior_decision.hpp"
using namespace behavior_planner;
int main(){
 int failures=0;auto expect=[&](bool ok,const char* name){std::cout<<(ok?"[PASS] ":"[FAIL] ")<<name<<'\n';failures+=!ok;};
 Fsm fsm;V2CommandManager wire;
 auto decision=fsm.updateManeuver(BehaviorType::MERGE,{},0,false);
 expect(decision.phase==ManeuverPhase::PREPARE,"MERGE starts PREPARE");
 BehaviorContextData context;context.selected_behavior=decision.behavior;context.maneuver_phase=decision.phase;context.target.current_link_id="entry";context.target.target_link_id="merge";context.target.desired_speed=25.0/3.6;
 auto command=wire.update(context,nullptr,ros::Time(100),.5);uint32_t id=command.command_id;
 expect(command.maneuver_type==asmc_msgs::BehaviorCommand::MERGE && command.merge_type==asmc_msgs::BehaviorCommand::MERGE_ROUNDABOUT && command.target_link_id=="merge" && command.preferred_front_object_id==-1,"wire names stable zone and leaves dynamic gap IDs unbound");
 {
  using Command=asmc_msgs::BehaviorCommand;using Feedback=asmc_msgs::FrenetFeedback;
  RoundaboutReadyNotice notice;notice.issuedCommand(command);
  Feedback ready;ready.command_id=id;ready.command_accepted=ready.plan_valid=true;
  ready.active_maneuver=Command::MERGE;ready.execution_state=Feedback::READY;
  auto waiting=ready;waiting.execution_state=Feedback::WAITING;waiting.plan_valid=false;
  notice.observe(ready,100);notice.observe(waiting,100.039);
  const bool pending=notice.consume(&waiting,100.1,1.0);
  expect(pending,"39ms READY survives WAITING until the next 100ms Behavior tick");
  Fsm pulse_fsm;V2CommandManager pulse_wire;BehaviorContextData pulse_context=context;
  auto phase=pulse_fsm.updateManeuver(BehaviorType::MERGE,{},0,false);
  pulse_context.maneuver_phase=phase.phase;
  auto pulse_command=pulse_wire.update(pulse_context,nullptr,ros::Time(100),.5);
  PlannerFeedbackData handshake;handshake.valid=true;handshake.command_id=pulse_command.command_id;
  handshake.execution_state=pending?PlannerExecutionState::READY:PlannerExecutionState::WAITING;
  phase=pulse_fsm.updateManeuver(BehaviorType::MERGE,handshake,pulse_command.command_id,false);
  pulse_context.maneuver_phase=phase.phase;waiting.command_id=pulse_command.command_id;
  auto requested=pulse_wire.update(pulse_context,&waiting,ros::Time(100.1),.5);
  expect(requested.command_id==pulse_command.command_id&&requested.execution_request==Command::EXECUTE,
    "remembered notification requests execution without rewriting the latest WAITING feedback");
  expect(!notice.consume(&waiting,100.11,1),"READY notification is consumed once");
  notice.observe(ready,200);expect(!notice.consume(&waiting,201.01,1),"expired READY cannot request execution");
  notice.observe(ready,202);expect(!notice.consume(&waiting,201.9,1),"backward receipt clock cannot request execution");
  auto replacement=command;replacement.command_id=id+1;
  notice.observe(ready,203);notice.issuedCommand(replacement);
  expect(!notice.consume(&waiting,203.1,1),"command replacement discards old READY");
  notice.observe(ready,203.2);
  expect(!notice.consume(&waiting,203.3,1),"late feedback for an old command cannot rearm the notice");
  notice.issuedCommand(command);notice.observe(ready,204);
  auto highway=command;highway.merge_type=Command::MERGE_HIGHWAY;
  notice.issuedCommand(highway);
  expect(!notice.consume(&waiting,204.1,1),"highway MERGE cannot consume roundabout READY");
  for(auto type:{Command::AVOID,Command::STOP,Command::EMERGENCY_STOP}) {
   notice.issuedCommand(command);notice.observe(ready,205);
   auto other=command;other.maneuver_type=type;notice.issuedCommand(other);
   expect(!notice.consume(&waiting,205.1,1),"another maneuver or emergency discards roundabout READY");
  }
  notice.issuedCommand(command);notice.observe(ready,206);
  auto cancel=command;cancel.execution_request=Command::CANCEL;notice.issuedCommand(cancel);
  expect(!notice.consume(&waiting,206.1,1),"CANCEL discards roundabout READY");
  for(auto state:{Feedback::COMMITTED,Feedback::EXECUTING,Feedback::COMPLETED,Feedback::FAILED}) {
   replacement.command_id++;ready.command_id=waiting.command_id=replacement.command_id;
   notice.issuedCommand(replacement);notice.observe(ready,207);
   auto closed=ready;closed.execution_state=state;notice.observe(closed,207.02);
   notice.observe(ready,207.03);
   expect(!notice.consume(&waiting,207.1,1),"commit or terminal feedback closes READY against late replay");
  }
  replacement.command_id++;notice.issuedCommand(replacement);
  ready.command_id=waiting.command_id=replacement.command_id;
  ready.plan_valid=false;notice.observe(ready,208);
  expect(!notice.consume(&waiting,208.1,1),"invalid READY is never cached");
  ready.plan_valid=true;ready.command_accepted=false;notice.observe(ready,208.2);
  expect(!notice.consume(&waiting,208.3,1),"rejected READY is never cached");
  ready.command_accepted=true;notice.observe(ready,209);
  expect(!notice.consume(nullptr,209.1,1)&&notice.consume(&waiting,209.2,1),
   "missing latest feedback does not consume a still-fresh notification");
  notice.observe(ready,210);
  auto failed=ready;failed.execution_state=Feedback::FAILED;failed.command_accepted=false;
  notice.observe(failed,210.01);notice.observe(ready,210.02);
  expect(!notice.consume(&waiting,210.1,1),"rejected terminal feedback still prevents late READY replay");
 }
 PlannerFeedbackData feedback;feedback.valid=true;feedback.command_id=id;feedback.execution_state=PlannerExecutionState::READY;
 decision=fsm.updateManeuver(BehaviorType::MERGE,feedback,id,false);context.maneuver_phase=decision.phase;
 auto execute=wire.update(context,nullptr,ros::Time(100.1),.5);
 expect(execute.command_id==id&&execute.execution_request==asmc_msgs::BehaviorCommand::EXECUTE,"READY changes request without replacing command ID");
 context.target.current_link_id="next_link";context.extra_flags["intersection_approach_speed_limit"]=true;
 auto refreshed=wire.update(context,nullptr,ros::Time(100.2),.5);
 expect(refreshed.command_id==id&&refreshed.source_link_id=="entry","changing current link or signal approach flag cannot restart MERGE");
 feedback.execution_state=PlannerExecutionState::EXECUTING;
 decision=fsm.updateManeuver(BehaviorType::FOLLOW,feedback,id,false);
 expect(decision.behavior==BehaviorType::MERGE&&decision.phase==ManeuverPhase::COMMITTED,"FOLLOW cannot erase executing MERGE");
 decision=fsm.updateManeuver(BehaviorType::KEEP,feedback,id,false);
 expect(decision.behavior==BehaviorType::MERGE,"ordinary KEEP cannot erase merge commitment");
 feedback.execution_state=PlannerExecutionState::COMPLETED;
 decision=fsm.updateManeuver(BehaviorType::KEEP,feedback,id,false);
 expect(decision.behavior==BehaviorType::KEEP,"completed MERGE hands off to KEEP");
 fsm.updateManeuver(BehaviorType::MERGE,{},0,false);
 decision=fsm.updateManeuver(BehaviorType::EMERGENCY_STOP,{},0,false);
 expect(decision.behavior==BehaviorType::EMERGENCY_STOP,"emergency preempts MERGE");
 const auto dir=std::filesystem::temp_directory_path()/std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
 std::filesystem::create_directories(dir);
 {std::ofstream f(dir/"links.json");f<<R"([{"idx":"approach","from_node_idx":"start","to_node_idx":"yield","max_speed":50,"points":[[0,0],[100,0]]},{"idx":"merge","from_node_idx":"yield","to_node_idx":"exit","max_speed":30,"points":[[100,0],[140,0]]}])";}
 {std::ofstream f(dir/"nodes.json");f<<R"([{"idx":"yield","point":[100,0],"on_stop_line":true}])";}
 MGeoLoader map;RouteManager route;expect(map.load((dir/"links.json").string(),(dir/"nodes.json").string())&&route.initialize(map,{"approach","merge"}),"synthetic map route exposes unsignalized yield stopline");
 struct NoSignals:TrafficLightProvider{TrafficLightState getState(const std::string&,SignalMovement)const override{return TrafficLightState::UNKNOWN;}bool hasRecentDetection(const std::string&)const override{return false;}} signals;
 FeatureBuilder builder;builder.setRoundaboutMerge(true,"yield","approach","merge");
 EgoState vehicle;vehicle.x=10;vehicle.y=0;vehicle.yaw=0;vehicle.speed=5;vehicle.stamp=200;vehicle.valid=true;
 auto feature=builder.build(vehicle,{},route,signals);expect(!feature.roundabout.approach,"slow ego does not activate merge ninety metres early");
 vehicle.speed=18;auto fast=builder.build(vehicle,{},route,signals);expect(fast.roundabout.approach,"high measured speed starts approach early enough to stop");
 vehicle.x=70;vehicle.speed=5;feature=builder.build(vehicle,{},route,signals);
 BehaviorDecision selector;auto selected=selector.decide(feature,PlannerFeedbackData{},0);
 expect(selected.selected_behavior==BehaviorType::MERGE&&selected.maneuver_phase==ManeuverPhase::PREPARE&&selected.target.desired_speed<=25.0/3.6,"approach selects MERGE PREPARE with twenty-five km/h ceiling");
 feature.risk.emergency_risk=true;selected=selector.decide(feature,PlannerFeedbackData{},0);expect(selected.selected_behavior==BehaviorType::EMERGENCY_STOP,"approach selection cannot suppress emergency risk");
 std::filesystem::remove_all(dir);
 return failures?1:0;
}
