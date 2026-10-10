#include "Behavior/Decision/fsm.hpp"
#include "Execution/IntentEncoder.hpp"
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
