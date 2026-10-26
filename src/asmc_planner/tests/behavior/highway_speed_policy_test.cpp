#include "Behavior/Decision/behavior_decision.hpp"
#include <cmath>
#include <iostream>

int main() {
    using namespace behavior_planner;
    int failures=0,checks=0;
    const auto expect=[&](bool ok,const char* message) {
        ++checks;if(!ok){++failures;std::cerr<<"FAIL: "<<message<<'\n';}
    };
    const double cap=70.0/3.6;
    UrbanFeature feature;feature.timestamp=1;
    feature.ego.valid=feature.ego.state_valid=true;feature.ego.speed=10;
    feature.route.current_link_valid=true;feature.route.current_link_id="A2256W000411";
    feature.route.speed_limit=40.0/3.6;
    feature.highway.valid=feature.highway.approach=true;
    feature.highway.source_link="A2256W000411";
    feature.highway.target_link="A2256W000153";feature.highway.speed_cap_mps=cap;
    BehaviorDecision decision;decision.setAbsoluteSpeedLimit(57.5/3.6);
    const auto merge=decision.decide(feature,PlannerFeedbackData{},0);
    expect(merge.selected_behavior==BehaviorType::MERGE,"highway approach selects MERGE");
    expect(std::fabs(merge.target.desired_speed-cap)<1e-6,"first request carries configured highway speed rather than frozen ramp speed");
    expect(merge.target.current_link_id==feature.highway.source_link&&
           merge.target.target_link_id==feature.highway.target_link,"speed policy preserves source and target identities");
    feature.highway={};
    BehaviorDecision ordinary;ordinary.setAbsoluteSpeedLimit(57.5/3.6);
    const auto keep=ordinary.decide(feature,PlannerFeedbackData{},0);
    expect(std::fabs(keep.target.desired_speed-40.0/3.6)<1e-6,"ordinary road retains its map speed cap");
    NormalCruiseConfig cruise;cruise.enabled=true;cruise.target_speed=cruise.max_speed=58/3.6;
    cruise.minimum_map_speed=40/3.6;
    expect(ordinary.setNormalCruiseConfig(cruise),"load ordinary 58 km/h profile");
    feature.highway.cruising=true;feature.highway.speed_cap_mps=cap;
    const auto highway_keep=ordinary.decide(feature,PlannerFeedbackData{},0);
    expect(highway_keep.selected_behavior==BehaviorType::KEEP&&
           std::fabs(highway_keep.target.desired_speed-cap)<1e-6,
           "an adopted highway lane keeps 70 rather than inheriting ordinary KEEP 58");
    feature.highway.cruising=false;
    expect(std::fabs(ordinary.decide(feature,PlannerFeedbackData{},0).target.desired_speed-57.5/3.6)<1e-6,
           "leaving the explicit active highway restores the main absolute ordinary cruise ceiling");
    feature.highway.valid=feature.highway.approach=true;feature.highway.speed_cap_mps=cap;
    feature.signal.has_stop_line=true;feature.signal.stop_line_s=100;
    feature.signal.distance_to_stop_line=5;
    HardRuleFilter rules;const auto approach=rules.apply(feature,nullptr);
    expect(approach.intersection_approach_active&&approach.max_speed<=25.0/3.6+1e-6,
           "signal approach still limits highway speed to 25 km/h");
    feature.signal.light_state=TrafficLightState::RED;
    expect(rules.apply(feature,nullptr).planned_stop,"red light still requires planned STOP");
    feature.risk.emergency_risk=true;
    expect(rules.apply(feature,nullptr).emergency_stop,"emergency stop still overrides highway policy");
    UrbanFeature scoped=feature;scoped.signal={};scoped.risk={};
    scoped.highway.valid=scoped.highway.approach=false;scoped.highway.cruising=true;
    scoped.dynamic.in_scope=true;scoped.dynamic.section_speed_limit=25.0/3.6;
    BehaviorDecision dynamic_section;dynamic_section.setAbsoluteSpeedLimit(57.5/3.6);
    const auto capped=dynamic_section.decide(scoped,PlannerFeedbackData{},0);
    expect(capped.selected_behavior==BehaviorType::KEEP &&
           capped.target.desired_speed<=25.0/3.6+1e-6,
           "main dynamic-section ceiling remains lower than highway cruise");
    scoped.dynamic.action=DynamicAction::EMERGENCY;
    const auto hazard=dynamic_section.decide(scoped,PlannerFeedbackData{},0);
    expect(hazard.selected_behavior==BehaviorType::EMERGENCY_STOP,
           "main imminent-dynamic hazard retains emergency ownership");
    std::cout<<checks<<" checks, "<<failures<<" failures\n";
    return failures?1:0;
}
