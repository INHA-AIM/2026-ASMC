#pragma once
#include "asmc_msgs/PlanFeedback.h"
#include "behavior_planner/decision/behavior_scorer.hpp"
#include "behavior_planner/decision/fsm.hpp"
#include "behavior_planner/rule/hard_rule_filter.hpp"
namespace behavior_planner { class BehaviorDecision { public: BehaviorContextData decide(const UrbanFeature&,const asmc_msgs::PlanFeedback&); void setEmergencyRelease(double exit_ttc,double hold_s){fsm_.setEmergencyRelease(exit_ttc,hold_s);} private:HardRuleFilter rules_;BehaviorScorer scorer_;Fsm fsm_; }; }
