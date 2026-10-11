#pragma once
#include "asmc_msgs/PlanFeedback.h"
#include "Behavior/Decision/behavior_scorer.hpp"
#include "Behavior/Decision/fsm.hpp"
#include "Behavior/Decision/planner_feedback_data.hpp"
#include "Behavior/Rule/hard_rule_filter.hpp"
namespace behavior_planner { class BehaviorDecision { public: BehaviorContextData decide(const UrbanFeature&,const PlannerFeedbackData&,uint32_t active_command_id=0); BehaviorContextData decide(const UrbanFeature&,const asmc_msgs::PlanFeedback&); void setEmergencyRelease(double exit_ttc,double hold_s){fsm_.setEmergencyRelease(exit_ttc,hold_s);} void setFollowReleaseDelay(double delay_s){fsm_.setFollowReleaseDelay(delay_s);} void setAvoidReleaseGraceCycles(int cycles){fsm_.setAvoidReleaseGraceCycles(cycles);} void setTrafficStopConfig(const TrafficStopConfig& config){rules_.setTrafficStopConfig(config);} private:HardRuleFilter rules_;BehaviorScorer scorer_;Fsm fsm_; }; }
