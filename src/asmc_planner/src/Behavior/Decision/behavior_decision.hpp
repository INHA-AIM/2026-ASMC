#pragma once
#include <map>
#include <string>
#include "asmc_msgs/PlanFeedback.h"
#include "Behavior/Decision/behavior_scorer.hpp"
#include "Behavior/Decision/fsm.hpp"
#include "Behavior/Decision/planner_feedback_data.hpp"
#include "Behavior/Rule/hard_rule_filter.hpp"
namespace behavior_planner {
// Explicit MORAI test profile; disabled unless configured. It may replace a
// map cap only on ordinary KEEP links at or above minimum_map_speed, or on
// explicitly configured links. Units inside Behavior are m/s.
struct NormalCruiseConfig {
  bool enabled = false;
  double target_speed = 0.0;
  double max_speed = 0.0;
  double minimum_map_speed = 0.0;
  std::map<std::string, double> link_speed_caps;
};
class BehaviorDecision { public: BehaviorContextData decide(const UrbanFeature&,const PlannerFeedbackData&,uint32_t active_command_id=0); BehaviorContextData decide(const UrbanFeature&,const asmc_msgs::PlanFeedback&); void setEmergencyRelease(double exit_ttc,double hold_s){fsm_.setEmergencyRelease(exit_ttc,hold_s);} void setFollowReleaseDelay(double delay_s){fsm_.setFollowReleaseDelay(delay_s);} void setAvoidReleaseGraceCycles(int cycles){fsm_.setAvoidReleaseGraceCycles(cycles);} void setTrafficStopConfig(const TrafficStopConfig& config){rules_.setTrafficStopConfig(config);} bool setNormalCruiseConfig(const NormalCruiseConfig& config); private:NormalCruiseConfig normal_cruise_;HardRuleFilter rules_;BehaviorScorer scorer_;Fsm fsm_; }; }
