#pragma once
#include <string>
#include <unordered_map>
#include <vector>
#include "behavior_planner/context/behavior_types.hpp"
namespace behavior_planner {
struct HardConstraint { bool force_stop=false, planned_stop=false, emergency_stop=false, forbid_left_change=false, forbid_right_change=false, forbid_lane_change=false, stop_line_constraint=false; double stop_before_s=1e9, max_speed=1e9; };
struct BehaviorEnable { bool keep=true, follow=false, left_change=false, right_change=false, avoid=false, stop=false, emergency_stop=false; };
struct BehaviorScore { double keep=0, follow=0, left_change=0, right_change=0, avoid=0, stop=0, emergency_stop=0; };
struct BehaviorTarget { std::string current_link_id, route_required_link_id, target_link_id; std::vector<std::string> candidate_link_ids; double desired_speed=0, stop_before_s=1e9, stop_x=0, stop_y=0, target_x=0, target_y=0, target_yaw=0; int lead_vehicle_id=-1, avoidance_object_id=-1; double follow_time_gap=1.5, follow_min_gap=5; bool has_stop_target=false, has_target_pose=false; };
struct BehaviorDebug { std::string selected_reason, hard_rule_reason; std::unordered_map<std::string,double> cost_terms; std::unordered_map<std::string,bool> flags; };
struct BehaviorContextData { double timestamp=0; BehaviorState state=BehaviorState::KEEP_LANE; ManeuverPhase maneuver_phase=ManeuverPhase::NONE; BehaviorType selected_behavior=BehaviorType::KEEP; HardConstraint hard; BehaviorEnable enable; BehaviorScore score; BehaviorTarget target; BehaviorDebug debug; std::unordered_map<std::string,double> extra_values; std::unordered_map<std::string,bool> extra_flags; };
}
