#pragma once
namespace asmc_planner::execution {
// Facts are computed in Frenet with the existing geometry/config/clocks.
// No phase, transition decision, projected coordinate or trajectory lives here.
struct CommitFacts { bool candidate_valid; bool selected_target_finite; };
struct AvoidEntryFacts { bool entry_reached; };
struct AvoidShiftFacts { bool longitudinally_cleared; bool lateral_target_reached; };
struct AvoidReturnFacts { bool lane_center_reached; };
struct LaneChangeFacts { bool position_ok; bool speed_ok; bool heading_ok; };
struct StopFacts { bool braking_trigger_reached; bool at_target; bool low_speed; };
}
