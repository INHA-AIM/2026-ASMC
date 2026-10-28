#pragma once
#include "Map/HighwayMergeRoute.hpp"
#include "Frenet/Common/PlannerCommand.hpp"
#include <chrono>

// Road ownership comes from the configured, validated lane graph, not from
// which reference core is active. Empty/unrecognized source links abstain.
inline bool IsHighwayCruiseCommand(const HighwayMergeConfig& highway,
    const asmc_planner::lane_reference::HdLaneMap* map,const PlannerCommand& command) {
    if(!map||command.source_link_id.empty()||
       (command.mode!=LANE_KEEPING&&command.mode!=FOLLOWING))return false;
    for(const auto& zone:highway.zones) {
        if(command.source_link_id==zone.source_link&&map->lane("lane:"+zone.source_link))return true;
        for(const auto& id:zone.target_lanes) {
            const auto* lane=map->lane(id);
            if(lane&&lane->link_id==command.source_link_id)return true;
        }
    }
    return false;
}

// The same highway policy follows KEEP/FOLLOW across reference handover.
// External target/max caps and physical checks remain authoritative.
inline CurveSpeedConfig ResolveHighwayCurveSpeed(
    const CurveSpeedConfig& ordinary, const HighwayMergeConfig& highway,
    bool enabled, const PlannerCommand& command,bool highway_cruise=false) {
    auto policy = ordinary;
    if (enabled && ((command.mode == MERGE && command.merge_type == MergeType::HIGHWAY)||
                    (highway_cruise&&(command.mode==LANE_KEEPING||command.mode==FOLLOWING))) &&
        std::isfinite(highway.speed_cap_mps) && highway.speed_cap_mps > 0) {
        policy.target_vel = highway.speed_cap_mps;
        policy.lateral_accel_safety_factor = highway.curve_lateral_accel_safety_factor;
        policy.comfortable_decel_factor = highway.curve_comfortable_decel_factor;
    }
    return policy;
}

struct HighwayCandidateCheck {
    bool safe = false, covers_completion = false;
    double entry_time_s = -1, clear_time_s = -1;
    double minimum_front_gap_m = INFINITY, minimum_rear_gap_m = INFINITY;
    double minimum_front_ttc_s = INFINITY, minimum_rear_ttc_s = INFINITY;
    double maximum_rear_decel_mps2 = 0;
    int front_id = -1, rear_id = -1, blocking_object_id = -1;
    std::size_t evaluated_samples = 0;
    // Prefix certified by this check, including the completion continuation.
    // Zero on failure; generation must retain exactly this checked prefix.
    std::size_t certified_path_samples = 0;
    std::string reason;
};

// This replaces the old FindHighwayMergeGap approximation. Candidate generation
// belongs to Frenet. No independent entry-time or surrogate ego-path scan here.
HighwayCandidateCheck EvaluateHighwayMergeCandidate(
    const CartesianPath&, const HighwayMergeRoute::Zone&, const RefLine& route,
    const std::vector<TrackedObject>&, const HighwayMergeConfig&,
    const VehicleShape&, const CollisionCheckConfig&, double sample_dt,
    bool entering,double object_time_offset_s=0,
    std::chrono::steady_clock::time_point deadline=std::chrono::steady_clock::time_point::max(),
    bool reference_transition=false);
