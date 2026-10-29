#pragma once

#include "Frenet/Config/FrenetConfig.hpp"
#include "Frenet/Evaluation/CollisionChecker.hpp"
#include "Map/HdLaneMap.hpp"
#include <memory>
#include <functional>
#include <cmath>
#include <string>

// Read-only, prepared once before enabling the normal Behavior -> Frenet path.
// Mission stations remain on the official route. Production may prepare a
// target-lane connector and adopt a separately validated lane reference.
struct HighwayMergeRoute {
    struct Zone {
        struct FollowingLane {
            std::shared_ptr<const RefLine> reference;
            std::shared_ptr<const asmc_planner::lane_reference::LaneRegion> region;
            // Per-call execution fact. Prepared/admission zones keep the full
            // allowance; a runtime copy subtracts observed physical entry age.
            double reaction_remaining_s=1.0;
            // Candidate-local response may use known incoming lane geometry
            // only up to its first unresolved successor. Zero means no hint.
            double response_lane_end_s=0;
        };
        HighwayMergeZone config;
        RefLine target_reference;
        std::shared_ptr<const asmc_planner::lane_reference::LaneRegion> target_region;
        // Last source-only stopping position, derived from the actual lane
        // footprint/target-overlap frontier, not an intermediate wait point.
        RefLine source_reference;
        std::shared_ptr<const asmc_planner::lane_reference::LaneRegion> source_region;
        // Same source lane through its common successor. This is only for
        // established following roles; source-only stop/entry ownership stays above.
        std::shared_ptr<const asmc_planner::lane_reference::LaneRegion> source_following_region;
        double entry_s=0,source_stop_s=0,source_stop_local_s=0;
        // Optional-change admission facts. Each candidate uses its own actual
        // arrival/progress against this bounded rear observation window.
        double admission_rear_observation_m=0,admission_unseen_speed_mps=0;
        // Traffic membership continues on the prepared predecessor chains.
        // It is separate from target_region, which owns handover/completion.
        // Only source and requested/adjacent joining lanes belong to this
        // certificate; unrelated parallel lanes never inherit a rear role.
        std::vector<FollowingLane> following_lanes;
        std::shared_ptr<const asmc_planner::lane_reference::LaneRegion> following_region;
        // Same runtime raster/physical body used when preparing lane references.
        // It certifies road containment at microscopic vector-polygon seams;
        // following membership is still bounded by following_region.
        asmc_planner::lane_reference::FootprintShape following_road_body;
        std::function<bool(double,double,double)> following_road_footprint;
    };
    RefLine reference;
    HighwayMergeConfig config;
    VehicleShape vehicle;
    double approach_decel_mps2=0;
    std::shared_ptr<const asmc_planner::lane_reference::HdLaneMap> map;
    std::vector<Zone> zones;
};

std::shared_ptr<const HighwayMergeRoute> PrepareHighwayMergeRoute(
    const RefLine&, const ReferenceConfig&, const HighwayMergeConfig&,
    const VehicleShape&, double max_curvature, const std::string& areas_path,double approach_decel_mps2,
    std::string& error,const std::string& markings_path="");

struct HighwayMergeApproach {
    bool valid = false, approach = false;
    std::size_t zone_index = 0;
    double route_s = 0, speed_cap_mps = 0;
    std::string source_link, target_link;
    // Explicit validated active-lane ownership; independent of another MERGE.
    bool cruising = false;
};
HighwayMergeApproach HighwayApproachAt(const HighwayMergeRoute&,
    const CartesianState&, const double* station_hint = nullptr,double max_lateral_distance=2.0);
// Preserve a recognized map section while slowing before its configured
// start. A speed-dependent recognition boundary must not chatter the request.
HighwayMergeApproach UpdateHighwayApproach(const HighwayMergeRoute&,
    const CartesianState&,HighwayMergeApproach& previous,double max_lateral_distance=2.0);

