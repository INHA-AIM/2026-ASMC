#pragma once

#include "Frenet/Config/FrenetConfig.hpp"
#include "Frenet/Evaluation/CollisionChecker.hpp"
#include "Map/HdLaneMap.hpp"
#include <memory>
#include <cmath>
#include <string>

// Read-only, prepared once before enabling the normal Behavior -> Frenet path.
// Mission stations remain on the official route. Production may prepare a
// target-lane connector and adopt a separately validated lane reference.
struct HighwayMergeRoute {
    struct Zone {
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

