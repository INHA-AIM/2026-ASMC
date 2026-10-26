#pragma once
#include <string>
#include <vector>
#include <memory>
#include "Frenet/Reference/Reference.hpp"
#include "Frenet/Evaluation/CollisionChecker.hpp"

// A single map-defined roundabout entry. Highway merge/reference switching
// remain separate callers of the transaction contract.
struct RoundaboutMergeConfig {
    bool enabled = false;
    std::string source_link = "A2256W000148";
    std::string target_link = "A2256W000146";
    std::string stop_node = "A1256W000709";
    std::vector<std::string> conflict_links{"A2256W000134", "A2256W000151"};
    double speed_cap = 25.0 / 3.6;
    double preview_distance = 45.0;
    double max_horizon = 10.0;
    // Roundabout-only headway. YAML omission inherits planner/merge's value.
    double min_front_time_gap = 1.0;
    double confirmation_time_tolerance = .6;
    // Preliminary absolute speed-error allowance. Two samples may differ by
    // twice this amount; this is separate from the departure time-gap policy.
    double speed_measurement_error = 5.0;
    // Existing passed-vehicle size assumption, also used for a supported
    // partial observation. Its centre may lie anywhere inside this body, so
    // a partial-only occupancy uses these FULL sizes as its half extents.
    double partial_vehicle_length = 6.0;
    double partial_vehicle_width = 3.0;
};

struct RoundaboutMergeZone {
    bool valid = false;
    double stop_s = 0, conflict_start_s = 0, rear_clear_s = 0;
    CartesianState waiting_pose{};
    std::vector<geometry::Polygon2D> conflict_areas;
    std::vector<geometry::Polygon2D> circulating_areas;
    std::vector<std::shared_ptr<const RefLine>> circulating_references;
    bool Load(const std::string& areas_path, const std::string& lines_path,
              const RefLine& ref, const VehicleShape& shape,
              const RoundaboutMergeConfig& config, double margin,
              std::string& error);
    bool Overlaps(const CartesianState& pose, const VehicleShape& shape,
                  double margin) const;
    bool SupportsCirculatingObservation(double x, double y) const;
};

struct MergeCandidateCheck {
    bool safe = false;
    bool covers_exit = false;
    double entry_time = -1, exit_time = -1;
    int blocking_object_id = -1;
};
// Uses the candidate's actual positions/times. Stationary objects are included.
// extra temporal buffers apply before entry only; committed replanning still
// checks all objects over the full path without inventing a new waiting phase.
MergeCandidateCheck CheckRoundaboutCandidate(
    const CartesianPath& path, const RoundaboutMergeZone& zone,
    const std::vector<TrackedObject>& objects, const VehicleShape& shape,
    const CollisionCheckConfig& collision, double dt,
    double front_time_gap, double rear_time_gap, bool entering);

// Vehicle-sized UNKNOWN observations on the mapped circulating lane cannot
// certify departure from an initial zero-velocity estimate.
int UnclassifiedCirculatingObject(const std::vector<TrackedObject>& objects,
    const RoundaboutMergeZone& zone,double conflict_x,double conflict_y);
