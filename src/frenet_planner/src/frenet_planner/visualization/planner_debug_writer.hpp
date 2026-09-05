#pragma once

#include <chrono>
#include <string>
#include <vector>

#include "frenet_planner/frenet/path_generator.hpp"
#include "frenet_planner/Reference/Reference.hpp"
#include "frenet_planner/global/global.hpp"
#include "frenet_planner/obstacle/tracked_obstacle.hpp"

struct PlannerVisualizationConfig {
    bool enabled = false;
    double publish_hz = 5.0;
    std::string snapshot_path = "/tmp/frenet_planner_debug.json";
};

class PlannerDebugWriter {
public:
    explicit PlannerDebugWriter(PlannerVisualizationConfig config);

    void Publish(const RefLine& ref,
                 const CartesianState& ego,
                 const FrenetState& start,
                 BehaviorState mode,
                 const char* phase,
                 double target_speed,
                 const std::vector<TrackedObstacle>& obstacles,
                 const std::vector<FrenetPath>& candidates,
                 int selected_index,
                 const PlannerDebugStats& stats,
                 const PlannerCommand& command);

private:
    PlannerVisualizationConfig config_;
    std::chrono::steady_clock::time_point last_publish_{};
};
