#pragma once

#include <chrono>
#include <string>
#include <vector>

#include "frenet_planner/Generator/CandidateCombiner.hpp"
#include "frenet_planner/Reference/Reference.hpp"
#include "frenet_planner/Common/BehaviorTypes.hpp"
#include "frenet_planner/Common/CartesianTypes.hpp"
#include "frenet_planner/Common/FrenetTypes.hpp"
#include "frenet_planner/Common/ObjectTypes.hpp"
#include "frenet_planner/Common/PlannerCommand.hpp"

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
                 const std::vector<TrackedObject>& obstacles,
                 const std::vector<FrenetPath>& candidates,
                 int selected_index,
                 const PlannerDebugStats& stats,
                 const PlannerCommand& command);

private:
    PlannerVisualizationConfig config_;
    std::chrono::steady_clock::time_point last_publish_{};
};
