#pragma once

#include <chrono>
#include <string>
#include <vector>

#include "Frenet/Generator/CandidateCombiner.hpp"
#include "Frenet/Reference/Reference.hpp"
#include "Frenet/Common/BehaviorTypes.hpp"
#include "Frenet/Common/CartesianTypes.hpp"
#include "Frenet/Common/FrenetTypes.hpp"
#include "Frenet/Common/ObjectTypes.hpp"
#include "Frenet/Common/PlannerCommand.hpp"

struct PlannerVisualizationConfig {
    bool enabled;
    double publish_hz;
    std::string snapshot_path;
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
