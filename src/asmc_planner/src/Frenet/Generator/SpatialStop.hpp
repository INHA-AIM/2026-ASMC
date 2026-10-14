#pragma once

#include "Frenet/Generator/LongitudinalGenerator.hpp"
#include "Frenet/Evaluation/KinematicFilter.hpp"

// Distance-native STOP. Every time horizon samples the same physical profile;
// choosing a longer horizon cannot defer braking. The endpoint includes the
// caller's stop buffer. Impossible stops return no target-reaching candidates.
std::vector<FrenetPath> GenerateSpatialStoppingCandidates(
    const FrenetState& start, double stop_s, double cruise_speed,
    double comfortable_decel, const PathGeneratorConfig& cfg,
    const KinematicLimits& limits, double acceleration_limit = 0.0);
