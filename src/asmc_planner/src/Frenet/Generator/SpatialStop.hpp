#pragma once

#include "Frenet/Generator/LongitudinalGenerator.hpp"
#include "Frenet/Evaluation/KinematicFilter.hpp"

// Distance-native STOP. Every time horizon samples the same physical profile;
// choosing a longer horizon cannot defer braking. The endpoint includes the
// caller's stop buffer. Impossible stops return no target-reaching candidates.
// Independent policies: full-stop sampling and longitudinal braking ownership.
// A named type prevents old positional bool arguments silently changing meaning.
struct SpatialStopOptions {
    bool braking_only = false;
    bool allow_partial_horizons = false;
    // Explicit partial-departure witness through the existing stop endpoint.
    // Ordinary STOP retains its configured receding horizon.
    bool complete_profile = false;
};

std::vector<FrenetPath> GenerateSpatialStoppingCandidates(
    const FrenetState& start, double stop_s, double cruise_speed,
    double comfortable_decel, const PathGeneratorConfig& cfg,
    const KinematicLimits& limits, double acceleration_limit = 0.0,
    SpatialStopOptions options = {});

// Shared jerk-limited minimum-distance braking primitive. No stop target or
// mode is inferred. The complete stop is sampled, then held through min_duration.
FrenetPath GenerateBrakingContinuation(const FrenetState& start,
    double decel, double jerk, double dt, double min_duration = 0.0);
