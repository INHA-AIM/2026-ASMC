#pragma once

#include "Global/Global.hpp"

// Predict delayed yaw response from the issued steering sequence. Acceleration
// is immediate. History is oldest first; missing history holds the last known
// command. Fractional sample delays are linearly interpolated.
std::vector<MPCControl> ApplySteeringDelay(
    const std::vector<MPCControl>& commands,
    const std::vector<double>& history, double fallback_steering,
    double delay_s, double dt);
