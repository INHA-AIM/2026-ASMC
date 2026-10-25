#pragma once
#include "Global/Global.hpp"

// Re-time validated spatial geometry for the fixed brake pedal. Beyond the
// finite path, continue its terminal tangent/curvature only while braking.
ReferencePath MakeBrakingReference(const ReferencePath& geometry,
    const MPCState& ego, double deceleration, const MPCParams& params,
    bool allow_terminal_extension = true);
double BrakePedalDeceleration(double pedal, const MPCParams& params);
