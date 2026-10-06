#include "MPC/Reference/trajectory_timing.hpp"

#include <algorithm>
#include <cmath>

double ComputeReferenceStartOffset(double trajectory_source_time_s,
                                   double vehicle_source_time_s) {
    const double offset = vehicle_source_time_s - trajectory_source_time_s;
    if (!std::isfinite(offset)) return 0.0;
    return std::max(0.0, offset);
}
