#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include "Frenet/Geometry/GeometryTypes.hpp"

// Shared sensor-corner projection for acquisition and static path generation.
// Offsets are relative to the supplied reference pose, never regenerated from
// the independently fitted scalar width/length/yaw fields.
struct AvoidFootprintBounds {
    double near_s=INFINITY, far_s=-INFINITY, min_d=INFINITY, max_d=-INFINITY;
    bool valid=false;
};

inline AvoidFootprintBounds ProjectAvoidFootprint(const geometry::Corners8& corners,
    double x,double y,double yaw) {
    AvoidFootprintBounds out;
    const double c=std::cos(yaw),s=std::sin(yaw);
    for(const auto& p:corners) {
        if(!std::isfinite(p.x)||!std::isfinite(p.y))return {};
        const double dx=p.x-x,dy=p.y-y;
        const double along=dx*c+dy*s,across=-dx*s+dy*c;
        out.near_s=std::min(out.near_s,along);out.far_s=std::max(out.far_s,along);
        out.min_d=std::min(out.min_d,across);out.max_d=std::max(out.max_d,across);
    }
    out.valid=out.far_s-out.near_s>1e-6&&out.max_d-out.min_d>1e-6;
    return out;
}
