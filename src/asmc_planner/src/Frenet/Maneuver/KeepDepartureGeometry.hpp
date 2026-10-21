#pragma once
#include "Frenet/Evaluation/CollisionChecker.hpp"
#include "Frenet/Reference/Reference.hpp"
#include <algorithm>
#include <cmath>

// Preserve a small existing in-lane offset during departure rather than
// completing a time-based centering S turn before meaningful travel. No state,
// mode inference or new threshold: callers supply the existing recovery band.
inline bool KeepDepartureOffsetHoldAllowed(const CartesianState& ego,
    const RefLine& ref,double s,VehicleShape shape,double lane_width,double recovery_speed) {
    if(!std::isfinite(ego.v)||std::fabs(ego.v)>=recovery_speed)return false;
    shape.width=std::max(shape.width,shape.road_outer_width);
    const auto rp=Interpolate(ref,s);
    for(const auto& p:MakeEgoFootprint(ego,shape,0)) {
        const double lateral=-std::sin(rp.theta)*(p.x-rp.x)+std::cos(rp.theta)*(p.y-rp.y);
        if(!std::isfinite(lateral)||std::fabs(lateral)>lane_width*.5)return false;
    }
    return true;
}
