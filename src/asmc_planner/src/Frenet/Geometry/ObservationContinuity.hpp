#pragma once
#include "Frenet/Geometry/ObjectTypes.hpp"
#include <algorithm>
#include <cmath>

// An ID is an association proposal. A different observed occupancy must not
// inherit its old motion/body certificate. This is a history gate, never an
// object deletion or a replacement of the authoritative current corners.
inline bool ObservationContinuous(const TrackedObject& previous,
                                  const TrackedObject& current,double dt) {
    if(!std::isfinite(dt)||dt<=0||dt>.5)return false;
    for(double v:{previous.x,previous.y,previous.vx,previous.vy,current.x,current.y})
        if(!std::isfinite(v))return false;
    const double size=std::max(std::hypot(previous.length,previous.width),
                              std::hypot(current.length,current.width));
    if(!std::isfinite(size))return false;
    if(size<.05)return true; // No usable size: cannot disprove association.
    // Visibility can move the fitted centre within half a body diagonal.
    // 0.35 m is the existing rigid-edge tolerance; acceleration allowance
    // avoids splitting a real manoeuvre between adjacent sensor frames.
    const double tolerance=.35+.5*size+4*dt*dt;
    return std::hypot(current.x-previous.x-previous.vx*dt,
                      current.y-previous.y-previous.vy*dt)<=tolerance;
}
