#pragma once
#include "Frenet/Maneuver/Merge/HighwayPrediction.hpp"

// Derived prediction only. The input message and its missing-velocity flag
// remain unchanged. A newborn/changed/stationary occupancy cannot borrow speed.
inline TrackedObject RecoverHighwayVelocity(const TrackedObject& object,
    const ObservedObjectMotion* track,const HighwayMergeConfig& config) {
    if(!object.velocity_unavailable||!track||track->continuity_reset||
       object.motion_state==ObjectMotionState::LOW_THREAT||track->samples.size()<3)
        return object;
    const auto& first=track->samples.front();const auto& last=track->samples.back();
    const double age=track->stamp-last.time;
    if(age<0||age>config.source_max_age_s||last.time-first.time<config.motion_fit_min_span_s)
        return object;
    std::vector<double> xs,ys;
    for(const auto& p:track->samples){xs.push_back(p.vx);ys.push_back(p.vy);}
    const double vx=highway_prediction_detail::Median(xs),vy=highway_prediction_detail::Median(ys);
    const double speed=std::hypot(vx,vy);
    if(speed<.5||!std::isfinite(speed))return object;
    const double tolerance=.35+.5*std::hypot(object.length,object.width);
    if(std::hypot(object.x-last.x-vx*age,object.y-last.y-vy*age)>tolerance)return object;
    // Actual travel must support moving occupancy; velocity jitter on roadside
    // structures is not enough to create a moving-object prediction.
    if(std::hypot(last.x-first.x,last.y-first.y)<.5*speed*(last.time-first.time))return object;
    auto derived=object;derived.vx=vx;derived.vy=vy;derived.speed=speed;
    derived.velocity_unavailable=false;derived.yaw_rate=0;
    // Recovery is a motion hypothesis, not a measured velocity. Preserve the
    // stationary possibility throughout the full swept collision occupancy.
    derived.prediction_min_speed=0;
    derived.prediction_max_speed=speed+config.speed_error_mps;
    return derived;
}
