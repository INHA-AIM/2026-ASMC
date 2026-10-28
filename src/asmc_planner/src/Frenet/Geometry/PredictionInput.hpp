#pragma once
#include "Frenet/Common/ObjectTypes.hpp"
#include <algorithm>
#include <cmath>

// Prediction authority, not an obstacle filter. Every rejected motion model
// still leaves the measured footprint in collision checking.
enum class PredictionInputStatus {
    MEASURED, VELOCITY_UNAVAILABLE, INVALID_VELOCITY,
    VELOCITY_DISAGREEMENT, MAP_UNCOVERED
};
inline PredictionInputStatus InspectPredictionVelocity(const TrackedObject& object) {
    if(object.velocity_unavailable)return PredictionInputStatus::VELOCITY_UNAVAILABLE;
    if(!std::isfinite(object.vx)||!std::isfinite(object.vy))
        return PredictionInputStatus::INVALID_VELOCITY;
    return PredictionInputStatus::MEASURED;
}
inline bool PredictionLaneCovers(double station,double end,double maximum_speed,double horizon) {
    return std::isfinite(station)&&std::isfinite(end)&&std::isfinite(maximum_speed)&&
        std::isfinite(horizon)&&station>=0&&end>station&&maximum_speed>=0&&horizon>=0&&
        maximum_speed*horizon<=end-station;
}
