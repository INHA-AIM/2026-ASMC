#pragma once

#include "Frenet/Maneuver/Merge/HighwayPrediction.hpp"

// Candidate-local admission hypothesis, never an observed object mutation.
// The one-second reaction allowance and maximum permitted rear braking are
// the same contract as the rear-gap check. No response before physical entry
// plus that allowance, and no response model for an unconfirmed lane/cut-in.
namespace highway_rear_response {
constexpr double reaction_s=1.0;
struct State {
    bool enabled=false;
    double start=0,low=0,high=0,floor=0,required_decel=0,maximum_decel=0;
    static std::pair<double,double> advance(double speed,double floor,double brake,double time) {
        floor=std::min(speed,floor);
        if(time<=0||brake<=0)return {speed*std::max(0.0,time),speed};
        const double slowing=std::min(time,(speed-floor)/brake);
        return {speed*slowing-.5*brake*slowing*slowing+floor*(time-slowing),
            std::max(floor,speed-brake*slowing)};
    }
    std::pair<double,double> distances(double time) const {
        const double coast=std::min(time,start),braking=std::max(0.0,time-start);
        return {low*coast+advance(low,0,maximum_decel,braking).first,
                high*coast+advance(high,floor,required_decel,braking).first};
    }
    std::pair<double,double> speeds(double time) const {
        return {advance(low,0,maximum_decel,std::max(0.0,time-start)).second,
                advance(high,floor,required_decel,std::max(0.0,time-start)).second};
    }
    geometry::Polygon2D footprint(const TrackedObject& object,double time) const {
        // Use the shared lane-interval sweep, including EVERY intervening
        // reference knot and its rotational guard (not just two rectangles).
        if(time<=0)return PredictObstacleFootprint(object,time);
        auto bounded=object;const auto distance=distances(time);
        const double average=.5*(distance.first+distance.second)/time;
        // The confirmed lane model uses only speed magnitude for station.
        // Setting it exactly also keeps zero-width intervals valid under
        // floating-point hypot/normalization rounding.
        bounded.vx=average;bounded.vy=0;
        bounded.prediction_min_speed=distance.first/time;
        bounded.prediction_max_speed=distance.second/time;
        return PredictObstacleFootprint(bounded,time);
    }
};
} // namespace highway_rear_response
