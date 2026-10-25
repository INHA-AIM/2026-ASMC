#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include "Util/BrakingSteering.hpp"

struct EmergencySteeringState {
    bool active=false,valid=false;
    std::uint64_t generation=0;
    double steering=0,last_output=0,geometry_stamp=0;
    void Capture(double last_delta,double now,bool usable,std::uint64_t epoch,
                 double source_stamp) {
        if(active)return;
        active=true;valid=usable && std::isfinite(last_delta);
        generation=epoch;steering=valid ? last_delta : 0;
        last_output=now;geometry_stamp=source_stamp;
    }
    double Fallback(double now,double speed,bool state_valid,const MPCParams& params) const {
        if(!valid || !state_valid)return 0;
        return std::clamp(BrakingSteering(steering,now-last_output,speed,params),
                          -params.steering_max,params.steering_max);
    }
    double Bound(double delta,double now,double speed,const MPCParams& params) const {
        const double range=SteeringRateLimit(speed,params)*std::max(0.,now-last_output);
        return std::clamp(std::clamp(delta,steering-range,steering+range),
                          -params.steering_max,params.steering_max);
    }
    void Published(double delta,double now) { steering=delta;last_output=now; }
};
