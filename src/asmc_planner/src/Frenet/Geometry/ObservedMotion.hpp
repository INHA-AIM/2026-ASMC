#pragma once
#include <algorithm>
#include <cmath>
#include <deque>
#include "Frenet/Geometry/ObjectTypes.hpp"
#include "Frenet/Geometry/ObservationContinuity.hpp"

// Sensor-frame time and velocity direction, never the fitted OBB angle.
struct ObservedObjectMotion {
    struct Sample { double time, direction, x, y, vx, vy; };
    std::deque<Sample> samples;
    TrackedObject last{};
    double stamp = -1, yaw_rate = 0;
    int missing_cycles = 0;
    bool continuity_reset = false;
    void Observe(const TrackedObject& object, double time, double clamp,
                 double minimum_direction_change=.05,std::size_t maximum_samples=6,
                 double history_window_s=.5) {
        if (!std::isfinite(time) || time <= stamp + 1e-9) return;
        continuity_reset=stamp>=0&&!ObservationContinuous(last,object,time-stamp);
        if (continuity_reset) { samples.clear(); yaw_rate=0; }
        last = object; stamp = time; missing_cycles = 0;
        const double speed = std::hypot(object.vx, object.vy);
        if (object.velocity_unavailable || object.motion_state == ObjectMotionState::LOW_THREAT || speed < .5) {
            samples.clear(); yaw_rate = 0; return;
        }
        double direction = std::atan2(object.vy, object.vx);
        if (!samples.empty()) direction = samples.back().direction +
            std::remainder(direction - samples.back().direction, 2*M_PI);
        samples.push_back({time,direction,object.x,object.y,object.vx,object.vy});
        while (samples.size()>maximum_samples || time-samples.front().time>history_window_s) samples.pop_front();
        yaw_rate = 0;
        if (samples.size()<3 || time-samples.front().time<.1) return;
        double mt=0,md=0;
        for (auto s:samples) { mt+=s.time-time; md+=s.direction; }
        mt/=samples.size();md/=samples.size();
        double xx=0,xy=0;
        for (auto s:samples) {const double x=s.time-time-mt;xx+=x*x;xy+=x*(s.direction-md);}
        if (xx<=1e-9) return;
        const double rate=xy/xx;
        // A small lane-centering/noise trend is not evidence of a sustained
        // turn over the collision horizon. Require a resolved direction change.
        if(std::fabs(rate)*(time-samples.front().time)<minimum_direction_change)return;
        if (!std::isfinite(rate)||std::fabs(rate)>clamp) return;
        for (auto s:samples)
            if (std::fabs(s.direction-md-rate*(s.time-time-mt))>.1) return;
        // Small direction changes must be resolvable in the observed travel,
        // not amplified into metres of lateral motion over a long horizon.
        const auto first=samples.front();
        const double travel=std::hypot(object.x-first.x,object.y-first.y);
        if (std::fabs(rate)*(time-first.time)<=std::atan2(.1,std::max(.1,travel))) return;
        double px=first.x,py=first.y;
        for(std::size_t i=1;i<samples.size();++i) {
            const auto a=samples[i-1],b=samples[i];const double dt=b.time-a.time;
            px+=.5*(a.vx+b.vx)*dt;py+=.5*(a.vy+b.vy)*dt;
            // Fit-center jumps or drift are not independent turn evidence.
            if(std::hypot(px-b.x,py-b.y)>.35) return;
        }
        yaw_rate=rate;
    }
};
