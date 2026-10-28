#ifndef FRENET_GENERATOR_BRAKING_PROFILE_HPP
#define FRENET_GENERATOR_BRAKING_PROFILE_HPP

#include <algorithm>
#include <array>
#include <cmath>

// Distance, velocity and acceleration of the existing three-stage brake.
// An optional initial release preserves achieved braking stronger than the
// planning deceleration instead of clipping the measured starting state.
struct BrakingProfile {
    bool valid = false;
    double speed = 0, acceleration = 0, jerk = 0;
    double release = 0, release_distance = 0;
    double peak = 0, down = 0, hold = 0, up = 0;
    double v0 = 0, a0 = 0, v1 = 0, v2 = 0, d1 = 0, d2 = 0;
    double duration = 0, distance = 0;

    std::array<double,3> sample(double t) const {
        if (t < release)
            return {speed*t + .5*acceleration*t*t + jerk*t*t*t/6,
                    speed + acceleration*t + .5*jerk*t*t,
                    acceleration + jerk*t};
        t -= release;
        std::array<double,3> state;
        if (t < down)
            state = {v0*t + .5*a0*t*t - jerk*t*t*t/6,
                     v0 + a0*t - .5*jerk*t*t, a0 - jerk*t};
        else {
            t -= down;
            if (t < hold) state = {d1 + v1*t - .5*peak*t*t, v1 - peak*t, -peak};
            else {
                t -= hold;
                if (t < up)
                    state = {d2 + v2*t - .5*peak*t*t + jerk*t*t*t/6,
                             v2 - peak*t + .5*jerk*t*t, -peak + jerk*t};
                else state = {distance-release_distance, 0.0, 0.0};
            }
        }
        state[0] += release_distance;
        return state;
    }
};

inline BrakingProfile MakeBrakingProfile(double speed, double acceleration,
    double decel, double jerk, bool recover_achieved_braking = false) {
    BrakingProfile p;
    if (!std::isfinite(speed) || !std::isfinite(acceleration) ||
        !std::isfinite(decel) || !std::isfinite(jerk) || speed <= .1 ||
        decel <= 0 || jerk <= 0 || acceleration > decel ||
        (acceleration < -decel && !recover_achieved_braking)) return p;
    p.speed=speed; p.acceleration=acceleration; p.jerk=jerk;
    p.v0=speed; p.a0=acceleration;
    if (acceleration < -decel) {
        p.release=(-decel-acceleration)/jerk;
        p.release_distance=speed*p.release + .5*acceleration*p.release*p.release +
            jerk*p.release*p.release*p.release/6;
        p.v0=speed + acceleration*p.release + .5*jerk*p.release*p.release;
        p.a0=-decel;
    }
    if (p.v0 <= 0) return p;
    p.peak=std::min(decel,std::sqrt(jerk*p.v0 + .5*p.a0*p.a0));
    p.down=(p.a0+p.peak)/jerk;
    p.v1=p.v0 + (p.a0*p.a0-p.peak*p.peak)/(2*jerk);
    const double raw_hold=(p.v1-p.peak*p.peak/(2*jerk))/p.peak;
    if (p.down < 0 || raw_hold < -1e-8) return p;
    p.hold=std::max(0.0,raw_hold); p.up=p.peak/jerk;
    p.d1=p.v0*p.down + .5*p.a0*p.down*p.down - jerk*p.down*p.down*p.down/6;
    p.d2=p.d1 + p.v1*p.hold - .5*p.peak*p.hold*p.hold;
    p.v2=p.v1-p.peak*p.hold;
    p.distance=p.release_distance+p.d2+p.v2*p.up-.5*p.peak*p.up*p.up+
        jerk*p.up*p.up*p.up/6;
    p.duration=p.release+p.down+p.hold+p.up;
    p.valid=std::isfinite(p.duration)&&std::isfinite(p.distance)&&p.distance>0;
    return p;
}
#endif
