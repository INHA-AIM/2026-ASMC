#include "Frenet/Generator/SpatialStop.hpp"
#include "Frenet/Generator/SamplingUtils.hpp"
#include "Signal/BrakingModel.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {
struct State { double s = 0, v = 0, a = 0; };
struct Phase { double duration, jerk; };
struct Profile {
    State initial, end;
    std::vector<Phase> phases;
    double duration = 0;
};

State Advance(State x, double t, double jerk) {
    x.s += x.v*t + .5*x.a*t*t + jerk*t*t*t/6;
    x.v += x.a*t + .5*jerk*t*t;
    x.a += jerk*t;
    return x;
}

void Append(Profile& p, double t, double jerk) {
    if (t <= 1e-10) return;
    p.phases.push_back({t,jerk});
    p.end = Advance(p.end,t,jerk);
    p.duration += t;
}

bool Brake(Profile& p, double decel, double jerk) {
    const double v = p.end.v, a = p.end.a;
    if (v <= 1e-9 && std::fabs(a) <= 1e-9) return true;
    const double peak = std::min(decel, std::sqrt(jerk*v + .5*a*a));
    if (peak <= 0) return false;
    const bool releasing = a < -peak;
    const double down = std::fabs(a+peak)/jerk;
    const double v1 = v + (releasing ? -1 : 1)*(a*a-peak*peak)/(2*jerk);
    const double hold = (v1-peak*peak/(2*jerk))/peak;
    if (hold < -1e-9) return false;
    Append(p,down,releasing ? jerk : -jerk);
    Append(p,std::max(0.0,hold),0);
    Append(p,peak/jerk,jerk);
    return std::fabs(p.end.v)<1e-7 && std::fabs(p.end.a)<1e-7;
}

// Transition between two speeds with zero acceleration at both ends.
void ChangeSpeed(Profile& p, double speed, double accel, double jerk) {
    const double dv = speed-p.end.v;
    if (std::fabs(dv)<1e-9) return;
    const double sign = dv>0 ? 1 : -1;
    const double peak = std::min(accel,std::sqrt(jerk*std::fabs(dv)));
    Append(p,peak/jerk,sign*jerk);
    Append(p,std::max(0.0,std::fabs(dv)/peak-peak/jerk),0);
    Append(p,peak/jerk,-sign*jerk);
}

Profile At(State x) { return {x,x,{},0}; }

bool BuildProfile(State initial, double target, double cruise,
                  double comfortable, double max_decel, double jerk, double max_accel,
                  Profile& result, bool braking_only) {
    if (target < initial.s-1e-8 || initial.v<0 ||
        std::fabs(initial.a)>max_decel+1e-9) return false;
    Profile hardest = At(initial);
    if (!Brake(hardest,max_decel,jerk) || hardest.end.s>target+1e-7)
        return false;
    if (braking_only) {
        // Once signal braking starts, a farther revision may reduce braking
        // strength but cannot coast back into an accelerating/reapproach phase.
        // Measured negative acceleration is a boundary condition, not a
        // minimum braking strength for every future sample. Releasing it with
        // bounded positive jerk while a<=0 can place the stop at the target
        // without accelerating the vehicle or starting a new approach.
        double lo=1e-4,hi=max_decel;
        result=hardest;
        for(int i=0;i<48;++i) {
            const double mid=.5*(lo+hi);
            Profile candidate=At(initial);
            if(!Brake(candidate,mid,jerk) || candidate.end.s>target)lo=mid;
            else {hi=mid;result=std::move(candidate);}
        }
        return result.end.s<=target+1e-7;
    }

    // The backward ceiling is the inverse of jerk-limited stopping distance.
    // Use comfortable braking when it fits. If the current speed is above that
    // ceiling, bisect the necessary braking strength, independent of horizon T.
    const double min_peak = comfortable;
    Profile gentle = At(initial);
    if (Brake(gentle,min_peak,jerk) && gentle.end.s>=target-1e-7) {
        double lo = std::min(min_peak,max_decel), hi = max_decel;
        result = hardest;
        for (int i=0;i<48;++i) {
            const double mid = .5*(lo+hi);
            Profile candidate = At(initial);
            if (!Brake(candidate,mid,jerk) || candidate.end.s>target) lo=mid;
            else { hi=mid; result=std::move(candidate); }
        }
        return std::fabs(result.end.s-target)<1e-5;
    }

    // Far approach / restart before the target: release the boundary
    // acceleration, reach a distance-admissible cruise speed, then brake.
    // This also allows a stopped vehicle to approach a still-distant target.
    Profile neutral = At(initial);
    Append(neutral,std::fabs(initial.a)/jerk,
           initial.a>0 ? -jerk : jerk);
    if (neutral.end.v < -1e-8) return false;
    auto moving = [&](double speed) {
        Profile p=neutral;
        ChangeSpeed(p,speed,max_accel,jerk);
        return p;
    };
    auto stop_end = [&](double speed) {
        Profile p=moving(speed);
        if (!Brake(p,comfortable,jerk))
            p.end.s=std::numeric_limits<double>::infinity();
        return p.end.s;
    };
    double speed=std::max(0.0,cruise);
    if (stop_end(speed)>target) {
        // At a distant target, avoid accelerating beyond the backward ceiling.
        double lo=0, hi=speed;
        if (stop_end(lo)>target+1e-7) {
            // Neutralizing acceleration would itself consume too much road.
            // Use immediate braking instead of postponing it with a coast.
            double lower=1e-6;
            double upper=max_decel;
            result=hardest;
            for(int i=0;i<48;++i) {
                const double mid=.5*(lower+upper);
                Profile p=At(initial);
                if (!Brake(p,mid,jerk) || p.end.s>target) lower=mid;
                else { upper=mid;result=std::move(p); }
            }
            return std::fabs(result.end.s-target)<1e-5;
        }
        for(int i=0;i<48;++i) {
            const double mid=.5*(lo+hi);
            if(stop_end(mid)>target) hi=mid; else lo=mid;
        }
        speed=lo;
    }
    result=moving(speed);
    Profile stop=result;
    if(!Brake(stop,comfortable,jerk))return false;
    const double slack=target-stop.end.s;
    if(slack < -1e-6 || (slack>1e-6 && speed<1e-6))return false;
    if(speed>1e-6)Append(result,std::max(0.0,slack)/speed,0);
    return Brake(result,comfortable,jerk) &&
           std::fabs(result.end.s-target)<1e-5;
}

State Sample(const Profile& p,double time) {
    State x=p.initial;
    for(const auto& phase:p.phases) {
        const double t=std::min(time,phase.duration);
        x=Advance(x,t,phase.jerk);
        time-=t;
        if(time<=1e-10)return x;
    }
    return p.end;
}
} // namespace

std::vector<FrenetPath> GenerateSpatialStoppingCandidates(
    const FrenetState& start,double stop_s,double cruise_speed,
    double comfortable_decel,const PathGeneratorConfig& cfg,
    const KinematicLimits& limits, double acceleration_limit,
    SpatialStopOptions options) {
    const bool allow_partial_horizons=options.allow_partial_horizons;
    if(!std::isfinite(start.s) || !std::isfinite(start.s_d) ||
       !std::isfinite(start.s_dd) || !std::isfinite(stop_s) ||
       !std::isfinite(cruise_speed) || !std::isfinite(comfortable_decel) ||
       !std::isfinite(cfg.dt) ||
       !std::isfinite(limits.max_longitudinal_accel) ||
       !std::isfinite(limits.max_longitudinal_jerk) ||
       !std::isfinite(limits.boundary_condition_accel_fraction) ||
       cfg.dt<=0 || limits.max_longitudinal_accel<=0 ||
       limits.max_longitudinal_jerk<=0 || comfortable_decel<=0)return {};
    const double headroom=std::clamp(limits.boundary_condition_accel_fraction,1e-6,1.0);
    const double max_decel=limits.max_longitudinal_accel*headroom;
    const double jerk=limits.max_longitudinal_jerk*headroom;
    const double comfortable=std::min(comfortable_decel,max_decel);
    const asmc_planner::signal::BrakeLimits braking{max_decel,jerk,max_decel};
    if (asmc_planner::signal::MinimumStoppingDistance(start.s_d,start.s_dd,braking) >
        stop_s-start.s+1e-7) return {};
    std::vector<FrenetPath> paths;
    for(double offset:SampleRange(cfg.delta_s)) {
        if(offset>1e-9)continue;
        Profile profile;
        if(!BuildProfile({start.s,start.s_d,start.s_dd},stop_s+offset,
                         cruise_speed,comfortable,max_decel,jerk,
                         std::min(limits.max_forward_accel,
                             acceleration_limit > 0 ? std::min(acceleration_limit,max_decel) : comfortable),profile,options.braking_only))continue;
        for(double horizon:SampleRange(cfg.time_horizon)) {
            if(horizon<=0)continue;
            // If the stop fits in the planning horizon, publish it through
            // zero speed. Otherwise use the full approach horizon, not a short
            // fragment that hides the upcoming stop from the controller.
            if(!allow_partial_horizons && profile.duration<=cfg.time_horizon.max+1e-9) {
                if(horizon+1e-9<profile.duration)continue;
            } else if(!allow_partial_horizons&&horizon+1e-9<cfg.time_horizon.max)continue;
            FrenetPath path{};
            path.valid=true;path.t=SampleTimes(horizon,cfg.dt);path.delta_s=offset;
            State previous=profile.initial;
            for(double time:path.t) {
                const State x=Sample(profile,time);
                path.s.push_back(x.s);
                path.s_d.push_back(std::max(0.0,x.v));
                path.s_dd.push_back(x.a);
                const double j=path.s.size()>1 ? (x.a-previous.a)/cfg.dt : 0;
                path.s_ddd.push_back(j);
                path.jerk_cost_lon+=j*j*cfg.dt;
                previous=x;
            }
            path.delta_s_dot=path.s_d.back();
            paths.push_back(std::move(path));
        }
    }
    return paths;
}

FrenetPath GenerateBrakingContinuation(const FrenetState& start,double decel,
    double jerk,double dt,double min_duration) {
    FrenetPath path{};path.valid=false;
    if(!std::isfinite(start.s)||!std::isfinite(start.s_d)||!std::isfinite(start.s_dd)||
       !std::isfinite(decel)||!std::isfinite(jerk)||!std::isfinite(dt)||
       !std::isfinite(min_duration)||start.s_d<0||decel<=0||jerk<=0||dt<=0||
       std::fabs(start.s_dd)>decel+1e-8)return path;
    auto profile=At({start.s,start.s_d,start.s_dd});
    if(!Brake(profile,decel,jerk)||profile.duration>60)return path;
    path.valid=true;
    const double duration=std::max({dt,min_duration,std::ceil(profile.duration/dt)*dt});
    path.t=SampleTimes(duration,dt);
    State previous=profile.initial;
    for(double t:path.t) {
        const auto value=Sample(profile,t);
        path.s.push_back(value.s);path.s_d.push_back(std::max(0.0,value.v));
        path.s_dd.push_back(value.a);
        path.s_ddd.push_back(t>0?(value.a-previous.a)/dt:0);
        previous=value;
    }
    return path;
}
