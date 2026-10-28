#pragma once
#include "Frenet/Generator/SpatialStop.hpp"
#include "Frenet/Geometry/CartesianTypes.hpp"
#include <algorithm>
#include <cmath>

// Search indices on an ALREADY generated curve, not another geometry/speed
// catalogue. A nominal collision supplies a response deadline. The caller must
// still certify the complete stop against ALL objects: this is no clearance.
inline double ProgressPrefixBeforeCollision(const CartesianPath& nominal,
    std::size_t collision_sample,double minimum_prefix,double dt,double decel,double jerk) {
    if(dt<=0||nominal.v.size()!=nominal.x.size()||nominal.a.size()!=nominal.x.size()||
       collision_sample>=nominal.x.size())return minimum_prefix;
    const auto first=static_cast<std::size_t>(std::ceil(minimum_prefix/dt));
    for(std::size_t i=collision_sample;i>first;--i) {
        FrenetState boundary{};boundary.s_d=nominal.v[i];boundary.s_dd=nominal.a[i];
        const auto stop=GenerateBrakingContinuation(boundary,decel,jerk,dt,0);
        if(stop.valid&&!stop.s.empty()&&i+stop.s.size()-1<=collision_sample)
            return i*dt;
    }
    return minimum_prefix;
}

// Execute a prefix only when a COMPLETE jerk-limited stop on the same geometry
// exists. Callers must certify the entire returned path, including its held
// endpoint, against road and every obstacle over this complete stop horizon.
inline CartesianPath MakeProgressStoppingCertificate(const CartesianPath& nominal,
    double prefix_time,double dt,double decel,double jerk,double min_horizon,
    double terminal_hold_time=0) {
    const auto n=nominal.x.size();
    if(n<3||dt<=0||prefix_time<=0||nominal.y.size()!=n||nominal.yaw.size()!=n||
       nominal.kappa.size()!=n||nominal.v.size()!=n||nominal.a.size()!=n)return {};
    const size_t prefix=std::max<size_t>(1,std::ceil(prefix_time/dt));
    if(prefix>=n-1)return {};
    std::vector<double> station(n,0);
    for(size_t i=1;i<n;++i)station[i]=station[i-1]+
        std::hypot(nominal.x[i]-nominal.x[i-1],nominal.y[i]-nominal.y[i-1]);
    FrenetState start{};start.s=station[prefix];
    start.s_d=nominal.v[prefix];start.s_dd=nominal.a[prefix];
    const auto stop=GenerateBrakingContinuation(start,decel,jerk,dt,
        std::max(0.0,min_horizon-prefix*dt));
    if(!stop.valid||stop.s.empty()||stop.s.back()>station.back()+1e-6)return {};
    CartesianPath out;
    for(size_t i=0;i<prefix;++i) {
        out.x.push_back(nominal.x[i]);out.y.push_back(nominal.y[i]);
        out.yaw.push_back(nominal.yaw[i]);out.kappa.push_back(nominal.kappa[i]);
        out.v.push_back(nominal.v[i]);out.a.push_back(nominal.a[i]);
    }
    for(size_t i=0;i<stop.s.size();++i) {
        auto it=std::upper_bound(station.begin(),station.end(),stop.s[i]);
        const size_t hi=std::clamp<size_t>(it-station.begin(),1,n-1),lo=hi-1;
        const double span=station[hi]-station[lo];
        const double q=span>1e-9?std::clamp((stop.s[i]-station[lo])/span,0.0,1.0):0;
        const auto blend=[&](const std::vector<double>& v){return v[lo]+q*(v[hi]-v[lo]);};
        out.x.push_back(blend(nominal.x));out.y.push_back(blend(nominal.y));
        out.yaw.push_back(nominal.yaw[lo]+q*std::remainder(nominal.yaw[hi]-nominal.yaw[lo],2*M_PI));
        out.kappa.push_back(blend(nominal.kappa));out.v.push_back(stop.s_d[i]);out.a.push_back(stop.s_dd[i]);
    }
    // Hold after reaching zero so the caller's input-expiry allowance is
    // covered as well. The horizon follows braking physics, not a fixed cutoff.
    for(size_t i=0;i<static_cast<size_t>(std::ceil(std::max(0.0,terminal_hold_time)/dt));++i) {
        out.x.push_back(out.x.back());out.y.push_back(out.y.back());
        out.yaw.push_back(out.yaw.back());out.kappa.push_back(out.kappa.back());
        out.v.push_back(0);out.a.push_back(0);
    }
    return out;
}
