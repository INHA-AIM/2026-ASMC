#include "Frenet/Evaluation/CollisionChecker.hpp"
#include "Frenet/Generator/SpatialStop.hpp"
#include "Frenet/Generator/LateralGenerator.hpp"
#include <algorithm>
#include <cmath>

bool BuildStoppingSafetyPath(const FrenetPath& prefix,const RefLine& ref,
    const StoppingContinuationPolicy& policy,double horizon,FrenetPath& checked) {
    const auto count=prefix.t.size();
    if(!prefix.valid||count<2||prefix.s.size()!=count||prefix.s_d.size()!=count||
       prefix.s_dd.size()!=count||prefix.d.size()!=count||
       prefix.s_ddd.size()!=count||
       prefix.d_d.size()!=count||prefix.d_dd.size()!=count||
       !std::isfinite(horizon)||horizon<0||!std::isfinite(policy.decel)||policy.decel<=0||
       !std::isfinite(policy.jerk)||policy.jerk<=0||
       !std::isfinite(policy.max_curvature)||policy.max_curvature<=0||
       !std::isfinite(policy.max_lateral_accel)||policy.max_lateral_accel<=0)return false;
    const double dt=prefix.t[1]-prefix.t[0];
    if(dt<=0||!std::isfinite(dt))return false;
    if(!prefix.spatial_lateral.enabled&&
       (std::fabs(prefix.d_d.back())>1e-5||std::fabs(prefix.d_dd.back())>1e-5))return false;
    FrenetState boundary{};
    boundary.s=prefix.s.back();boundary.s_d=prefix.s_d.back();boundary.s_dd=prefix.s_dd.back();
    FrenetPath tail;
    double decel=policy.decel;
    bool physical=false;
    for(int attempt=0;attempt<3&&!physical;++attempt) {
        tail=GenerateBrakingContinuation(boundary,decel,policy.jerk,dt,
            std::max(0.0,horizon-prefix.t.back()));
        if(!tail.valid)return false;
        if(prefix.spatial_lateral.enabled)tail=SampleSpatialLateralMotion(tail,prefix.spatial_lateral);
        else {
            tail.d.assign(tail.t.size(),prefix.d.back());
            tail.d_d.assign(tail.t.size(),0);tail.d_dd.assign(tail.t.size(),0);
        }
        if(!tail.valid)return false;
        const auto cart=ConvertToCartesianPath(tail,ref);
        double max_acceleration=0;
        for(std::size_t i=0;i<cart.x.size();++i) {
            if(!std::isfinite(cart.x[i])||!std::isfinite(cart.y[i])||!std::isfinite(cart.yaw[i])||
               !std::isfinite(cart.kappa[i])||!std::isfinite(cart.v[i])||!std::isfinite(cart.a[i])||
               cart.v[i]<-1e-6||std::fabs(cart.kappa[i])>policy.max_curvature+1e-6||
               cart.v[i]*cart.v[i]*std::fabs(cart.kappa[i])>policy.max_lateral_accel+1e-6)return false;
            max_acceleration=std::max(max_acceleration,std::fabs(cart.a[i]));
        }
        physical=max_acceleration<=policy.decel+1e-6;
        // Frenet acceleration is not Cartesian tangential acceleration on an
        // offset curve. Resolve that coupling by retiming the SAME geometry;
        // never waive the physical limit or reset the boundary acceleration.
        if(!physical)decel*=policy.decel/(max_acceleration+1e-3);
    }
    if(!physical)return false;
    checked=prefix;
    for(std::size_t i=1;i<tail.t.size();++i) {
        checked.t.push_back(prefix.t.back()+tail.t[i]);
        checked.s.push_back(tail.s[i]);checked.s_d.push_back(tail.s_d[i]);
        checked.s_dd.push_back(tail.s_dd[i]);checked.s_ddd.push_back(tail.s_ddd[i]);
        checked.d.push_back(tail.d[i]);checked.d_d.push_back(tail.d_d[i]);
        checked.d_dd.push_back(tail.d_dd[i]);
    }
    return true;
}
