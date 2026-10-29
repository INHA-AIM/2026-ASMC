#include "Frenet/Generator/StaticAvoidPath.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

bool RetimeStaticAvoidGeometry(const CartesianPath& input,double speed,
    double acceleration,double target,double dt,double wheelbase,
    const KinematicLimits& limits,CartesianPath& out,std::string* rejection) {
    out={};
    const auto fail=[&](const char* stage){if(rejection)*rejection=stage;return false;};
    if(input.x.size()<3||input.y.size()!=input.x.size()||
       input.yaw.size()!=input.x.size()||input.kappa.size()!=input.x.size()||
       !std::isfinite(speed)||speed<0||!std::isfinite(acceleration)||
       !std::isfinite(target)||target<=.05||!std::isfinite(dt)||dt<=0)return fail("input");
    CartesianPath geometry;std::vector<double> arc;
    for(std::size_t i=0;i<input.x.size();++i) {
        if(!std::isfinite(input.x[i])||!std::isfinite(input.y[i])||
           !std::isfinite(input.yaw[i])||!std::isfinite(input.kappa[i])||
           std::fabs(input.kappa[i])>limits.max_curvature+1e-9)return fail("geometry");
        const double ds=geometry.x.empty()?0:std::hypot(input.x[i]-geometry.x.back(),input.y[i]-geometry.y.back());
        if(!geometry.x.empty()&&ds<1e-6)continue;
        arc.push_back(arc.empty()?0:arc.back()+ds);
        geometry.x.push_back(input.x[i]);geometry.y.push_back(input.y[i]);
        geometry.yaw.push_back(input.yaw[i]);geometry.kappa.push_back(input.kappa[i]);
    }
    if(arc.size()<3)return fail("geometry");
    const auto n=arc.size();
    std::vector<double> caps(n,target);
    const double scale=wheelbase/limits.steering.curvature_gain;
    for(std::size_t i=0;i<n;++i) {
        caps[i]=std::min(caps[i],std::sqrt(limits.max_lateral_accel/std::max(1e-9,std::fabs(geometry.kappa[i])))*.98);
        if(!i)continue;
        const double before=geometry.kappa[i-1],after=geometry.kappa[i];
        const double nearest=before*after<=0?0:std::min(std::fabs(before),std::fabs(after));
        const double derivative=scale*std::fabs(after-before)/
            ((arc[i]-arc[i-1])*(1+scale*scale*nearest*nearest));
        double lo=0,hi=std::max(target,speed);
        for(int j=0;j<24;++j) {
            const double v=.5*(lo+hi);
            if(derivative*v<=.85*SteeringProfileRateLimit(v,limits.steering))lo=v;else hi=v;
        }
        caps[i-1]=std::min(caps[i-1],lo);caps[i]=std::min(caps[i],lo);
    }
    // Include a full stop in the accepted tail. Completion occurs before this
    // tail, so a successful handoff does not require stopping after each group.
    caps.back()=0;
    const double decel=limits.max_longitudinal_accel*.8;
    for(std::size_t i=n-1;i>0;--i)
        caps[i-1]=std::min(caps[i-1],std::sqrt(caps[i]*caps[i]+decel*(arc[i]-arc[i-1])));
    auto interval=[&](double u) {
        const auto p=std::upper_bound(arc.begin(),arc.end(),u);
        return std::min(n-2,p==arc.begin()?std::size_t(0):std::size_t(p-arc.begin()-1));
    };
    auto ceiling=[&](double u) {
        const auto i=interval(std::clamp(u,0.0,arc.back()));
        return std::min(caps[i],caps[i+1]);
    };
    auto value=[&](const std::vector<double>& values,double u,bool angle=false) {
        const auto i=interval(u);
        const double f=std::clamp((u-arc[i])/(arc[i+1]-arc[i]),0.0,1.0);
        return values[i]+f*(angle?std::remainder(values[i+1]-values[i],2*M_PI):values[i+1]-values[i]);
    };
    double u=0,v=speed,a=std::clamp(acceleration,-decel,limits.max_longitudinal_accel);
    bool complete=false;
    for(int tick=0;tick<1600;++tick) {
        const double k=value(geometry.kappa,u);
        if(v*v*std::fabs(k)>limits.max_lateral_accel+1e-6)return fail("lateral_acceleration");
        out.x.push_back(value(geometry.x,u));out.y.push_back(value(geometry.y,u));
        out.yaw.push_back(value(geometry.yaw,u,true));out.kappa.push_back(k);
        out.v.push_back(v);out.a.push_back(a);
        if(u>=arc.back()-1.0&&v<.02) {
            if(std::fabs(a)<1e-6){complete=out.x.size()>2;break;}
            a+=std::clamp(-a,-limits.max_longitudinal_jerk*dt,limits.max_longitudinal_jerk*dt);
            v=0;continue;
        }
        const double ramp=v*std::max(0.0,a+decel)/std::max(.1,limits.max_longitudinal_jerk);
        const double ahead=std::min(arc.back(),u+std::max(.25,ramp+v*.5));
        double cap=std::min(ceiling(u),ceiling(ahead));
        for(std::size_t i=interval(u);i<=interval(ahead);++i)cap=std::min(cap,caps[i]);
        const double desired=std::clamp((cap-v)/.65,-decel,
            std::min(limits.max_longitudinal_accel,limits.max_forward_accel));
        const double next_a=std::clamp(desired,a-limits.max_longitudinal_jerk*dt,a+limits.max_longitudinal_jerk*dt);
        const double next_v=std::max(0.0,v+(a+next_a)*.5*dt);
        const double distance=(v+next_v)*.5*dt;
        if(u+distance>arc.back()+.05)return fail("stopping_distance");
        u=std::min(arc.back(),u+distance);v=next_v;a=next_a;
    }
    if(!complete)return fail("stop_sampling");
    if(!PathObeysSteeringRate(out,dt,wheelbase,limits.steering))return fail("steering_rate");
    if(rejection)rejection->clear();
    return true;
}
