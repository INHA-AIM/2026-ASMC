#include "Frenet/Maneuver/Merge/HighwayTransitionPath.hpp"
#include "Frenet/Math/FrenetConverter.hpp"
#include "Frenet/Math/Polynomial.hpp"
#include <algorithm>
#include <cmath>

namespace {
double offset(const RefLine& mission,const RefLine& lane,double s) {
    const auto p=Interpolate(mission,s);
    const double t=FindClosestS(lane,p.x,p.y,nullptr,100);
    const auto q=Interpolate(lane,t);
    return -(q.x-p.x)*std::sin(p.theta)+(q.y-p.y)*std::cos(p.theta);
}
struct Offset { double d,first,second; };
Offset derivatives(const RefLine& mission,const RefLine& lane,double s) {
    constexpr double h=.5;
    const double d=offset(mission,lane,s),a=offset(mission,lane,s-h),b=offset(mission,lane,s+h);
    return {d,(b-a)/(2*h),(b-2*d+a)/(h*h)};
}
}
namespace {
Offset geometryOffset(const RefLine& mission,const RefLine& lane,
    const HighwayTransitionGeometry& geometry,double s) {
    if(s>=geometry.terminal_s)return derivatives(mission,lane,s);
    const double q=s-geometry.begin_s;
    return {EvalPos(geometry.lateral,q),EvalVel(geometry.lateral,q),EvalAcc(geometry.lateral,q)};
}
}
HighwayTransitionGeometry MakeHighwayTransitionGeometry(const RefLine& mission,
    const RefLine& lane,double s,double d,double first,double second,double terminal) {
    HighwayTransitionGeometry geometry;
    if(lane.points.size()<3||mission.points.size()<3||!std::isfinite(s)||
       !std::isfinite(d)||!std::isfinite(first)||!std::isfinite(second)||
       !std::isfinite(terminal)||terminal-s<.5)return geometry;
    const auto end=derivatives(mission,lane,terminal);
    geometry.lateral=MakeQuintic(d,first,second,end.d,end.first,end.second,terminal-s);
    geometry.begin_s=s;geometry.terminal_s=terminal;geometry.valid=true;return geometry;
}
bool SampleHighwayTransitionPath(const RefLine& mission,const RefLine& lane,
    const FrenetPath& motion,const HighwayTransitionGeometry& geometry,CartesianPath& out) {
    out={};const auto n=motion.s.size();
    if(!geometry.valid||n<2||lane.points.size()<3||mission.points.size()<3||
       motion.s_d.size()!=n||motion.s_dd.size()!=n||motion.s_ddd.size()!=n||
       motion.s.front()<geometry.begin_s-1e-6)return false;
    for(std::size_t i=0;i<n;++i) {
        const double s=motion.s[i];
        if(!std::isfinite(s)||!std::isfinite(motion.s_d[i])||!std::isfinite(motion.s_dd[i])||
           !std::isfinite(motion.s_ddd[i])||motion.s_d[i]<-1e-6||
           (i&&s<motion.s[i-1]-1e-6))return false;
        const auto z=geometryOffset(mission,lane,geometry,s);
        const auto p=FrenetToCartesian(Interpolate(mission,s),s,
            std::max(0.0,motion.s_d[i]),motion.s_dd[i],z.d,z.first,z.second);
        if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.yaw)||
           !std::isfinite(p.kappa)||!std::isfinite(p.v)||!std::isfinite(p.a))return false;
        out.x.push_back(p.x);out.y.push_back(p.y);out.yaw.push_back(p.yaw);
        out.kappa.push_back(p.kappa);out.v.push_back(p.v);out.a.push_back(p.a);
    }
    return true;
}
bool HighwayTransitionKinematicsValid(const CartesianPath& path,const FrenetPath& motion,
    const KinematicLimits& limits,std::size_t samples) {
    if(samples<2||path.x.size()<samples||path.y.size()<samples||path.yaw.size()<samples||
       path.kappa.size()<samples||path.v.size()<samples||path.a.size()<samples||
       motion.s_ddd.size()<samples)return false;
    for(std::size_t i=0;i<samples;++i)
        if(!std::isfinite(path.x[i])||!std::isfinite(path.y[i])||!std::isfinite(path.yaw[i])||
           !std::isfinite(path.kappa[i])||!std::isfinite(path.v[i])||!std::isfinite(path.a[i])||
           !std::isfinite(motion.s_ddd[i])||path.v[i]<0||
           std::abs(path.kappa[i])>limits.max_curvature||
           path.v[i]*path.v[i]*std::abs(path.kappa[i])>limits.max_lateral_accel||
           std::abs(path.a[i])>limits.max_longitudinal_accel+1e-6||
           std::abs(motion.s_ddd[i])>limits.max_longitudinal_jerk+1e-6)return false;
    return true;
}
bool HighwayTransitionTrackingGuard(const RefLine& mission,const RefLine& lane,
    const HighwayTransitionGeometry& geometry,double s,const CartesianState& ego,
    const VehicleShape& shape,double position_limit,double heading_limit,double curvature_limit,
    double& guard) {
    guard=0;
    if(!geometry.valid||s<geometry.begin_s||!std::isfinite(ego.kappa))return false;
    const auto z=geometryOffset(mission,lane,geometry,s);
    const auto p=FrenetToCartesian(Interpolate(mission,s),s,1,0,z.d,z.first,z.second);
    const double position=std::hypot(p.x-ego.x,p.y-ego.y);
    const double heading=std::abs(std::remainder(p.yaw-ego.yaw,2*M_PI));
    if(!std::isfinite(position)||!std::isfinite(heading)||
       position>position_limit||heading>heading_limit||
       std::abs(p.kappa-ego.kappa)>curvature_limit)return false;
    // A rigid rotation moves any body point by at most 2*r*sin(angle/2).
    // Inflating all four sides by this bound also covers the actual start body.
    const double rear=shape.rear_overhang<0?shape.length*.5:shape.rear_overhang;
    const double radius=std::hypot(std::max(rear,shape.length-rear),shape.width*.5);
    guard=position+2*radius*std::sin(heading*.5);return true;
}
bool BuildHighwayTransitionPath(const RefLine& mission,const RefLine& lane,
    const FrenetPath& motion,double d,double first,double second,double terminal,
    const KinematicLimits& limits,CartesianPath& out) {
    if(motion.s.empty())return false;
    const auto geometry=MakeHighwayTransitionGeometry(mission,lane,motion.s.front(),d,first,second,terminal);
    return SampleHighwayTransitionPath(mission,lane,motion,geometry,out)&&
        HighwayTransitionKinematicsValid(out,motion,limits,out.x.size());
}
