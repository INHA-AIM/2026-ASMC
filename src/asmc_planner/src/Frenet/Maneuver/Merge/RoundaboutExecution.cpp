#include "Frenet/Maneuver/Merge/ProgressStoppingCertificate.hpp"
#include "Frenet/frenet_planner.hpp"
#include "Frenet/Generator/BoundaryConditionPolicy.hpp"
#include "Frenet/Generator/LongitudinalGenerator.hpp"
#include "Frenet/Generator/SpatialStop.hpp"
#include "Frenet/Generator/StoppingLateralGeometry.hpp"
#include "Frenet/Math/FrenetConverter.hpp"
#include "Frenet/Speed/CurveSpeed.hpp"
#include "Frenet/Math/Polynomial.hpp"
#include "Frenet/Maneuver/ManeuverUtils.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace {
// Keep the selected stop's rear-axle endpoint and longitudinal schedule. Shape
// its approach in distance, where a terminal heading remains defined at v=0.
bool AlignWaitingApproach(const RefLine& ref,const CartesianState& ego,
    const AvoidRoadMap& road,const VehicleShape& shape,const KinematicLimits& limits,
    double stop_s,double s,double d,double service_decel,double dt,
    const CartesianPath& original,CartesianPath& aligned) {
    if(original.x.size()<2||original.v.back()>.05)return false;
    std::vector<double> stations;double hint=s;
    for(size_t i=0;i<original.x.size();++i){
        hint=FindClosestS(ref,original.x[i],original.y[i],&hint,5);
        stations.push_back(hint);
    }
    const double end=stations.back(),span=end-s;
    if(span<.15||std::fabs(end-stop_s)>1)return false;
    const auto goal=Interpolate(ref,end),origin=Interpolate(ref,s);
    const double target_d=(original.y.back()-goal.y)*std::cos(goal.theta)-
        (original.x.back()-goal.x)*std::sin(goal.theta);
    double heading=goal.theta,best=-1;
    // Bounded map search includes the front/rear body and paint, rather than
    // aligning just the rear axle with a single centreline tangent.
    for(int k=-10;k<=10;++k){
        const double yaw=goal.theta+k*2*M_PI/180;
        if(!road.PoseClear(original.x.back(),original.y.back(),yaw,shape))continue;
        // Containment is monotone as the body expands. Binary search the
        // eight clearance levels to bound map work per heading.
        int low=0,high=8;
        while(low<high){
            const int mid=(low+high+1)/2;const double margin=.05*mid;
            auto expanded=shape;expanded.width+=2*margin;expanded.length+=2*margin;
            expanded.rear_overhang+=margin;
            if(road.PoseClear(original.x.back(),original.y.back(),yaw,expanded))low=mid;
            else high=mid-1;
        }
        const double clearance=.05*low;
        const double score=clearance-.001*std::abs(k);
        if(score>best){best=score;heading=yaw;}
    }
    if(best<0)return false;
    const double first=(1-origin.kappa*d)*std::tan(std::remainder(ego.yaw-origin.theta,2*M_PI));
    const double last=(1-goal.kappa*target_d)*std::tan(std::remainder(heading-goal.theta,2*M_PI));
    if(!std::isfinite(first)||!std::isfinite(last))return false;
    double ps,pv,pa,pd,prime,second;
    CartesianToFrenet(ref,ego,ps,pv,pa,pd,prime,second,&s,5);
    if(!std::isfinite(second))return false;
    const auto lateral=MakeQuintic(d,first,second,target_d,last,0,span);
    for(size_t i=0;i<stations.size();++i){
        const auto rp=Interpolate(ref,stations[i]);
        const double q=std::clamp(stations[i]-s,0.0,span);
        const double offset=EvalPos(lateral,q),prime=EvalVel(lateral,q),second=EvalAcc(lateral,q);
        const double metric=std::hypot(1-rp.kappa*offset,prime);
        if(metric<.1)return false;
        const double sd=original.v[i]/metric;
        const auto base=FrenetToCartesian(rp,stations[i],sd,0,offset,prime,second);
        const auto point=FrenetToCartesian(rp,stations[i],sd,(original.a[i]-base.a)/metric,
            offset,prime,second);
        if(!std::isfinite(point.kappa)||std::fabs(point.kappa)>limits.max_curvature||
           point.v*point.v*std::fabs(point.kappa)>limits.max_lateral_accel||
           std::fabs(point.a)>std::max(service_decel,limits.max_longitudinal_accel))return false;
        aligned.x.push_back(point.x);aligned.y.push_back(point.y);aligned.yaw.push_back(point.yaw);
        aligned.kappa.push_back(point.kappa);aligned.v.push_back(point.v);aligned.a.push_back(point.a);
    }
    const auto before=road.EvaluatePath(original,shape,dt),after=road.EvaluatePath(aligned,shape,dt);
    return after.drivable&&after.paint_contact_seconds<=before.paint_contact_seconds+1e-6;
}
// A waiting certificate is distinct from a departure certificate. Generate
// complete stops at the rear-axle frontier, rather than velocity-keeping
// horizons which can extend into the circulating lane. Keep every object.
bool MakeFrontierWaitingPath(const RefLine& ref,const CartesianState& ego,
    const FrenetState& start,double s,double d,double stop_s,double cap,
    const AvoidRoadMap& road,const VehicleShape& shape,const KinematicLimits& limits,
    const CollisionCheckConfig& collision,const std::vector<TrackedObject>& objects,
    double service_decel,double comfortable_decel,double approach_accel,double measured_s_accel,
    double dt,double horizon,double target_d,double initial_second,double lateral_tolerance,CartesianPath& selected) {
    const double span=stop_s-s;
    if(span<.15||span>45)return false;
    const auto origin=Interpolate(ref,s);
    const double slope=(1-origin.kappa*d)*std::tan(std::remainder(ego.yaw-origin.theta,2*M_PI));
    if(!std::isfinite(slope))return false;
    const auto lateral_geometries=MakeStoppingLateralGeometries(d,slope,initial_second,
        target_d,span,ego.v,limits);
    if(lateral_geometries.empty())return false;
    PathGeneratorConfig grid{};grid.dt=dt;grid.delta_s={0,0,0};
    grid.time_horizon={2,std::min(8.0,horizon),.5};
    auto boundary=start;
    // Match the service-brake generator's non-reversing boundary policy.
    // A measured brake near standstill may require more jerk to unwind than
    // the remaining speed permits; do not rebuild every candidate in reverse.
    if(boundary.s_dd<0)boundary.s_dd=std::max(boundary.s_dd,
        -std::sqrt(2*limits.max_longitudinal_jerk*std::max(0.0,boundary.s_d)));
    if(ego.v<.1&&boundary.s_dd<0)boundary.s_dd=0;
    auto motions=GenerateStoppingCandidates(boundary,stop_s,grid);
    double best=std::numeric_limits<double>::infinity();
    auto evaluate=[&](const decltype(motions)::value_type& motion,bool partial=false){
        for(const auto& geometry:lateral_geometries) {
            const auto& lateral=geometry.polynomial;
            CartesianPath path;bool valid=!motion.s.empty();double effort=0;
            double curvature_effort=0,curvature_change=0;
            double previous_curvature=ego.kappa;
            for(size_t i=0;i<motion.s.size()&&valid;++i){
                if(motion.s[i]<s-1e-6||motion.s[i]>stop_s+1e-6||motion.s_d[i]<-1e-6||
                   (i&&motion.s[i]<motion.s[i-1]-1e-6)||
                   std::fabs(motion.s_ddd[i])>limits.max_longitudinal_jerk+1e-6) {valid=false;break;}
                const auto rp=Interpolate(ref,motion.s[i]);const double q=std::clamp(motion.s[i]-s,0.0,geometry.distance);
                const auto point=FrenetToCartesian(rp,motion.s[i],motion.s_d[i],motion.s_dd[i],
                    EvalPos(lateral,q),EvalVel(lateral,q),EvalAcc(lateral,q));
                const double brake_bound=std::max(service_decel,std::max(0.0,-ego.a));
                if(!std::isfinite(point.kappa)||std::fabs(point.kappa)>limits.max_curvature||
                   point.v>std::max(cap,ego.v)+.05||point.a>limits.max_longitudinal_accel||
                   point.a < -brake_bound||point.v*point.v*std::fabs(point.kappa)>limits.max_lateral_accel)
                    {valid=false;break;}
                path.x.push_back(point.x);path.y.push_back(point.y);path.yaw.push_back(point.yaw);
                path.kappa.push_back(point.kappa);path.v.push_back(point.v);path.a.push_back(point.a);
                effort+=point.a*point.a*dt;
                const double travel=(i?std::hypot(path.x[i]-path.x[i-1],path.y[i]-path.y[i-1]):0);
                curvature_effort+=point.kappa*point.kappa*travel;
                curvature_change+=std::fabs(point.kappa-previous_curvature);
                previous_curvature=point.kappa;
            }
            if(!valid)continue;
            path.v.back()=0;path.a.back()=0;
            if(!road.EvaluatePath(path,shape,dt).drivable||
               !CartesianTrajectoryClear(path,objects,shape,collision,dt))continue;
            // Rank only fully certified paths. Penalize steering effort/change
            // before centering within the ordinary STOP tolerance. Larger offsets
            // retain the centering preference; partial stops still reward progress.
            const double offset_penalty=std::fabs(geometry.target_d)<=lateral_tolerance
                ?.01*std::fabs(geometry.target_d):std::fabs(geometry.target_d);
            const double tracking=curvature_effort+curvature_change+offset_penalty;
            const double cost=partial?-(motion.s.back()-s)+.01*motion.t.back()+tracking:
                motion.t.back()+.1*effort+tracking;
            if(cost<best){best=cost;selected=std::move(path);}
        }
    };
    // Reuse ordinary STOP's distance-native longitudinal calculation without
    // changing that generator or its caller. A longer horizon samples the same
    // braking profile, rather than postponing the necessary deceleration.
    auto stopping_limits=limits;
    stopping_limits.max_longitudinal_accel=std::max(limits.max_longitudinal_accel,service_decel);
    stopping_limits.boundary_condition_accel_fraction=1;
    auto spatial_boundary=boundary;
    spatial_boundary.s_dd=std::clamp(measured_s_accel,
        -stopping_limits.max_longitudinal_accel,
        std::min(limits.max_longitudinal_accel,approach_accel));
    // Preserve achieved service braking only in the new shared profile. Legacy
    // waiting alternatives keep their original initial boundary unchanged.
    if(spatial_boundary.s_dd<0)spatial_boundary.s_dd=std::max(spatial_boundary.s_dd,
        -std::sqrt(2*limits.max_longitudinal_jerk*std::max(0.0,spatial_boundary.s_d)));
    if(ego.v<.1&&spatial_boundary.s_dd<0)spatial_boundary.s_dd=0;
    auto spatial_grid=grid;
    spatial_grid.time_horizon={horizon,horizon,.5};
    const auto spatial=GenerateSpatialStoppingCandidates(spatial_boundary,stop_s,cap,
        comfortable_decel,spatial_grid,stopping_limits,approach_accel);
    for(const auto& motion:spatial) {
        // PREPARE must publish the complete stop, including the stationary tail.
        if(!motion.s_d.empty()&&motion.s_d.back()<=.05)evaluate(motion);
    }
    if(!selected.x.empty())return true;
    for(const auto& motion:motions)evaluate(motion);
    if(!selected.x.empty())return true;
    const double max_duration=std::min(10.0,horizon);
    // Decelerate to a low rolling speed, then stop at the same frontier.
    // Keeping both stages in the checked output avoids an unchecked coast
    // beyond a short approach horizon. No object or road check is bypassed.
    for(double crawl:{2.0,1.0,.5}) {
        if(crawl>cap||crawl>=std::max(.1,boundary.s_d))continue;
        for(double slow_time:{2.0,3.0}) {
            const auto approach=MakeQuartic(boundary.s,boundary.s_d,boundary.s_dd,
                crawl,0,slow_time);
            const double join_s=EvalPos(approach,slow_time);
            if(join_s>=stop_s-.15)continue;
            for(double stop_time=2.0;slow_time+stop_time<=max_duration+1e-6;stop_time+=1.0) {
                FrenetPath motion{};
                motion.t.clear();motion.s.clear();motion.s_d.clear();
                motion.s_dd.clear();motion.s_ddd.clear();
                const auto finish=MakeQuintic(join_s,crawl,0,stop_s,0,0,stop_time);
                const int steps=static_cast<int>(std::ceil((slow_time+stop_time)/dt));
                for(int i=0;i<=steps;++i) {
                    const double t=std::min(i*dt,slow_time+stop_time);
                    motion.t.push_back(t);
                    if(t<=slow_time) {
                        motion.s.push_back(EvalPos(approach,t));motion.s_d.push_back(EvalVel(approach,t));
                        motion.s_dd.push_back(EvalAcc(approach,t));motion.s_ddd.push_back(EvalJerk(approach,t));
                    } else {
                        const double q=t-slow_time;
                        if(EvalVel(finish,q)>crawl+.05){motion.s.clear();break;}
                        motion.s.push_back(EvalPos(finish,q));motion.s_d.push_back(EvalVel(finish,q));
                        motion.s_dd.push_back(EvalAcc(finish,q));motion.s_ddd.push_back(EvalJerk(finish,q));
                    }
                }
                evaluate(motion);
            }
        }
    }
    if(!selected.x.empty())return true;
    // If reaching the frontier is still blocked, certify only safe forward
    // progress. The rolling segment includes its own jerk-limited stop tail;
    // every sample remains before the frontier and passes the same checks.
    for(double crawl:{2.0,1.0,.5}) {
        if(crawl>cap||crawl>=std::max(.1,boundary.s_d))continue;
        const double brake_time=std::max(1.0,std::sqrt(6*crawl/limits.max_longitudinal_jerk)*1.2);
        for(double slow_time:{1.5,2.0,3.0,4.0}) {
            const auto approach=MakeQuartic(boundary.s,boundary.s_d,boundary.s_dd,crawl,0,slow_time);
            const double join_s=EvalPos(approach,slow_time);
            for(double cruise_time:{1.0,2.0}) {
                const double brake_s=join_s+crawl*cruise_time;
                const auto brake=MakeQuartic(brake_s,crawl,0,0,0,brake_time);
                const double end_s=EvalPos(brake,brake_time);
                const double total=slow_time+cruise_time+brake_time;
                if(end_s>stop_s||end_s<s+.5||total>max_duration)continue;
                FrenetPath motion{};motion.t.clear();motion.s.clear();motion.s_d.clear();
                motion.s_dd.clear();motion.s_ddd.clear();
                const int steps=static_cast<int>(std::ceil(total/dt));
                for(int i=0;i<=steps;++i) {
                    const double t=std::min(i*dt,total);motion.t.push_back(t);
                    if(t<=slow_time) {
                        motion.s.push_back(EvalPos(approach,t));motion.s_d.push_back(EvalVel(approach,t));
                        motion.s_dd.push_back(EvalAcc(approach,t));motion.s_ddd.push_back(EvalJerk(approach,t));
                    } else if(t<=slow_time+cruise_time) {
                        motion.s.push_back(join_s+crawl*(t-slow_time));motion.s_d.push_back(crawl);
                        motion.s_dd.push_back(0);motion.s_ddd.push_back(0);
                    } else {
                        const double q=t-slow_time-cruise_time;
                        motion.s.push_back(EvalPos(brake,q));motion.s_d.push_back(EvalVel(brake,q));
                        motion.s_dd.push_back(EvalAcc(brake,q));motion.s_ddd.push_back(EvalJerk(brake,q));
                    }
                }
                evaluate(motion,true);
            }
        }
    }
    if(!selected.x.empty())return true;
    // Direct stops fill the gap between frontier arrival and strongest braking.
    // Duration determines the stop position. The stationary tail is checked to
    // the same horizon, so stopping early cannot hide a later collision.
    std::vector<double> durations;
    // At the non-reversing braking boundary, only the exact jerk-release
    // duration can stop monotonically. A coarse half-second grid misses it.
    if(boundary.s_dd<0) {
        const double release=-2*boundary.s_d/boundary.s_dd;
        if(release>0&&release<=max_duration)durations.push_back(release);
    }
    for(double duration=.5;duration<=max_duration+1e-6;duration+=.5)durations.push_back(duration);
    for(double duration:durations) {
        const auto brake=MakeQuartic(boundary.s,boundary.s_d,boundary.s_dd,0,0,duration);
        const double end_s=EvalPos(brake,duration);
        if(end_s<=s+1e-6||end_s>stop_s)continue;
        FrenetPath motion{};motion.t.clear();motion.s.clear();motion.s_d.clear();
        motion.s_dd.clear();motion.s_ddd.clear();
        const int steps=static_cast<int>(std::ceil(max_duration/dt));
        for(int i=0;i<=steps;++i) {
            const double t=std::min(i*dt,max_duration),q=std::min(t,duration);
            motion.t.push_back(t);motion.s.push_back(EvalPos(brake,q));
            motion.s_d.push_back(t<duration?EvalVel(brake,q):0);
            motion.s_dd.push_back(t<duration?EvalAcc(brake,q):0);
            motion.s_ddd.push_back(t<duration?EvalJerk(brake,q):0);
        }
        evaluate(motion,true);
    }
    return !selected.x.empty();
}

// STOP's braking fallback retains the current d. During a long
// approach that repeatedly freezes a tracking offset instead of correcting it.
// Rebuild only that constant-offset fallback, retaining its longitudinal stop
// schedule and the observed initial heading. Short stops stay unchanged.
bool RecenterWaitingFallback(const RefLine& ref,const CartesianState& ego,
    double s,double d,const KinematicLimits& limits,double service_decel,
    const CartesianPath& original,CartesianPath& centered) {
    if(original.x.size()<2||std::fabs(d)<.2)return false;
    std::vector<double> stations;double hint=s;
    for(size_t i=0;i<original.x.size();++i) {
        hint=FindClosestS(ref,original.x[i],original.y[i],&hint,5);
        const auto rp=Interpolate(ref,hint);
        double offset=(original.y[i]-rp.y)*std::cos(rp.theta)-
            (original.x[i]-rp.x)*std::sin(rp.theta);
        if(std::fabs(offset-d)>.005)return false;
        stations.push_back(hint);
    }
    const double settle=std::max(10.0,std::min(25.0,ego.v*2.5));
    if(stations.back()-s<settle)return false;
    const auto origin=Interpolate(ref,s);
    const double slope=(1-origin.kappa*d)*std::tan(std::remainder(ego.yaw-origin.theta,2*M_PI));
    if(!std::isfinite(slope))return false;
    const auto lateral=MakeQuintic(d,slope,0,0,0,0,settle);
    for(size_t i=0;i<stations.size();++i) {
        const auto rp=Interpolate(ref,stations[i]);
        const double metric=std::fabs(1-rp.kappa*d);
        if(metric<.1)return false;
        const double sd=original.v[i]/metric;
        const auto base=FrenetToCartesian(rp,stations[i],sd,0,d,0,0);
        const double sdd=(original.a[i]-base.a)/metric;
        const double q=std::min(settle,std::max(0.0,stations[i]-s));
        const auto point=FrenetToCartesian(rp,stations[i],sd,sdd,
            EvalPos(lateral,q),EvalVel(lateral,q),EvalAcc(lateral,q));
        if(!std::isfinite(point.kappa)||std::fabs(point.kappa)>limits.max_curvature||
           point.v*point.v*std::fabs(point.kappa)>limits.max_lateral_accel||
           std::fabs(point.a)>std::max(service_decel,limits.max_longitudinal_accel))return false;
        centered.x.push_back(point.x);centered.y.push_back(point.y);centered.yaw.push_back(point.yaw);
        centered.kappa.push_back(point.kappa);centered.v.push_back(point.v);centered.a.push_back(point.a);
    }
    return !centered.x.empty();
}
// Fixed-reference spatial centering plus a bounded longitudinal profile. A
// long quartic from rest spends the whole horizon accelerating; it cannot
// certify the downstream exit at a modest end speed. This profile reaches
// cruise speed, follows the curve envelope, and checks the complete exit.
bool MakeMergeProfile(const RefLine& ref, const CartesianState& ego,
    const FrenetState& start, double cap, double duration, double dt,
    const CurveSpeedConfig& curve, const KinematicLimits& limits,
    double settle, double target_d, bool release_positive_accel, CartesianPath& path,
    double accepted_acceleration = std::numeric_limits<double>::quiet_NaN()) {
    const auto origin=Interpolate(ref,start.s);
    const double slope=(1-origin.kappa*start.d)*
        std::tan(std::remainder(ego.yaw-origin.theta,2*M_PI));
    if(!std::isfinite(slope)||std::fabs(start.d)>1.25)return false;
    auto lateral=MakeQuintic(start.d,slope,0,target_d,0,0,settle);
    // Preview the actual offset geometry, rather than only reference curvature.
    // The shared envelope reserves distance for unwinding positive acceleration
    // under the jerk limit before the curve's braking boundary.
    RefLine geometry;
    double arc=0,previous_metric=1;
    const double span=std::max(80.0,std::max(cap,ego.v)*duration+30.0);
    for(double ds=0;ds<=span;ds+=.25) {
        auto rp=Interpolate(ref,start.s+ds);double q=std::min(ds,settle);
        double d=EvalPos(lateral,q),dp=EvalVel(lateral,q),dpp=EvalAcc(lateral,q);
        double m=std::hypot(1-rp.kappa*d,dp);
        auto point=FrenetToCartesian(rp,start.s+ds,0,0,d,dp,dpp);
        if(!std::isfinite(point.kappa)||m<.1)return false;
        if(ds>0)arc+=.125*(previous_metric+m);
        geometry.points.push_back({point.x,point.y,arc,point.yaw,point.kappa,0});
        previous_metric=m;
    }
    geometry.length=arc;
    const double headroom=std::clamp(limits.boundary_condition_accel_fraction,1e-6,1.0);
    const double preview_acceleration=std::isfinite(accepted_acceleration)
        ?std::max(ego.a,accepted_acceleration):ego.a;
    auto envelope=BuildKeepSpeedEnvelope(geometry,0,ego.v,release_positive_accel?std::min(0.0,preview_acceleration):preview_acceleration,duration,dt,
        cap,curve,limits.max_longitudinal_accel,limits.max_lateral_accel,
        limits.max_longitudinal_jerk*headroom);
    // Use the same spatial-envelope integrator as KEEP. Its feed-forward
    // acceleration follows the declining v^2 ceiling, instead of waiting for
    // a speed error in the middle of the curve. Generate in physical arc length.
    FrenetState motion_start{};motion_start.s_d=std::max(0.0,ego.v);
    motion_start.s_dd=release_positive_accel?std::min(0.0,ego.a):ego.a;
    // Do not release a measured brake when the current spatial ceiling still
    // calls for slowing down. An accepted seed is only a normal speed-recovery
    // option; the fresh geometry envelope and all safety checks remain binding.
    if(ego.a<0&&std::isfinite(accepted_acceleration)&&envelope.size()>1&&
       cap>ego.v+limits.accel_intent_deadband_mps&&
       std::sqrt(std::max(0.0,envelope[0].speed_squared))>ego.v+limits.accel_intent_deadband_mps&&
       std::sqrt(std::max(0.0,envelope[1].speed_squared))>ego.v+limits.accel_intent_deadband_mps)
        motion_start.s_dd=accepted_acceleration;
    if(motion_start.s_d<.01&&motion_start.s_dd<0)motion_start.s_dd=0;
    // A measured strong brake near standstill may already be outside the
    // jerk-limited stopping boundary. Release that initial memory sufficiently
    // to avoid generating reverse motion; current speed/pose stay unchanged.
    if(motion_start.s_dd<0)motion_start.s_dd=std::max(motion_start.s_dd,
        -std::sqrt(2*limits.max_longitudinal_jerk*headroom*motion_start.s_d));
    PathGeneratorConfig grid{};grid.dt=dt;grid.time_horizon={duration,duration,0};
    grid.delta_s_dot={0,0,0};
    auto motions=GenerateEnvelopeVelocityCandidates(motion_start,cap,envelope,grid,limits);
    if(motions.empty()||!motions.front().valid)return false;
    const auto& motion=motions.front();
    for(size_t i=0;i<motion.s.size();++i) {
        const double travelled=motion.s[i];
        auto it=std::lower_bound(geometry.points.begin(),geometry.points.end(),travelled,
            [](const RefPoint& point,double position){return point.s<position;});
        if(it==geometry.points.end())return false;
        double distance=0;
        if(it!=geometry.points.begin()) {
            const auto index=static_cast<size_t>(it-geometry.points.begin());
            const auto& before=*(it-1);
            distance=.25*(index-1)+(travelled-before.s)/(it->s-before.s)*.25;
        }
        auto rp=Interpolate(ref,start.s+distance);
        double q=std::min(distance,settle);
        double d=EvalPos(lateral,q),dp=EvalVel(lateral,q),dpp=EvalAcc(lateral,q);
        double metric=std::hypot(1-rp.kappa*d,dp);
        if(metric<.1)return false;
        const double speed=std::max(0.0,motion.s_d[i]),accel=motion.s_dd[i];
        double sd=speed/metric;
        auto base=FrenetToCartesian(rp,start.s+distance,sd,0,d,dp,dpp);
        auto point=FrenetToCartesian(rp,start.s+distance,sd,(accel-base.a)/metric,d,dp,dpp);
        if(!std::isfinite(point.kappa)||std::fabs(point.kappa)>limits.max_curvature||
           speed*speed*std::fabs(point.kappa)>limits.max_lateral_accel+1e-6) {
            const double initial=ego.v*ego.v*std::fabs(geometry.points.front().kappa);
            const double current=speed*speed*std::fabs(point.kappa);
            const double previous=path.v.empty()?initial:
                path.v.back()*path.v.back()*std::fabs(path.kappa.back());
            // This cannot fix the observed initial state retroactively. Retain
            // steering only while recovering a small, decreasing existing excess
            // which is removed within half a second. Future curvature stays hard.
            if(!std::isfinite(point.kappa)||std::fabs(point.kappa)>limits.max_curvature||
               initial>limits.max_lateral_accel+.15||
               i*dt>=.5||current>initial+1e-6||current>previous+1e-6||
               (i>0&&accel>=0))return false;
        }
        path.x.push_back(point.x);path.y.push_back(point.y);path.yaw.push_back(point.yaw);
        path.kappa.push_back(point.kappa);path.v.push_back(speed);path.a.push_back(accel);
    }
    return !path.x.empty();
}
}

void FrenetPlanner::ResetRoundaboutMerge(bool preserve_exit_prediction) {
    execution_.resetMerge();
    roundabout_preferred_d_=0;
    roundabout_acceleration_memory_.Reset();
    if(!preserve_exit_prediction||!roundabout_exit_prediction_active_) {
        roundabout_prediction_.Reset();
        roundabout_exit_prediction_active_=false;
    }
    roundabout_bridge_active_=roundabout_bridge_used_=false;
    roundabout_bridge_travel_=roundabout_bridge_settled_=0;
    roundabout_safe_observations_=0;
    roundabout_last_observation_=roundabout_last_entry_=-1;
    ++roundabout_epoch_;
    last_roundabout_result_=MergePrepareResult{};
}
void FrenetPlanner::ObserveRoundaboutExit(const std::vector<TrackedObject>& objects,double stamp) {
    if(!roundabout_exit_prediction_active_)return;
    if(!std::isfinite(stamp)||stamp>roundabout_exit_prediction_until_) {
        roundabout_exit_prediction_active_=false;roundabout_prediction_.Reset();return;
    }
    roundabout_prediction_.Observe(objects,stamp,object_tracking_cfg_.yaw_rate_clamp_rad_s,
        roundabout_merge_cfg_.speed_measurement_error);
}
void FrenetPlanner::RevokeRoundaboutReadiness() {
    if (execution_->merge.committed) return;
    ++roundabout_epoch_;
    roundabout_safe_observations_=0;
    roundabout_last_observation_=roundabout_last_entry_=-1;
    last_roundabout_result_=MergePrepareResult{};
}
bool FrenetPlanner::CanCommitRoundaboutMerge(const MergePrepareResult& result) const {
    return execution_.canCommitMerge(roundabout_merge_enabled(),
        result.epoch==roundabout_epoch_, result.candidate_valid, result.ready,
        last_roundabout_result_.ready, !result.candidate_path.x.empty());
}
void FrenetPlanner::ApplyRoundaboutMergeCommit(const MergePrepareResult&) {
    execution_.beginMergeCommit();
    stop_=StopContext{};
    execution_.resetStopTarget();
}
bool FrenetPlanner::PlanRoundaboutMerge(const CartesianState& ego,
    const std::vector<TrackedObject>& obstacles,const PlannerCommand& command,
    double planning_time,double observation_stamp,CartesianPath& out_path,
    MergePrepareResult& result,double ego_observation_time) {
    result=MergePrepareResult{};result.epoch=++roundabout_epoch_;
    last_roundabout_result_=result;
    last_failure_reason_=PlanFailureReason::NONE;
    if(!roundabout_merge_enabled() || command.mode!=MERGE ||
       command.merge_type!=MergeType::ROUNDABOUT ||
       command.source_link_id!=roundabout_merge_cfg_.source_link ||
       command.target_link_id!=roundabout_merge_cfg_.target_link ||
       command.merge_sa_id!=-1 || command.merge_sb_id!=-1 ||
       !std::isfinite(planning_time)||!std::isfinite(observation_stamp)||
       !std::isfinite(ego.x)||!std::isfinite(ego.y)||!std::isfinite(ego.yaw)||
       !std::isfinite(ego.v)||!std::isfinite(ego.a)||ego.v<0) {
        roundabout_acceleration_memory_.Reset();
        last_failure_reason_=PlanFailureReason::INVALID_COMMAND;return false;
    }
    speed_profiler_.Reset();
    last_profiled_acceleration_=0.0;
    const double evaluation_time=ego_observation_time>=0?ego_observation_time:planning_time;
    if(!std::isfinite(evaluation_time))return false;
    SetObjectObservationTime(observation_stamp,observation_stamp);
    auto ctx=PreparePlanningContext(ego,obstacles,command);
    roundabout_prediction_.Observe(obstacles,observation_stamp,
        object_tracking_cfg_.yaw_rate_clamp_rad_s,roundabout_merge_cfg_.speed_measurement_error);
    for(auto& object:ctx.predicted_obstacles) {
        object.yaw_rate=roundabout_prediction_.Rate(object.id);
        if(execution_->merge.committed)
            object=roundabout_prediction_.MappedMotion(object,roundabout_zone_.circulating_references,
                roundabout_merge_cfg_.max_horizon);
    }
    auto departure_obstacles=ctx.predicted_obstacles;
    std::vector<TrackedObject> departure_bounds;
    if(!execution_->merge.committed) {
        ctx.predicted_obstacles=roundabout_prediction_.ApproachOccupancies(ctx.predicted_obstacles);
        departure_obstacles=ctx.predicted_obstacles;
        departure_bounds=roundabout_prediction_.DepartureOccupancies(obstacles,
            [&](double x,double y){return roundabout_zone_.SupportsCirculatingObservation(x,y);},
            roundabout_merge_cfg_.partial_vehicle_length,roundabout_merge_cfg_.partial_vehicle_width);
        const auto bodies=roundabout_prediction_.ApproachOccupancies(departure_bounds);
        departure_obstacles.insert(departure_obstacles.end(),bodies.begin(),bodies.end());
        // Expand once per planning cycle for the final departure certificate,
        // not per candidate or as a Cartesian product of vehicle motions.
        departure_obstacles=RoundaboutPrediction::ExpandTurnModels(
            roundabout_prediction_.WithObservedSpeedRanges(departure_obstacles),
            object_tracking_cfg_.yaw_rate_clamp_rad_s);
    }
    const auto sensor_model_obstacles=ctx.predicted_obstacles;
    const double observation_age=std::max(0.0,evaluation_time-observation_stamp);
    for(auto& object:ctx.predicted_obstacles)object=AdvanceObjectObservation(object,observation_age);
    for(auto& object:departure_obstacles)object=AdvanceObjectObservation(object,observation_age);
    last_s_=ctx.s;last_d_=ctx.d;has_last_s_=true;
    double dd,ddd;ArcDerivToTimeDeriv(ctx.s_dot,ctx.s_ddot,ctx.d_prime,ctx.d_pprime,dd,ddd);
    auto start=ClampInitialBoundaryState(ctx.s,ctx.s_dot,ctx.s_ddot,ctx.d,dd,ddd,limits_);
    result.progress=std::clamp((ctx.s-roundabout_zone_.stop_s)/
        (roundabout_zone_.rear_clear_s-roundabout_zone_.stop_s),0.0,1.0);
    const double cap=std::min(ctx.resolved_target_speed,roundabout_merge_cfg_.speed_cap);
    CartesianState traversal_ego=ego;
    if(execution_->merge.committed)
        traversal_ego.a=roundabout_acceleration_memory_.Seed(ego.a,planning_time);
    last_target_speed_=cap;
    if(execution_->merge.committed && ctx.s>=roundabout_zone_.rear_clear_s &&
       !roundabout_zone_.Overlaps(ego,vehicle_shape_,collision_cfg_.safety_margin)) {
        roundabout_acceleration_memory_.Reset();
        PlannerCommand keep=command;keep.mode=LANE_KEEPING;keep.target_speed=cap;keep.has_target_speed=true;
        bool ok=KeepProcess(ego,start,ctx.s,ctx.d,start.s_d,start.s_dd,start.d_d,start.d_dd,
            obstacles,ctx.predicted_obstacles,cap,keep,out_path);
        result.maneuver_completed=ok;result.progress=ok?1:result.progress;
        if(ok) {roundabout_exit_prediction_active_=true;
            roundabout_exit_prediction_until_=observation_stamp+5;}
        last_roundabout_result_=result;return ok;
    }
    // Consume the one bridge episode from actual boundary contact, not from a
    // predicted corner metres ahead. Subsequent plans retain the same budget.
    if(execution_->merge.committed&&!roundabout_bridge_used_ &&
       !avoid_road_map_.FootprintInside(ego.x,ego.y,ego.yaw,vehicle_shape_)) {
        roundabout_bridge_active_=roundabout_bridge_used_=true;
        roundabout_bridge_start_time_=planning_time;
        roundabout_bridge_x_=ego.x;roundabout_bridge_y_=ego.y;
    }
    if(roundabout_bridge_active_) {
        double movement=std::hypot(ego.x-roundabout_bridge_x_,ego.y-roundabout_bridge_y_);
        roundabout_bridge_travel_+=movement;
        roundabout_bridge_x_=ego.x;roundabout_bridge_y_=ego.y;
        VehicleShape settled=vehicle_shape_;
        settled.width+=.10;settled.length+=.10;settled.rear_overhang+=.05;
        if(avoid_road_map_.FootprintInside(ego.x,ego.y,ego.yaw,settled))
            roundabout_bridge_settled_+=movement;
        else roundabout_bridge_settled_=0;
        if(roundabout_bridge_settled_>=.5)roundabout_bridge_active_=false;
    }
    bool early_departure_clear=true;int uncertain=-1;
    if(!execution_->merge.committed) {
    // MakeMergeProfile starts at the measured speed/acceleration and unwinds
    // braking under the existing jerk limit. Certify that actual schedule
    // against every departure model instead of vetoing all braking approaches.
    // PREPARE keeps its stop-capable output until a fresh candidate is committed.
    const auto conflict=Interpolate(ref_,roundabout_zone_.conflict_start_s);
    // Confidence is needed for approaching vehicles. A fragment may stop
    // delaying departure only after reliable motion and a conservative whole
    // vehicle stay clear of the mapped conflict corridor for every turn model.
    const auto passed=[&](int id) {
        if(!roundabout_prediction_.MovingAwayReliable(id,conflict.x,conflict.y))return false;
        auto it=std::find_if(obstacles.begin(),obstacles.end(),[&](const auto& o){return o.id==id;});
        if(it==obstacles.end())return false;
        VehicleShape bound=vehicle_shape_;bound.length=std::max(roundabout_merge_cfg_.partial_vehicle_length,it->length);
        bound.width=std::max(roundabout_merge_cfg_.partial_vehicle_width,it->width);bound.rear_overhang=bound.length/2;
        const double speed=std::hypot(it->vx,it->vy);
        const auto speed_range=roundabout_prediction_.ObservedSpeedRange(id,speed);
        const double speed_spread=std::max(speed-speed_range.first,speed_range.second-speed);
        for(double rate:RoundaboutPrediction::TurnModelRates(
                roundabout_prediction_.Rate(id),object_tracking_cfg_.yaw_rate_clamp_rad_s)) {
            CartesianState pose{};pose.x=it->x;pose.y=it->y;pose.yaw=std::atan2(it->vy,it->vx);
            for(double t=0;t<=2.0;t+=.1) {
                // A passed-vehicle certificate must remain clear for the
                // observed speed interval, not just its latest point estimate.
                if(roundabout_zone_.Overlaps(pose,bound,
                    collision_cfg_.safety_margin+speed_spread*t))return false;
                const double angle=pose.yaw+.05*rate;
                pose.x+=.1*std::hypot(it->vx,it->vy)*std::cos(angle);
                pose.y+=.1*std::hypot(it->vx,it->vy)*std::sin(angle);pose.yaw+=.1*rate;
            }
        }
        return true;
    };
    const auto checked_occupancy=[&](int id) {
        return std::any_of(departure_bounds.begin(),departure_bounds.end(),
            [&](const auto& bound){return bound.id==id;})||passed(id);
    };
    uncertain=roundabout_prediction_.UncertainNear(conflict.x,conflict.y,checked_occupancy);
    // UNKNOWN with zero velocity is not evidence of a stationary vehicle.
    // Gate vehicle-sized observations on the circulating lane until perception
    // establishes motion/classification. Small roadside fixtures are excluded.
    if(uncertain<0)uncertain=UnclassifiedCirculatingObject(obstacles,roundabout_zone_,
        conflict.x,conflict.y);
    early_departure_clear=uncertain<0&&
        planning_time-observation_stamp<=.25&&observation_stamp<=planning_time+.05;
    if(uncertain>=0)result.blocking_object_id=uncertain;
    }
    double best_cost=std::numeric_limits<double>::infinity();
    bool nominal_candidate_found=false;
    // A departure certificate must also be obtainable from a slightly displaced
    // waiting pose. It does not execute movement during PREPARE: the stop path
    // remains authoritative, and the complete departure still needs gap and
    // physical collision checks. Never authorize a fresh boundary excursion
    // from an already contained pre-commit pose.
    const bool waiting_boundary_recovery = !execution_->merge.committed &&
        ctx.s >= roundabout_zone_.stop_s-1.0 &&
        !avoid_road_map_.FootprintInside(ego.x,ego.y,ego.yaw,vehicle_shape_);
    bool selected_bridge=false, profile_failed=false;
    CartesianPath progress_certificate;bool progress_checked=false;
    double selected_d=roundabout_preferred_d_;
    last_candidate_paths_.clear();last_candidate_validity_.clear();
    const double ordinary_settle=std::max(10.0,std::min(25.0,ego.v*2.5));
    // Keep the same mission reference and prefer the last selected local
    // offset, avoiding repeated left/right target changes at a raster corner.
    // Strict grid containment is preferred; an execution/waiting recovery bridge is bounded
    // by 15 cm, 10% of the footprint, 6 m and 5 s before settled re-entry.
    for(bool allow_bridge:{false,true}) {
    if(allow_bridge&&((!execution_->merge.committed&&!waiting_boundary_recovery)||nominal_candidate_found))break;
    for(bool release_positive_accel:{false,true}) {
    if(release_positive_accel&&(ego.a<=0||!profile_failed||nominal_candidate_found))break;
    std::vector<std::pair<double,double>> geometries{
        {ordinary_settle,roundabout_preferred_d_},{6,roundabout_preferred_d_},
        {10,.35},{10,-.35},{6,.35},{6,-.35}};
    if(roundabout_preferred_d_!=0){geometries.push_back({ordinary_settle,0});geometries.push_back({6,0});}
    for(const auto geometry:geometries) {
    double nominal_family_cost=std::numeric_limits<double>::infinity(),nominal_family_paint=0;
    bool nominal_family_valid=false;
    for(double fraction:{1.0,.8,.6,.4}) {
        ++result.generated;
        CartesianPath path;
        bool valid=MakeMergeProfile(ref_,ego,start,cap*fraction,
            roundabout_merge_cfg_.max_horizon,path_cfg_.dt,curve_speed_cfg_,limits_,
            geometry.first,geometry.second,release_positive_accel,path,traversal_ego.a);
        profile_failed|=!valid;
        AvoidRoadMap::PathEvaluation road;bool bridge=false;
        if(valid) {
            road=avoid_road_map_.EvaluatePath(path,vehicle_shape_,path_cfg_.dt);
            if(!road.drivable && allow_bridge && (execution_->merge.committed||waiting_boundary_recovery) &&
               (roundabout_bridge_active_||!roundabout_bridge_used_)) {
                road=avoid_road_map_.EvaluateMergeBoundaryBridge(path,vehicle_shape_,path_cfg_.dt,
                    6.0-roundabout_bridge_travel_,
                    roundabout_bridge_active_?5.0-(planning_time-roundabout_bridge_start_time_):5.0,
                    roundabout_bridge_settled_);
                bridge=road.drivable;
            }
            valid=road.drivable;
        }
        MergeCandidateCheck check;
        if(valid) {
            check=CheckRoundaboutCandidate(path,roundabout_zone_,ctx.predicted_obstacles,
                vehicle_shape_,collision_cfg_,path_cfg_.dt,roundabout_merge_cfg_.min_front_time_gap,
                merge_cfg_.min_rear_time_gap,!execution_->merge.committed);
            valid=check.safe;
            if(check.blocking_object_id>=0)result.blocking_object_id=check.blocking_object_id;
            // One bounded alternative, only after COMMIT and on an ordinary
            // road-contained preferred geometry. Keep every object and the
            // full stationary tail; entry/gap approval is never shortened.
            if(!valid&&execution_->merge.committed&&!bridge&&!progress_checked) {
                progress_checked=true;
                const double prefix_time=std::max(path_cfg_.dt,replanning_period_s_+observation_age);
                auto certified=MakeProgressStoppingCertificate(path,prefix_time,path_cfg_.dt,
                    stop_cfg_.max_service_decel,limits_.max_longitudinal_jerk,
                    prefix_time,.25);
                bool physical=!certified.x.empty()&&certified.v.back()<=.05;
                for(size_t i=0;i<certified.x.size()&&physical;++i)
                    physical=std::isfinite(certified.kappa[i])&&std::fabs(certified.kappa[i])<=limits_.max_curvature&&
                        certified.v[i]*certified.v[i]*std::fabs(certified.kappa[i])<=limits_.max_lateral_accel&&
                        std::fabs(certified.a[i])<=stop_cfg_.max_service_decel;
                if(physical&&avoid_road_map_.EvaluatePath(certified,vehicle_shape_,path_cfg_.dt).drivable) {
                    auto progress_obstacles=sensor_model_obstacles;
                    const double horizon=(certified.x.size()-1)*path_cfg_.dt+observation_age;
                    for(auto& object:progress_obstacles) {
                        object=roundabout_prediction_.MappedMotion(object,roundabout_zone_.circulating_references,horizon);
                        object=AdvanceObjectObservation(object,observation_age);
                    }
                    auto safety=EvaluateCartesianTrajectorySafety(certified,progress_obstacles,
                        vehicle_shape_,collision_cfg_,path_cfg_.dt);
                    if(!safety.clear&&safety.blocking_sample_index==0)
                        safety=EvaluateCartesianTrajectorySafety(certified,progress_obstacles,
                            vehicle_shape_,collision_cfg_,path_cfg_.dt,safety.blocking_object_id);
                    if(safety.clear)progress_certificate=std::move(certified);
                }
            }
        }
        double cost=check.exit_time+AvoidRoadMap::PaintCost(road)+std::fabs(geometry.second);
        if(valid) {
            nominal_candidate_found=nominal_family_valid=true;
            if(cost<nominal_family_cost){nominal_family_cost=cost;nominal_family_paint=road.paint_contact_seconds;}
            // Select among the existing candidates AFTER the full certificate.
            // No extra speed/geometry catalogue or delayed-entry samples.
            if(!execution_->merge.committed&&early_departure_clear) {
                const auto final=CheckRoundaboutCandidate(path,roundabout_zone_,departure_obstacles,
                    vehicle_shape_,collision_cfg_,path_cfg_.dt,roundabout_merge_cfg_.min_front_time_gap,
                    merge_cfg_.min_rear_time_gap,true);
                valid=final.safe;
                if(!valid&&final.blocking_object_id>=0)result.blocking_object_id=final.blocking_object_id;
            }
        }
        last_candidate_paths_.push_back(path);last_candidate_validity_.push_back(valid);
        if(!valid)continue;
        ++result.valid;
        if(cost<best_cost){best_cost=cost;selected_d=geometry.second;selected_bridge=bridge;result.candidate_valid=true;
            result.entry_time=check.entry_time;result.exit_time=check.exit_time;
            result.candidate_path=std::move(path);}
    }
    // Prefer the original reference geometry when it is feasible. Alternate
    // local shapes rescue geometry or prolonged paint contact, with bounded work.
    if(nominal_family_valid&&nominal_family_paint<.3)break;
    }
    if(nominal_candidate_found)break;
    }
    if(nominal_candidate_found)break;
    }
    if(!execution_->merge.committed&&uncertain>=0)result.blocking_object_id=uncertain;
    else if(result.candidate_valid)result.blocking_object_id=-1;
    if(execution_->merge.committed) {
        if(!result.candidate_valid&&!progress_certificate.x.empty()) {
            result.progress_stopping=true;
            result.candidate_path=std::move(progress_certificate);++result.valid;
        }
        if(result.candidate_valid||result.progress_stopping){
            if(selected_bridge&&!roundabout_bridge_active_&&
               !avoid_road_map_.FootprintInside(ego.x,ego.y,ego.yaw,vehicle_shape_)) {
                roundabout_bridge_active_=roundabout_bridge_used_=true;
                roundabout_bridge_start_time_=planning_time;
                roundabout_bridge_x_=ego.x;roundabout_bridge_y_=ego.y;
            }
            roundabout_preferred_d_=selected_d;
            roundabout_acceleration_memory_.Accept(result.candidate_path.a,planning_time,path_cfg_.dt);
            out_path=result.candidate_path;last_roundabout_result_=result;return true;
        }
        last_roundabout_result_=result;
        roundabout_acceleration_memory_.Reset();
        // Keep a steerable, freshly checked stop trajectory when a complete
        // traversal is unavailable. This MERGE-only service-brake fallback can
        // retain an already measured service deceleration; the generic KEEP
        // fallback rejects |a| above the ordinary acceleration limit.
        bool braking=false;
        KinematicLimits braking_limits=limits_;
        braking_limits.max_longitudinal_accel=std::max(limits_.max_longitudinal_accel,
            stop_cfg_.max_service_decel);
        braking_limits.boundary_condition_accel_fraction=1.0;
        const double duration=std::max(collision_cfg_.reactive_lookahead,
            ego.v/std::max(.1,stop_cfg_.comfortable_decel)+
            2*braking_limits.max_longitudinal_accel/braking_limits.max_longitudinal_jerk);
        for(double settle:{ordinary_settle,6.0,3.0}) {
            for(double target:{roundabout_preferred_d_,0.0,.35,-.35}) {
                CartesianPath stop;
                if(!MakeMergeProfile(ref_,ego,start,0,duration,path_cfg_.dt,
                    curve_speed_cfg_,braking_limits,settle,target,false,stop))continue;
                auto road=avoid_road_map_.EvaluatePath(stop,vehicle_shape_,path_cfg_.dt);
                if(!road.drivable&&(roundabout_bridge_active_||!roundabout_bridge_used_))
                    road=avoid_road_map_.EvaluateMergeBoundaryBridge(stop,vehicle_shape_,path_cfg_.dt,
                        6.0-roundabout_bridge_travel_,
                        roundabout_bridge_active_?5.0-(planning_time-roundabout_bridge_start_time_):5.0,
                        roundabout_bridge_settled_);
                if(!road.drivable)continue;
                auto safety=EvaluateCartesianTrajectorySafety(stop,ctx.predicted_obstacles,
                    vehicle_shape_,collision_cfg_,path_cfg_.dt);
                if(!safety.clear&&safety.blocking_sample_index==0)
                    safety=EvaluateCartesianTrajectorySafety(stop,ctx.predicted_obstacles,
                        vehicle_shape_,collision_cfg_,path_cfg_.dt,safety.blocking_object_id);
                if(!safety.clear)continue;
                out_path=std::move(stop);braking=true;break;
            }
            if(braking)break;
        }
        // Retain an existing collision-checked braking option when it is
        // feasible; the new service profiles add alternatives, not a regression
        // to a one-point stop where the old braking path was still valid.
        if(!braking)braking=PlanCollisionBrakingPath(ego,ctx.s,ctx.d,
            ctx.predicted_obstacles,out_path);
        last_failure_reason_=braking?PlanFailureReason::NONE:PlanFailureReason::COLLISION_BLOCKED;
        return braking;
    }
    // Every selected departure has already passed the same final models.
    if(!result.candidate_valid||!early_departure_clear) {
        roundabout_safe_observations_=0;roundabout_last_entry_=-1;
    } else if(observation_stamp>roundabout_last_observation_+1e-6) {
        double absolute_entry=planning_time+result.entry_time;
        if(roundabout_last_entry_>=0 && std::fabs(absolute_entry-roundabout_last_entry_)>
            roundabout_merge_cfg_.confirmation_time_tolerance)roundabout_safe_observations_=0;
        ++roundabout_safe_observations_;roundabout_last_entry_=absolute_entry;
    } else if(observation_stamp<roundabout_last_observation_-1e-6) {
        roundabout_safe_observations_=0;roundabout_last_entry_=-1;
    }
    roundabout_last_observation_=observation_stamp;
    // Restrict early commitment to one body length before the frontier. A
    // distant forecast is not permission to commit while still approaching.
    result.ready=result.candidate_valid && early_departure_clear && roundabout_safe_observations_>=merge_cfg_.safe_confirm_cycles &&
        ctx.s>=roundabout_zone_.stop_s-vehicle_shape_.length &&
        ctx.s<=roundabout_zone_.stop_s+1.0;
    // Once actually stopped in the departure zone, hold here until a fresh
    // full departure is authorized. Do not inch forward just to stop again.
    execution_.observeMergeWaiting(ego.v<=.1&&
        ctx.s>=roundabout_zone_.stop_s-vehicle_shape_.length&&
        ctx.s<=roundabout_zone_.stop_s+.02);
    if(execution_->merge.waiting_latched){
        PlanExternalStopHold(ego,ctx.s,out_path);
        last_failure_reason_=PlanFailureReason::NONE;
        last_roundabout_result_=result;return true;
    }
    // The PREPARE output remains a stop-capable approach/hold trajectory.
    // Its target is the rear axle at the map stopline. The frontier generator
    // checks approach/hold independently from the complete departure candidate.
    PlannerCommand waiting=command;waiting.mode=STOP;waiting.has_target_speed=true;
    waiting.target_speed=cap;waiting.stop_position=roundabout_zone_.stop_s;
    const bool lateral_hold=StopLateralHoldAllowed(ctx.s_dot,ctx.d,
        stop_cfg_.lateral_hold_speed,stop_cfg_.lateral_error_tolerance);
    const double waiting_d=lateral_hold?ctx.d:0;
    // This target preference does not authorize an unsafe offset. Both terminal
    // choices still pass the original road, collision and physical constraints.
    waiting.has_stop_lateral_hold=lateral_hold;
    waiting.stop_lateral_hold_d=waiting_d;
    bool ok=MakeFrontierWaitingPath(ref_,ego,start,ctx.s,ctx.d,
        roundabout_zone_.stop_s,cap,avoid_road_map_,vehicle_shape_,limits_,collision_cfg_,
        ctx.predicted_obstacles,stop_cfg_.max_service_decel,stop_cfg_.comfortable_decel,
        stop_cfg_.reapproach_accel,ctx.s_ddot,path_cfg_.dt,
        roundabout_merge_cfg_.max_horizon,waiting_d,ctx.d_pprime,
        stop_cfg_.lateral_error_tolerance,out_path);
    if(!ok)ok=StopProcess(ego,start,ctx.s,ctx.d,start.s_d,start.s_dd,start.d_d,start.d_dd,
        obstacles,ctx.predicted_obstacles,cap,waiting,out_path);
    // The legacy STOP fallback may produce an overshooting horizon. Never
    // execute it across the waiting frontier before commitment, and check all
    // objects even if generic STOP scoped them by current longitudinal position.
    if(ok){double hint=ctx.s;
        for(size_t i=0;i<out_path.x.size();++i){
            hint=FindClosestS(ref_,out_path.x[i],out_path.y[i],&hint,5);
            if(hint>std::max(ctx.s,roundabout_zone_.stop_s)+.02){ok=false;break;}
        }
        if(ok&&!CartesianTrajectoryClear(out_path,ctx.predicted_obstacles,vehicle_shape_,
            collision_cfg_,path_cfg_.dt))ok=false;
    }
    if(!ok){
        // A rejected frontier arrival still needs a steering-capable safe stop.
        // Reuse the committed service-braking generator, bounded by this line.
        auto braking_limits=limits_;
        braking_limits.max_longitudinal_accel=std::max(limits_.max_longitudinal_accel,
            stop_cfg_.max_service_decel);
        braking_limits.boundary_condition_accel_fraction=1;
        for(double settle:{ordinary_settle,6.0}){
            CartesianPath brake;
            if(!MakeMergeProfile(ref_,ego,start,0,std::min(8.0,roundabout_merge_cfg_.max_horizon),
                path_cfg_.dt,curve_speed_cfg_,braking_limits,settle,ctx.d,false,brake))continue;
            bool bounded=true;double hint=ctx.s;
            for(size_t i=0;i<brake.x.size();++i){
                hint=FindClosestS(ref_,brake.x[i],brake.y[i],&hint,5);
                if(hint>std::max(ctx.s,roundabout_zone_.stop_s)+.02){bounded=false;break;}
            }
            if(!bounded||!avoid_road_map_.EvaluatePath(brake,vehicle_shape_,path_cfg_.dt).drivable||
               !CartesianTrajectoryClear(brake,ctx.predicted_obstacles,vehicle_shape_,collision_cfg_,path_cfg_.dt))continue;
            out_path=std::move(brake);ok=true;break;
        }
    }
    if(ok) {
        CartesianPath centered;
        if(RecenterWaitingFallback(ref_,ego,ctx.s,ctx.d,limits_,
                stop_cfg_.max_service_decel,out_path,centered)&&
           avoid_road_map_.EvaluatePath(centered,vehicle_shape_,path_cfg_.dt).drivable&&
           CartesianTrajectoryClear(centered,ctx.predicted_obstacles,vehicle_shape_,
                collision_cfg_,path_cfg_.dt))out_path=std::move(centered);
        CartesianPath aligned;
        if(AlignWaitingApproach(ref_,ego,avoid_road_map_,vehicle_shape_,limits_,
            roundabout_zone_.stop_s,ctx.s,ctx.d,stop_cfg_.max_service_decel,
            path_cfg_.dt,out_path,aligned)&&
           CartesianTrajectoryClear(aligned,ctx.predicted_obstacles,vehicle_shape_,
               collision_cfg_,path_cfg_.dt))out_path=std::move(aligned);
    }
    if(!ok&&result.ready&&ego.v<.1){PlanExternalStopHold(ego,ctx.s,out_path);ok=true;}
    if(!ok)result.ready=false;
    last_roundabout_result_=result;
    return ok;
}


