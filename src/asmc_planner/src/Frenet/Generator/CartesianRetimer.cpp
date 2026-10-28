#include "Frenet/Generator/CartesianRetimer.hpp"
#include "Frenet/Generator/LongitudinalGenerator.hpp"
#include "Frenet/Speed/CurveSpeed.hpp"
#include "Frenet/Reference/Reference.hpp"
#include <algorithm>
#include <cmath>

namespace {
bool PhysicalReference(const CartesianPath& geometry,RefLine& physical) {
    const auto n=geometry.x.size();physical={};
    if(n<2||geometry.y.size()!=n||geometry.yaw.size()!=n||geometry.kappa.size()!=n)return false;
    for(std::size_t i=0;i<n;++i) {
        if(!std::isfinite(geometry.x[i])||!std::isfinite(geometry.y[i])||
           !std::isfinite(geometry.yaw[i])||!std::isfinite(geometry.kappa[i]))return false;
        const double distance=i?std::hypot(geometry.x[i]-physical.points.back().x,
            geometry.y[i]-physical.points.back().y):0;
        if(i&&distance<1e-6)continue;
        const double s=physical.points.empty()?0:physical.points.back().s+distance;
        physical.points.push_back({geometry.x[i],geometry.y[i],s,geometry.yaw[i],geometry.kappa[i],0});
    }
    if(physical.points.size()<3)return false;
    physical.length=physical.points.back().s;
    return true;
}
bool SamplePhysicalMotion(const RefLine& physical,const FrenetPath& motion,CartesianPath& output) {
    output={};
    for(std::size_t i=0;i<motion.s.size();++i) {
        if(motion.s[i]>physical.length+1e-6)break;
        const auto point=Interpolate(physical,motion.s[i]);
        output.x.push_back(point.x);output.y.push_back(point.y);output.yaw.push_back(point.theta);
        output.kappa.push_back(point.kappa);output.v.push_back(motion.s_d[i]);output.a.push_back(motion.s_dd[i]);
    }
    return output.x.size()>1;
}
}
bool RetimeCartesianGeometry(const CartesianPath& geometry,double speed,double acceleration,
    double target,const PathGeneratorConfig& grid,const KinematicLimits& limits,
    const CurveSpeedConfig& curve,CartesianPath& output,FrenetPath& motion) {
    output={};motion={};RefLine physical;
    if(!PhysicalReference(geometry,physical))return false;
    const auto envelope=BuildKeepSpeedEnvelope(physical,0,speed,acceleration,
        grid.time_horizon.max,grid.dt,target,curve,limits.max_longitudinal_accel,
        limits.max_lateral_accel,limits.max_longitudinal_jerk);
    if(envelope.empty())return false;
    FrenetState start{};start.s_d=speed;start.s_dd=acceleration;
    const auto candidates=GenerateEnvelopeVelocityCandidates(start,target,envelope,grid,limits);
    if(candidates.empty())return false;
    motion=candidates.front();
    return SamplePhysicalMotion(physical,motion,output);
}

std::vector<CartesianFollowingCandidate> RetimeCartesianFollowingGeometry(
    const CartesianPath& geometry,double speed,double acceleration,double target,
    const std::vector<TrackedObject>& objects,const VehicleShape& body,
    const FollowingConfig& following,const PathGeneratorConfig& grid,
    const KinematicLimits& limits,const CurveSpeedConfig& curve,double age) {
    std::vector<CartesianFollowingCandidate> result;RefLine physical;
    if(!std::isfinite(age)||age<0||!std::isfinite(target)||target<=0||
       !std::isfinite(speed)||speed<0||!std::isfinite(acceleration)||
       !PhysicalReference(geometry,physical))return result;
    const TrackedObject* leader=nullptr;double nearest=INFINITY,leader_speed=0,approach=target;
    for(const auto& object:objects) {
        const bool moving_unknown=object.type==int(asmc_planner::ObjectType::UNKNOWN)&&
            (object.motion_state==ObjectMotionState::DYNAMIC||object.motion_state==ObjectMotionState::DYNAMIC_PENDING);
        if((object.type!=int(asmc_planner::ObjectType::VEHICLE)&&!moving_unknown)||!object.geometry_valid)continue;
        auto nominal=object;nominal.prediction_min_speed=nominal.prediction_max_speed=NAN;
        const auto pose=PredictTrackedObjectPose(nominal,age);
        const double station=FindClosestS(physical,pose.x,pose.y,nullptr,100);
        const auto rp=Interpolate(physical,station);
        const auto velocity=PredictTrackedObjectVelocity(nominal,age);
        const double along=velocity.first*std::cos(rp.theta)+velocity.second*std::sin(rp.theta);
        if(along<0||station<=0||station>following.max_leader_search_s||
           station>=physical.length-1e-3)continue;
        double lo=INFINITY,hi=-INFINITY,front=INFINITY;
        for(const auto& corner:PredictObstacleFootprint(nominal,age)) {
            const double s=FindClosestS(physical,corner.x,corner.y,nullptr,100);
            const auto p=Interpolate(physical,s);
            const double d=-(corner.x-p.x)*std::sin(p.theta)+(corner.y-p.y)*std::cos(p.theta);
            lo=std::min(lo,d);hi=std::max(hi,d);front=std::min(front,s);
        }
        // The future connector, not ego's tilted body axis or an adjacent
        // lane's center, determines which front actor actually constrains it.
        if(front<=body.length-body.rear_overhang||lo>body.width*.5||hi<-body.width*.5)continue;
        const double bound=std::clamp(along+following.gap_gain*
            (station-following.min_gap-following.time_gap*along),0.0,target);
        if(bound>approach||(bound==approach&&station>=nearest))continue;
        leader=&object;nearest=station;leader_speed=along;approach=bound;
    }
    if(!leader)return result;
    if(approach>=target-1e-3)return result;
    FrenetState start{};start.s_d=speed;start.s_dd=acceleration;
    auto bounded=grid;bounded.delta_s={0,0,1};
    bounded.time_horizon.step=std::max(grid.dt,grid.time_horizon.max-bounded.time_horizon.min);
    auto motions=GenerateFollowingCandidates(start,nearest,leader_speed,0,
        following.time_gap,following.min_gap,target,following.gap_gain,bounded);
    FilterLongitudinalByKinematicLimits(motions,limits);
    bool exact=false;for(const auto& m:motions)exact=exact||m.valid;
    if(!exact)motions=GenerateFollowingRecoveryCandidates(start,nearest,leader_speed,0,
        following.time_gap,following.min_gap,target,following.gap_gain,bounded,limits);
    FilterLongitudinalByKinematicLimits(motions,limits);
    for(auto& m:motions) {
        CartesianPath path;
        if(!m.valid||!SamplePhysicalMotion(physical,m,path))continue;
        bool admissible=true;
        for(std::size_t i=0;i<path.v.size();++i) {
            const double ceiling=VelocityFromCurvature(std::abs(path.kappa[i]),curve,limits.max_lateral_accel);
            // An achieved initial overspeed needs a braking envelope, not a
            // newly invented speed boundary. The caller still checks all hard
            // curvature/acceleration/jerk limits on the certified span.
            if(path.v[i]>std::max(speed,ceiling)+1e-6||
               path.a[i]>limits.max_forward_accel+1e-6){admissible=false;break;}
        }
        if(admissible)result.push_back({std::move(path),std::move(m),leader->id});
    }
    return result;
}
