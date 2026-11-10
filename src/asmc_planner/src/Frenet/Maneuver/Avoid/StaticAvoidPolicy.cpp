#include "Frenet/Maneuver/Avoid/StaticAvoidPolicy.hpp"
#include "Frenet/Maneuver/ManeuverUtils.hpp"
#include "Frenet/Geometry/PolygonUtils.hpp"
#include <algorithm>
#include <cmath>

bool IsStaticAvoidProgressObject(const TrackedObject& o) {
    for(const char* mobile:{"person","pedestrian","car","vehicle","truck","bus",
            "bicycle","bike","motorcycle","rider","stroller","dog","deer","wheelchair"})
        if(o.camera_class==mobile||o.classification==mobile)return false;
    return o.geometry_valid&&geometry::ValidateCorners8(o.corners).valid&&!o.velocity_unavailable&&
        (o.type==0||o.type==3)&&o.motion_state==ObjectMotionState::LOW_THREAT&&
        o.classification!="dynamic_obstacle";
}

namespace {
bool Eligible(const TrackedObject& o,const StaticAvoidPolicy& policy) {
    return IsStaticAvoidProgressObject(o)&&
        std::find(policy.object_ids.begin(),policy.object_ids.end(),o.id)!=policy.object_ids.end();
}
double Penetration(const geometry::Polygon2D& a,const geometry::Polygon2D& b) {
    double depth=INFINITY;
    for(const auto* polygon:{&a,&b})for(std::size_t i=0;i<polygon->size();++i) {
        const auto& p=(*polygon)[i];const auto& q=(*polygon)[(i+1)%polygon->size()];
        const double length=std::hypot(q.x-p.x,q.y-p.y);if(length<1e-9)continue;
        const double nx=-(q.y-p.y)/length,ny=(q.x-p.x)/length;
        double amin=INFINITY,amax=-INFINITY,bmin=INFINITY,bmax=-INFINITY;
        for(const auto& v:a){const double s=v.x*nx+v.y*ny;amin=std::min(amin,s);amax=std::max(amax,s);}
        for(const auto& v:b){const double s=v.x*nx+v.y*ny;bmin=std::min(bmin,s);bmax=std::max(bmax,s);}
        if(amax<bmin||bmax<amin)return 0;
        depth=std::min(depth,std::min(amax-bmin,bmax-amin));
    }
    return std::isfinite(depth)?std::max(0.0,depth):0;
}
}

bool StaticAvoidTrajectoryClear(const CartesianPath& path,
    const std::vector<TrackedObject>& objects,const VehicleShape& shape,
    const CollisionCheckConfig& nominal,double dt,const StaticAvoidPolicy* policy,
    int* blocker,bool* recovery,const std::vector<int>* initial_recovery_ids) {
    if(!policy||policy->mode==StaticAvoidPolicy::NORMAL)
        return CartesianAvoidTrajectoryClear(path,objects,shape,nominal,dt,blocker,-1,recovery,initial_recovery_ids);
    std::vector<TrackedObject> fixed,strict;
    for(const auto& o:objects)(Eligible(o,*policy)?fixed:strict).push_back(o);
    bool strict_recovery=false,fixed_recovery=false;
    if(!CartesianAvoidTrajectoryClear(path,strict,shape,nominal,dt,blocker,-1,&strict_recovery,initial_recovery_ids))return false;
    if(policy->mode!=StaticAvoidPolicy::CONTACT_PROGRESS) {
        auto reduced=nominal;reduced.safety_margin=std::min(nominal.safety_margin,policy->margin);
        if(!CartesianAvoidTrajectoryClear(path,fixed,shape,reduced,dt,blocker,-1,&fixed_recovery,initial_recovery_ids))return false;
    }
    if(recovery)*recovery=strict_recovery||fixed_recovery;
    return true;
}

StaticAvoidContact MeasureStaticAvoidContact(const CartesianPath& path,
    const std::vector<TrackedObject>& objects,const VehicleShape& shape,
    const StaticAvoidPolicy& policy) {
    StaticAvoidContact result;
    std::vector<geometry::Polygon2D> polygons;
    for(const auto& o:objects)if(Eligible(o,policy))polygons.push_back(PredictObstacleFootprint(o,0));
    for(std::size_t i=0;i<path.x.size();++i) {
        CartesianState pose{};pose.x=path.x[i];pose.y=path.y[i];pose.yaw=path.yaw[i];
        const auto body=MakeEgoFootprint(pose,shape,0);
        const double ds=i?std::hypot(path.x[i]-path.x[i-1],path.y[i]-path.y[i-1]):.1;
        for(const auto& polygon:polygons) {
            const double gap=geometry::ConvexPolygonDistance(body,polygon);
            result.minimum_clearance=std::min(result.minimum_clearance,gap);
            if(gap<=1e-9) {
                result.physical_overlap=true;
                const double depth=Penetration(body,polygon);
                result.cost+=(1+10*depth*depth)*ds;
            }
        }
    }
    return result;
}
