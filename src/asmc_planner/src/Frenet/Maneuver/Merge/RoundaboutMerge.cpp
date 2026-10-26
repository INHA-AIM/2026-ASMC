#include "Frenet/Maneuver/Merge/RoundaboutMerge.hpp"
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include "Frenet/Geometry/PolygonUtils.hpp"

namespace {
double Cross(geometry::Point2D a, geometry::Point2D b, geometry::Point2D c) {
    return (b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x);
}
bool OnSegment(geometry::Point2D a, geometry::Point2D b, geometry::Point2D p) {
    return std::fabs(Cross(a,b,p)) <= 1e-8 &&
        p.x >= std::min(a.x,b.x)-1e-8 && p.x <= std::max(a.x,b.x)+1e-8 &&
        p.y >= std::min(a.y,b.y)-1e-8 && p.y <= std::max(a.y,b.y)+1e-8;
}
bool Inside(geometry::Point2D p, const geometry::Polygon2D& polygon) {
    bool in=false;
    for(size_t i=0,j=polygon.size()-1;i<polygon.size();j=i++) {
        auto a=polygon[j], b=polygon[i];
        if(OnSegment(a,b,p))return true;
        if((a.y>p.y)!=(b.y>p.y) &&
           p.x<(b.x-a.x)*(p.y-a.y)/(b.y-a.y)+a.x)in=!in;
    }
    return in;
}
bool Overlap(const geometry::Polygon2D& a, const geometry::Polygon2D& b) {
    if(a.size()<3 || b.size()<3)return true;
    double ax0=1e30,ax1=-1e30,ay0=1e30,ay1=-1e30;
    double bx0=1e30,bx1=-1e30,by0=1e30,by1=-1e30;
    for(auto p:a){ax0=std::min(ax0,p.x);ax1=std::max(ax1,p.x);ay0=std::min(ay0,p.y);ay1=std::max(ay1,p.y);}
    for(auto p:b){bx0=std::min(bx0,p.x);bx1=std::max(bx1,p.x);by0=std::min(by0,p.y);by1=std::max(by1,p.y);}
    if(ax1<bx0 || bx1<ax0 || ay1<by0 || by1<ay0)return false;
    for(auto p:a)if(Inside(p,b))return true;
    for(auto p:b)if(Inside(p,a))return true;
    for(size_t i=0;i<a.size();++i)for(size_t j=0;j<b.size();++j){
        auto p=a[i],q=a[(i+1)%a.size()],r=b[j],s=b[(j+1)%b.size()];
        if(OnSegment(p,q,r)||OnSegment(p,q,s)||OnSegment(r,s,p)||OnSegment(r,s,q))return true;
        if(Cross(p,q,r)*Cross(p,q,s)<0 && Cross(r,s,p)*Cross(r,s,q)<0)return true;
    }
    return false;
}
geometry::Polygon2D ReadPolygon(const YAML::Node& points) {
    geometry::Polygon2D polygon;
    for(auto p:points){
        const double x=p[0].as<double>(),y=p[1].as<double>();
        if(!std::isfinite(x)||!std::isfinite(y))throw std::runtime_error("non-finite map coordinate");
        polygon.push_back({x,y});
    }
    if(polygon.size()<3)throw std::runtime_error("invalid conflict polygon");
    return polygon;
}
CartesianState Pose(const CartesianPath& path, size_t i) {
    CartesianState p{};p.x=path.x[i];p.y=path.y[i];p.yaw=path.yaw[i];return p;
}
}

bool RoundaboutMergeZone::Overlaps(const CartesianState& pose,
    const VehicleShape& shape,double margin) const {
    if(!valid)return true;
    auto footprint=MakeEgoFootprint(pose,shape,margin);
    for(const auto& area:conflict_areas)if(Overlap(footprint,area))return true;
    return false;
}
bool RoundaboutMergeZone::SupportsCirculatingObservation(double x,double y) const {
    if(!valid||!std::isfinite(x)||!std::isfinite(y))return false;
    // The same two-metre observation guard as the UNKNOWN confidence query.
    // This identifies traffic evidence only; it grants no ego road clearance.
    for(const auto& area:circulating_areas) {
        if(Inside({x,y},area))return true;
        for(std::size_t i=0;i<area.size();++i) {
            const auto a=area[i],b=area[(i+1)%area.size()];
            const double dx=b.x-a.x,dy=b.y-a.y,length2=dx*dx+dy*dy;
            const double u=length2>0?std::clamp(((x-a.x)*dx+(y-a.y)*dy)/length2,0.0,1.0):0;
            if(std::hypot(x-a.x-u*dx,y-a.y-u*dy)<=2)return true;
        }
    }
    return false;
}
bool RoundaboutMergeZone::Load(const std::string& areas_path,
    const std::string& lines_path,const RefLine& ref,const VehicleShape& shape,
    const RoundaboutMergeConfig& config,double margin,std::string& error) {
    *this=RoundaboutMergeZone{};
    try {
        auto areas=YAML::LoadFile(areas_path), lines=YAML::LoadFile(lines_path);
        bool found_stop=false,found_source=false,found_target=false;
        double scan_end=-1;
        for(auto lane:areas["lanes"]){
            auto id=lane["link_id"].as<std::string>();
            found_source|=id==config.source_link;found_target|=id==config.target_link;
            if(std::find(config.conflict_links.begin(),config.conflict_links.end(),id)==config.conflict_links.end())continue;
            auto ps=lane["centerline"];auto last=ps[ps.size()-1];
            scan_end=std::max(scan_end,FindClosestS(ref,last[0].as<double>(),last[1].as<double>(),nullptr,1e6));
        }
        for(auto feature:lines["features"]){
            if(feature["feature_type"].as<std::string>("")!="stopline" ||
               feature["provenance"]["source_id"].as<std::string>("")!=config.stop_node)continue;
            bool source=false;for(auto id:feature["topology"]["current_link_ids"])source|=id.as<std::string>()==config.source_link;
            if(!source)continue;
            if(found_stop)throw std::runtime_error("ambiguous roundabout stopline");
            auto ps=feature["geometry"]["coordinates"];if(ps.size()!=2)throw std::runtime_error("stopline must have two endpoints");
            stop_s=FindClosestS(ref,.5*(ps[0][0].as<double>()+ps[1][0].as<double>()),.5*(ps[0][1].as<double>()+ps[1][1].as<double>()),nullptr,1e6);
            found_stop=true;
        }
        for(const auto& id:config.conflict_links){
            bool found=false;
            for(auto area:areas["drivable_areas"]){
                // Shared junction surface polygons describe available pavement, not
                // the circulating lane corridor. Use the lane-specific polygon.
                bool match=area["id"].as<std::string>("")=="drivable:"+id;
                if(match){conflict_areas.push_back(ReadPolygon(area["polygon"]));found=true;}
            }
            if(!found)throw std::runtime_error("missing conflict link: "+id);
        }
        // Read the SAME compact product as all ego modes. An ego exit link
        // (A151) is not the circulating actor's successor. Do not invent an
        // actor route by following the route-compact successor list.
        for(const auto& lane:areas["lanes"]) {
            if(lane["drivable_group"].as<std::string>("")!="roundabout_maneuver")continue;
            circulating_areas.push_back(ReadPolygon(lane["polygon"]));
            auto line=std::make_shared<RefLine>();
            for(const auto& p:lane["centerline"]) {
                const double x=p[0].as<double>(),y=p[1].as<double>();
                if(!std::isfinite(x)||!std::isfinite(y))throw std::runtime_error("invalid circulating centerline");
                const double ds=line->points.empty()?0:std::hypot(x-line->points.back().x,y-line->points.back().y);
                if(!line->points.empty()&&ds<1e-5)continue;
                line->length+=ds;line->points.push_back({x,y,line->length,0,0,0});
            }
            if(line->points.size()<3)continue;
            for(size_t i=0;i<line->points.size();++i) {
                const auto& a=line->points[i?i-1:0];
                const auto& b=line->points[std::min(i+1,line->points.size()-1)];
                line->points[i].theta=std::atan2(b.y-a.y,b.x-a.x);
            }
            for(size_t i=0;i<line->points.size();++i) {
                const auto& a=line->points[i?i-1:0];
                const auto& b=line->points[std::min(i+1,line->points.size()-1)];
                line->points[i].kappa=std::remainder(b.theta-a.theta,2*M_PI)/(b.s-a.s);
            }
            circulating_references.push_back(std::move(line));
        }
        if(!found_source||!found_target||!found_stop||scan_end<=stop_s)
            throw std::runtime_error("roundabout topology/stopline is missing");
        valid=true;
        auto p=Interpolate(ref,stop_s);CartesianState waiting{};waiting.x=p.x;waiting.y=p.y;waiting.yaw=p.theta;waiting_pose=waiting;
        if(Overlaps(waiting,shape,margin))throw std::runtime_error("rear axle at stopline overlaps circulating corridor");
        bool seen=false;double last_overlap=stop_s;
        for(double s=stop_s;s<=scan_end+shape.length+margin+2;s+=.1){
            auto rp=Interpolate(ref,s);CartesianState pose{};pose.x=rp.x;pose.y=rp.y;pose.yaw=rp.theta;
            if(Overlaps(pose,shape,margin)){if(!seen)conflict_start_s=s;seen=true;last_overlap=s;}
        }
        if(!seen)throw std::runtime_error("reference never enters conflict corridor");
        rear_clear_s=last_overlap+.2;
        error.clear();return true;
    }catch(const std::exception& e){valid=false;error=e.what();return false;}
}

MergeCandidateCheck CheckRoundaboutCandidate(const CartesianPath& path,
    const RoundaboutMergeZone& zone,const std::vector<TrackedObject>& objects,
    const VehicleShape& shape,const CollisionCheckConfig& collision,double dt,
    double front_time_gap,double rear_time_gap,bool entering) {
    MergeCandidateCheck result;
    const size_t count=path.x.size();
    if(!zone.valid||count<2||path.y.size()!=count||path.yaw.size()!=count||!std::isfinite(dt)||dt<=0 ||
       !std::isfinite(front_time_gap)||front_time_gap<0 ||
       !std::isfinite(rear_time_gap)||rear_time_gap<0)return result;
    for(size_t i=0;i<count;++i)
        if(!std::isfinite(path.x[i])||!std::isfinite(path.y[i])||!std::isfinite(path.yaw[i]))return result;
    bool occupied=false;
    for(size_t i=0;i<count;++i){
        bool hit=zone.Overlaps(Pose(path,i),shape,collision.safety_margin);
        if(hit){if(!occupied)result.entry_time=i*dt;occupied=true;result.exit_time=-1;}
        else if(occupied && result.exit_time<0)result.exit_time=i*dt;
    }
    result.covers_exit=occupied && result.exit_time>=0;
    // A post-commit path already beyond the zone may hand off normally.
    if(!entering&&!occupied)result.covers_exit=true;
    if(!result.covers_exit)return result;
    auto safety=EvaluateCartesianTrajectorySafety(path,objects,shape,collision,dt);
    // A circulating car can stop beside us after commitment. If only its
    // expanded margin touches the current pose, preserve physical collision
    // checks and require increasing clearance while leaving that margin.
    if(!entering&&!safety.clear&&safety.blocking_sample_index==0)
        safety=EvaluateCartesianTrajectorySafety(path,objects,shape,collision,dt,
                                                 safety.blocking_object_id);
    if(!safety.clear){result.blocking_object_id=safety.blocking_object_id;return result;}
    if(entering)for(size_t i=0;i<count;++i){
        auto p=Pose(path,i);if(!zone.Overlaps(p,shape,collision.safety_margin))continue;
        // Check intermediate time shifts, not only the two endpoints: a fast
        // vehicle may cross this footprint between those endpoints.
        for(double shift=-front_time_gap;shift<=rear_time_gap+1e-8;shift+=dt){
            double t=std::max(0.0,i*dt+shift);
            auto footprint=MakeEgoFootprint(p,shape,collision.safety_margin+collision.margin_growth_rate*t);
            for(const auto& object:objects)if(CheckObstacleOverlap(footprint,{},object,t)){
                result.blocking_object_id=object.id;return result;
            }
        }
    }
    result.safe=true;return result;
}

int UnclassifiedCirculatingObject(const std::vector<TrackedObject>& objects,
    const RoundaboutMergeZone& zone,double conflict_x,double conflict_y) {
    if(!zone.valid)return -1;
    for(const auto& object:objects) {
        if(object.motion_state!=ObjectMotionState::UNKNOWN||!object.geometry_valid||
           object.length<2||object.width<.8||std::hypot(object.vx,object.vy)>=.5)continue;
        if(std::hypot(object.x-conflict_x,object.y-conflict_y)>25)continue;
        const auto footprint=geometry::ComputeConvexHull(geometry::ProjectCornersToXY(object.corners));
        for(const auto& area:zone.conflict_areas)
            // Perception fragments can lie just outside the map lane polygon.
            // This two-metre observation guard affects confidence only; it
            // never expands the drivable road or removes collision checking.
            if(Overlap(footprint,area)||geometry::ConvexPolygonDistance(footprint,area)<=2)
                return object.id;
    }
    return -1;
}
