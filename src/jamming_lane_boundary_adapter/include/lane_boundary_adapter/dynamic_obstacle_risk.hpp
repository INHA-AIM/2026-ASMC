#pragma once
// Class-independent short-horizon risk check; no ROS or simulator dependencies.
#include <asmc_geometry/motion_evidence.hpp>
#include <algorithm>
#include <cmath>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace lane_boundary_adapter::dynamic_risk {
using asmc_geometry::motion_evidence::Point;
using asmc_geometry::motion_evidence::Box;
using asmc_geometry::motion_evidence::Motion;
struct Object {
    Box box;
    Motion motion;
    int id=-1;
    std::vector<Point> footprint{}; // LiDAR OBB XY hull, not an image-derived box
    double uncertainty=0;
};
struct Config {
    double front=3.845,rear=.79,width=1.892,margin=.3;
    double horizon=2.,reaction=.3,deceleration=2.;
    double stationary_margin=.3;
    // MORAI pedal runs measured ~3.6 m/s^2 over 0.4 s; retain an upper bound.
    double acceleration=4.;
};
struct Risk {bool clear=true;int id=-1;double ttc=0;};
using MotionHistory=asmc_geometry::motion_evidence::MotionHistory;
inline double cross(Point a,Point b,Point c){return (b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x);}
inline std::vector<Point> hull(std::vector<Point> p){
    std::sort(p.begin(),p.end(),[](auto a,auto b){return a.x<b.x||(a.x==b.x&&a.y<b.y);});
    p.erase(std::unique(p.begin(),p.end(),[](auto a,auto b){return a.x==b.x&&a.y==b.y;}),p.end());
    if(p.size()<3)return p;
    std::vector<Point> out;
    for(auto v:p){while(out.size()>1&&cross(out[out.size()-2],out.back(),v)<=0)out.pop_back();out.push_back(v);}
    const auto n=out.size();
    for(auto it=p.rbegin()+1;it!=p.rend();++it){while(out.size()>n&&cross(out[out.size()-2],out.back(),*it)<=0)out.pop_back();out.push_back(*it);}
    out.pop_back();return out;
}
inline std::vector<Point> rectangle(Box b){return {{b.x0,b.y0},{b.x1,b.y0},{b.x1,b.y1},{b.x0,b.y1}};}
// Advance one scan into the current ego frame while retaining its OBB shape.
inline Object alignToEgo(Object object,double age,double speed,double yaw_rate){
    if(object.footprint.empty())object.footprint=rectangle(object.box);
    const double angle=yaw_rate*age,c=std::cos(angle),s=std::sin(angle);
    const double distance=speed*age;
    object.box={1e100,-1e100,1e100,-1e100};
    for(auto& p:object.footprint){
        if(object.motion.valid){p.x+=object.motion.velocity.x*age;p.y+=object.motion.velocity.y*age;}
        const double x=p.x-distance*std::cos(angle/2.),y=p.y-distance*std::sin(angle/2.);
        p={c*x+s*y,-s*x+c*y};
        object.box.x0=std::min(object.box.x0,p.x);object.box.x1=std::max(object.box.x1,p.x);
        object.box.y0=std::min(object.box.y0,p.y);object.box.y1=std::max(object.box.y1,p.y);
    }
    if(object.motion.valid){
        auto v=object.motion.velocity;object.motion.velocity={c*v.x+s*v.y,-s*v.x+c*v.y};
    }
    return object;
}
inline bool overlaps(const std::vector<Point>& a,const std::vector<Point>& b){
    if(a.size()<3||b.size()<3)return true;
    for(const auto* polygon:{&a,&b})for(std::size_t i=0;i<polygon->size();++i){
        auto p=(*polygon)[i],q=(*polygon)[(i+1)%polygon->size()];Point axis{q.y-p.y,p.x-q.x};
        if(std::hypot(axis.x,axis.y)<1e-9)continue;
        double amin=1e100,amax=-1e100,bmin=1e100,bmax=-1e100;
        for(auto v:a){double t=v.x*axis.x+v.y*axis.y;amin=std::min(amin,t);amax=std::max(amax,t);}
        for(auto v:b){double t=v.x*axis.x+v.y*axis.y;bmin=std::min(bmin,t);bmax=std::max(bmax,t);}
        if(amax<bmin||bmax<amin)return false;
    }return true;
}
inline std::vector<Point> vehicleBody(Point center,double yaw,const Config& cfg){
    const double c=std::cos(yaw),s=std::sin(yaw);
    std::vector<Point> out;
    for(auto p:rectangle({-cfg.rear,cfg.front,-cfg.width/2-cfg.margin,cfg.width/2+cfg.margin}))
        out.push_back({center.x+c*p.x-s*p.y,center.y+s*p.x+c*p.y});
    return out;
}
struct PathHit {bool clear=true;int id=-1;Point point;};
// Validate the actual path, including the area swept between its samples.
inline PathHit checkStaticPath(const std::vector<Point>& path,const std::vector<Object>& objects,const Config& cfg){
    if(path.size()<2)return {false,-1,{}};
    std::vector<Point> previous;
    for(std::size_t i=0;i<path.size();++i){
        auto p=path[i],a=path[i?i-1:0],b=path[std::min(i+1,path.size()-1)];
        if(!std::isfinite(p.x)||!std::isfinite(p.y))return {false,-1,p};
        auto body=vehicleBody(p,std::atan2(b.y-a.y,b.x-a.x),cfg),swept=body;
        swept.insert(swept.end(),previous.begin(),previous.end());
        swept=hull(std::move(swept));
        for(const auto& o:objects)
            if(overlaps(swept,o.footprint.empty()?rectangle(o.box):o.footprint))return {false,o.id,p};
        previous=std::move(body);
    }
    return {};
}
// Lateral movement relative to the road, not a class-name trigger. A parallel
// vehicle on a sloped/curved lane must not become a crossing actor just because
// base_link velocity has a nonzero y component.
inline bool lateralMover(const Object& object,const std::vector<Point>& road,double threshold=.35){
    if(!object.motion.valid||road.size()<2)return false;
    const Point center{(object.box.x0+object.box.x1)/2.,(object.box.y0+object.box.y1)/2.};
    double best=1e100,yaw=0;
    for(std::size_t i=1;i<road.size();++i){
        auto a=road[i-1],b=road[i];double dx=b.x-a.x,dy=b.y-a.y,n=dx*dx+dy*dy;
        if(n<1e-8)continue;
        double w=std::clamp(((center.x-a.x)*dx+(center.y-a.y)*dy)/n,0.,1.);
        double distance=std::hypot(center.x-a.x-w*dx,center.y-a.y-w*dy);
        if(distance<best){best=distance;yaw=std::atan2(dy,dx);}
    }
    return std::abs(-std::sin(yaw)*object.motion.velocity.x+std::cos(yaw)*object.motion.velocity.y)>threshold;
}
inline Risk assess(const std::vector<Point>& path,const std::vector<Object>& objects,
                   double speed,double target,const Config& cfg){
    if(objects.empty())return {};
    if(path.size()<2||!std::isfinite(speed)||!std::isfinite(target)||speed<0||target<0||
       !std::isfinite(cfg.horizon)||cfg.horizon<=0||
       !std::isfinite(cfg.deceleration)||cfg.deceleration<=0||
       !std::isfinite(cfg.acceleration)||cfg.acceleration<=0)return {false,-1,0};
    for(auto p:path)if(!std::isfinite(p.x)||!std::isfinite(p.y))return {false,-1,0};
    for(const auto& o:objects){
        const auto b=o.box;
        if(!std::isfinite(b.x0)||!std::isfinite(b.x1)||!std::isfinite(b.y0)||!std::isfinite(b.y1)||
           b.x1<b.x0||b.y1<b.y0||
           (o.motion.valid&&(!std::isfinite(o.motion.velocity.x)||!std::isfinite(o.motion.velocity.y))))
            return {false,o.id,0};
        for(auto p:o.footprint)if(!std::isfinite(p.x)||!std::isfinite(p.y))return {false,o.id,0};
    }
    std::vector<double> lengths(path.size(),0);
    for(std::size_t i=1;i<path.size();++i)lengths[i]=lengths[i-1]+std::hypot(path[i].x-path[i-1].x,path[i].y-path[i-1].y);
    if(lengths.back()<1e-6)return {false,-1,0};
    auto body=[&](double distance,double margin){
        auto upper=std::upper_bound(lengths.begin(),lengths.end(),distance);
        std::size_t i=std::min<std::size_t>(path.size()-1,std::max<std::size_t>(1,upper-lengths.begin()));
        auto a=path[i-1],b=path[i];double w=std::clamp((distance-lengths[i-1])/std::max(1e-6,lengths[i]-lengths[i-1]),0.,1.);
        Point center{a.x+w*(b.x-a.x),a.y+w*(b.y-a.y)};double yaw=std::atan2(b.y-a.y,b.x-a.x),c=std::cos(yaw),s=std::sin(yaw);
        std::vector<Point> out;
        for(double x:{-cfg.rear,cfg.front})for(double y:{-cfg.width/2-margin,cfg.width/2+margin})
            out.push_back({center.x+c*x-s*y,center.y+s*x+c*y});
        return out;
    };
    const double static_margin=std::max(0.,std::min(cfg.margin,cfg.stationary_margin));
    double distance=0,previous_time=0;auto previous_body=body(0,cfg.margin),previous_static=body(0,static_margin);
    const double horizon=std::max(cfg.horizon,speed/cfg.deceleration+cfg.reaction);
    for(double time=0;time<=horizon+1e-6;time+=.05){
        if(distance>lengths.back())break; // this check covers only the observed path
        auto current_body=body(distance,cfg.margin),swept=current_body;
        swept.insert(swept.end(),previous_body.begin(),previous_body.end());swept=hull(swept);
        auto current_static=body(distance,static_margin),static_swept=current_static;
        static_swept.insert(static_swept.end(),previous_static.begin(),previous_static.end());static_swept=hull(static_swept);
        for(const auto& o:objects){
            std::vector<Point> occupied;
            // After continuous stationary evidence, retain the measured shape
            // and normal collision margin instead of growing a static prop or
            // wall into an otherwise clear path. Unknown motion still expands.
            const double growth=o.motion.valid&&o.motion.stationary?0.:
                (o.motion.valid?.1:.5);
            const double uncertainty=o.uncertainty+growth*time;
            for(double t:{previous_time,time}){
                double dx=o.motion.valid?o.motion.velocity.x*t:0.,dy=o.motion.valid?o.motion.velocity.y*t:0.;
                const auto corners=o.footprint.empty()?rectangle(o.box):o.footprint;
                for(auto p:corners)for(double ex:{-uncertainty,uncertainty})for(double ey:{-uncertainty,uncertainty})
                    occupied.push_back({p.x+dx+ex,p.y+dy+ey});
            }
            if(overlaps(o.motion.valid&&o.motion.stationary?static_swept:swept,hull(occupied)))return {false,o.id,time};
        }
        previous_body=std::move(current_body);previous_static=std::move(current_static);previous_time=time;
        // Brake latency must not delay the predicted start of acceleration:
        // doing so made a stopped ego arrive much later than the actual car.
        const double requested=target>=speed||time>=cfg.reaction?target:speed;
        speed=std::max(0.,speed+std::clamp(requested-speed,-cfg.deceleration*.05,cfg.acceleration*.05));
        distance+=speed*.05;
    }return {};
}
} // namespace lane_boundary_adapter::dynamic_risk
