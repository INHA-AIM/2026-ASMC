#pragma once
// Class-independent short-horizon risk check; no ROS or simulator dependencies.
#include <algorithm>
#include <cmath>
#include <map>
#include <string>
#include <vector>

namespace lane_boundary_adapter::dynamic_risk {
struct Point {double x=0,y=0;};
struct Box {double x0=0,x1=0,y0=0,y1=0;};
struct Motion {bool valid=false;Point velocity;};
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
};
struct Risk {bool clear=true;int id=-1;double ttc=0;};
class MotionHistory {
    struct Sample {double stamp=0,speed=0,yaw_rate=0;Point center,velocity;int consistent=0;bool ego_valid=false;};
    std::map<std::pair<std::string,int>,Sample> samples_;
 public:
    void clear(){samples_.clear();}
    Motion update(const std::string& source,int id,double stamp,Point center,
                  double ego_speed,double yaw_rate,bool ego_valid){
        auto& old=samples_[{source,id}];Motion motion;
        const double dt=stamp-old.stamp;
        Sample next;next.stamp=stamp;next.center=center;next.speed=ego_speed;next.yaw_rate=yaw_rate;next.ego_valid=ego_valid;
        if(ego_valid&&old.ego_valid&&old.stamp>0&&dt>=.03&&dt<=.5){
            const double angle=(old.yaw_rate+yaw_rate)*dt/2.,distance=(old.speed+ego_speed)*dt/2.;
            const double x=old.center.x-distance*std::cos(angle/2.),y=old.center.y-distance*std::sin(angle/2.);
            const double c=std::cos(angle),s=std::sin(angle);
            Point v{(center.x-(c*x+s*y))/dt,(center.y-(-s*x+c*y))/dt};
            Point previous{c*old.velocity.x+s*old.velocity.y,-s*old.velocity.x+c*old.velocity.y};
            if(std::isfinite(v.x)&&std::isfinite(v.y)&&std::hypot(v.x,v.y)<25.){
                next.consistent=old.consistent&&std::hypot(v.x-previous.x,v.y-previous.y)>2.5?1:old.consistent+1;
                next.velocity=next.consistent>1?Point{.5*(v.x+previous.x),.5*(v.y+previous.y)}:v;
                motion={next.consistent>=2,next.velocity};
            }
        }
        old=next;
        for(auto it=samples_.begin();it!=samples_.end();)
            if(stamp-it->second.stamp>3.)it=samples_.erase(it);else ++it;
        return motion;
    }
};
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
       !std::isfinite(cfg.deceleration)||cfg.deceleration<=0)return {false,-1,0};
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
    auto body=[&](double distance){
        auto upper=std::upper_bound(lengths.begin(),lengths.end(),distance);
        std::size_t i=std::min<std::size_t>(path.size()-1,std::max<std::size_t>(1,upper-lengths.begin()));
        auto a=path[i-1],b=path[i];double w=std::clamp((distance-lengths[i-1])/std::max(1e-6,lengths[i]-lengths[i-1]),0.,1.);
        Point center{a.x+w*(b.x-a.x),a.y+w*(b.y-a.y)};double yaw=std::atan2(b.y-a.y,b.x-a.x),c=std::cos(yaw),s=std::sin(yaw);
        std::vector<Point> out;
        for(double x:{-cfg.rear,cfg.front})for(double y:{-cfg.width/2-cfg.margin,cfg.width/2+cfg.margin})
            out.push_back({center.x+c*x-s*y,center.y+s*x+c*y});
        return out;
    };
    double distance=0,previous_time=0;auto previous_body=body(0);
    const double horizon=std::max(cfg.horizon,speed/cfg.deceleration+cfg.reaction);
    for(double time=0;time<=horizon+1e-6;time+=.05){
        if(distance>lengths.back())break; // this check covers only the observed path
        auto current_body=body(distance),swept=current_body;
        swept.insert(swept.end(),previous_body.begin(),previous_body.end());swept=hull(swept);
        for(const auto& o:objects){
            std::vector<Point> occupied;
            const double uncertainty=o.uncertainty+(o.motion.valid?.1*time:.5*time);
            for(double t:{previous_time,time}){
                double dx=o.motion.valid?o.motion.velocity.x*t:0.,dy=o.motion.valid?o.motion.velocity.y*t:0.;
                const auto corners=o.footprint.empty()?rectangle(o.box):o.footprint;
                for(auto p:corners)for(double ex:{-uncertainty,uncertainty})for(double ey:{-uncertainty,uncertainty})
                    occupied.push_back({p.x+dx+ex,p.y+dy+ey});
            }
            if(overlaps(swept,hull(occupied)))return {false,o.id,time};
        }
        previous_body=std::move(current_body);previous_time=time;
        const double requested=time<cfg.reaction?speed:target;
        speed=std::max(0.,speed+std::clamp(requested-speed,-cfg.deceleration*.05,.8*.05));
        distance+=speed*.05;
    }return {};
}
} // namespace lane_boundary_adapter::dynamic_risk
