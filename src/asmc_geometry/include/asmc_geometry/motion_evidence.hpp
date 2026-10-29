#pragma once
#include <algorithm>
#include <cmath>
#include <map>
#include <string>
#include <utility>
#include <vector>
namespace asmc_geometry::motion_evidence {
struct Point {double x=0,y=0;};
struct Box {double x0=0,x1=0,y0=0,y1=0;};
struct Motion {bool valid=false;Point velocity;bool stationary=false;bool extrapolated=false;double age=0.;};
class MotionHistory {
    struct Sample {
        double stamp=0,speed=0,yaw_rate=0,stationary_sec=0;
        Point center,velocity,stationary_anchor;
        std::vector<Point> stationary_shape;
        std::vector<std::pair<double,Point>> window;
        int consistent=0;bool ego_valid=false;
    };
    std::map<std::pair<std::string,int>,Sample> samples_;
 public:
    void clear(){samples_.clear();}
    Motion predict(const std::string& source,int id,double stamp,double yaw_rate,bool ego_valid)const{
        const auto found=samples_.find({source,id});
        if(found==samples_.end()||!ego_valid)return {};
        const auto& old=found->second;const double dt=stamp-old.stamp;
        if(!old.ego_valid||old.consistent<2||dt<=0.||dt>.5)return {};
        const double angle=(old.yaw_rate+yaw_rate)*dt/2.,c=std::cos(angle),s=std::sin(angle);
        const bool stationary=old.stationary_sec>=1.;
        return {true,stationary?Point{}:Point{c*old.velocity.x+s*old.velocity.y,
            -s*old.velocity.x+c*old.velocity.y},stationary,true,dt};
    }
    Motion update(const std::string& source,int id,double stamp,Point center,
                  double ego_speed,double yaw_rate,bool ego_valid,const Point* measured_velocity=nullptr,
                  const std::vector<Point>* shape=nullptr){
        auto& old=samples_[{source,id}];Motion motion;
        const double dt=stamp-old.stamp;
        // Re-evaluating one sensor packet is not another observation.
        if(old.stamp>0&&dt==0)return {old.consistent>=2,old.stationary_sec>=1.?Point{}:old.velocity,old.stationary_sec>=1.};
        Sample next;next.stamp=stamp;next.center=center;next.speed=ego_speed;next.yaw_rate=yaw_rate;next.ego_valid=ego_valid;
        if(ego_valid&&old.ego_valid&&old.stamp>0&&dt>=.03&&dt<=.5){
            const double angle=(old.yaw_rate+yaw_rate)*dt/2.,distance=(old.speed+ego_speed)*dt/2.;
            const double x=old.center.x-distance*std::cos(angle/2.),y=old.center.y-distance*std::sin(angle/2.);
            const double c=std::cos(angle),s=std::sin(angle);
            Point v{(center.x-(c*x+s*y))/dt,(center.y-(-s*x+c*y))/dt};
            if(measured_velocity)v=*measured_velocity;
            const auto transform=[&](Point p){
                p.x-=distance*std::cos(angle/2.);p.y-=distance*std::sin(angle/2.);
                return Point{c*p.x+s*p.y,-s*p.x+c*p.y};
            };
            for(const auto& sample:old.window)
                if(stamp-sample.first<=.4)next.window.push_back({sample.first,transform(sample.second)});
            next.window.push_back({stamp,center});
            // Low-speed OBB jitter must not become a new moving object every
            // time ego starts. Fit source-time displacement over a short window;
            // retain the immediate estimate for fast motion/abrupt starts.
            if(!measured_velocity&&std::hypot(v.x,v.y)<3.&&std::hypot(old.velocity.x,old.velocity.y)<3.1&&
               next.window.size()>=3&&stamp-next.window.front().first>=.14){
                double mean_t=0.,mean_x=0.,mean_y=0.;
                for(const auto& p:next.window){mean_t+=p.first-stamp;mean_x+=p.second.x;mean_y+=p.second.y;}
                const double n=next.window.size();mean_t/=n;mean_x/=n;mean_y/=n;
                double denominator=0.,vx=0.,vy=0.;
                for(const auto& p:next.window){
                    const double t=p.first-stamp-mean_t;
                    denominator+=t*t;vx+=t*(p.second.x-mean_x);vy+=t*(p.second.y-mean_y);
                }
                if(denominator>1e-8)v={vx/denominator,vy/denominator};
            }
            Point previous{c*old.velocity.x+s*old.velocity.y,-s*old.velocity.x+c*old.velocity.y};
            if(std::isfinite(v.x)&&std::isfinite(v.y)&&std::hypot(v.x,v.y)<25.){
                next.consistent=old.consistent&&std::hypot(v.x-previous.x,v.y-previous.y)>2.5?1:old.consistent+1;
                next.velocity=next.consistent>1?Point{.5*(v.x+previous.x),.5*(v.y+previous.y)}:v;
                bool retained=old.stationary_sec>=1.&&
                    std::hypot(next.velocity.x,next.velocity.y)<.35&&
                    std::hypot(center.x-transform(old.stationary_anchor).x,
                               center.y-transform(old.stationary_anchor).y)<.15;
                for(auto p:old.stationary_shape)next.stationary_shape.push_back(transform(p));
                if(old.stationary_sec>=1.&&shape&&!shape->empty()&&!next.stationary_shape.empty()){
                    const auto bounds=[](const std::vector<Point>& points){
                        Box b{points[0].x,points[0].x,points[0].y,points[0].y};
                        for(auto p:points){b.x0=std::min(b.x0,p.x);b.x1=std::max(b.x1,p.x);
                            b.y0=std::min(b.y0,p.y);b.y1=std::max(b.y1,p.y);}
                        return b;
                    };
                    const auto a=bounds(next.stationary_shape),b=bounds(*shape);
                    const auto common=[](double low,double high){
                        return low*high>0.?std::min(std::abs(low),std::abs(high)):0.;
                    };
                    const double dx=common(b.x0-a.x0,b.x1-a.x1),dy=common(b.y0-a.y0,b.y1-a.y1);
                    const auto comparable=[](double a,double b){
                        return std::max(a,b)<=2.*std::max(.1,std::min(a,b));
                    };
                    // Compare both edges with the frozen, ego-corrected shape.
                    // A partly occluded box can shift its centre while one face
                    // stays fixed. Actual translation moves both faces together.
                    // This is a cumulative reference, never a per-frame minimum
                    // that would systematically subtract shape noise from speed.
                    retained=std::hypot(next.velocity.x,next.velocity.y)<3.&&std::hypot(dx,dy)<.15&&
                        comparable(a.x1-a.x0,b.x1-b.x0)&&comparable(a.y1-a.y0,b.y1-b.y0);
                }
                if(retained||(std::hypot(v.x,v.y)<.2&&
                              std::hypot(next.velocity.x,next.velocity.y)<.2))
                    next.stationary_sec=old.stationary_sec+dt;
                next.stationary_anchor=next.stationary_sec>0&&old.stationary_sec>0?
                    transform(old.stationary_anchor):center;
                if(next.stationary_sec>=1.&&old.stationary_sec<1.&&shape)
                    next.stationary_shape=*shape;
                if(next.stationary_sec==0.)next.stationary_shape.clear();
                motion={next.consistent>=2,next.stationary_sec>=1.?Point{}:next.velocity,next.stationary_sec>=1.};
            }
        }
        if(measured_velocity&&std::isfinite(measured_velocity->x)&&std::isfinite(measured_velocity->y)){
            // Producer-certified velocity already has its own warmup/filter.
            // A missing consumer history or a clock epoch change must not turn
            // that current measurement into unknown motion for several scans.
            next.velocity=*measured_velocity;next.consistent=std::max(2,next.consistent);
            if(std::hypot(measured_velocity->x,measured_velocity->y)>=.2){
                next.stationary_sec=0.;next.stationary_shape.clear();
            }
            motion={true,*measured_velocity,next.stationary_sec>=1.};
        }
        if(next.window.empty())next.window.push_back({stamp,center});
        old=next;
        for(auto it=samples_.begin();it!=samples_.end();)
            if(stamp-it->second.stamp>3.)it=samples_.erase(it);else ++it;
        return motion;
    }
};
}
