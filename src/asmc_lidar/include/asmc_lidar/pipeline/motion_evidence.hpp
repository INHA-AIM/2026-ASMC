#pragma once
#include <cmath>
#include <deque>
namespace asmc { namespace lidar { namespace pipeline { namespace motion_display { namespace motion_evidence {
struct Sample {
    double time,min_x,max_x,min_y,max_y,vx,vy;
};
// A fast tracker estimate alone never establishes motion during warm-up.
// Three independent map OBB observations must translate rigidly in the same
// direction as the reported map velocity. Growing/shrinking boxes fail closed.
inline bool Certify(const std::deque<Sample>& samples,double edge_tolerance,
                    double speed_error,double minimum_speed,double direction_cosine) {
    if(samples.size()<3 || !std::isfinite(edge_tolerance)||edge_tolerance<0 ||
       !std::isfinite(speed_error)||speed_error<0 || !std::isfinite(minimum_speed)||minimum_speed<=0 ||
       !std::isfinite(direction_cosine)||direction_cosine<0||direction_cosine>1)return false;
    const auto first=samples.front(),last=samples.back();
    const double span=last.time-first.time;
    if(span<.1||span>.4)return false;
    double travelled_x=0,travelled_y=0;
    for(std::size_t i=1;i<samples.size();++i) {
        const auto a=samples[i-1],b=samples[i];const double dt=b.time-a.time;
        if(!std::isfinite(dt)||dt<=.01||dt>.3)return false;
        for(double v:{a.min_x,a.max_x,a.min_y,a.max_y,b.min_x,b.max_x,b.min_y,b.max_y,a.vx,a.vy,b.vx,b.vy})
            if(!std::isfinite(v))return false;
        if(a.max_x<=a.min_x||a.max_y<=a.min_y||b.max_x<=b.min_x||b.max_y<=b.min_y)return false;
        const double x0=b.min_x-a.min_x,x1=b.max_x-a.max_x;
        const double y0=b.min_y-a.min_y,y1=b.max_y-a.max_y;
        if(std::fabs(x0-x1)>edge_tolerance||std::fabs(y0-y1)>edge_tolerance)return false;
        const double dx=.5*(x0+x1),dy=.5*(y0+y1);
        const double vx=.5*(a.vx+b.vx),vy=.5*(a.vy+b.vy);
        const double observed=std::hypot(dx,dy)/dt,reported=std::hypot(vx,vy);
        if(observed<minimum_speed||reported<minimum_speed||
           std::fabs(observed-reported)>speed_error||
           (dx*vx+dy*vy)/(std::hypot(dx,dy)*reported)<direction_cosine)return false;
        travelled_x+=dx;travelled_y+=dy;
    }
    return std::hypot(travelled_x,travelled_y)>edge_tolerance;
}
}

}}}}
