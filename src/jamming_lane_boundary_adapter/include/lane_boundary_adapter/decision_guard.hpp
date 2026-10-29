#pragma once
#include <cmath>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace lane_boundary_adapter::decision {
// Time may advance between control cycles, but evidence counts only distinct
// valid sensor observations. No geometry or object lifetime is stored here.
struct ClearEvidence {
    double since=-1;
    std::uint64_t sequence=0;
    unsigned frames=0;
    std::string source;
    std::string reset_reason="initial";
    unsigned resets=0;

    void reset(const char* reason="reset"){since=-1;frames=0;reset_reason=reason;++resets;}
    void observe(double now,std::uint64_t scan,const std::string& producer,bool valid,bool clear){
        if(!valid||!clear){reset(valid?"occupied":"invalid_input");return;}
        if(source!=producer||now<since){reset(source!=producer?"source_changed":"clock_rewind");source=producer;}
        if(since<0){since=now;frames=1;sequence=scan;return;}
        if(scan!=sequence){++frames;sequence=scan;}
    }
    bool ready(double now,double duration,unsigned minimum=3)const{
        return since>=0&&now>=since&&now-since>=duration&&frames>=minimum;
    }
};

struct StopHold {
    bool active=false;
    double rest_since=-1;
    double confirmation_at=-1;
    std::string plan;
    ClearEvidence clear;
    std::vector<std::pair<double,double>> path_shape;

    void engage(){active=true;confirmation_at=-1;clear.reset("risk_or_unavailable");path_shape.clear();}
    void observeSpeed(double now,double speed,bool valid,double stop_speed){
        if(!active)return;
        if(!valid||!std::isfinite(speed)||std::abs(speed)>stop_speed){rest_since=-1;clear.reset("ego_not_at_rest");return;}
        if(rest_since<0||now<rest_since)rest_since=now;
    }
    bool resting(double now,double duration)const{
        return active&&rest_since>=0&&now>=rest_since&&now-rest_since>=duration;
    }
    // Compare against the first confirming shape, not the previous frame: slow
    // geometric drift must not inherit another path's accumulated evidence.
    void observePath(const std::vector<std::pair<double,double>>& path,double tolerance=.25){
        bool changed=path.size()!=path_shape.size();
        for(std::size_t i=0;!changed&&i<path.size();++i)
            changed=!std::isfinite(path[i].first)||!std::isfinite(path[i].second)||
                std::hypot(path[i].first-path_shape[i].first,path[i].second-path_shape[i].second)>tolerance;
        if(changed){clear.reset("path_geometry_changed");path_shape=path;}
    }
    bool release(double now,bool valid,bool safe,const std::string& path,
                 std::uint64_t scan,const std::string& source,double rest_time,double clear_time){
        if(!active)return true;
        // A forward clock step or a missed control interval is not continuous
        // clear evidence. In the live bag a 1.5 s ROS time step advanced a
        // two-scan confirmation to release after only ~0.15 s of real time.
        if(confirmation_at>=0&&(now<confirmation_at||now-confirmation_at>.500001)){
            clear.reset("confirmation_time_gap");rest_since=now;
        }
        confirmation_at=now;
        if(plan!=path){plan=path;clear.reset("path_changed");}
        clear.observe(now,scan,source,valid&&resting(now,rest_time),safe);
        if(!clear.ready(now,clear_time))return false;
        active=false;rest_since=confirmation_at=-1;clear.reset("released");path_shape.clear();return true;
    }
};

inline bool oppositeSide(double current,double candidate,double home){
    return (current-home)*(candidate-home)<-1e-6;
}
inline bool canSwitchSide(double current,double candidate,double home,double now,double changed,double hold){
    return !oppositeSide(current,candidate,home)||changed<0||(now>=changed&&now-changed>=hold);
}
inline double lineCost(double candidate,double current,double home){
    return 2.*std::abs(candidate-current)+.15*std::abs(candidate-home)+
           (oppositeSide(current,candidate,home)?3.:0.);
}
} // namespace lane_boundary_adapter::decision
