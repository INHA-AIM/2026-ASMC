#pragma once
#include <cmath>
#include <cstdint>
#include <string>

namespace lane_boundary_adapter::decision {
// Time may advance between control cycles, but evidence counts only distinct
// valid sensor observations. No geometry or object lifetime is stored here.
struct ClearEvidence {
    double since=-1;
    std::uint64_t sequence=0;
    unsigned frames=0;
    std::string source;

    void reset(){since=-1;frames=0;}
    void observe(double now,std::uint64_t scan,const std::string& producer,bool valid,bool clear){
        if(!valid||!clear){reset();return;}
        if(source!=producer||now<since){reset();source=producer;}
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
    std::string plan;
    ClearEvidence clear;

    void engage(){active=true;clear.reset();}
    void observeSpeed(double now,double speed,bool valid,double stop_speed){
        if(!active)return;
        if(!valid||!std::isfinite(speed)||std::abs(speed)>stop_speed){rest_since=-1;clear.reset();return;}
        if(rest_since<0||now<rest_since)rest_since=now;
    }
    bool resting(double now,double duration)const{
        return active&&rest_since>=0&&now>=rest_since&&now-rest_since>=duration;
    }
    bool release(double now,bool valid,bool safe,const std::string& path,
                 std::uint64_t scan,const std::string& source,double rest_time,double clear_time){
        if(!active)return true;
        if(plan!=path){plan=path;clear.reset();}
        clear.observe(now,scan,source,valid&&resting(now,rest_time),safe);
        if(!clear.ready(now,clear_time))return false;
        active=false;rest_since=-1;clear.reset();return true;
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
