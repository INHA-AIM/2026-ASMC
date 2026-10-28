#pragma once
#include <algorithm>
#include <cmath>

// Confirmation evidence is not permission to execute. Every executing path
// still needs a complete certificate for the current ego/object snapshot.
struct HighwayReadiness {
    int confirmations=0;
    double last_safe_time=-1,last_safe_observation=-1,issued_time=-1;

    void Reset() { *this={}; }
    void Expire(double now,double maximum_gap) {
        if(!std::isfinite(now)||last_safe_time<0||now<last_safe_time||
           now-last_safe_time>maximum_gap)Reset();
    }
    bool ObserveSafe(double now,double observation,int required,double maximum_gap) {
        Expire(now,maximum_gap);
        if(last_safe_observation>=0&&observation<last_safe_observation)Reset();
        if(observation>last_safe_observation)confirmations=std::min(confirmations+1,std::max(1,required));
        last_safe_time=now;last_safe_observation=observation;
        return confirmations>=required;
    }
    bool WasIssued(double now,double maximum_gap) const {
        return confirmations>0&&issued_time>=0&&last_safe_time>=0&&now>=last_safe_time&&
            now>=issued_time&&now-last_safe_time<=maximum_gap&&now-issued_time<=maximum_gap;
    }
    void MarkIssued(double now) { issued_time=now; }
};
