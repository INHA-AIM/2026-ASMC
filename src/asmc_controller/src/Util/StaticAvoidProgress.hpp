#pragma once

#include <asmc_msgs/Trajectory.h>
#include <algorithm>
#include <cmath>

// The planner owns geometry validation. The consumer independently checks the
// declared policy, explicit simulator opt-in, and the bounded speed profile.
inline bool StaticAvoidProgressProfileAllowed(const asmc_msgs::Trajectory& path,
    bool allow_contact,double margin_cap,double contact_cap) {
    if(path.execution_policy==asmc_msgs::Trajectory::POLICY_NORMAL)return true;
    if(path.execution_policy!=asmc_msgs::Trajectory::POLICY_STATIC_MARGIN_RECOVERY&&
       path.execution_policy!=asmc_msgs::Trajectory::POLICY_STATIC_CONTACT_PROGRESS)return false;
    if(path.execution_policy==asmc_msgs::Trajectory::POLICY_STATIC_CONTACT_PROGRESS&&!allow_contact)return false;
    const double cap=path.execution_policy==asmc_msgs::Trajectory::POLICY_STATIC_CONTACT_PROGRESS?contact_cap:margin_cap;
    if(!path.valid||path.failure_reason!=asmc_msgs::Trajectory::FAILURE_NONE||path.points.size()<2||
       !std::isfinite(cap)||cap<=0||!std::isfinite(path.dt)||path.dt<=0||
       !std::isfinite(path.points.front().acceleration)||path.points.back().velocity>.02)return false;
    const double initial_speed=path.points.front().velocity;
    const double initial_acceleration=std::max(0.0,path.points.front().acceleration);
    const double ramp_time=initial_acceleration/6;
    const double ramp_overshoot=initial_acceleration*initial_acceleration/12;
    const double release_deadline=1.5+std::max(0.0,initial_speed-cap)+ramp_time+ramp_overshoot;
    double ceiling=cap;
    bool settled=false;
    for(std::size_t i=0;i<path.points.size();++i) {
        const double speed=path.points[i].velocity;
        if(!std::isfinite(speed)||speed<0)return false;
        // A moving entry must decelerate from the observed speed, including
        // unavoidable jerk-ramp transients, rather than teleporting to crawl.
        if(!i)ceiling=std::max(cap,speed+ramp_overshoot+.1);
        if(speed>ceiling+.01)return false;
        if(i*path.dt>=ramp_time&&speed<=cap+.05)settled=true;
        if(!settled&&i*path.dt>release_deadline)return false;
        if(settled&&speed>cap+.1)return false;
    }
    return settled;
}
