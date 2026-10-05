#pragma once
#include "frenet_planner/Common/ObjectTypes.hpp"
#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <unordered_map>
#include <vector>

// Roundabout-only history. OBB orientation describes a visible fragment, not
// the vehicle's direction of travel. Keep at most eight fresh sensor samples
// and fit unwrapped velocity direction against their actual timestamps.
class RoundaboutPrediction {
    struct Sample { double t, direction, speed, length, width; };
    struct Track {
        std::deque<Sample> samples;
        double stamp=-1, rate=0, x=0, y=0, vx=0, vy=0, speed=0, last_motion=-1;
        bool moving=false, reliable=false, complete=false;
    };
    std::unordered_map<int,Track> tracks_;
    double stamp_=-1;
public:
    void Reset() { tracks_.clear(); stamp_=-1; }
    void Observe(const std::vector<TrackedObject>& objects,double stamp,double clamp) {
        if(!std::isfinite(stamp))return;
        if(stamp<stamp_-1e-6)Reset();
        if(stamp<=stamp_+1e-6)return;
        stamp_=stamp;
        for(const auto& object:objects) {
            auto& track=tracks_[object.id];
            if(track.stamp>=0&&stamp-track.stamp>.5)track=Track{};
            track.stamp=stamp;
            const double speed=std::hypot(object.vx,object.vy);
            track.x=object.x;track.y=object.y;track.speed=speed;track.vx=object.vx;track.vy=object.vy;
            track.moving=speed>=.5||object.motion_state==ObjectMotionState::DYNAMIC||
                object.motion_state==ObjectMotionState::DYNAMIC_PENDING;
            if(track.moving)track.last_motion=stamp;
            else track.moving=stamp-track.last_motion<=1;
            track.reliable=track.complete=false;
            if(!std::isfinite(speed)||speed<.5) {
                track.samples.clear();track.rate=0;continue;
            }
            double direction=std::atan2(object.vy,object.vx);
            if(!track.samples.empty())direction=track.samples.back().direction+
                std::remainder(direction-track.samples.back().direction,2*M_PI);
            track.samples.push_back({stamp,direction,speed,object.length,object.width});
            while(track.samples.size()>8||stamp-track.samples.front().t>.8)
                track.samples.pop_front();
            double mt=0,ma=0;
            for(const auto& sample:track.samples){mt+=sample.t-stamp;ma+=sample.direction;}
            mt/=track.samples.size();ma/=track.samples.size();
            double numerator=0,denominator=0;
            for(const auto& sample:track.samples){
                const double dt=sample.t-stamp-mt;
                numerator+=dt*(sample.direction-ma);denominator+=dt*dt;
            }
            const double fit=denominator>1e-9?numerator/denominator:0;
            track.rate=std::clamp(fit,-clamp,clamp);
            double residual=0,min_speed=speed,max_speed=speed;
            double min_length=object.length,max_length=object.length;
            double min_width=object.width,max_width=object.width;
            for(const auto& sample:track.samples){
                residual=std::max(residual,std::fabs(sample.direction-ma-fit*(sample.t-stamp-mt)));
                min_speed=std::min(min_speed,sample.speed);max_speed=std::max(max_speed,sample.speed);
                min_length=std::min(min_length,sample.length);max_length=std::max(max_length,sample.length);
                min_width=std::min(min_width,sample.width);max_width=std::max(max_width,sample.width);
            }
            track.reliable=track.samples.size()>=4&&stamp-track.samples.front().t>=.25&&
                residual<=.20&&std::fabs(fit)<=clamp&&max_speed-min_speed<=2;
            // This is a confidence gate, never a replacement for the perception
            // corners. A partial vehicle must not authorize early departure.
            track.complete=track.reliable&&object.geometry_valid&&min_length>=3&&min_width>=1.2&&
                max_length-min_length<=1&&max_width-min_width<=.6;
        }
        for(auto it=tracks_.begin();it!=tracks_.end();)
            if(stamp-it->second.stamp>1)it=tracks_.erase(it);else ++it;
    }
    double Rate(int id) const {
        const auto it=tracks_.find(id);return it==tracks_.end()?0:it->second.rate;
    }
    // Check vehicles within roughly two seconds of the conflict area. A
    // recently seen moving vehicle which vanishes is retained for one second.
    bool MovingAwayReliable(int id,double x,double y) const {
        auto it=tracks_.find(id);if(it==tracks_.end())return false;
        const auto& t=it->second;
        return t.reliable&&stamp_-t.stamp<=1e-6&&t.speed>=.5&&
            (t.x-x)*t.vx+(t.y-y)*t.vy>.7*t.speed*std::hypot(t.x-x,t.y-y);
    }
    int UncertainNear(double x,double y,
        const std::function<bool(int)>& passed = {}) const {
        for(const auto& item:tracks_){
            const auto& track=item.second;
            if(!track.moving||(track.complete&&stamp_-track.stamp<=1e-6))continue;
            const double radius=12+2*track.speed;
            if(std::hypot(track.x-x,track.y-y)<=radius&&(!passed||!passed(item.first)))return item.first;
        }
        return -1;
    }
};
