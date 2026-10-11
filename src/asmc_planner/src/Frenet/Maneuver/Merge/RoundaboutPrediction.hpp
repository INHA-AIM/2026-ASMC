#pragma once
#include "Frenet/Common/ObjectTypes.hpp"
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
    struct BodySample { double t, half_length, half_width; };
    struct Track {
        std::deque<Sample> samples;
        double stamp=-1, rate=0, x=0, y=0, vx=0, vy=0, speed=0, last_motion=-1;
        bool moving=false, reliable=false, complete=false;
        std::deque<BodySample> bodies;
        double body_stamp=-1, body_x=0, body_y=0;
        double half_length=0, half_width=0;
    };
    std::unordered_map<int,Track> tracks_;
    double stamp_=-1;
    static bool GeometryUsable(const TrackedObject& object) {
        if(!object.geometry_valid)return false;
        double min_x=INFINITY,max_x=-INFINITY,min_y=INFINITY,max_y=-INFINITY;
        for(const auto& p:object.corners) {
            if(!std::isfinite(p.x)||!std::isfinite(p.y))return false;
            min_x=std::min(min_x,p.x);max_x=std::max(max_x,p.x);
            min_y=std::min(min_y,p.y);max_y=std::max(max_y,p.y);
        }
        return (max_x-min_x)*(max_y-min_y)>1e-6;
    }
    static bool BodyContainsObservation(const Track& track,const TrackedObject& object,double stamp) {
        if(track.body_stamp<0||stamp-track.body_stamp>1.0||!GeometryUsable(object))return false;
        const double angle=std::atan2(track.vy,track.vx),c=std::cos(angle),s=std::sin(angle);
        const double dx=object.x-track.body_x,dy=object.y-track.body_y;
        const double growth=.25+.5*(stamp-track.body_stamp);
        return std::fabs(dx*c+dy*s)<=track.half_length+growth&&
            std::fabs(-dx*s+dy*c)<=track.half_width+growth;
    }
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
            // Advect the last full-body centre with fresh measured velocity.
            // A fragment's centre can move across the body as visibility changes.
            if(track.body_stamp>=0&&track.stamp>=0) {
                const double dt=stamp-track.stamp;
                track.body_x+=.5*(track.vx+object.vx)*dt;
                track.body_y+=.5*(track.vy+object.vy)*dt;
            }
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
            for(const auto& sample:track.samples){
                residual=std::max(residual,std::fabs(sample.direction-ma-fit*(sample.t-stamp-mt)));
                min_speed=std::min(min_speed,sample.speed);max_speed=std::max(max_speed,sample.speed);
            }
            track.reliable=track.samples.size()>=4&&stamp-track.samples.front().t>=.25&&
                residual<=.20&&std::fabs(fit)<=clamp&&max_speed-min_speed<=2;
            // Fresh whole-body observations establish a conservative size.
            // Width variation is normal for LiDAR; it enlarges occupancy rather
            // than erasing otherwise reliable motion. History is bounded by
            // eight body samples and one second, independently of motion fit.
            while(!track.bodies.empty()&&stamp-track.bodies.front().t>1.0)
                track.bodies.pop_front();
            if(GeometryUsable(object)&&std::isfinite(object.length)&&std::isfinite(object.width)&&
               object.length>=3&&object.width>=1.2) {
                const double angle=std::atan2(object.vy,object.vx),c=std::cos(angle),s=std::sin(angle);
                double half_length=object.length/2,half_width=object.width/2;
                for(const auto& p:object.corners) {
                    const double dx=p.x-object.x,dy=p.y-object.y;
                    half_length=std::max(half_length,std::fabs(dx*c+dy*s));
                    half_width=std::max(half_width,std::fabs(-dx*s+dy*c));
                }
                track.bodies.push_back({stamp,half_length,half_width});
                if(track.bodies.size()>8)track.bodies.pop_front();
                track.half_length=track.half_width=0;
                for(const auto& body:track.bodies) {
                    track.half_length=std::max(track.half_length,body.half_length);
                    track.half_width=std::max(track.half_width,body.half_width);
                }
                track.body_stamp=stamp;track.body_x=object.x;track.body_y=object.y;
            }
            track.complete=track.reliable&&BodyContainsObservation(track,object,stamp);
        }
        for(auto it=tracks_.begin();it!=tracks_.end();)
            if(stamp-it->second.stamp>1)it=tracks_.erase(it);else ++it;
    }
    double Rate(int id) const {
        const auto it=tracks_.find(id);return it==tracks_.end()?0:it->second.rate;
    }
    // Additional planning occupancy, never a replacement for authoritative
    // perception corners. Use it in every precommit collision/time-gap model
    // whenever retained geometry is allowed to satisfy the confidence gate.
    std::vector<TrackedObject> ConservativeOccupancies(const std::vector<TrackedObject>& objects) const {
        std::vector<TrackedObject> result;
        for(const auto& object:objects) {
            const auto it=tracks_.find(object.id);
            if(it==tracks_.end())continue;
            const auto& track=it->second;
            if(!track.complete||stamp_-track.stamp>1e-6||
               !BodyContainsObservation(track,object,stamp_))continue;
            TrackedObject bound=object;
            bound.x=track.body_x;bound.y=track.body_y;
            bound.heading=std::atan2(track.vy,track.vx);bound.yaw_rate=track.rate;
            const double growth=.25+.5*(stamp_-track.body_stamp);
            const double hl=track.half_length+growth,hw=track.half_width+growth;
            bound.length=2*hl;bound.width=2*hw;bound.geometry_valid=true;bound.z_valid=false;
            const double c=std::cos(bound.heading),s=std::sin(bound.heading);
            for(int i=0;i<8;++i) {
                const double dx=(i%4==0||i%4==3)?-hl:hl;
                const double dy=i%4<2?-hw:hw;
                bound.corners[i]={bound.x+dx*c-dy*s,bound.y+dx*s+dy*c,object.z};
            }
            result.push_back(bound);
        }
        return result;
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
