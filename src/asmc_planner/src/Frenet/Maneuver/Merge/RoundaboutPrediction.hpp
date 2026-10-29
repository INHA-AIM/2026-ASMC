#pragma once
#include "Frenet/Common/ObjectTypes.hpp"
#include "Frenet/Geometry/ObservationContinuity.hpp"
#include "Frenet/Geometry/PredictionInput.hpp"
#include "Frenet/Evaluation/CollisionChecker.hpp"
#include "Frenet/Reference/Reference.hpp"
#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <unordered_map>
#include <utility>
#include <vector>

// Roundabout-only history. OBB orientation describes a visible fragment, not
// the vehicle's direction of travel. Keep at most eight fresh sensor samples
// and fit unwrapped velocity direction against their actual timestamps.
class RoundaboutPrediction {
    struct Sample { double t, direction, speed, length, width, x, y; };
    struct BodySample { double t, half_length, half_width; };
    struct Track {
        std::deque<Sample> samples;
        double stamp=-1, rate=0, x=0, y=0, vx=0, vy=0, speed=0, last_motion=-1;
        bool moving=false, reliable=false, complete=false, motion_bounded=false;
        std::deque<BodySample> bodies;
        TrackedObject last{};
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
    static bool WholeBodyObserved(const TrackedObject& object) {
        if(!GeometryUsable(object)||!std::isfinite(object.length)||
           !std::isfinite(object.width)||object.length<3||object.width<1.2)return false;
        // Departure confidence only: keep low fragments as collision objects,
        // but do not let a long tyre/ground strip certify a complete vehicle.
        // Measured corner height cannot be replaced by nominal metadata.
        double low=INFINITY,high=-INFINITY;
        for(const auto& p:object.corners) {
            if(!std::isfinite(p.z))return false;
            low=std::min(low,p.z);high=std::max(high,p.z);
        }
        return high-low>=.5;
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
    // Include the continuum's interior between straight travel and the
    // fitted turn band. Preserve the original endpoints, with at most thirteen
    // samples per object; never construct
    // combinations across vehicles. Collision checking is object-separable.
    static std::vector<double> TurnModelRates(double rate,double clamp) {
        if(!std::isfinite(rate)||!std::isfinite(clamp)||clamp<=0)return {0};
        rate=std::clamp(rate,-clamp,clamp);
        const double low=std::min(0.0,std::max(-clamp,rate-.15));
        const double high=std::max(0.0,std::min(clamp,rate+.15));
        const int steps=static_cast<int>(std::clamp(std::ceil((high-low)/.15),1.0,8.0));
        std::vector<double> rates{0,rate,std::max(-clamp,rate-.15),
            std::min(clamp,rate+.15)};
        for(int i=0;i<=steps;++i)rates.push_back(low+(high-low)*i/steps);
        std::sort(rates.begin(),rates.end());
        rates.erase(std::unique(rates.begin(),rates.end(),
            [](double a,double b){return std::fabs(a-b)<1e-9;}),rates.end());
        return rates;
    }
    static std::vector<TrackedObject> ExpandTurnModels(
        const std::vector<TrackedObject>& objects,double clamp) {
        std::vector<TrackedObject> expanded;
        for(const auto& object:objects) {
            if(std::hypot(object.vx,object.vy)<.5) {
                expanded.push_back(object);continue;
            }
            for(double rate:TurnModelRates(object.yaw_rate,clamp)) {
                auto alternative=object;alternative.yaw_rate=rate;
                expanded.push_back(std::move(alternative));
            }
        }
        return expanded;
    }
    void Reset() { tracks_.clear(); stamp_=-1; }
    void Observe(const std::vector<TrackedObject>& objects,double stamp,double clamp,
                 double speed_measurement_error=1.0) {
        if(!std::isfinite(stamp))return;
        if(stamp<stamp_-1e-6)Reset();
        if(stamp<=stamp_+1e-6)return;
        stamp_=stamp;
        for(const auto& object:objects) {
            auto& track=tracks_[object.id];
            if(track.stamp>=0&&!ObservationContinuous(track.last,object,stamp-track.stamp))
                track=Track{};
            track.last=object;
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
            track.reliable=track.complete=track.motion_bounded=false;
            if(InspectPredictionVelocity(object)!=PredictionInputStatus::MEASURED||speed<.5) {
                track.samples.clear();track.rate=0;continue;
            }
            double direction=std::atan2(object.vy,object.vx);
            if(!track.samples.empty())direction=track.samples.back().direction+
                std::remainder(direction-track.samples.back().direction,2*M_PI);
            track.samples.push_back({stamp,direction,speed,object.length,object.width,object.x,object.y});
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
                residual<=.20&&std::fabs(fit)<=clamp&&
                std::isfinite(speed_measurement_error)&&speed_measurement_error>0&&
                max_speed-min_speed<=2*speed_measurement_error;
            track.motion_bounded=track.reliable;
            double position_vx=0,position_vy=0;
            if(DisputedVelocity(track,position_vx,position_vy))track.reliable=false;
            // Fresh whole-body observations establish a conservative size.
            // Width variation is normal for LiDAR; it enlarges occupancy rather
            // than erasing otherwise reliable motion. History is bounded by
            // eight body samples and one second, independently of motion fit.
            while(!track.bodies.empty()&&stamp-track.bodies.front().t>1.0)
                track.bodies.pop_front();
            if(WholeBodyObserved(object)) {
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
    // Map constrains a supported circulating actor's FUTURE displacement.
    // Preserve the measured current center/corners and do not follow ego's exit.
    TrackedObject MappedMotion(const TrackedObject& object,
        const std::vector<std::shared_ptr<const RefLine>>& references,double horizon=0,
        PredictionInputStatus* status=nullptr) const {
        if(status)*status=InspectPredictionVelocity(object);
        const auto found=tracks_.find(object.id);
        if(object.velocity_unavailable||object.motion_state==ObjectMotionState::LOW_THREAT||
           (object.type!=0&&object.type!=1)||found==tracks_.end())return object;
        // A lane match alone cannot repair disputed velocity or an unstable
        // fragment. Keep the original prediction until independent motion and
        // a recently observed whole body support this road constraint.
        double disputed_x=0,disputed_y=0;
        if(DisputedVelocity(found->second,disputed_x,disputed_y)) {
            if(status)*status=PredictionInputStatus::VELOCITY_DISAGREEMENT;
            return object;
        }
        if(!found->second.reliable||!found->second.complete)return object;
        double position_vx=0,position_vy=0,position_error=0;
        if(!PositionVelocity(found->second,position_vx,position_vy,position_error))return object;
        if(std::hypot(position_vx-object.vx,position_vy-object.vy)>position_error) {
            if(status)*status=PredictionInputStatus::VELOCITY_DISAGREEMENT;
            return object;
        }
        const auto& samples=found->second.samples;
        const double speed=std::hypot(object.vx,object.vy);
        if(samples.size()<3||samples.back().t-samples.front().t<.2||speed<.5)return object;
        std::shared_ptr<const RefLine> best;double best_s=0,best_d=0,distance=INFINITY;
        for(const auto& line:references) {
            const double station=FindClosestS(*line,object.x,object.y,nullptr,1e6);
            const auto point=Interpolate(*line,station);
            const double dx=object.x-point.x,dy=object.y-point.y;
            const double error=std::hypot(dx,dy);
            const double upper=std::isfinite(object.prediction_max_speed)?
                std::max(speed,object.prediction_max_speed):speed;
            if(!PredictionLaneCovers(station,line->length,upper,horizon)) {
                if(status)*status=PredictionInputStatus::MAP_UNCOVERED;
                continue;
            }
            if(error>2||station>=line->length-.1||
               (object.vx*std::cos(point.theta)+object.vy*std::sin(point.theta))/speed<.7)continue;
            bool stable=true;size_t count=0;
            for(auto it=samples.rbegin();it!=samples.rend()&&count<3;++it,++count) {
                const auto q=Interpolate(*line,FindClosestS(*line,it->x,it->y,nullptr,1e6));
                if(std::hypot(it->x-q.x,it->y-q.y)>2){stable=false;break;}
            }
            if(stable&&error<distance){best=line;best_s=station;best_d=-dx*std::sin(point.theta)+dy*std::cos(point.theta);distance=error;}
        }
        if(!best)return object;
        if(status)*status=PredictionInputStatus::MEASURED;
        auto result=object;const auto at=Interpolate(*best,best_s);
        result.prediction_lane=best;result.prediction_lane_s=best_s;result.prediction_lane_d=best_d;
        result.prediction_lane_anchor_x=at.x-best_d*std::sin(at.theta);
        result.prediction_lane_anchor_y=at.y+best_d*std::cos(at.theta);
        result.prediction_lane_heading=at.theta;result.prediction_lane_end_s=best->length;
        // Caller certifies only spans fully covered by this supplied lane.
        // No unobserved circular successor or ego exit is invented.
        result.yaw_rate=0;return result;
    }
    double Rate(int id) const {
        const auto it=tracks_.find(id);return it==tracks_.end()?0:it->second.rate;
    }
    std::pair<double,double> ObservedSpeedRange(int id,double nominal) const {
        double low=nominal,high=nominal;
        const auto it=tracks_.find(id);
        if(it!=tracks_.end()&&stamp_-it->second.stamp<=1e-6)
            for(const auto& sample:it->second.samples) {
                low=std::min(low,sample.speed);high=std::max(high,sample.speed);
            }
        return {low,high};
    }
    std::vector<TrackedObject> WithObservedSpeedRanges(
        const std::vector<TrackedObject>& objects) const {
        auto bounded=objects;
        for(auto& object:bounded) {
            const double speed=std::hypot(object.vx,object.vy);
            if(object.motion_state==ObjectMotionState::LOW_THREAT||speed<.5)continue;
            auto range=ObservedSpeedRange(object.id,speed);
            const auto it=tracks_.find(object.id);double vx=0,vy=0,error=0;
            // A supported speed disagreement enlarges the departure interval;
            // it must not disappear when the binary confidence veto is waived.
            double disputed_vx=0,disputed_vy=0;
            if(it!=tracks_.end()&&DisputedVelocity(it->second,disputed_vx,disputed_vy)&&
               BoundedDisagreement(it->second,vx,vy,error)) {
                const double position_speed=std::hypot(vx,vy);
                range.first=std::min(range.first,std::max(0.0,position_speed-error));
                range.second=std::max(range.second,position_speed+error);
            }
            if(std::isfinite(object.prediction_min_speed))
                range.first=std::min(range.first,std::max(0.0,object.prediction_min_speed));
            if(std::isfinite(object.prediction_max_speed))
                range.second=std::max(range.second,object.prediction_max_speed);
            object.prediction_min_speed=range.first;object.prediction_max_speed=range.second;
        }
        return bounded;
    }
    // Fit translation independently from reported velocity. A fragment may
    // shift within the body, so require bounded position residuals and actual
    // travel before using this as an additional motion estimate. Geometry is
    // never replaced, and the departure confidence gate is not relaxed.
    bool PositionVelocity(const Track& track,double& vx,double& vy,double& error) const {
        if(track.samples.size()<4||track.samples.back().t-track.samples.front().t<.25)return false;
        double mt=0,mx=0,my=0;
        for(const auto& p:track.samples){mt+=p.t-track.stamp;mx+=p.x;my+=p.y;}
        mt/=track.samples.size();mx/=track.samples.size();my/=track.samples.size();
        double xx=0,xy=0,tt=0;
        for(const auto& p:track.samples){const double t=p.t-track.stamp-mt;
            tt+=t*t;xx+=t*(p.x-mx);xy+=t*(p.y-my);}
        if(tt<1e-9)return false;
        vx=xx/tt;vy=xy/tt;
        if(std::hypot(vx,vy)*(track.samples.back().t-track.samples.front().t)<.5)return false;
        double residual_squared=0;
        for(const auto& p:track.samples) {
            const double t=p.t-track.stamp-mt;
            const double residual=std::hypot(p.x-mx-vx*t,p.y-my-vy*t);
            if(residual>.5)return false;
            residual_squared+=residual*residual;
        }
        // A longer, less noisy history supports a tighter velocity comparison.
        // This is a fit-consistency band, not a probabilistic safety guarantee.
        error=std::max(.5,2*std::sqrt(residual_squared/((track.samples.size()-2)*tt)));
        return std::isfinite(vx)&&std::isfinite(vy)&&std::isfinite(error);
    }
    bool DisputedVelocity(const Track& track,double& vx,double& vy) const {
        double error=0;
        return PositionVelocity(track,vx,vy,error)&&std::hypot(vx,vy)>track.speed+error&&
            std::hypot(vx-track.vx,vy-track.vy)>error;
    }
    bool BoundedDisagreement(const Track& track,double& vx,double& vy,double& error) const {
        if(!track.motion_bounded||!PositionVelocity(track,vx,vy,error))return false;
        const double position_speed=std::hypot(vx,vy);
        // Both estimates must describe the same direction. A position fit
        // contradicting a spurious tight turn still has no departure authority.
        return position_speed>.5&&track.speed>=.5&&
            (vx*track.vx+vy*track.vy)/(position_speed*track.speed)>=std::cos(.20);
    }
    // Extra occupancies, never replacements for measured corners. A caller may
    // waive this ID's confidence veto ONLY if these bounds also participate in
    // the complete candidate's final collision and temporal-gap checks.
    std::vector<TrackedObject> DepartureOccupancies(
        const std::vector<TrackedObject>& objects,
        const std::function<bool(double,double)>& on_circulating_lane,
        double partial_length,double partial_width) const {
        auto result=ConservativeOccupancies(objects);
        if(!std::isfinite(partial_length)||partial_length<=0||
           !std::isfinite(partial_width)||partial_width<=0)return result;
        for(const auto& object:objects) {
            const auto it=tracks_.find(object.id);if(it==tracks_.end())continue;
            const auto& track=it->second;
            if(track.complete||object.motion_state==ObjectMotionState::LOW_THREAT||
               stamp_-track.stamp>1e-6||!track.moving||
               !track.motion_bounded||!GeometryUsable(object))continue;
            double vx=0,vy=0,error=0;
            if(!BoundedDisagreement(track,vx,vy,error))continue;
            const bool remembered=BodyContainsObservation(track,object,stamp_);
            // Partial-only support requires actual displacement and mapped
            // circulating-lane observations. A stationary roadside false
            // velocity, an unsupported route or a new ID cannot qualify.
            bool mapped=false;
            if(on_circulating_lane)for(const auto& sample:track.samples)
                mapped|=on_circulating_lane(sample.x,sample.y);
            if(!remembered&&!mapped)continue;
            auto bound=object;
            bound.heading=std::atan2(track.vy,track.vx);bound.yaw_rate=track.rate;
            double hl,hw;
            if(remembered) {
                bound.x=track.body_x;bound.y=track.body_y;
                const double age=stamp_-track.body_stamp;
                const double growth=.25+(.5+std::hypot(vx-track.vx,vy-track.vy)+error)*age;
                hl=track.half_length+growth;hw=track.half_width+growth;
            } else {
                // The fragment is NOT a whole body and its centre is NOT a
                // vehicle centre. If a body touches the measured fragment,
                // its centre can be displaced by half a body in either
                // direction. Full dimensions as radii bound that uncertainty.
                hl=partial_length+.25;hw=partial_width+.25;
            }
            const double c=std::cos(bound.heading),s=std::sin(bound.heading);
            for(const auto& p:object.corners) {
                const double dx=p.x-bound.x,dy=p.y-bound.y;
                hl=std::max(hl,std::fabs(dx*c+dy*s)+.25);
                hw=std::max(hw,std::fabs(-dx*s+dy*c)+.25);
            }
            bound.length=2*hl;bound.width=2*hw;bound.geometry_valid=true;bound.z_valid=false;
            const auto speeds=ObservedSpeedRange(object.id,track.speed);
            const double position_speed=std::hypot(vx,vy);
            bound.prediction_min_speed=std::min(speeds.first,std::max(0.0,position_speed-error));
            bound.prediction_max_speed=std::max(speeds.second,position_speed+error);
            for(int i=0;i<8;++i) {
                const double dx=(i%4==0||i%4==3)?-hl:hl,dy=i%4<2?-hw:hw;
                bound.corners[i]={bound.x+dx*c-dy*s,bound.y+dx*s+dy*c,object.z};
            }
            result.push_back(std::move(bound));
        }
        return result;
    }
    struct MissingDepartureObservation {
        int id=-1;
        double source_stamp=-1, age=0;
        std::size_t observations=0;
        bool motion_supported=false, body_supported=false;
        TrackedObject observation{};
        const char* reason="motion_insufficient";
        std::vector<TrackedObject> models;
        // Relevance only: never upgrades body confidence or a sensor sample.
        std::vector<TrackedObject> relevance_models;
    };
    // Read-only use of the existing one-second history. Construct at the real
    // observation time, then align once to the ego time. Never renew authority.
    std::vector<MissingDepartureObservation> MissingDepartureOccupancies(
        double evaluation_time,const std::function<bool(double,double)>& on_lane,
        double partial_length,double partial_width,double clamp,
        double conflict_x=NAN,double conflict_y=NAN) const {
        std::vector<MissingDepartureObservation> result;
        if(!std::isfinite(evaluation_time))return result;
        for(const auto& item:tracks_) {
            const auto& track=item.second;
            const double age=std::max(0.0,evaluation_time-track.stamp);
            if(stamp_-track.stamp<=1e-6||age>1||!track.moving)continue;
            MissingDepartureObservation missing;
            missing.id=item.first;missing.source_stamp=track.stamp;missing.age=age;
            missing.observations=track.samples.size();
            missing.observation=track.last;
            double vx=0,vy=0,error=0;
            missing.motion_supported=BoundedDisagreement(track,vx,vy,error);
            missing.body_supported=track.body_stamp>=0&&evaluation_time-track.body_stamp<=1;
            RoundaboutPrediction original;
            original.stamp_=track.stamp;original.tracks_.emplace(item.first,track);
            if(missing.motion_supported&&(track.body_stamp<0||missing.body_supported)) {
                auto bodies=original.DepartureOccupancies({track.last},on_lane,
                    partial_length,partial_width);
                if(!bodies.empty()) {
                    auto models=original.ApproachOccupancies({track.last});
                    bodies=original.ApproachOccupancies(bodies);
                    models.insert(models.end(),bodies.begin(),bodies.end());
                    models=ExpandTurnModels(original.WithObservedSpeedRanges(models),clamp);
                    for(const auto& model:models)
                        missing.models.push_back(AdvanceObjectObservation(model,age));
                }
            }
            if(!missing.models.empty())missing.reason="none";
            else if(InspectPredictionVelocity(track.last)!=PredictionInputStatus::MEASURED)
                missing.reason="velocity_missing";
            else if(!missing.motion_supported)missing.reason="motion_insufficient";
            else {
                missing.reason=track.body_stamp>=0&&!missing.body_supported?
                    "body_expired":"body_missing";
                // Source-time motion evidence is immutable. Missing time is
                // accounted for only by AdvanceObjectObservation below.
                if(!std::isfinite(conflict_x)||!std::isfinite(conflict_y)||
                   !original.MovingAwayReliable(item.first,conflict_x,conflict_y))
                    missing.reason="moving_away_unproven";
                else if(!GeometryUsable(track.last)||!std::isfinite(partial_length)||
                        partial_length<=0||!std::isfinite(partial_width)||partial_width<=0)
                    missing.reason="prediction_uncovered";
                else {
                    auto bound=track.last;
                    bound.heading=std::atan2(track.vy,track.vx);bound.yaw_rate=track.rate;
                    // A fragment centre can be anywhere on the body: use full
                    // dimensions as radii, including larger observed bodies.
                    double hl=std::max({partial_length,track.last.length,2*track.half_length})+.25;
                    double hw=std::max({partial_width,track.last.width,2*track.half_width})+.25;
                    const double c=std::cos(bound.heading),s=std::sin(bound.heading);
                    for(const auto& p:track.last.corners) {
                        const double dx=p.x-bound.x,dy=p.y-bound.y;
                        hl=std::max(hl,std::fabs(dx*c+dy*s)+.25);
                        hw=std::max(hw,std::fabs(-dx*s+dy*c)+.25);
                    }
                    bound.length=2*hl;bound.width=2*hw;bound.z_valid=false;
                    const auto speeds=original.ObservedSpeedRange(item.first,track.speed);
                    const double position_speed=std::hypot(vx,vy);
                    bound.prediction_min_speed=std::min(speeds.first,std::max(0.0,position_speed-error));
                    bound.prediction_max_speed=std::max(speeds.second,position_speed+error);
                    for(int k=0;k<8;++k) {
                        const double dx=(k%4==0||k%4==3)?-hl:hl,dy=k%4<2?-hw:hw;
                        bound.corners[k]={bound.x+dx*c-dy*s,bound.y+dx*s+dy*c,bound.z};
                    }
                    auto raw=track.last;raw.yaw_rate=track.rate;
                    auto models=ExpandTurnModels(original.WithObservedSpeedRanges(
                        original.ApproachOccupancies({raw,bound})),clamp);
                    for(const auto& model:models)
                        missing.relevance_models.push_back(AdvanceObjectObservation(model,age));
                }
            }
            result.push_back(std::move(missing));
        }
        std::sort(result.begin(),result.end(),[](const auto& a,const auto& b){return a.id<b.id;});
        return result;
    }
    // PREPARE checks both position-supported translation and a short sensor
    // velocity/turn hypothesis. A disputed fit cannot extrapolate a tight loop
    // for the entire stopping horizon. All original corners and margins remain.
    std::vector<TrackedObject> ApproachOccupancies(const std::vector<TrackedObject>& objects) const {
        std::vector<TrackedObject> result;
        result.reserve(objects.size()*2);
        for(const auto& object:objects) {
            auto primary=object;
            const auto it=tracks_.find(object.id);
            if(it==tracks_.end()){result.push_back(primary);continue;}
            const auto& track=it->second;
            double vx=0,vy=0;
            // Use the positional fit only when it contradicts an apparent
            // velocity decay: the observed centre still travels substantially
            // faster. A changing visible fragment can move its centre in the
            // opposite direction, so a slower position fit does not replace
            // the sensor's faster collision hypothesis.
            const bool disputed=DisputedVelocity(track,vx,vy);
            if(!disputed){result.push_back(primary);continue;}
            primary.vx=vx;primary.vy=vy;primary.speed=std::hypot(vx,vy);
            // Translation is supported, but these observations do not support
            // the reported heading-change fit. Keep both straight translation
            // and a short-turn sensor-velocity hypothesis in the safety check.
            primary.yaw_rate=0;
            result.push_back(primary);
            auto alternative=object;
            alternative.yaw_rate=track.rate;
            alternative.turn_prediction_duration=std::min(.5,
                track.samples.back().t-track.samples.front().t);
            alternative.continuation_vx=primary.vx;alternative.continuation_vy=primary.vy;
            result.push_back(alternative);
        }
        return result;
    }
    // A position fit does not authorize free-space extrapolation. Refine a
    // committed circulating actor only when a full-body/history-supported map
    // model covers the COMPLETE requested certificate, including the faster
    // measured/position speed bound. Otherwise retain the sensor model.
    std::vector<TrackedObject> ExecutionOccupancies(const std::vector<TrackedObject>& objects,
        const std::vector<std::shared_ptr<const RefLine>>& references,double horizon) const {
        auto result=objects;
        for(auto& object:result) {
            const auto it=tracks_.find(object.id);
            double vx=0,vy=0,error=0;
            if((object.type!=0&&object.type!=1)||
               InspectPredictionVelocity(object)!=PredictionInputStatus::MEASURED||
               object.motion_state==ObjectMotionState::LOW_THREAT||it==tracks_.end()||
               !BoundedDisagreement(it->second,vx,vy,error)||
               std::hypot(vx-object.vx,vy-object.vy)<=error)continue;
            auto supported=object;
            supported.vx=vx;supported.vy=vy;supported.speed=std::hypot(vx,vy);
            const double measured=std::hypot(object.vx,object.vy);
            supported.prediction_min_speed=std::max(0.0,std::min(supported.speed,measured)-error);
            supported.prediction_max_speed=std::max(supported.speed,measured)+error;
            if(std::isfinite(object.prediction_min_speed))supported.prediction_min_speed=
                std::min(supported.prediction_min_speed,object.prediction_min_speed);
            if(std::isfinite(object.prediction_max_speed))supported.prediction_max_speed=
                std::max(supported.prediction_max_speed,object.prediction_max_speed);
            const auto mapped=MappedMotion(supported,references,horizon);
            if(mapped.prediction_lane)object=mapped;
        }
        return result;
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

