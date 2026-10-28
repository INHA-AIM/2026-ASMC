#pragma once

#include "Frenet/Config/FrenetConfig.hpp"
#include "Frenet/Geometry/ObservedMotion.hpp"
#include "Frenet/Evaluation/CollisionChecker.hpp"

// Current observation-derived uncertainty covers the FULL certified path.
// Only new evidence can reduce it; a future time threshold is not evidence.
// Values are prediction assumptions, not calibrated sensor guarantees.
inline double HighwayPositionError(double initial_error,double time,
                                   const HighwayMergeConfig& /*config*/) {
    return initial_error*std::fabs(time);
}
inline double HighwayVelocityError(double initial_error,double /*time*/,
                                   const HighwayMergeConfig& /*config*/) {
    return initial_error;
}
inline geometry::Polygon2D PredictHighwayObstacleFootprint(
    const TrackedObject& object,double time,double initial_error,
    const HighwayMergeConfig& config) {
    auto bounded=object;const double speed=std::hypot(object.vx,object.vy);
    if(object.prediction_lane) {
        bounded.prediction_min_speed=std::max(0.0,speed-initial_error);
        bounded.prediction_max_speed=speed+initial_error;
        return PredictObstacleFootprint(bounded,time);
    }
    if(object.motion_state!=ObjectMotionState::LOW_THREAT&&speed>1e-6) {
        const double error=std::fabs(time)>1e-9?HighwayPositionError(initial_error,time,config)/std::fabs(time):initial_error;
        bounded.prediction_min_speed=std::max(0.0,speed-error);
        bounded.prediction_max_speed=speed+error;
        if(std::isfinite(object.continuation_vx)&&std::isfinite(object.continuation_vy)) {
            // This highway hypothesis has a lane-aligned continuation. Scale
            // BOTH phases, so its persistent longitudinal uncertainty is not
            // accidentally limited to the short provisional lateral phase.
            geometry::Polygon2D points;
            for(double value:{bounded.prediction_min_speed,bounded.prediction_max_speed}) {
                auto hypothesis=object;const double ratio=value/speed;
                hypothesis.vx*=ratio;hypothesis.vy*=ratio;
                hypothesis.continuation_vx*=ratio;hypothesis.continuation_vy*=ratio;
                hypothesis.prediction_min_speed=hypothesis.prediction_max_speed=NAN;
                const auto pose=PredictTrackedObjectPose(hypothesis,time);
                const auto corners=geometry::TranslateCorners(object.corners,
                    pose.x-object.x,pose.y-object.y,1.0);
                const auto xy=geometry::ProjectCornersToXY(corners);
                points.insert(points.end(),xy.begin(),xy.end());
            }
            return geometry::ComputeConvexHull(points);
        }
    }
    return PredictObstacleFootprint(bounded,time);
}

// The initial value is a tunable model prior, not a sensor-error guarantee.
// Same-direction traffic uses robust source-timed observations. A single OBB
// center jump must not discard an otherwise consistent velocity history.
namespace highway_prediction_detail {
inline double Median(std::vector<double> values) {
    if(values.empty())return 0;
    std::sort(values.begin(),values.end());
    const auto n=values.size();return n%2?values[n/2]:.5*(values[n/2-1]+values[n/2]);
}
inline double Scatter(const std::vector<double>& values,double center) {
    std::vector<double> deviations;
    for(double value:values)deviations.push_back(std::fabs(value-center));
    return 1.4826*Median(std::move(deviations));
}
}
inline TrackedObject PrepareHighwayObjectPrediction(
    const TrackedObject& object,const ObservedObjectMotion* track,
    const HighwayMergeConfig& config,const RefLine* lane=nullptr,double traffic_heading=NAN,
    double direction_resolution_rad=.05) {
    auto predicted=object;
    if(object.motion_state==ObjectMotionState::LOW_THREAT)return predicted;
    const double speed=std::hypot(object.vx,object.vy);
    if(!std::isfinite(speed)||speed<1e-6)return predicted;
    double error=config.speed_error_mps;
    if(track&&(std::isfinite(traffic_heading)||(lane&&lane->points.size()>2))&&track->samples.size()>=3&&
       std::fabs(object.yaw_rate)<1e-3&&
       track->stamp-track->samples.front().time>=config.motion_fit_min_span_s) {
        const double heading=std::isfinite(traffic_heading)?traffic_heading:
            Interpolate(*lane,FindClosestS(*lane,object.x,object.y,nullptr,100)).theta;
        const double tx=std::cos(heading),ty=std::sin(heading);
        const double along=object.vx*tx+object.vy*ty;
        if(along/speed>=config.min_heading_alignment) {
            using namespace highway_prediction_detail;
            std::vector<double> times,positions,velocities,slopes,lateral_positions;
            for(const auto& sample:track->samples) {
                times.push_back(sample.time-track->stamp);
                positions.push_back(sample.x*tx+sample.y*ty);
                velocities.push_back(sample.vx*tx+sample.vy*ty);
                lateral_positions.push_back(-sample.x*ty+sample.y*tx);
            }
            const double observed=Median(velocities);
            const double velocity_error=std::max(config.speed_error_floor_mps,
                3*Scatter(velocities,observed));
            double estimate=observed;
            error=std::max(velocity_error,std::fabs(along-observed));
            // Theil-Sen initialization followed by inlier-only least squares.
            // At most twelve observations / sixty-six pairs, once per object;
            // no candidate-level fit.
            for(std::size_t i=0;i<times.size();++i)
                for(std::size_t j=i+1;j<times.size();++j)
                    if(times[j]-times[i]>=config.motion_fit_min_span_s*.5)
                        slopes.push_back((positions[j]-positions[i])/(times[j]-times[i]));
            if(!slopes.empty()) {
                const double slope=Median(slopes);
                std::vector<double> intercepts;
                for(std::size_t i=0;i<times.size();++i)
                    intercepts.push_back(positions[i]-slope*times[i]);
                const double intercept=Median(intercepts);
                const double residual_gate=std::max(.1,3*Scatter(intercepts,intercept));
                std::vector<std::size_t> inliers;
                for(std::size_t i=0;i<times.size();++i)
                    if(std::fabs(intercepts[i]-intercept)<=residual_gate)inliers.push_back(i);
                if(inliers.size()>=3&&inliers.size()*3>=times.size()*2&&
                   times[inliers.back()]-times[inliers.front()]>=config.motion_fit_min_span_s) {
                    double mt=0,mp=0,tt=0,tp=0;
                    for(auto i:inliers){mt+=times[i];mp+=positions[i];}
                    mt/=inliers.size();mp/=inliers.size();
                    for(auto i:inliers){tt+=(times[i]-mt)*(times[i]-mt);tp+=(times[i]-mt)*(positions[i]-mp);}
                    if(tt>1e-9) {
                        const double fit=tp/tt;
                        std::vector<double> residuals;
                        for(auto i:inliers)residuals.push_back(std::fabs(positions[i]-mp-fit*(times[i]-mt)));
                        const double position_error=2*Median(residuals)/
                            (times[inliers.back()]-times[inliers.front()]);
                        const double disagreement=std::fabs(fit-observed);
                        if(std::isfinite(fit)&&fit>0) {
                            estimate=fit;
                            error=std::max({velocity_error,position_error,disagreement});
                        }
                    }
                }
            }
            // Refine only longitudinal speed. An actual observed cut-in is
            // never zeroed by a straight-road or low-scatter assumption.
            const double lateral=-object.vx*ty+object.vy*tx;
            predicted.vx=estimate*tx-lateral*ty;
            predicted.vy=estimate*ty+lateral*tx;
            predicted.speed=std::hypot(predicted.vx,predicted.vy);
            // The same observed-direction resolution used by motion tracking:
            // a tiny angular error has a larger m/s component at highway speed.
            const double lateral_resolution=std::max(config.provisional_lateral_speed_mps,
                speed*std::sin(std::clamp(direction_resolution_rad,0.0,M_PI/2)));
            if(std::fabs(lateral)<=lateral_resolution) {
                std::vector<double> transverse_slopes;
                for(std::size_t i=0;i<times.size();++i)
                    for(std::size_t j=i+1;j<times.size();++j)
                        if(times[j]-times[i]>=config.motion_fit_min_span_s*.5)
                            transverse_slopes.push_back((lateral_positions[j]-lateral_positions[i])/(times[j]-times[i]));
                // A small velocity component contradicted by position history
                // is a short observation hypothesis, not evidence of a several
                // second lane crossing. Current motion is retained through the
                // noise window. Strong or position-supported cut-ins continue.
                const double transverse=Median(transverse_slopes);
                std::vector<double> offsets;
                for(std::size_t i=0;i<times.size();++i)
                    offsets.push_back(lateral_positions[i]-transverse*times[i]);
                const double span=times.back()-times.front();
                const bool position_supported=std::fabs(transverse)*span>
                    std::max(.1,3*Scatter(offsets,Median(offsets)));
                if(!transverse_slopes.empty()&&(!position_supported||
                   std::fabs(transverse-lateral)>config.provisional_lateral_speed_mps)) {
                    predicted.turn_prediction_duration=config.speed_error_correlation_s;
                    predicted.continuation_vx=estimate*tx;
                    predicted.continuation_vy=estimate*ty;
                }
            }
        }
    }
    const double predicted_speed=std::hypot(predicted.vx,predicted.vy);
    predicted.prediction_min_speed=std::max(0.0,predicted_speed-error);
    predicted.prediction_max_speed=predicted_speed+error;
    if(std::isfinite(object.prediction_min_speed))
        predicted.prediction_min_speed=std::min(predicted.prediction_min_speed,object.prediction_min_speed);
    if(std::isfinite(object.prediction_max_speed))
        predicted.prediction_max_speed=std::max(predicted.prediction_max_speed,object.prediction_max_speed);
    return predicted;
}
