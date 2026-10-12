#pragma once

#include <algorithm>
#include <cmath>

// Bounded online correction of the pedal map, only used during ordinary STOP.
// MPC controls stay in physical acceleration units; only the request passed to
// the existing pedal map is divided by the observed brake response gain.
struct StopBrakeResponseConfig {
    bool enabled = false;
    double acceleration_limit = .5;
    double acceleration_minimum_request = .15;
    double learning_time_constant = .3;
    double response_time_constant = .1;
    double minimum_gain = .5;
    double maximum_gain = 2.5;
    double minimum_request = .5;
    double minimum_speed = .5;
};

class StopBrakeResponse {
public:
    void Reset() { *this = StopBrakeResponse{}; }
    double gain() const { return gain_; }
    void Observe(double acceleration, double speed, double stamp,
                 const StopBrakeResponseConfig& cfg) {
        if (!std::isfinite(acceleration) || !std::isfinite(speed) ||
            !std::isfinite(stamp) || stamp <= 0) return;
        if (last_observation_ > 0 && stamp <= last_observation_) return;
        const double dt = last_observation_ > 0 ? stamp-last_observation_ : 0;
        last_observation_ = stamp;
        if (!has_request_ || stamp <= request_stamp_ || dt <= 0 || dt > .2 ||
            speed < cfg.minimum_speed || acceleration >= 0 ||
            filtered_request_ > -cfg.minimum_request) return;
        const double observed_gain = acceleration/filtered_request_;
        // A delayed sign change or isolated sensor spike is not calibration.
        // Learn only responses inside the configured identifiable range.
        if (!std::isfinite(observed_gain) || observed_gain < cfg.minimum_gain ||
            observed_gain > cfg.maximum_gain) return;
        const double weight = dt / (cfg.learning_time_constant + dt);
        numerator_ += weight * (filtered_request_*acceleration - numerator_);
        denominator_ += weight * (filtered_request_*filtered_request_ - denominator_);
        if (denominator_ > 1e-9)
            gain_ = std::clamp(numerator_/denominator_,
                               cfg.minimum_gain,cfg.maximum_gain);
    }
    double Compensate(double desired_acceleration, double accel_min) const {
        if (!std::isfinite(desired_acceleration) || desired_acceleration >= 0)
            return desired_acceleration;
        return std::clamp(desired_acceleration/gain_,accel_min,0.0);
    }
    void Issued(double nominal_request, double stamp,
                const StopBrakeResponseConfig& cfg) {
        if (!std::isfinite(nominal_request) || !std::isfinite(stamp) || stamp <= 0) {
            Reset();return;
        }
        if (nominal_request >= 0) {
            // Do not identify residual braking after switching to throttle.
            has_request_ = false;filtered_request_=0;request_stamp_=stamp;return;
        }
        const double dt = has_request_ ? stamp-request_stamp_ : 0;
        if (!has_request_ || dt <= 0 || dt > .2) {
            filtered_request_ = nominal_request;
        } else {
            const double weight = dt/(cfg.response_time_constant+dt);
            filtered_request_ += weight*(nominal_request-filtered_request_);
        }
        has_request_=true;request_stamp_=stamp;
    }
private:
    double gain_=1,numerator_=.25,denominator_=.25;
    double last_observation_=0,request_stamp_=0,filtered_request_=0;
    bool has_request_=false;
};

// Independent throttle identification: reflect positive responses into the
// brake estimator's signed domain. Never share a gain between the two pedals.
class StopAccelerationResponse {
public:
    void Reset() { response_.Reset(); }
    double gain() const { return response_.gain(); }
    void Observe(double acceleration,double speed,double stamp,
                 const StopBrakeResponseConfig& config) {
        auto throttle=config;throttle.minimum_request=config.acceleration_minimum_request;
        response_.Observe(-acceleration,speed,stamp,throttle);
    }
    double Compensate(double desired,double maximum) const {
        return desired > 0 ? -response_.Compensate(-desired,-maximum) : desired;
    }
    void Issued(double request,double stamp,const StopBrakeResponseConfig& config) {
        auto throttle=config;throttle.minimum_request=config.acceleration_minimum_request;
        response_.Issued(-request,stamp,throttle);
    }
private:
    StopBrakeResponse response_;
};
