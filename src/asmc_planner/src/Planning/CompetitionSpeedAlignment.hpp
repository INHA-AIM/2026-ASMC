#pragma once
#include <algorithm>
#include <cmath>
#include <deque>
#include <optional>

namespace planning {
// Align the permitted body-longitudinal speed with the GPS/IMU pose time.
// Source time is used only for alignment; steady time only for freshness.
// This owns a bounded speed history, not a second pose estimator.
class CompetitionSpeedAlignment {
 public:
  struct Config { double source_age_s=.10, receive_age_s=.15, restart_jump_s=.5, stationary_deadband_mps=.05; };
  CompetitionSpeedAlignment() = default;
  explicit CompetitionSpeedAlignment(Config config):config_(config) {}
  bool observe(double source, double received, double speed) {
    if(!std::isfinite(source)||source<=0||!std::isfinite(received)||
       !std::isfinite(speed)||speed< -config_.stationary_deadband_mps||speed>60)return false;
    // Measured signed jitter around standstill is a known near-zero speed,
    // not missing data. Keep its source stamp fresh; genuine reverse remains
    // outside this forward-driving input contract.
    if(std::abs(speed)<=config_.stationary_deadband_mps)speed=0;
    if(!samples_.empty() && source<samples_.back().source-config_.restart_jump_s)
      samples_.clear();
    if(!samples_.empty() && source<=samples_.back().source)return false;
    samples_.push_back({source,received,speed});
    while(samples_.size()>64 || source-samples_.front().source>.5)samples_.pop_front();
    return true;
  }
  std::optional<double> align(double source, double acceleration, double steady_now) {
    if(!std::isfinite(source)||source<=0||!std::isfinite(acceleration)||
       !std::isfinite(steady_now))return {};
    if(last_state_source_>0 && source<last_state_source_-config_.restart_jump_s) {
      // Keep only measurements from the new time epoch, if they arrived first.
      while(!samples_.empty() && samples_.back().source>source+config_.source_age_s)
        samples_.pop_back();
    }
    last_state_source_=source;
    for(auto it=samples_.rbegin();it!=samples_.rend();++it) {
      if(it->source>source)continue;
      const double dt=source-it->source,age=steady_now-it->received;
      if(dt>config_.source_age_s || age<0 || age>config_.receive_age_s)return {};
      return std::max(0.0,it->speed+std::clamp(acceleration,-12.0,8.0)*dt);
    }
    return {};
  }
 private:
  struct Sample { double source,received,speed; };
  Config config_;
  std::deque<Sample> samples_;
  double last_state_source_=0;
};
}  // namespace planning
