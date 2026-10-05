#pragma once
#include <cmath>
#include <deque>
#include <optional>

// Source-time interpolation; receive time is deliberately not a substitute.
class HeadingHistory {
 public:
  void Add(double stamp, double yaw) {
    if (!std::isfinite(stamp) || !std::isfinite(yaw) || stamp <= 0 ||
        (!samples_.empty() && stamp <= samples_.back().stamp)) return;
    samples_.push_back({stamp, yaw});
    while (samples_.size() > 1 && stamp - samples_.front().stamp > 2.0)
      samples_.pop_front();
  }
  std::optional<double> At(double stamp) const {
    if (!std::isfinite(stamp) || samples_.empty()) return std::nullopt;
    for (std::size_t i=1; i<samples_.size(); ++i) {
      const auto& a=samples_[i-1]; const auto& b=samples_[i];
      if (a.stamp <= stamp && stamp <= b.stamp) {
        if (b.stamp-a.stamp > .2) return std::nullopt;
        const double u=(stamp-a.stamp)/(b.stamp-a.stamp);
        return a.yaw+u*std::remainder(b.yaw-a.yaw,2*3.14159265358979323846);
      }
    }
    const auto& nearest=stamp < samples_.front().stamp ? samples_.front() : samples_.back();
    if (std::abs(nearest.stamp-stamp) > .05) return std::nullopt;
    return nearest.yaw;
  }
 private:
  struct Sample { double stamp,yaw; };
  std::deque<Sample> samples_;
};
