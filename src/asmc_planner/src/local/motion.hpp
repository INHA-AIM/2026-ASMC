#pragma once
#include <algorithm>
#include <cmath>
#include <deque>
#include <optional>

namespace local_planning {
struct Motion {
  double t = 0, x = 0, y = 0, yaw = 0, v = 0, w = 0, roll = 0, pitch = 0, a = 0;
};
// Source-time integration only. Steady receive freshness belongs to the ROS
// boundary. A missing/retrograde sample starts a new epoch, not map fallback.
class MotionHistory {
public:
  unsigned epoch = 0;
  double latest_stamp() const {
    return samples.empty() ? 0. : samples.back().t;
  }
  void reset() {
    samples.clear();
    ++epoch;
  }
  bool add(double t, double v, double w, double roll = 0, double pitch = 0) {
    if (!std::isfinite(t + v + w + roll + pitch) || t <= 0 ||
        std::abs(w) > 2.) {
      reset();
      return false;
    }
    if (!samples.empty() && t == samples.back().t)
      return false;
    if (!samples.empty() &&
        (t < samples.back().t || t - samples.back().t > .15))
      reset();
    Motion m;
    m.t = t;
    // Observation alignment is signed even though the current controller only
    // plans forward. Reversing must not retire camera/LiDAR epochs or drawings.
    m.v = v;
    m.w = w;
    m.roll = roll;
    m.pitch = pitch;
    if (!samples.empty()) {
      const auto &p = samples.back();
      double dt = t - p.t, turn = .5 * (p.w + w) * dt;
      m.a = .7 * p.a + .3 * std::clamp((m.v - p.v) / dt, -5., 2.);
      m.yaw = p.yaw + turn;
      double d = .5 * (p.v + m.v) * dt;
      m.x = p.x + d * std::cos(p.yaw + turn * .5);
      m.y = p.y + d * std::sin(p.yaw + turn * .5);
    }
    samples.push_back(m);
    while (samples.size() > 256 || samples.back().t - samples.front().t > 2.)
      samples.pop_front();
    return true;
  }
  // LOCAL-2A arrival audit: valid IMU gaps reached83ms. A100ms short
  // constant-twist forecast covers that cadence; this is not tunnel map pose.
  std::optional<Motion> at(double t) const {
    if (samples.empty() || t < samples.front().t || t > samples.back().t + .10)
      return {};
    if (t >= samples.back().t) {
      auto m = samples.back();
      double dt = t - m.t;
      m.x += m.v * dt * std::cos(m.yaw + .5 * m.w * dt);
      m.y += m.v * dt * std::sin(m.yaw + .5 * m.w * dt);
      m.yaw += m.w * dt;
      m.t = t;
      return m;
    }
    for (size_t i = 1; i < samples.size(); ++i)
      if (samples[i].t >= t) {
        const auto &a = samples[i - 1];
        const auto &b = samples[i];
        double q = (t - a.t) / (b.t - a.t);
        return Motion{t,
                      a.x + q * (b.x - a.x),
                      a.y + q * (b.y - a.y),
                      a.yaw + q * (b.yaw - a.yaw),
                      a.v + q * (b.v - a.v),
                      a.w + q * (b.w - a.w),
                      a.roll + q * (b.roll - a.roll),
                      a.pitch + q * (b.pitch - a.pitch),
                      a.a + q * (b.a - a.a)};
      }
    return {};
  }

private:
  std::deque<Motion> samples;
};
} // namespace local_planning
