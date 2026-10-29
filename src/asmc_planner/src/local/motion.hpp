#pragma once
#include <algorithm>
#include <cmath>
#include <deque>
#include <optional>

namespace local_planning {
inline bool BoundedSourceReorder(double t,double previous,double tolerance) {
  return std::isfinite(t+previous+tolerance)&&tolerance>0&&
         t<previous&&previous-t<=tolerance;
}
struct Motion {
  double t = 0, x = 0, y = 0, yaw = 0, v = 0, w = 0, roll = 0, pitch = 0, a = 0;
};
// One source-timed relative pose owner. LOCAL keeps gyro-rate yaw by default;
// Localization can measure relative yaw from the IMU quaternion instead of
// accumulating its rate bias. Translation is integrated once in either mode.
// A missing/retrograde sample starts a new epoch; receive freshness is external.
class MotionHistory {
public:
  unsigned epoch = 0;
  double reorder_tolerance_s=0; // opt-in; default LOCAL epoch policy unchanged
  double velocity_reference_offset_m=0;
  double velocity_turn_gain_m2ps=0; // opt-in empirical Molit model; zero keeps constant offset
  double velocity_turn_speed_floor_mps=1.;
  // Localization may track a small real brake rollback without authorizing a
  // reverse trajectory. Generic LOCAL retains its former rejection/clamp.
  double reverse_tracking_limit_mps=.05;
  double reference_offset(double speed) const {
    return velocity_turn_gain_m2ps>0?
        velocity_turn_gain_m2ps/std::max(speed,velocity_turn_speed_floor_mps):velocity_reference_offset_m;
  }
  void reset() {
    samples.clear();
    last_orientation_yaw=NAN;
    ++epoch;
  }
  bool add(double t, double v, double w, double roll = 0, double pitch = 0,
           double orientation_yaw = NAN) {
    if (!std::isfinite(t + v + w + roll + pitch) || t <= 0 || v < -reverse_tracking_limit_mps ||
        std::abs(w) > 2.) {
      reset();
      return false;
    }
    if (!samples.empty() && t == samples.back().t)
      return false;
    if(!samples.empty()&&BoundedSourceReorder(t,samples.back().t,reorder_tolerance_s))return false;
    if (!samples.empty() &&
        (t < samples.back().t || t - samples.back().t > .15))
      reset();
    Motion m;
    m.t = t;
    m.v = reverse_tracking_limit_mps>.05?v:std::max(0., v);
    m.w = w;
    m.roll = roll;
    m.pitch = pitch;
    if (!samples.empty()) {
      const auto &p = samples.back();
      double dt = t - p.t, turn = .5 * (p.w + w) * dt;
      if(std::isfinite(orientation_yaw)) {
        if(!std::isfinite(last_orientation_yaw)){reset();return false;}
        turn=std::remainder(orientation_yaw-last_orientation_yaw,2*M_PI);
        // Permit short packet bursts, but never integrate a heading jump.
        if(std::abs(turn)>std::max(.05,2.*dt)){reset();return false;}
      }
      m.a = .7 * p.a + .3 * std::clamp((m.v - p.v) / dt, -5., 2.);
      m.yaw = p.yaw + turn;
      double d = .5 * (p.v + m.v) * dt;
      m.x = p.x + d * std::cos(p.yaw + turn * .5);
      m.y = p.y + d * std::sin(p.yaw + turn * .5);
      // Exact rigid-point translation from the measured heading change, not
      // another integrator. Localization calibrates this velocity-reference
      // offset from pre-loss GNSS/IMU/speed; generic LOCAL keeps zero.
      // The optional Molit turn model scales that incremental lateral term
      // with speed. It is empirical, not a speed-dependent physical lever arm;
      // acceleration alone must never introduce a dk/dt translation.
      const double offset=reference_offset(.5*(p.v+m.v));
      m.x+=offset*(std::cos(m.yaw)-std::cos(p.yaw));
      m.y+=offset*(std::sin(m.yaw)-std::sin(p.yaw));
    }
    last_orientation_yaw=orientation_yaw;
    samples.push_back(m);
    while (samples.size() > 256 || samples.back().t - samples.front().t > 2.)
      samples.pop_front();
    return true;
  }
  std::optional<Motion> at(double t) const {
    if (samples.empty() || t < samples.front().t || t > samples.back().t + .05)
      return {};
    if (t >= samples.back().t) {
      auto m = samples.back();
      double dt = t - m.t;
      m.x += m.v * dt * std::cos(m.yaw + .5 * m.w * dt);
      m.y += m.v * dt * std::sin(m.yaw + .5 * m.w * dt);
      const double offset=reference_offset(m.v);
      m.x+=offset*(std::cos(m.yaw+m.w*dt)-std::cos(m.yaw));
      m.y+=offset*(std::sin(m.yaw+m.w*dt)-std::sin(m.yaw));
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
  double last_orientation_yaw=NAN;
};
} // namespace local_planning
