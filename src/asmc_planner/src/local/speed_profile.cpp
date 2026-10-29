#include "local/speed_profile.hpp"
#include "Frenet/Evaluation/CollisionChecker.hpp"
#include <algorithm>
#include <cmath>

namespace local_planning {
// Exact constant-jerk ramp from current acceleration to the configured brake.
// Positive acceleration is part of stopping distance, not instantaneous
// braking.
double BrakingDistance(double v, double a, const Config &c) {
  if (v <= 0)
    return 0.;
  a = std::max(-c.decel, a);
  double ramp = (a + c.decel) / c.jerk;
  double zero = (a + std::sqrt(a * a + 2 * c.jerk * v)) / c.jerk;
  double t = std::min(ramp, zero);
  double d = v * t + .5 * a * t * t - c.jerk * t * t * t / 6.;
  double rest = std::max(0., v + a * t - .5 * c.jerk * t * t);
  return d + rest * rest / (2 * c.decel);
}
double SpaceSpeed(double distance, double acceleration, double delay,
                  const Config &c) {
  double lo = 0., hi = std::max(15., c.speed_cap);
  for (int i = 0; i < 24; ++i) {
    double v = .5 * (lo + hi);
    if (v * delay + BrakingDistance(v, acceleration, c) <=
        std::max(0., distance))
      lo = v;
    else
      hi = v;
  }
  return lo;
}

bool BuildSpeedProfile(const Scene &s, const Config &c, Result &out) {
  if (out.path.size() < 2) { out.reason="no_speed_geometry"; return false; }
  if(c.maneuver_reserve>0.) {
    const auto &p=out.path.back();
    const auto body=MakeEgoFootprint(CartesianState{p.x,p.y,p.yaw,0,0,0},VehicleShape{c.width,c.length,c.rear},0.);
    bool approaching=false;
    for(const auto &o:s.obstacles) {
      const auto &poly=o.motion_sweep.empty()?o.polygon:o.motion_sweep;
      double front=-1e9;
      for(auto q:poly)front=std::max(front,q.x);
      if(p.x-c.rear>front)continue; // A verified path beyond the object is a bypass.
      if(geometry::ConvexPolygonDistance(body,poly)<=c.margin+o.extra_clearance+o.uncertainty+.25)
        approaching=true;
    }
    // Do not consume all turn room by repeatedly selecting a straight prefix
    // ending just before an obstacle. A genuinely turning bounded recovery can
    // still reposition within today's hard body/OBB/exterior checks.
    const double nose_shift=std::abs(p.y+(c.length-c.rear)*std::sin(p.yaw));
    if(approaching && !(out.recovery && nose_shift>.5)) {
      const double keep=std::max(.1,p.s-c.maneuver_reserve),old=p.s;
      auto it=std::lower_bound(out.path.begin(),out.path.end(),keep,[](const Pose& p,double s){return p.s<s;});
      if(it!=out.path.end() && it!=out.path.begin()) {
        auto b=*it,a=*(it-1);double t=(keep-a.s)/std::max(1e-8,b.s-a.s);
        Pose end{a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),a.yaw+t*(b.yaw-a.yaw),a.k+t*(b.k-a.k),keep};
        out.path.erase(it,out.path.end());out.path.push_back(end);
        out.maneuver_reserve_m=old-keep;
      }
    }
  }
  out.trajectory.clear();
  double maxk = 0;
  for (auto &p : out.path)
    maxk = std::max(maxk, std::abs(p.k));
  // Geometry can turn sharply while crawling, but its speed must respect the
  // calibrated wheel-angle slew. For linearly interpolated curvature, the
  // largest |d atan(L*k/gain)/ds| is at the smallest |k| (also across zero).
  // Keep the physical steering/curvature bounds; do not invent a larger radius
  // limit or clip the measured initial speed/acceleration instantaneously.
  double steering_gradient = 0.;
  std::vector<double> gradients(out.path.size(),0.);
  for (size_t i = 1; i < out.path.size(); ++i) {
    const auto &a = out.path[i-1], &b = out.path[i];
    const double dk = std::abs(b.k-a.k), ds = b.s-a.s;
    if (ds <= 1e-8) {
      if (dk > 1e-8) { out.reason="zero_length_steering_change"; return false; }
      continue;
    }
    const double mink = a.k*b.k <= 0. ? 0. : std::min(std::abs(a.k),std::abs(b.k));
    const double scale = c.wheelbase/c.gain;
    gradients[i]=scale/(1.+scale*scale*mink*mink)*dk/ds;
    steering_gradient = std::max(steering_gradient,gradients[i]);
  }
  out.steering_cap = !c.native_frenet_timing && steering_gradient > 1e-8 ?
      .9*c.max_steer_rate/steering_gradient : c.speed_cap;
  out.raw_cap = c.speed_cap;
  out.maneuver_cap=c.speed_cap;
  if(c.maneuver_speed_cap>0. && c.maneuver_speed_cap<c.speed_cap && !out.recovery &&
     !(out.assisted && c.maneuver_speed_cap>=5./3.6)) {
    for(const auto &o:s.obstacles) {
      double near=1e9,far=-1e9;
      for(auto p:o.polygon){near=std::min(near,p.x);far=std::max(far,p.x);}
      if(near>10. || far < -c.rear)continue;
      // This is current selected-path geometry, not YOLO presence or a motion
      // class. Slow before a tight maneuver so the calibrated steering delay
      // and repeated replans do not consume the remaining turn room at10km/h.
      for(size_t i=1;i<out.path.size();++i) {
        const auto &a=out.path[i-1],&b=out.path[i];
        if(a.x>far+c.rear+.5)break;
        const int count=std::clamp(int(std::ceil(std::hypot(b.x-a.x,b.y-a.y)/.5)),1,100);
        for(int k=0;k<=count;++k) {
          const double t=double(k)/count;
          const auto body=MakeEgoFootprint(CartesianState{a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),a.yaw+t*(b.yaw-a.yaw),0,0,0},VehicleShape{c.width,c.length,c.rear},0.);
          if(geometry::ConvexPolygonDistance(body,o.polygon)<=c.preferred_clearance+o.uncertainty+o.extra_clearance+.10) {
            out.maneuver_cap=std::min(c.speed_cap,c.maneuver_speed_cap);break;
          }
        }
        if(out.maneuver_cap<c.speed_cap)break;
      }
      if(out.maneuver_cap<c.speed_cap)break;
    }
  }
  out.curve_cap = std::sqrt(c.lateral_accel * .95 / std::max(1e-4, maxk));
  // Path s is already rear-axle travel after a full-body collision check; do
  // not subtract the front overhang a second time. Reserve jerk ramp +
  // execution lag.
  const double distance = out.path.back().s;
  out.available_stopping_m = distance;
  // In aligned scenes the vehicle's source->plan travel already shortens the
  // available sweep. Keep execution/jerk delay, do not charge that travel
  // twice.
  double lag = c.native_frenet_timing?c.delay+s.age+c.decel/c.jerk:c.delay + (s.aligned_to_plan ? 0. : s.age);
  out.comfort_stopping_m = s.speed * lag + (c.native_frenet_timing?s.speed*s.speed/(2*c.decel):BrakingDistance(s.speed, s.acceleration, c));
  out.required_stopping_m = out.comfort_stopping_m;
  Config braking=c;
  // Comfort is not the vehicle's physical braking bound. Keep the original
  // cruise governor, but before invalidating a current path try the separately
  // configured/calibrated controlled brake (max3m/s², MPC bound is5m/s²).
  // No distance, delay, jerk, source age or collision gate is relaxed.
  if(out.required_stopping_m>distance && c.controlled_decel>c.decel) {
    braking.decel=c.controlled_decel;
    out.required_stopping_m=s.speed*lag+BrakingDistance(s.speed,s.acceleration,braking);
  }
  out.braking_limit=braking.decel;
  // A braking frame must not instantly raise the cruise cap and restart the
  // throttle/brake cycle. Keep a positive-acceleration envelope, plus measured
  // LOCAL-2B tracking overshoot reserve (0.35m/s). Actual-state stopping
  // feasibility below remains separate and is never relaxed by this governor.
  out.stopping_cap = SpaceSpeed(distance - .1, s.acceleration, lag, braking);
  out.space_cap =
      std::max(0., SpaceSpeed(distance - .1, c.accel, lag, c) - .35);
  if(c.native_frenet_timing)out.space_cap=std::max(0.,std::sqrt(c.decel*c.decel*lag*lag+2*c.decel*std::max(0.,distance-.1))-c.decel*lag);
  out.chosen_cap = std::min({out.raw_cap, out.maneuver_cap, out.curve_cap, out.steering_cap, out.space_cap});
  out.speed_reason = out.chosen_cap == out.space_cap ? "verified_stopping_space"
                     : out.chosen_cap == out.steering_cap ? "steering_slew"
                     : out.chosen_cap == out.curve_cap ? "curvature"
                                                       : "requested_cap";
  if(out.chosen_cap==out.maneuver_cap && out.maneuver_cap<out.raw_cap)
    out.speed_reason="close_swept_path_maneuver";
  if (out.assisted && !out.recovery && out.chosen_cap > 5. / 3.6) {
    out.chosen_cap = 5. / 3.6;
    out.speed_reason = "experimental_near_blind_assist";
  }
  if (out.recovery) {
    out.chosen_cap = std::min(c.recovery_speed, out.chosen_cap);
    out.speed_reason = c.recovery_speed>3./3.6?"near_field_recovery_5kph":"near_field_recovery_3kph";
  }
  if (out.required_stopping_m > distance) {
    out.reason = "insufficient_stopping_space";
    return false;
  }
  if(braking.decel>c.decel)out.speed_reason="controlled_braking";
  if(out.maneuver_reserve_m>0.)out.speed_reason=out.chosen_cap<.15?"waiting_for_bypass_space":"maneuver_space_reserve";
  double travelled = 0, v = s.speed, a = c.native_frenet_timing?std::clamp(s.acceleration,-c.decel,c.accel):std::max(-braking.decel, s.acceleration);
  SpeedProfiler native_profiler;
  size_t native_stop_begin=s.native_speed_limits.size();
  while(native_stop_begin>0 && s.native_speed_limits[native_stop_begin-1]<.03)--native_stop_begin;
  size_t index = 1;
  for (int i = 0; i < 400; ++i) {
    while (index < out.path.size() - 1 && out.path[index].s < travelled)
      ++index;
    const auto &left = out.path[index - 1], &right = out.path[index];
    double q = std::clamp(
        (travelled - left.s) / std::max(1e-6, right.s - left.s), 0., 1.);
    out.trajectory.push_back({left.x + q * (right.x - left.x),
                              left.y + q * (right.y - left.y),
                              left.yaw + q * (right.yaw - left.yaw),
                              left.k + q * (right.k - left.k), v, a, i * .05});
    double remaining = std::max(0., distance - travelled);
    double cap =
        std::min(out.chosen_cap, SpaceSpeed(remaining - .1, a, lag, braking));
    if(c.native_frenet_timing)cap=std::min(out.chosen_cap,std::max(0.,std::sqrt(c.decel*c.decel*lag*lag+2*c.decel*remaining)-c.decel*lag));
    if(s.native_speed_limits.size()==out.path.size())
      cap=std::min(cap,std::max(0.,s.native_speed_limits[index-1]+q*(s.native_speed_limits[index]-s.native_speed_limits[index-1])));
    // LOCAL-2C: scalar SpeedProfiler clips a lower cap immediately and is
    // deliberately unsuitable for a physical v/a trajectory. Use its shared
    // analytical transition, retaining endpoint acceleration (not average
    // delta-v/dt). Otherwise a tiny overspeed provokes a brake/reaccelerate
    // loop.
    auto step = c.native_frenet_timing?
        native_profiler.Update({cap,v,a,.05},{c.accel,c.decel,c.jerk}):
        AdvancePhysicalSpeed({cap, v, a, .05}, {c.accel, braking.decel, c.jerk});
    if(c.native_frenet_timing){
      step.target_acceleration=std::clamp((step.target_speed-v)/.05,std::max(-c.decel,a-c.jerk*.05),std::min(c.accel,a+c.jerk*.05));
      step.target_speed=std::max(0.,v+step.target_acceleration*.05);
    }
    const double new_v = step.target_speed;
    // A sharp FUTURE segment must not invalidate today's straight braking
    // interval. Check only the actually swept s range for this physical step;
    // every later step is checked in turn. Keep the global desired cap, source
    // state and exact curvature derivative; never clip speed instantaneously.
    const double next_s=travelled+.5*(v+new_v)*.05;
    double current_gradient=0.;
    for(size_t k=index;k<out.path.size() && out.path[k-1].s<=next_s;++k) {
      current_gradient=std::max(current_gradient,gradients[k]);
      if(out.path[k].s>=next_s)break;
    }
    // The jerk bound also covers an interior speed peak between time samples.
    const double speed_bound=std::max(v,new_v)+.5*c.jerk*.05*.05;
    if (!c.native_frenet_timing && current_gradient * speed_bound > c.max_steer_rate+1e-6) {
      out.reason="initial_steering_slew_infeasible";
      out.trajectory.clear(); return false;
    }
    a = step.target_acceleration;
    travelled += .5 * (v + new_v) * .05;
    v = new_v;
    if (travelled > distance + .05) {
      out.reason = "terminal_braking_violation";
      out.trajectory.clear();
      return false;
    }
    if (i > 40 && v < .03 && (remaining < .4 || (!s.native_speed_limits.empty() && index-1>=native_stop_begin))) {
      out.trajectory.back()[4] = 0;
      break;
    }
  }
  return true;
}
Result TimeParameterize(const Scene& scene,const Config& cfg,Result result,const std::vector<double>& limits){
  auto timed=scene;timed.native_speed_limits=limits;auto native=cfg;native.native_frenet_timing=true;
  if(!limits.empty()&&limits.size()!=result.path.size()){result.valid=false;result.reason="native_speed_limit_shape";return result;}
  for(double v:limits)if(!std::isfinite(v)||v<0){result.valid=false;result.reason="native_speed_limit_invalid";return result;}
  result.valid=BuildSpeedProfile(timed,native,result);
  if(result.valid)result.reason="ok";
  return result;
}
} // namespace local_planning
