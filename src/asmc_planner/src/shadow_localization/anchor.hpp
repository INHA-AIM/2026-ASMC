#pragma once
#include "local/motion.hpp"
#include <string>
#include <array>

namespace shadow_localization {
struct Transform {
  double x=0, y=0, yaw=0, stamp=0, distance=0;
  bool valid=false;
  std::string source="unavailable";
  std::array<double,2> apply(double px,double py) const {
    return {x+std::cos(yaw)*px-std::sin(yaw)*py,y+std::sin(yaw)*px+std::cos(yaw)*py};
  }
  std::array<double,2> inverse(double px,double py) const {
    px-=x;py-=y;return {std::cos(yaw)*px+std::sin(yaw)*py,-std::sin(yaw)*px+std::cos(yaw)*py};
  }
};
// One external odometry owner. This class estimates only the SE2 anchor. All
// time is aligned measurement time, never ROS receipt time or simulator GT.
class Anchor {
 public:
  unsigned generation=0;
  std::string state="warming",reason="initial_anchor_warmup";
  double innovation_m=0,innovation_rad=0,raw_innovation_m=0;
  // Research-only opt-in. Default consumers retain immediate single-fix gates.
  bool validate_recovery_window=false;
  size_t samples() const {return fixes_.size();}
  void reset() {
    ++generation;fixes_.clear();history_.clear();last_motion_={};travel_=0;
    last_fix_=window_start_=blend_start_=recovery_start_=validated_stamp_=validated_distance_=0;
    blocked_=false;state="warming";reason="epoch_reset";
  }
  void invalidate(const std::string& why) {
    reset();blocked_=true;state="reanchor_stop";reason=why;
  }
  void motion(const local_planning::Motion& m) {
    if (last_motion_.t && m.t<last_motion_.t) reset();
    if (last_motion_.t && m.t==last_motion_.t)return;
    if (last_motion_.t)travel_+=std::hypot(m.x-last_motion_.x,m.y-last_motion_.y);
    last_motion_=m;
  }
  bool observe(const local_planning::Motion& m,double x,double y,double yaw) {
    if (!std::isfinite(m.t+m.x+m.y+m.yaw+m.v+x+y+yaw) || m.t<=0 || m.t<=last_fix_) {
      reason="gps_invalid_or_nonmonotonic";return false;
    }
    const double gap=last_fix_?m.t-last_fix_:0;
    last_fix_=m.t;
    Transform raw;raw.yaw=std::remainder(yaw-m.yaw,2*M_PI);raw.stamp=m.t;raw.distance=travel_;raw.valid=true;
    const auto relative=raw.apply(m.x,m.y);raw.x=x-relative[0];raw.y=y-relative[1];
    if (blocked_) {
      // A failed recovery must first stop. Moving GPS cannot silently start a
      // new anchor and thereby release a previously invalidated trajectory.
      if (std::abs(m.v)>.1) {fixes_.clear();window_start_=0;reason="reanchor_wait_stopped";return false;}
      if (gap>.3) {fixes_.clear();window_start_=0;}
      collect(raw);
      if (!ready_window()) {reason="stopped_reanchor_warmup";return false;}
      blocked_=false;state="gps_anchored";reason="stopped_reanchor_ready";
      auto q=mean();q.source=state;remember(q);return true;
    }
    if (history_.empty()) {
      if (gap>.3) {fixes_.clear();window_start_=0;}
      collect(raw);
      if (!ready_window()) {state="warming";reason="initial_anchor_warmup";return false;}
      auto q=mean();q.source=state="gps_anchored";reason="anchor_ready";remember(q);return true;
    }
    auto prior=value_at(history_.back(),m.t);const auto expected=prior.apply(m.x,m.y);
    innovation_m=std::hypot(expected[0]-x,expected[1]-y);
    raw_innovation_m=innovation_m;
    innovation_rad=std::abs(std::remainder(raw.yaw-prior.yaw,2*M_PI));
    if(validate_recovery_window&&gap<=.6&&state=="gps_anchored"&&innovation_m>.25) {
      // An isolated noisy tracking fix is not a validated pose jump. Retain
      // the last anchor and apply the same bounded candidate policy as after
      // a loss. Gross jumps still fail immediately inside recovery_window.
      fixes_.clear();window_start_=blend_start_=0;recovery_start_=m.t;state="recovery_verify";
    }
    if(validate_recovery_window&&(gap>.6||state=="recovery_verify"||state=="recovery_blend"))
      return recovery_window(m,raw,prior,gap);
    if (innovation_m>.25 || innovation_rad>2*M_PI/180) {
      return reject_recovery("gps_innovation_exceeded");
    }
    if (gap>.6) {
      // Loss during verification/blending restarts validation against the last
      // published transform. There is never an unvalidated correction jump.
      fixes_.clear();window_start_=0;blend_start_=0;state="recovery_verify";
    } else if (state=="recovery_verify" && gap>.3) {fixes_.clear();window_start_=0;}
    if (state=="recovery_verify") {
      collect(raw);reason="recovery_validating";
      if (ready_window(2)) {
        blend_from_=prior;blend_to_=mean();blend_start_=m.t;state="recovery_blend";reason="recovery_blending";
      }
      prior.stamp=m.t;prior.distance=travel_;prior.source=state;remember(prior);return true;
    }
    if (state=="recovery_blend") {
      const double u=std::clamp((m.t-blend_start_)/1.,0.,1.);
      Transform q=blend_from_;q.x+=u*(blend_to_.x-q.x);q.y+=u*(blend_to_.y-q.y);
      q.yaw+=u*std::remainder(blend_to_.yaw-q.yaw,2*M_PI);q.stamp=m.t;q.distance=travel_;
      if (u>=1.) {state="gps_anchored";fixes_.clear();window_start_=0;collect(raw);reason="recovery_complete";}
      q.source=state;remember(q);return true;
    }
    collect(raw);auto q=mean();q.source=state="gps_anchored";reason="gps_tracking";remember(q);return true;
  }
  Transform at(double t,std::string* failure=nullptr) const {
    const auto unavailable=[&](const std::string& why){if(failure)*failure=why;return Transform{};};
    if(!std::isfinite(t)||t<=0)return unavailable("anchor_query_time_invalid");
    if(blocked_)return unavailable(reason);
    if(history_.empty())return unavailable(reason);
    for(auto i=history_.rbegin();i!=history_.rend();++i)if(i->value.stamp<=t){
      if(t-i->validated_stamp>120.)return unavailable("anchor_time_limit");
      if(travel_-i->validated_distance>250.)return unavailable("anchor_distance_limit");
      auto q=value_at(*i,t);
      if(t-q.stamp>.6)q.source="dead_reckoning";
      if(failure)failure->clear();
      return q;
    }
    return unavailable("anchor_no_past_sample");
  }

 private:
  local_planning::Motion last_motion_;
  double travel_=0,last_fix_=0,window_start_=0,blend_start_=0,recovery_start_=0;
  double validated_stamp_=0,validated_distance_=0;
  bool blocked_=false;
  Transform blend_from_,blend_to_;
  struct History {
    Transform value,from,to;
    double blend_start=0;
    double validated_stamp=0,validated_distance=0;
  };
  std::deque<Transform> fixes_;
  std::deque<History> history_;
  static Transform value_at(const History& entry,double t) {
    auto q=entry.value;
    if(entry.blend_start>0&&t>=entry.blend_start) {
      // Only the already validated target is interpolated. A missing GPS
      // freezes progress after the existing 300ms source-freshness window;
      // neither future fixes nor later recovery targets can rewrite history.
      const double u=std::clamp((std::min(t,q.stamp+.3)-entry.blend_start)/1.,0.,1.);
      q.x=entry.from.x+u*(entry.to.x-entry.from.x);
      q.y=entry.from.y+u*(entry.to.y-entry.from.y);
      q.yaw=entry.from.yaw+u*std::remainder(entry.to.yaw-entry.from.yaw,2*M_PI);
    }
    return q;
  }
  bool reject_recovery(const std::string& why) {
    ++generation;blocked_=true;history_.clear();fixes_.clear();window_start_=blend_start_=0;
    state="reanchor_stop";reason=why;return false;
  }
  bool recovery_window(const local_planning::Motion& m,const Transform& raw,Transform prior,double gap) {
    if(gap>.6) {
      fixes_.clear();window_start_=blend_start_=0;recovery_start_=m.t;state="recovery_verify";
    } else if(state=="recovery_verify"&&gap>.3) {
      fixes_.clear();window_start_=0;
    }
    // A recovered fix never changes the DR anchor before one full second of
    // source-aligned fixes has been validated. Gross jumps stop immediately;
    // smaller noisy samples are judged as a candidate window, not as a new pose.
    // The accepted candidate still has the original strict 25cm / 2deg gate.
    // Once blending starts, the published anchor is already being corrected.
    // Judge new evidence against that current anchor, as in normal tracking;
    // the frozen DR origin is not a second recovery gate. The blend target
    // itself remains the originally validated <=25cm correction.
    const auto baseline=prior;
    const auto a=baseline.apply(m.x,m.y),b=raw.apply(m.x,m.y);
    raw_innovation_m=std::hypot(a[0]-b[0],a[1]-b[1]);
    if(raw_innovation_m>=2*.25||innovation_rad>2*M_PI/180)
      return reject_recovery("gps_innovation_exceeded");
    collect(raw);
    if(state=="recovery_verify"&&!ready_window(2)) {
      if(m.t-recovery_start_>1.5)return reject_recovery("gps_recovery_unstable");
      prior.stamp=m.t;prior.distance=travel_;prior.source=state;remember(prior);
      reason="recovery_validating";return true;
    }
    const auto candidate=mean();const auto c=candidate.apply(m.x,m.y);
    innovation_m=std::hypot(a[0]-c[0],a[1]-c[1]);
    innovation_rad=std::abs(std::remainder(candidate.yaw-baseline.yaw,2*M_PI));
    if(innovation_m>.25||innovation_rad>2*M_PI/180)
      return reject_recovery("gps_innovation_exceeded");
    if(state=="recovery_verify") {
      blend_from_=prior;blend_to_=candidate;blend_start_=m.t;state="recovery_blend";
    }
    const double u=std::clamp((m.t-blend_start_)/1.,0.,1.);
    Transform q=blend_from_;q.x+=u*(blend_to_.x-q.x);q.y+=u*(blend_to_.y-q.y);
    q.yaw+=u*std::remainder(blend_to_.yaw-q.yaw,2*M_PI);q.stamp=m.t;q.distance=travel_;
    reason="recovery_blending";
    if(u>=1.) {state="gps_anchored";fixes_.clear();window_start_=0;collect(raw);reason="recovery_complete";}
    q.source=state;remember(q);return true;
  }
  void collect(const Transform& q) {
    if (!fixes_.empty()) {
      const auto a=fixes_.back().apply(last_motion_.x,last_motion_.y);
      const auto b=q.apply(last_motion_.x,last_motion_.y);
      if (std::hypot(a[0]-b[0],a[1]-b[1])>.25 ||
          std::abs(std::remainder(q.yaw-fixes_.back().yaw,2*M_PI))>2*M_PI/180) {
        fixes_.clear();window_start_=0;
      }
    }
    if (fixes_.empty())window_start_=q.stamp;
    fixes_.push_back(q);
    // Initial/stopped anchoring requires BOTH elapsed time and ten fixes.
    // A one-second-only buffer can never hold ten fixes from the real 5Hz GPS.
    // After anchoring, retain the original one-second tracking/recovery window.
    const size_t minimum=(history_.empty() || blocked_)?10:1;
    while (fixes_.size()>256 || (fixes_.size()>minimum && q.stamp-fixes_[1].stamp>1.))fixes_.pop_front();
  }
  bool ready_window(size_t minimum=10) const {
    return fixes_.size()>=minimum && fixes_.back().stamp-window_start_>=1.-1e-9;
  }
  Transform mean() const {
    auto q=fixes_.back();q.x=q.y=0;double cs=0,sn=0;
    for(const auto& f:fixes_) {q.x+=f.x;q.y+=f.y;cs+=std::cos(f.yaw);sn+=std::sin(f.yaw);}
    q.x/=fixes_.size();q.y/=fixes_.size();q.yaw=std::atan2(sn,cs);return q;
  }
  void remember(const Transform& q) {
    History h;h.value=q;
    if(q.source=="gps_anchored") {validated_stamp_=q.stamp;validated_distance_=q.distance;}
    h.validated_stamp=validated_stamp_;h.validated_distance=validated_distance_;
    if(q.source=="recovery_blend") {h.from=blend_from_;h.to=blend_to_;h.blend_start=blend_start_;}
    history_.push_back(h);while(history_.size()>512)history_.pop_front();
  }
};
} // namespace shadow_localization
