#include "Signal/SignalPolicy.hpp"

#include <algorithm>
#include <cmath>

namespace asmc_planner::signal {
namespace {
bool Finite(const Pose& p) {
  return p.valid && std::isfinite(p.stamp) && std::isfinite(p.x) &&
      std::isfinite(p.y) && std::isfinite(p.yaw) && std::isfinite(p.speed) &&
      std::isfinite(p.acceleration) && p.speed>=0;
}
bool LineValid(const Geometry& g) {
  return g.reviewed && g.key.valid() && std::isfinite(g.line.x) &&
      std::isfinite(g.line.y) && std::isfinite(g.approach.x) &&
      std::isfinite(g.approach.y) &&
      std::abs(std::hypot(g.approach.x,g.approach.y)-1)<1e-5;
}
bool ShapeValid(const Config& c) {
  return std::isfinite(c.front_extent) && c.front_extent>=0 &&
      std::isfinite(c.rear_extent) && c.rear_extent>=0 &&
      std::isfinite(c.half_width) && c.half_width>0 &&
      std::isfinite(c.position_error) && c.position_error>=0 &&
      std::isfinite(c.clearance) && c.clearance>=0;
}
double Before(Point p, Point line, Point direction) {
  return (line.x-p.x)*direction.x+(line.y-p.y)*direction.y;
}
Point Transform(const Pose& p, double longitudinal, double lateral) {
  return {p.x+longitudinal*std::cos(p.yaw)-lateral*std::sin(p.yaw),
          p.y+longitudinal*std::sin(p.yaw)+lateral*std::cos(p.yaw)};
}
std::array<Point,4> Corners(const Pose& p, const Config& c) {
  return {Transform(p,c.front_extent+c.position_error,c.half_width+c.position_error),
          Transform(p,c.front_extent+c.position_error,-c.half_width-c.position_error),
          Transform(p,-c.rear_extent-c.position_error,-c.half_width-c.position_error),
          Transform(p,-c.rear_extent-c.position_error,c.half_width+c.position_error)};
}
bool Inside(Point p, const std::vector<Point>& polygon) {
  bool inside=false;
  for (std::size_t i=0,j=polygon.size()-1; i<polygon.size(); j=i++) {
    const auto a=polygon[i], b=polygon[j];
    const double cross=(p.x-a.x)*(b.y-a.y)-(p.y-a.y)*(b.x-a.x);
    if(std::abs(cross)<1e-9 && p.x>=std::min(a.x,b.x)-1e-9 &&
       p.x<=std::max(a.x,b.x)+1e-9 && p.y>=std::min(a.y,b.y)-1e-9 &&
       p.y<=std::max(a.y,b.y)+1e-9) return true;
    if ((a.y>p.y)!=(b.y>p.y) && p.x<(b.x-a.x)*(p.y-a.y)/(b.y-a.y)+a.x)
      inside=!inside;
  }
  return inside;
}
double Cross(Point a, Point b, Point p) {
  return (b.x-a.x)*(p.y-a.y)-(b.y-a.y)*(p.x-a.x);
}
bool Intersects(Point a,Point b,Point c,Point d) {
  return std::max(std::min(a.x,b.x),std::min(c.x,d.x))<=
             std::min(std::max(a.x,b.x),std::max(c.x,d.x))+1e-9 &&
      std::max(std::min(a.y,b.y),std::min(c.y,d.y))<=
             std::min(std::max(a.y,b.y),std::max(c.y,d.y))+1e-9 &&
      Cross(a,b,c)*Cross(a,b,d)<=0 && Cross(c,d,a)*Cross(c,d,b)<=0;
}
bool IsClearing(Motion m) {
  return m==Motion::NORMAL_CLEAR || m==Motion::RECOVERY_CLEAR || m==Motion::EXITED ||
      m==Motion::ENTRY_PENDING_CLEAR || m==Motion::GREEN_CROSSING_CLEAR;
}
}  // namespace

void PoseHistory::Push(const Pose& p) {
  if(!Finite(p) || p.stamp<=0) return;
  if(!poses_.empty() && p.stamp<poses_.back().stamp) poses_.clear();
  if(!poses_.empty() && p.stamp==poses_.back().stamp) return;
  poses_.push_back(p);
  while(!poses_.empty() && p.stamp-poses_.front().stamp>config_.history_seconds)
    poses_.pop_front();
}
bool PoseHistory::At(double stamp, Pose& out) const {
  if(!std::isfinite(stamp)) return false;
  const auto next=std::lower_bound(poses_.begin(),poses_.end(),stamp,
      [](const Pose& p,double t){return p.stamp<t;});
  if(next==poses_.end()) return false;
  if(next->stamp==stamp) { out=*next;return true; }
  if(next==poses_.begin()) return false;
  const auto& a=*std::prev(next); const auto& b=*next;
  if(b.stamp-a.stamp>config_.interpolation_gap) return false;
  const double f=(stamp-a.stamp)/(b.stamp-a.stamp);
  out=a;out.stamp=stamp;
  out.x+=f*(b.x-a.x);out.y+=f*(b.y-a.y);
  out.yaw+=f*std::remainder(b.yaw-a.yaw,2*std::acos(-1.0));
  out.speed+=f*(b.speed-a.speed);out.acceleration+=f*(b.acceleration-a.acceleration);
  out.route_s+=f*(b.route_s-a.route_s);
  return true;
}
bool PoseHistory::Covers(double begin,double end) const {
  Pose first,last;
  if(begin<=0 || end<begin || !At(begin,first) || !At(end,last))return false;
  double previous=begin;
  for(const auto& p:poses_)if(p.stamp>begin && p.stamp<=end) {
    if(p.stamp-previous>config_.interpolation_gap)return false;
    previous=p.stamp;
  }
  return end-previous<=config_.interpolation_gap;
}
bool PoseHistory::MaxQualificationSpeed(double begin,double end,double horizon,double& speed) const {
  if(!std::isfinite(horizon) || horizon<0 || !Covers(begin,end))return false;
  Pose first,last;if(!At(begin,first) || !At(end,last))return false;
  const auto qualified=[&](const Pose& p){return p.speed+std::max(0.,p.acceleration)*horizon;};
  speed=std::max(qualified(first),qualified(last));
  for(const auto& p:poses_)if(begin<p.stamp && p.stamp<end)speed=std::max(speed,qualified(p));
  return std::isfinite(speed);
}

Distance FrontWheelDistance(const Pose& p,const Geometry& g,const Config& c) {
  Distance d;
  if(!Finite(p) || !LineValid(g) || !ShapeValid(c) || !std::isfinite(c.wheelbase) ||
     c.wheelbase<=0 || !std::isfinite(c.half_track) ||
     c.half_track<=0 || c.half_track>c.half_width)
    return d;
  const double left=Before(Transform(p,c.wheelbase,c.half_track),g.line,g.approach);
  const double right=Before(Transform(p,c.wheelbase,-c.half_track),g.line,g.approach);
  d.min=std::min(left,right)-c.position_error;
  d.max=std::max(left,right)+c.position_error;d.valid=true;
  return d;
}
bool BodyBeforeLine(const Pose& p,const Geometry& g,const Config& c) {
  if(!Finite(p) || !LineValid(g) || !ShapeValid(c)) return false;
  for(auto corner:Corners(p,c)) if(Before(corner,g.line,g.approach)<c.clearance-1e-8) return false;
  return true;
}
bool BodyOutsideExit(const Pose& p,const Geometry& g,const Config& c) {
  if(!Finite(p) || !ShapeValid(c) || !g.conflict_reviewed ||
     !std::isfinite(g.exit.x) || !std::isfinite(g.exit.y) ||
     !std::isfinite(g.exit_direction.x) || !std::isfinite(g.exit_direction.y) ||
     std::abs(std::hypot(g.exit_direction.x,g.exit_direction.y)-1)>1e-5) return false;
  for(auto corner:Corners(p,c)) if(Before(corner,g.exit,g.exit_direction)>=0) return false;
  return true;
}
bool BodyOccupiesConflict(const Pose& p,const Geometry& g,const Config& c) {
  if(!Finite(p) || !ShapeValid(c) || !g.conflict_reviewed || g.conflict_polygon.size()<3) return false;
  for(auto point:g.conflict_polygon) if(!std::isfinite(point.x) || !std::isfinite(point.y))return false;
  const auto body=Corners(p,c);
  const std::vector<Point> rectangle(body.begin(),body.end());
  for(auto point:body) if(Inside(point,g.conflict_polygon)) return true;
  for(auto point:g.conflict_polygon) if(Inside(point,rectangle)) return true;
  for(std::size_t i=0;i<body.size();++i)
    for(std::size_t j=0;j<g.conflict_polygon.size();++j)
      if(Intersects(body[i],body[(i+1)%body.size()],g.conflict_polygon[j],
                    g.conflict_polygon[(j+1)%g.conflict_polygon.size()])) return true;
  return false;
}
bool BodyDefinitelyOccupiesConflict(const Pose& p,const Geometry& g,const Config& c) {
  if(!ShapeValid(c) || c.half_width<=c.position_error || c.front_extent<=c.position_error)return false;
  auto inner=c;
  inner.front_extent-=c.position_error;
  inner.rear_extent=std::max(0.,c.rear_extent-c.position_error);
  inner.half_width-=c.position_error;inner.position_error=0;
  return BodyOccupiesConflict(p,g,inner);
}
StopTarget SelectLateTarget(const Geometry& g,const Config& c,
    const std::function<bool(double,Pose&)>& pose_at_s,double nominal_offset) {
  StopTarget out;
  if(!LineValid(g) || !std::isfinite(g.route_s)) return out;
  const double straight=c.front_extent+c.clearance;
  // Scan all candidates instead of assuming clearance is monotone on a curve.
  for(double offset=straight;offset<=nominal_offset+1e-6;offset+=.005) {
    Pose candidate;
    if(pose_at_s(g.route_s-offset,candidate) && BodyBeforeLine(candidate,g,c)) {
      out.valid=true;out.offset=offset;out.pose=candidate;return out;
    }
  }
  return out;
}

void GreenGate::Revoke() {
  count_=0;color_=Color::UNKNOWN;++revocations_;
}
void GreenGate::Bind(const Key& k) {
  if(k==key_) return;
  key_=k;Revoke();epoch_=0;stamp_=received_=last_green_=first_stop_=invalid_stamp_=0;
  invalid_received_=0;
  frame_.clear();
}
void GreenGate::Tick(double ros_now,double steady_now) {
  if(!std::isfinite(ros_now) || !std::isfinite(steady_now) || stamp_<=0 ||
     ros_now<stamp_ || ros_now-stamp_>config_.freshness ||
     steady_now<received_ || steady_now-received_>config_.freshness) {
    if(count_ || color_!=Color::UNKNOWN) Revoke();
  }
}
void GreenGate::Observe(const Observation& o,double ros_now,double steady_now) {
  // A tombstone is actionable even when it carries the same source header.
  if(!o.valid || o.stale || o.color==Color::UNKNOWN) {
    Revoke();
    if(std::isfinite(o.received))invalid_received_=std::max(invalid_received_,o.received);
    if(epoch_==0 || o.source_epoch>epoch_) {
      epoch_=o.source_epoch;frame_=o.frame;stamp_=received_=invalid_stamp_=0;
      last_green_=first_stop_=0;
    }
    if(o.source_epoch==epoch_ && o.frame==frame_) invalid_stamp_=std::max(invalid_stamp_,o.stamp);
    if(o.stamp>0 && (!first_stop_ || o.stamp<first_stop_)) first_stop_=o.stamp;
    return;
  }
  if(o.color==Color::GREEN &&
     (o.received<=invalid_received_ || o.source_epoch<epoch_ || (o.source_epoch==epoch_ && o.frame==frame_ &&
       (o.stamp<=stamp_ || o.stamp<=invalid_stamp_)))) return;
  if(!o.clock_verified || !o.association_valid || o.source_epoch==0 || o.key!=key_ || !key_.valid() ||
     !std::isfinite(o.stamp) || o.stamp<=0 || !std::isfinite(o.received) ||
     !std::isfinite(ros_now) || !std::isfinite(steady_now) || ros_now<o.stamp ||
     ros_now-o.stamp>config_.freshness || steady_now<o.received ||
     steady_now-o.received>config_.freshness) { Revoke();return; }
  if(o.source_epoch!=epoch_ || o.frame!=frame_) {
    if(o.source_epoch<epoch_) { Revoke();return; }
    Revoke();epoch_=o.source_epoch;frame_=o.frame;stamp_=received_=invalid_stamp_=0;
    last_green_=first_stop_=0;
  }
  // Negative aspects revoke even if a producer reuses a header incorrectly.
  if(o.color==Color::YELLOW || o.color==Color::RED) {
    Revoke();color_=o.color;invalid_stamp_=std::max(invalid_stamp_,o.stamp);
    invalid_received_=std::max(invalid_received_,o.received);
    if(!first_stop_ || o.stamp<first_stop_) first_stop_=o.stamp;
  }
  if(o.stamp<=stamp_ || o.stamp<=invalid_stamp_) return;
  if((stamp_>0 && o.stamp-stamp_>config_.max_gap) ||
     (received_>0 && o.received-received_>config_.max_gap)) Revoke();
  stamp_=o.stamp;received_=o.received;color_=o.color;
  if(o.color==Color::GREEN) { count_=std::min(3,count_+1);last_green_=o.stamp; }
}

Severity StopCauses::Final() const {
  return *std::max_element(causes_.begin(),causes_.end());
}
bool StopCauses::OthersClear() const {
  for(unsigned i=1;i<causes_.size();++i) if(causes_[i]!=Severity::NONE) return false;
  return true;
}
Policy::Policy(Config c):config_(c),gate_(c) {}
void Policy::Bind(const Geometry& g) {
  if(g.key==geometry_.key) return;  // Ordinary local replanning is not an epoch.
  geometry_=g;gate_.Bind(g.key);motion_=Motion::APPROACH;
  revision_=0;target_=std::numeric_limits<double>::quiet_NaN();previous_stamp_=0;
  brake_started_=crossed_=uncertain_=normal_=target_failed_=false;
  emergency_latched_=false;
  signal_brake_started_=false;
  braking_at_crossing_=braking_cross_recorded_=false;
  entry_pending_=crossing_ordered_=false;
  green_crossing_committed_=false;
  crossing_after_confirmed_=false;
  recovery_committed_=false;
  green_reentry_committed_=false;
  previous_authorized_=false;stopped_since_=-1;response_=ResponseMonitor{};
  entry_error_=config_.position_error;previous_entry_qualification_speed_=INFINITY;
  entry_window_started_=false;
  last_before_stamp_=0;
  previous_revocations_=gate_.revocations();
}
void Policy::Observe(const Observation& o,double ros_now,double steady_now,const PoseHistory* history) {
  gate_.Observe(o,ros_now,steady_now);
  gate_.Tick(ros_now,steady_now);
  // Classify each delivered original in order. A later red in the SAME
  // Behavior batch cannot erase a green image that already proves passage.
  // Conversely, a red BEFORE that proof invalidates the permission chain.
  if(!history || normal_ || uncertain_ || target_failed_ || signal_brake_started_ ||
     !o.valid || o.stale || o.color!=Color::GREEN || !o.clock_verified || !o.association_valid ||
     o.key!=geometry_.key || !gate_.authorized() || o.stamp!=gate_.last_green() ||
     !previous_authorized_ || previous_stamp_<=0 || last_before_stamp_<=0 ||
     (!crossed_ && previous_distance_<=0) ||
     gate_.revocations()!=(entry_pending_ ? crossing_revocations_ : previous_revocations_))return;
  Pose capture;double speed=0;
  if(!history->At(o.stamp,capture) || !history->MaxQualificationSpeed(last_before_stamp_,o.stamp,
      config_.interpolation_gap,speed))return;
  const bool qualified=config_.entry_observation_error>=0 && config_.entry_observation_speed_max>0 &&
      speed<=config_.entry_observation_speed_max;
  const double error=std::max(entry_error_,qualified ?
      std::min(config_.position_error,config_.entry_observation_error) : config_.position_error);
  auto config=config_;config.position_error=error;
  const auto distance=FrontWheelDistance(capture,geometry_,config);
  if(!distance.valid || distance.max>0)return;
  crossed_=normal_=crossing_after_confirmed_=true;entry_pending_=false;
  crossed_begin_=last_before_stamp_;crossed_end_=o.stamp;entry_error_=error;
  Record(EventKind::NORMAL_ENTRY_CONFIRMED,std::max(previous_stamp_,o.stamp),crossed_begin_,crossed_end_);
}
void Policy::SetTarget(double offset,bool late) {
  if(!std::isfinite(offset)) return;
  if(!std::isfinite(target_) || std::abs(target_-offset)>1e-6 || late_!=late) {
    target_=offset;late_=late;++revision_;++virtual_id_;
    // Deliberately retain braking and stationary state and response deadline.
  }
}
void Policy::Record(EventKind kind,double stamp,double begin,double end) {
  events_.push_back({geometry_.key,kind,stamp,begin,end});
}
bool Policy::RecoveryPositionEligible(const Pose& p,bool continuous_history) const {
  return BodyDefinitelyOccupiesConflict(p,geometry_,config_) ||
      (green_crossing_committed_ && continuous_history &&
       !BodyBeforeLine(p,geometry_,config_) && !BodyOutsideExit(p,geometry_,config_)) ||
      (recovery_committed_ && continuous_history &&
       !BodyBeforeLine(p,geometry_,config_) && !BodyOutsideExit(p,geometry_,config_));
}
Output Policy::Step(const Pose& p,double ros_now,double steady_now,
                    const Evidence& evidence,StopCauses causes) {
  gate_.Tick(ros_now,steady_now);
  const auto decision_state=ProjectMeasuredState(p.speed,p.acceleration,evidence.state_age);
  const auto distance=FrontWheelDistance(p,geometry_,config_);
  auto nominal_config=config_;nominal_config.position_error=0;
  const auto nominal=FrontWheelDistance(p,geometry_,nominal_config);
  const bool position_ok=Finite(p) && evidence.position_valid && LineValid(geometry_) &&
      decision_state.valid && distance.valid;
  const auto entry_error_for=[&](double speed) {
    const bool qualified=std::isfinite(config_.entry_observation_error) &&
        config_.entry_observation_error>=0 && config_.entry_observation_speed_max>0 &&
        std::isfinite(speed) && speed<=config_.entry_observation_speed_max;
    return qualified ? std::min(config_.position_error,config_.entry_observation_error) : config_.position_error;
  };
  const double qualification_speed=position_ok ? std::max(p.speed,decision_state.speed)+
      std::max(0.,p.acceleration)*config_.interpolation_gap : INFINITY;
  if(!crossed_) {
    const double current_error=entry_error_for(qualification_speed);
    if(entry_window_started_)entry_error_=std::max(entry_error_,current_error);
    else entry_error_=current_error;
    // The possible crossing interval begins before the nominal zero crossing.
    // Slowing within that interval must not narrow earlier position evidence.
    if(position_ok && nominal.valid && nominal.min<=config_.position_error)
      entry_window_started_=true;
  }
  if(recovery_committed_ && (!position_ok || !evidence.position_history_continuous) &&
     !evidence.localization_continuity_verified)
    recovery_committed_=false;
  if(!position_ok || !evidence.position_history_continuous || !gate_.authorized())
    green_reentry_committed_=false;
  if(!position_ok || !evidence.risks_fresh)
    causes.Set(Cause::INPUT_INVALID,Severity::EMERGENCY);
  if(position_ok && distance.valid && previous_stamp_>0 && !crossed_ &&
     previous_distance_>0 && nominal.valid && nominal.min<=0) {
    crossed_=true;crossed_begin_=last_before_stamp_>0?last_before_stamp_:previous_stamp_;crossed_end_=p.stamp;
    entry_error_=std::max(entry_error_,entry_error_for(previous_entry_qualification_speed_));
    braking_at_crossing_=signal_brake_started_;
    const bool ordered=p.stamp>previous_stamp_ &&
        (p.stamp-previous_stamp_<=config_.interpolation_gap+1e-9 || evidence.position_history_continuous);
    crossing_ordered_=ordered;crossing_revocations_=gate_.revocations();
    entry_pending_=ordered && previous_authorized_ && gate_.authorized() && !signal_brake_started_ &&
        gate_.revocations()==previous_revocations_;
    // The release was in force when the front wheel crossed. A later image
    // may make the event uncertain, but cannot reapply this signal's brake.
    green_crossing_committed_=entry_pending_ && evidence.position_history_continuous &&
        !brake_started_ && evidence.risks_fresh && causes.OthersClear();
    if(!entry_pending_) {
      uncertain_=true;Record(EventKind::ENTRY_UNCERTAIN,p.stamp,crossed_begin_,crossed_end_);
    }
  }
  if(entry_pending_) {
    const double required_error=entry_error_for(qualification_speed);
    if(required_error>entry_error_) {
      entry_error_=required_error;crossing_after_confirmed_=false;
    }
    auto entry_config=config_;entry_config.position_error=entry_error_;
    const auto entry_distance=FrontWheelDistance(p,geometry_,entry_config);
    if(!crossing_after_confirmed_) {
      crossed_end_=p.stamp;
      crossing_after_confirmed_=entry_distance.valid && entry_distance.max<=0;
    }
    // The most recent vehicle sample is often newer than the original image.
    // A verified pose AT that image can already prove passage; requiring an
    // additional image after the later sample introduces an artificial delay
    // and can strand a lawful entrant when the light leaves camera view.
    const auto& capture=evidence.green_capture_pose;
    if(position_ok && evidence.position_history_continuous && evidence.green_capture_pose_valid &&
       Finite(capture) && capture.stamp==gate_.last_green() &&
       capture.stamp>=crossed_begin_ && capture.stamp<=p.stamp) {
      const auto captured_distance=FrontWheelDistance(capture,geometry_,entry_config);
      if(captured_distance.valid && captured_distance.max<=0) {
        crossing_after_confirmed_=true;
        crossed_end_=capture.stamp;
      }
    }
    // Camera inference normally arrives after the latest pose sample. Wait
    // for a distinct original observation covering the crossing, rather than
    // permanently labelling every green passage uncertain on its first tick.
    // No recorded uncertain/overrun event is ever reclassified or deleted.
    if(!gate_.authorized() || gate_.revocations()!=crossing_revocations_) {
      entry_pending_=false;uncertain_=true;
      Record(EventKind::ENTRY_UNCERTAIN,p.stamp,crossed_begin_,p.stamp);
    } else if(position_ok && crossing_after_confirmed_ && gate_.last_green()>=crossed_end_ && crossing_ordered_) {
      entry_pending_=false;normal_=true;
      Record(EventKind::NORMAL_ENTRY_CONFIRMED,p.stamp,crossed_begin_,crossed_end_);
    }
  }
  // Starting/repositioning past the line supplies no proof of normal entry.
  if(position_ok && distance.valid && !crossed_ && previous_stamp_==0 && distance.min<=0) {
    crossed_=uncertain_=true;Record(EventKind::ENTRY_UNCERTAIN,p.stamp,p.stamp,p.stamp);
  }
  if(position_ok && distance.max<=0 && braking_at_crossing_ && !braking_cross_recorded_) {
    braking_cross_recorded_=true;
    Record(EventKind::STOP_LINE_CROSS_DURING_BRAKING,p.stamp,crossed_begin_,p.stamp);
  }
  const double measured_remaining=geometry_.route_s-p.route_s-target_;
  const double remaining=measured_remaining-decision_state.distance;
  if(position_ok && signal_brake_started_ && !recovery_committed_ &&
     std::isfinite(measured_remaining) && measured_remaining<-.02 && !target_failed_) {
    target_failed_=true;Record(EventKind::TARGET_OVERRUN,p.stamp,p.stamp,p.stamp);
  }
  const bool stationary=position_ok && p.speed<=config_.standstill_speed &&
      decision_state.speed<=config_.standstill_speed;
  if(stationary) { if(stopped_since_<0) stopped_since_=steady_now; }
  else stopped_since_=-1;
  const bool stopped=stopped_since_>=0 && steady_now-stopped_since_>=config_.standstill_time;
  if(response_.Observe(steady_now,p.stamp,p.speed,p.acceleration,position_ok)==BrakeResponse::FAILED) {
    causes.Set(Cause::EXECUTION_FAILURE,Severity::EMERGENCY);
    if(!response_failed_) {
      response_failed_=true;Record(EventKind::RESPONSE_FAILURE,p.stamp,p.stamp,p.stamp);
    }
  }
  if(response_failed_) causes.Set(Cause::EXECUTION_FAILURE,Severity::EMERGENCY);
  if(evidence.execution_failed) {
    causes.Set(Cause::EXECUTION_FAILURE,Severity::EMERGENCY);
    if(evidence.execution_failure_mask==2) {
      if(!external_path_failed_)Record(EventKind::PATH_FAILURE,p.stamp,p.stamp,p.stamp);
      external_path_failed_=true;
    } else {
      if(!response_failed_)Record(EventKind::RESPONSE_FAILURE,p.stamp,p.stamp,p.stamp);
      response_failed_=true;
    }
  }
  if(evidence.planning_checked) {
    if(!evidence.stop_path_valid || !evidence.path_goal_satisfied) {
      if(!path_failed_) Record(EventKind::PATH_FAILURE,p.stamp,p.stamp,p.stamp);
      path_failed_=true;
    } else if(position_ok && evidence.risks_fresh) path_failed_=false;
  }
  if(path_failed_ || external_path_failed_) causes.Set(Cause::EXECUTION_FAILURE,Severity::EMERGENCY);
  const bool inside=BodyDefinitelyOccupiesConflict(p,geometry_,config_);
  const bool conflict_known=evidence.conflict_known && geometry_.conflict_reviewed &&
      evidence.inside_conflict==inside;
  const bool recovery_position=RecoveryPositionEligible(p,evidence.position_history_continuous);
  bool recovered=position_ok && conflict_known && recovery_position &&
      evidence.exit_path_valid && evidence.risks_fresh && causes.OthersClear();
  const bool connected=evidence.linked && evidence.command_accepted &&
      evidence.trajectory_belongs_to_command && evidence.command_id!=0;
  const double delay=evidence.response_delay_known ? std::max(0.,evidence.response_delay_remaining) :
      response_.status()!=BrakeResponse::IDLE ? response_.remaining(steady_now) :
      ResponseBudget(config_.measured_max_delay);
  const double boundary=StopBoundaryAcceleration(decision_state.speed,p.acceleration,config_.brake);
  const double minimum_distance=std::max(
      MinimumStoppingDistance(decision_state.speed,p.acceleration,config_.brake,delay),
      MinimumStoppingDistance(decision_state.speed,boundary,config_.brake,delay));
  bool owner_recovery_accepted=false;
  if(evidence.execution_recovery_acknowledged && connected && position_ok &&
     evidence.risks_fresh && stopped && !path_failed_) {
    response_failed_=false;response_=ResponseMonitor{};
    external_path_failed_=false;
    owner_recovery_accepted=true;
    // Only this owner-acknowledged proof clears the internal failure cause.
    causes.Set(Cause::EXECUTION_FAILURE,Severity::NONE);
    recovered=position_ok && conflict_known && recovery_position && evidence.exit_path_valid &&
        evidence.risks_fresh && causes.OthersClear();
  }
  const Motion old_motion=motion_;
  if(!causes.OthersClear())green_reentry_committed_=false;
  if(position_ok && evidence.exited && BodyOutsideExit(p,geometry_,config_) &&
      (normal_ || crossed_)) motion_=Motion::EXITED;
  else if(normal_) motion_=Motion::NORMAL_CLEAR;
  else if(green_crossing_committed_ && position_ok && evidence.position_history_continuous &&
          !brake_started_) motion_=Motion::GREEN_CROSSING_CLEAR;
  else if(entry_pending_ && position_ok && gate_.authorized()) motion_=Motion::ENTRY_PENDING_CLEAR;
  else if(crossed_ || uncertain_) {
    if(recovered) motion_=connected ? Motion::RECOVERY_CLEAR : Motion::RECOVERY_CLEAR_PENDING;
    else if(position_ok && conflict_known && !inside &&
            gate_.authorized() && causes.OthersClear() &&
            ((stationary && (!emergency_latched_ || stopped)) ||
             (green_reentry_committed_ && connected && evidence.exit_path_valid)))
      motion_=Motion::RELEASE_PENDING;
    else motion_=Motion::RECOVERY_BRAKE;
  } else if(gate_.authorized() && causes.OthersClear()) {
    // A latched emergency finishes standing still before ordinary release.
    if(brake_started_ && (!emergency_latched_ || stopped))
      motion_=Motion::RELEASE_PENDING;
    else if(!brake_started_) motion_=Motion::APPROACH;
    else motion_=stationary ? Motion::STATIONARY_HOLD : Motion::BRAKING;
  } else if(!brake_started_ && !stationary && position_ok && evidence.target_geometry_valid &&
            causes.OthersClear() && remaining>minimum_distance+3.) {
    // STOP can approach its still-distant target without authorizing entry.
    // Start actual braking before the common stopping envelope + 3m margin.
    // Once BRAKING/HOLD began this branch is permanently closed until release.
    motion_=Motion::APPROACH;
  } else motion_=stationary ? Motion::STATIONARY_HOLD : Motion::BRAKING;
  if(motion_==Motion::RECOVERY_CLEAR)recovery_committed_=true;
  const bool signal_exempt=IsClearing(motion_) ||
      (green_crossing_committed_ && position_ok && evidence.position_history_continuous) ||
      (motion_==Motion::RELEASE_PENDING && connected &&
       (!(crossed_ || uncertain_) || evidence.exit_path_valid)) ||
      (!crossed_ && gate_.authorized() && !signal_brake_started_);
  causes.Set(Cause::SIGNAL,signal_exempt ? Severity::NONE : Severity::STOP);
  Output out;
  out.execution_recovery_acknowledged=owner_recovery_accepted;
  out.minimum_distance=minimum_distance;
  out.entry_observation_error=entry_error_;
  out.model_feasible=position_ok && evidence.target_geometry_valid && std::isfinite(remaining) &&
      out.minimum_distance<=remaining && !target_failed_ && !response_failed_;
  if(!signal_exempt && !stationary && !out.model_feasible)
    causes.Set(Cause::SIGNAL,Severity::EMERGENCY);
  const bool release_verified=connected && causes.OthersClear() &&
      (motion_==Motion::RECOVERY_CLEAR ||
       (((motion_==Motion::RELEASE_PENDING &&
          (!(crossed_ || uncertain_) || evidence.exit_path_valid)) || motion_==Motion::NORMAL_CLEAR) &&
        (!emergency_latched_ || stopped)));
  if(release_verified && motion_==Motion::RELEASE_PENDING && (crossed_ || uncertain_) &&
     evidence.position_history_continuous)green_reentry_committed_=true;
  Severity final_requirement=causes.Final();
  // Maintain the actuator release barrier independently of which hazard owns
  // the stop. A green collision stop must not invent a SIGNAL stop cause.
  if(emergency_latched_ && !release_verified) final_requirement=Severity::EMERGENCY;
  else if(final_requirement==Severity::NONE && brake_started_ && !release_verified)
    final_requirement=Severity::STOP;
  if(final_requirement!=Severity::NONE &&
     (motion_!=Motion::APPROACH || final_requirement==Severity::EMERGENCY)) {
    brake_started_=true;
    signal_brake_started_ |= causes.Get(Cause::SIGNAL)!=Severity::NONE;
    emergency_latched_ |= final_requirement==Severity::EMERGENCY;
    if(evidence.actuation_issued) response_.Begin(final_requirement==Severity::EMERGENCY ? 2 : 1,
        steady_now,p.speed,p.acceleration,ResponseBudget(config_.measured_max_delay),p.stamp);
  } else if(release_verified) {
    // An explicitly accepted green release ends active braking. A later legal
    // crossing is not mislabeled as braking overshoot; past events stay intact.
    brake_started_=false;signal_brake_started_=false;emergency_latched_=false;response_=ResponseMonitor{};
  }
  if(motion_!=old_motion) ++virtual_id_;
  out.motion=motion_;out.severity=final_requirement;out.signal_requirement=causes.Get(Cause::SIGNAL);
  for(unsigned i=0;i<out.causes.size();++i)out.causes[i]=causes.Get(static_cast<Cause>(i));
  out.release_barrier=final_requirement!=causes.Final();
  out.target_revision=revision_;
  out.virtual_command_id=virtual_id_;out.stationary=stationary;
  out.target_reached=position_ok && stationary && std::abs(measured_remaining)<=.7 &&
      BodyBeforeLine(p,geometry_,config_) && !target_failed_;
  out.entry_authorized=gate_.authorized() && position_ok && causes.OthersClear() && !crossed_;
  out.entry_pending=entry_pending_;
  // The pure policy cannot claim actuator authority; its adapter owns that.
  out.actual_control_allowed=false;
  if(position_ok && distance.valid) {
    if(!crossed_ && distance.min>0)last_before_stamp_=p.stamp;
    previous_stamp_=p.stamp;previous_distance_=nominal.min;
    previous_entry_qualification_speed_=qualification_speed;
    previous_authorized_=gate_.authorized();
    previous_revocations_=gate_.revocations();
  }
  return out;
}
const char* MotionName(Motion m) {
  switch(m) {
    case Motion::APPROACH:return "APPROACH";
    case Motion::BRAKING:return "BRAKING";
    case Motion::STATIONARY_HOLD:return "STATIONARY_HOLD";
    case Motion::RELEASE_PENDING:return "RELEASE_PENDING";
    case Motion::NORMAL_CLEAR:return "NORMAL_CLEAR";
    case Motion::RECOVERY_BRAKE:return "RECOVERY_BRAKE";
    case Motion::RECOVERY_CLEAR_PENDING:return "RECOVERY_CLEAR_PENDING";
    case Motion::RECOVERY_CLEAR:return "RECOVERY_CLEAR";
    case Motion::EXITED:return "EXITED";
    case Motion::ENTRY_PENDING_CLEAR:return "ENTRY_PENDING_CLEAR";
    case Motion::GREEN_CROSSING_CLEAR:return "GREEN_CROSSING_CLEAR";
  }
  return "UNKNOWN";
}
}  // namespace asmc_planner::signal
