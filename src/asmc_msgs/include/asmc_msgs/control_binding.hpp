#pragma once

// Explicit additive transport contract. Existing .msg definitions, header
// timestamps, frame IDs and diagnostic strings retain their original meaning.
// /behavior/control_request and /planning/control_plan use UInt8MultiArray
// with an empty layout and the versioned payload below. A plan contains the
// exact accepted request AND its trajectory atomically; no topic ordering or
// timestamp matching is used as evidence of command ownership.
#include <asmc_msgs/BehaviorCommand.h>
#include <asmc_msgs/FrenetFeedback.h>
#include <asmc_msgs/Trajectory.h>
#include <ros/serialization.h>
#include <std_msgs/UInt8MultiArray.h>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace asmc_msgs::binding {
constexpr std::uint32_t magic=0x41534d43;
constexpr std::uint8_t version=2;
constexpr std::size_t max_bytes=4*1024*1024;
enum class Kind : std::uint8_t { REQUEST=1, PLAN=2, STATUS=3 };

inline std::string NewEpoch() {
  std::array<unsigned char,16> bytes{};
  std::ifstream random("/dev/urandom",std::ios::binary);
  if(!random.read(reinterpret_cast<char*>(bytes.data()),bytes.size()))
    throw std::runtime_error("cannot create control binding epoch");
  std::ostringstream out;
  for(auto b:bytes)out<<std::hex<<std::setw(2)<<std::setfill('0')<<unsigned(b);
  return out.str();
}
struct Token {
  std::string controller_epoch, behavior_epoch;
  std::uint64_t revision=0;
  bool valid() const { return controller_epoch.size()==32 && behavior_epoch.size()==32 && revision>0; }
  bool operator==(const Token& b) const {
    return controller_epoch==b.controller_epoch && behavior_epoch==b.behavior_epoch && revision==b.revision;
  }
  bool operator!=(const Token& b) const { return !(*this==b); }
};
struct SignalStop {
  bool enabled=false;
  std::uint64_t route_generation=0,target_revision=0;
  std::string stop_line,movement;
  double line_x=0,line_y=0,normal_x=0,normal_y=0;
  double clearance=.5,position_error=.5;
  bool braking_only=false;
  bool recovery_geometry=false;
  double exit_x=0,exit_y=0,exit_normal_x=0,exit_normal_y=0;
};
struct Request {
  Token token;
  asmc_msgs::BehaviorCommand command;
  SignalStop signal;
  bool recovery_release=false;
  bool release_authorized=true;
  bool execution_recovery_ack=false;
  std::uint64_t brake_generation=0;
  std::uint8_t brake_level=0;
  double brake_decision_ros=0,brake_state_stamp=0,brake_speed=0,brake_acceleration=0,brake_budget=.5;
};
struct Plan {
  Request request;
  std::string planner_epoch;
  std::uint64_t sequence=0;
  asmc_msgs::FrenetFeedback feedback;
  asmc_msgs::Trajectory trajectory;
  bool goal_satisfied=false,exit_path_valid=false;
  bool stationary_hold=false;
  bool execution_recovery_safe=false;
  bool merge_waiting_path_valid=false;
};
struct Status {
  Token token;
  std::string planner_epoch;
  std::uint64_t plan_sequence=0,output_sequence=0;
  bool linked=false,planning_checked=false,stop_path_valid=false;
  bool goal_satisfied=false,exit_path_valid=false,actuated=false;
  bool execution_failed=false;
  std::uint8_t failure_mask=0;
  bool execution_recovery_safe=false;
  std::uint64_t brake_generation=0;
  std::uint8_t brake_level=0;
  bool response_observed=false;
  double response_remaining=0;
  double brake=0,steering=0,accel=0;
};

class Writer {
 public:
  std_msgs::UInt8MultiArray message;
  template<class T> void Put(const T& v) {
    const auto n=ros::serialization::serializationLength(v);
    const auto old=message.data.size();
    if(n>max_bytes || old>max_bytes-n)throw std::length_error("control packet too large");
    message.data.resize(old+n);
    ros::serialization::OStream stream(message.data.data()+old,n);
    ros::serialization::serialize(stream,v);
  }
};
class Reader {
 public:
  explicit Reader(const std_msgs::UInt8MultiArray& m):message(m) {
    if(m.data.size()>max_bytes || !m.layout.dim.empty() || m.layout.data_offset!=0)
      throw std::runtime_error("invalid control packet layout/size");
  }
  template<class T> void Get(T& v) {
    ros::serialization::IStream stream(const_cast<std::uint8_t*>(message.data.data()+offset),message.data.size()-offset);
    const auto before=stream.getLength();
    ros::serialization::deserialize(stream,v);
    offset+=before-stream.getLength();
  }
  void End() const { if(offset!=message.data.size())throw std::runtime_error("trailing control packet data"); }
 private:
  const std_msgs::UInt8MultiArray& message;
  std::size_t offset=0;
};
inline void PutRequest(Writer& w,const Request& r) {
  w.Put(r.token.controller_epoch);w.Put(r.token.behavior_epoch);w.Put(r.token.revision);
  w.Put(r.command);w.Put(r.signal.enabled);w.Put(r.signal.route_generation);w.Put(r.signal.target_revision);
  w.Put(r.signal.stop_line);w.Put(r.signal.movement);
  w.Put(r.signal.line_x);w.Put(r.signal.line_y);w.Put(r.signal.normal_x);w.Put(r.signal.normal_y);
  w.Put(r.signal.clearance);w.Put(r.signal.position_error);w.Put(r.signal.braking_only);w.Put(r.recovery_release);w.Put(r.release_authorized);
  w.Put(r.signal.recovery_geometry);w.Put(r.signal.exit_x);w.Put(r.signal.exit_y);w.Put(r.signal.exit_normal_x);w.Put(r.signal.exit_normal_y);
  w.Put(r.brake_generation);w.Put(r.brake_level);w.Put(r.brake_decision_ros);w.Put(r.brake_state_stamp);
  w.Put(r.brake_speed);w.Put(r.brake_acceleration);w.Put(r.brake_budget);
  w.Put(r.execution_recovery_ack);
}
inline void GetRequest(Reader& r,Request& v) {
  r.Get(v.token.controller_epoch);r.Get(v.token.behavior_epoch);r.Get(v.token.revision);
  r.Get(v.command);r.Get(v.signal.enabled);r.Get(v.signal.route_generation);r.Get(v.signal.target_revision);
  r.Get(v.signal.stop_line);r.Get(v.signal.movement);
  r.Get(v.signal.line_x);r.Get(v.signal.line_y);r.Get(v.signal.normal_x);r.Get(v.signal.normal_y);
  r.Get(v.signal.clearance);r.Get(v.signal.position_error);r.Get(v.signal.braking_only);r.Get(v.recovery_release);r.Get(v.release_authorized);
  r.Get(v.signal.recovery_geometry);r.Get(v.signal.exit_x);r.Get(v.signal.exit_y);r.Get(v.signal.exit_normal_x);r.Get(v.signal.exit_normal_y);
  r.Get(v.brake_generation);r.Get(v.brake_level);r.Get(v.brake_decision_ros);r.Get(v.brake_state_stamp);
  r.Get(v.brake_speed);r.Get(v.brake_acceleration);r.Get(v.brake_budget);
  r.Get(v.execution_recovery_ack);
}
inline void Prefix(Writer& w,Kind k) {w.Put(magic);w.Put(version);w.Put(static_cast<std::uint8_t>(k));}
inline void Prefix(Reader& r,Kind k) {
  std::uint32_t m=0;std::uint8_t v=0,t=0;r.Get(m);r.Get(v);r.Get(t);
  if(m!=magic || v!=version || t!=static_cast<std::uint8_t>(k))throw std::runtime_error("unknown control packet schema");
}
inline std_msgs::UInt8MultiArray Encode(const Request& r) {
  Writer w;Prefix(w,Kind::REQUEST);PutRequest(w,r);return std::move(w.message);
}
inline std_msgs::UInt8MultiArray Encode(const Plan& p) {
  Writer w;Prefix(w,Kind::PLAN);PutRequest(w,p.request);w.Put(p.planner_epoch);w.Put(p.sequence);
  w.Put(p.feedback);w.Put(p.trajectory);w.Put(p.goal_satisfied);w.Put(p.exit_path_valid);w.Put(p.stationary_hold);w.Put(p.execution_recovery_safe);w.Put(p.merge_waiting_path_valid);return std::move(w.message);
}
inline bool Decode(const std_msgs::UInt8MultiArray& m,Request& out) {
  try {Reader r(m);Request v;Prefix(r,Kind::REQUEST);GetRequest(r,v);r.End();out=std::move(v);return true;}catch(const std::exception&){return false;}
}
inline bool Decode(const std_msgs::UInt8MultiArray& m,Plan& out) {
  try {Reader r(m);Plan v;Prefix(r,Kind::PLAN);GetRequest(r,v.request);r.Get(v.planner_epoch);r.Get(v.sequence);
    r.Get(v.feedback);r.Get(v.trajectory);r.Get(v.goal_satisfied);r.Get(v.exit_path_valid);r.Get(v.stationary_hold);r.Get(v.execution_recovery_safe);r.Get(v.merge_waiting_path_valid);r.End();out=std::move(v);return true;
  }catch(const std::exception&){return false;}
}
inline std_msgs::UInt8MultiArray Encode(const Status& s) {
  Writer w;Prefix(w,Kind::STATUS);w.Put(s.token.controller_epoch);w.Put(s.token.behavior_epoch);w.Put(s.token.revision);
  w.Put(s.planner_epoch);w.Put(s.plan_sequence);w.Put(s.output_sequence);
  w.Put(s.linked);w.Put(s.planning_checked);w.Put(s.stop_path_valid);w.Put(s.goal_satisfied);w.Put(s.exit_path_valid);
  w.Put(s.actuated);w.Put(s.execution_failed);w.Put(s.failure_mask);w.Put(s.execution_recovery_safe);
  w.Put(s.brake_generation);w.Put(s.brake_level);w.Put(s.response_observed);w.Put(s.response_remaining);
  w.Put(s.brake);w.Put(s.steering);w.Put(s.accel);return std::move(w.message);
}
inline bool Decode(const std_msgs::UInt8MultiArray& m,Status& out) {
  try {Reader r(m);Status s;Prefix(r,Kind::STATUS);r.Get(s.token.controller_epoch);r.Get(s.token.behavior_epoch);r.Get(s.token.revision);
    r.Get(s.planner_epoch);r.Get(s.plan_sequence);r.Get(s.output_sequence);
    r.Get(s.linked);r.Get(s.planning_checked);r.Get(s.stop_path_valid);r.Get(s.goal_satisfied);r.Get(s.exit_path_valid);
    r.Get(s.actuated);r.Get(s.execution_failed);r.Get(s.failure_mask);r.Get(s.execution_recovery_safe);
    r.Get(s.brake_generation);r.Get(s.brake_level);r.Get(s.response_observed);r.Get(s.response_remaining);
    r.Get(s.brake);r.Get(s.steering);r.Get(s.accel);r.End();out=std::move(s);return true;
  }catch(const std::exception&){return false;}
}
inline std::vector<std::uint8_t> SemanticBytes(Request r) {
  r.token={};r.command.header.stamp={};r.command.header.seq=0;r.command.valid_until={};r.command.reason.clear();
  Writer w;PutRequest(w,r);return std::move(w.message.data);
}
inline bool SameRequest(const Request& a,const Request& b) {
  return a.token==b.token && SemanticBytes(a)==SemanticBytes(b);
}
inline bool CertifiedMergeWaitingPath(const Plan& p) {
  const auto& c=p.request.command;const auto& f=p.feedback;
  if(!p.merge_waiting_path_valid || c.maneuver_type!=asmc_msgs::BehaviorCommand::MERGE ||
     c.merge_type!=asmc_msgs::BehaviorCommand::MERGE_ROUNDABOUT ||
     c.execution_request!=asmc_msgs::BehaviorCommand::PREPARE ||
     p.request.signal.enabled || p.request.recovery_release ||
     (f.execution_state!=asmc_msgs::FrenetFeedback::PREPARING &&
      f.execution_state!=asmc_msgs::FrenetFeedback::WAITING) ||
     f.trajectory_source!=asmc_msgs::FrenetFeedback::TRAJECTORY_NEW ||
     p.trajectory.points.empty() || std::abs(p.trajectory.points.back().velocity)>.01)return false;
  for(const auto& point:p.trajectory.points)
    if(!std::isfinite(point.velocity) || point.velocity<-.01)return false;
  return true;
}

// V2 AVOID PREPARE publishes a freshly collision/road-checked maintenance
// trajectory as NEW/RESULT_NONE even when the lateral candidate is WAITING.
// Candidate readiness and this stopping trajectory are different outcomes.
// No new wire field is needed: these existing producer fields identify that
// path unambiguously; failed/reused/committed paths do not match this contract.
inline bool CertifiedAvoidWaitingPath(const Plan& p) {
  const auto& c=p.request.command;const auto& f=p.feedback;
  if(c.maneuver_type!=asmc_msgs::BehaviorCommand::AVOID ||
     (c.execution_request!=asmc_msgs::BehaviorCommand::PREPARE &&
      c.execution_request!=asmc_msgs::BehaviorCommand::EXECUTE) ||
     p.request.signal.enabled || p.request.recovery_release ||
     p.request.brake_level!=0 ||
     (f.execution_state!=asmc_msgs::FrenetFeedback::PREPARING &&
      f.execution_state!=asmc_msgs::FrenetFeedback::WAITING) ||
     f.result_code!=asmc_msgs::FrenetFeedback::RESULT_NONE ||
     f.trajectory_source!=asmc_msgs::FrenetFeedback::TRAJECTORY_NEW ||
     !p.trajectory.valid || p.trajectory.failure_reason!=0 ||
     p.trajectory.points.size()<2 ||
     !std::isfinite(p.trajectory.points.back().velocity) ||
     std::abs(p.trajectory.points.back().velocity)>.01)return false;
  for(const auto& point:p.trajectory.points)
    if(!std::isfinite(point.velocity) || point.velocity<0)return false;
  return true;
}

// Reusable consumer state machine. Freshness/road/dynamics are independent
// checks performed by the Controller; they never replace this identity proof.
class Gate {
 public:
  Gate():epoch_(NewEpoch()){}
  explicit Gate(std::string epoch):epoch_(std::move(epoch)){}
  const std::string& epoch() const {return epoch_;}
  bool RequestReceived(const Request& r) {
    if(!r.token.valid() || r.token.controller_epoch!=epoch_ || retired_.count(r.token.behavior_epoch))return false;
    if(have_request_) {
      if(r.token.behavior_epoch==request_.token.behavior_epoch) {
        if(r.token.revision<request_.token.revision)return false;
        if(r.token.revision==request_.token.revision) {
          if(!SameRequest(r,request_))return false;
          request_.command.header=r.command.header;request_.command.valid_until=r.command.valid_until;
          return true;  // Heartbeat: never resets the link or plan sequence.
        }
      } else retired_.insert(request_.token.behavior_epoch);
    }
    request_=r;have_request_=true;linked_=false;
    if(have_pending_)TryLink(pending_);
    return true;
  }
  bool PlanReceived(const Plan& p) {
    if(p.request.token.controller_epoch!=epoch_ || !p.request.token.valid() ||
       p.planner_epoch.size()!=32 || !p.sequence || retired_planners_.count(p.planner_epoch))return false;
    if(planner_epoch_==p.planner_epoch && p.sequence<=last_sequence_)return false;
    if(!planner_epoch_.empty() && planner_epoch_!=p.planner_epoch)retired_planners_.insert(planner_epoch_);
    planner_epoch_=p.planner_epoch;last_sequence_=p.sequence;
    pending_=p;have_pending_=true;
    return TryLink(p);
  }
  bool linked() const {return linked_;}
  bool has_request() const {return have_request_;}
  const Request& request() const {return request_;}
  const Plan& plan() const {return plan_;}
  const Plan& pending() const {return pending_;}
  void Invalidate() {linked_=false;}
 private:
  bool TryLink(const Plan& p) {
    if(!have_request_ || !SameRequest(request_,p.request))return false;
    linked_=false;
    // An accepted request must also be the actual planning owner. Rejected
    // requests/old locked paths cannot be relabelled with the latest token.
    if(!p.feedback.command_accepted || p.feedback.command_id!=request_.command.command_id ||
       p.feedback.active_maneuver!=request_.command.maneuver_type ||
       (!p.feedback.plan_valid && !p.stationary_hold && !CertifiedMergeWaitingPath(p) &&
        !CertifiedAvoidWaitingPath(p)) || !p.trajectory.valid || p.trajectory.failure_reason!=0 ||
       p.trajectory.points.empty() ||
       (p.feedback.trajectory_source!=asmc_msgs::FrenetFeedback::TRAJECTORY_NEW &&
        p.feedback.trajectory_source!=asmc_msgs::FrenetFeedback::TRAJECTORY_LOCKED))
      return false;
    if(p.stationary_hold) {
      if(request_.command.maneuver_type!=asmc_msgs::BehaviorCommand::STOP ||
         p.feedback.execution_state!=asmc_msgs::FrenetFeedback::COMPLETED)return false;
      for(const auto& point:p.trajectory.points)if(!std::isfinite(point.velocity) || std::abs(point.velocity)>.01)return false;
    }
    plan_=p;linked_=true;return true;
  }
  std::string epoch_,planner_epoch_;
  std::set<std::string> retired_,retired_planners_;
  std::uint64_t last_sequence_=0;
  Request request_;Plan plan_,pending_;
  bool have_request_=false,have_pending_=false,linked_=false;
};
}  // namespace asmc_msgs::binding
