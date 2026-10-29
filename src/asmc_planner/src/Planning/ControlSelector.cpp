#include "Planning/ControlSelector.hpp"

#include <utility>
#include <cmath>

namespace asmc_planner {
ControlSelector::ControlSelector(ros::NodeHandle nh) : nh_(std::move(nh)) {
  ros::NodeHandle("~").param("smooth_handoff",smooth_handoff_,false);
  ros::NodeHandle("~").param<std::string>("jamming_controller",jamming_controller_,"/localization_controller");
  ros::NodeHandle("~").param<std::string>("jamming_frame_prefix",jamming_frame_prefix_,"local/");
  std::string anchor_topic;ros::NodeHandle("~").param<std::string>("initial_ready_topic",anchor_topic,"");
  if(smooth_handoff_) {
    initial_anchor_ready_=anchor_topic.empty();
    if(!anchor_topic.empty())anchor_sub_=nh_.subscribe<std_msgs::Bool>(anchor_topic,1,
      [this](const std_msgs::Bool::ConstPtr& m){initial_anchor_ready_|=m->data;});
    request_sub_=nh_.subscribe("/planning/jamming/handoff_request",1,&ControlSelector::handoffRequest,this);
    candidate_sub_=nh_.subscribe("/planning/jamming/handoff_candidate",4,&ControlSelector::handoffCandidate,this);
    handoff_trace_pub_=nh_.advertise<std_msgs::String>("/planning/jamming/handoff_trace",10);
  }
  ros::NodeHandle("~").param("command_timeout_s", command_timeout_s_, 0.25);
  if (!std::isfinite(command_timeout_s_) || command_timeout_s_ <= 0.0)
    command_timeout_s_ = 0.25;
  control_pub_ = nh_.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd", 1);
  diagnostics_pub_ = nh_.advertise<std_msgs::String>(
      "/planning/control_selector/diagnostics", 1, true);
  mode_sub_ = nh_.subscribe("/planning/mode", 1,
                            &ControlSelector::modeCallback, this, ros::TransportHints().tcpNoDelay());
  normal_sub_ = nh_.subscribe("/ctrl_cmd/normal", 1,
                              &ControlSelector::normalCallback, this, ros::TransportHints().tcpNoDelay());
  jamming_sub_ = nh_.subscribe("/ctrl_cmd/jamming", 1,
                               &ControlSelector::jammingCallback, this, ros::TransportHints().tcpNoDelay());
  diagnostics_timer_ = nh_.createSteadyTimer(
      ros::WallDuration(1.0), &ControlSelector::diagnosticsCallback, this);
  watchdog_timer_ = nh_.createSteadyTimer(
      ros::WallDuration(0.05), &ControlSelector::watchdogCallback, this);
  publishDiagnostics();
}

void ControlSelector::modeCallback(const std_msgs::UInt8::ConstPtr& message) {
  if (!message) return;
  if (message->data != static_cast<std::uint8_t>(PlanningMode::NORMAL) &&
      message->data != static_cast<std::uint8_t>(PlanningMode::JAMMING)) {
    ROS_ERROR_THROTTLE(1.0, "Control selector rejected unknown mode %u",
                       static_cast<unsigned>(message->data));
    has_mode_ = false;
    awaiting_selected_command_ = true;
    publishStop();
    publishDiagnostics();
    return;
  }
  const auto next = static_cast<PlanningMode>(message->data);
  // Mode and preparation arrive on different topics. Allow the same bounded
  // 100ms race when mode arrives first; otherwise it inserts full brake even
  // when the prepared controller request/candidate follows immediately.
  if(smooth_handoff_ && has_mode_ && next!=mode_) {
    pending_handoff_=true;pending_mode_=next;pending_started_=ros::SteadyTime::now();
    completeHandoff();publishDiagnostics();return;
  }
  if (!has_mode_ || next != mode_) {
    ROS_WARN("Control selector source: %s -> %s", has_mode_ ?
             PlanningModeName(mode_) : "NONE", PlanningModeName(next));
    awaiting_selected_command_ = true;
    publishStop();
  }
  has_mode_ = true;
  mode_ = next;
  publishDiagnostics();
}

void ControlSelector::normalCallback(
    const morai_msgs::CtrlCmd::ConstPtr& message) {
  if (!message) return;
  normal_received_ = true;
  if(!initial_anchor_ready_){publishStop();return;}
  if(pending_handoff_)return;
  if (has_mode_ && mode_ == PlanningMode::NORMAL) {
    selected_received_at_ = ros::SteadyTime::now();
    awaiting_selected_command_ = false;
    control_pub_.publish(*message);
  }
}

void ControlSelector::jammingCallback(
    const morai_msgs::CtrlCmd::ConstPtr& message) {
  if (!message) return;
  jamming_received_ = true;
  if(!initial_anchor_ready_){publishStop();return;}
  if(pending_handoff_)return;
  if (has_mode_ && mode_ == PlanningMode::JAMMING) {
    selected_received_at_ = ros::SteadyTime::now();
    awaiting_selected_command_ = false;
    control_pub_.publish(*message);
  }
}

bool ControlSelector::commandFresh() const {
  return has_mode_ && !awaiting_selected_command_ &&
      (ros::SteadyTime::now() - selected_received_at_).toSec() <= command_timeout_s_;
}

void ControlSelector::publishStop() {
  morai_msgs::CtrlCmd stop;
  stop.longlCmdType = 1;
  stop.brake = 1.0;
  control_pub_.publish(stop);
}

void ControlSelector::watchdogCallback(const ros::SteadyTimerEvent&) {
  if(pending_handoff_) {
    if(completeHandoff())return;
    // Allow only the bounded transport race between candidate and mode topics.
    // Never refresh/re-publish the old source, and never extend the watchdog.
    if((ros::SteadyTime::now()-pending_started_).toSec()<=.1 && commandFresh())return;
    publishStop();return;
  }
  // A silent controller must not leave the simulator holding its last throttle.
  // Unselected input traffic never refreshes this selected-source watchdog.
  if (!commandFresh()) publishStop();
}

void ControlSelector::handoffRequest(const asmc_msgs::ControlHandoff::ConstPtr& m) {
  if(m->target_mode>1)return;
  if(request_.epoch && m->epoch<request_.epoch)return;
  if(m->epoch!=request_.epoch)candidates_={};
  request_=*m;request_received_=ros::SteadyTime::now();
  if(m->valid&&has_mode_&&awaiting_selected_command_&&m->target_mode==static_cast<uint8_t>(mode_)){
    pending_handoff_=true;pending_mode_=mode_;pending_started_=ros::SteadyTime::now();
  }
  if(!m->valid && pending_handoff_) {
    // /planning/mode already selected this target. A preparation deadline
    // cancels permission to use its candidate, not the logical mode change.
    // Otherwise mode-before-cancel delivery leaves the selector in NORMAL;
    // a later valid JAMMING request can never complete the stopped handoff.
    mode_=pending_mode_;has_mode_=true;
    pending_handoff_=false;awaiting_selected_command_=true;publishStop();
    publishDiagnostics();
  }
}

void ControlSelector::handoffCandidate(const asmc_msgs::ControlHandoff::ConstPtr& m) {
  if(m->target_mode>1 || m->header.frame_id!=(m->target_mode==0?"/mpc_controller":jamming_controller_))return;
  if(m->target_mode==0 ? m->state_frame!="map" : m->state_frame.rfind(jamming_frame_prefix_,0)!=0)return;
  if(m->epoch==request_.epoch && !m->valid) {
    candidates_[m->target_mode]={};return;
  }
  if(!asmc_msgs::handoff::Candidate(*m,request_,ros::Time::now()))return;
  candidates_[m->target_mode]=*m;if(pending_handoff_)completeHandoff();
}

bool ControlSelector::completeHandoff() {
  if(!pending_handoff_||!initial_anchor_ready_)return false;
  const auto index=static_cast<uint8_t>(pending_mode_);
  if(!request_.valid || request_.target_mode!=index || request_received_.isZero() ||
      (ros::SteadyTime::now()-request_received_).toSec()>.25)return false;
  const auto& c=candidates_[index];
  if(!asmc_msgs::handoff::Candidate(c,request_,ros::Time::now()))return false;
  morai_msgs::CtrlCmd cmd;cmd.longlCmdType=c.longl_cmd_type;
  cmd.steering=c.steering;cmd.accel=c.accel;cmd.brake=c.brake;
  mode_=pending_mode_;pending_handoff_=false;awaiting_selected_command_=false;
  selected_received_at_=ros::SteadyTime::now();control_pub_.publish(cmd);
  std_msgs::String trace;trace.data="{\"epoch\":"+std::to_string(c.epoch)+
      ",\"target\":"+std::to_string(index)+",\"command_age\":"+
      std::to_string((ros::Time::now()-c.header.stamp).toSec())+
      ",\"transport_wait_ms\":"+std::to_string((ros::SteadyTime::now()-pending_started_).toSec()*1000)+"}";
  handoff_trace_pub_.publish(trace);publishDiagnostics();return true;
}

void ControlSelector::diagnosticsCallback(const ros::SteadyTimerEvent&) {
  publishDiagnostics();
}

void ControlSelector::publishDiagnostics() {
  std_msgs::String message;
  message.data = std::string("mode=") +
      (has_mode_ ? PlanningModeName(mode_) : "UNKNOWN") +
      " selected_source=" +
      (has_mode_ ? (mode_ == PlanningMode::NORMAL ? "/ctrl_cmd/normal" :
                    "/ctrl_cmd/jamming") : "NONE") +
      " normal_received=" + (normal_received_ ? "true" : "false") +
      " jamming_received=" + (jamming_received_ ? "true" : "false") +
      " command_fresh=" + (commandFresh() ? "true" : "false");
  diagnostics_pub_.publish(message);
}
}  // namespace asmc_planner
