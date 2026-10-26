#include "Signal/SignalShadow.hpp"
#include <yaml-cpp/yaml.h>
#include <std_msgs/String.h>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace behavior_planner {
namespace {
namespace sig=asmc_planner::signal;
std::string Quote(const std::string& value) {
  std::ostringstream out;out<<'"';
  for(unsigned char c:value) {
    if(c=='"' || c=='\\')out<<'\\'<<c;
    else if(c<0x20)out<<"\\u"<<std::hex<<std::setw(4)<<std::setfill('0')<<int(c)<<std::dec;
    else out<<c;
  }
  out<<'"';return out.str();
}
sig::Point ReadPoint(const YAML::Node& n) {
  if(!n.IsSequence() || n.size()!=2) throw std::runtime_error("signal point must be [x,y]");
  sig::Point p{n[0].as<double>(),n[1].as<double>()};
  if(!std::isfinite(p.x) || !std::isfinite(p.y)) throw std::runtime_error("nonfinite signal point");
  return p;
}
std::string Movement(SignalMovement m) {
  switch(m) {
    case SignalMovement::STRAIGHT:return "straight";
    case SignalMovement::LEFT:return "left";
    case SignalMovement::LEFT_UNPROTECTED:return "left_unprotected";
    case SignalMovement::RIGHT_UNPROTECTED:return "right_unprotected";
    case SignalMovement::UTURN:return "uturn";
    default:return "";
  }
}
sig::Pose PoseOf(const EgoState& e,const RouteManager& route,bool project=true) {
  sig::Pose p;
  p.stamp=e.stamp;p.x=e.x;p.y=e.y;p.yaw=e.yaw;p.speed=e.speed;p.acceleration=e.acceleration;
  p.valid=e.valid && e.acceleration_valid;
  double d=0;std::string link;
  if(project) p.valid=p.valid && route.projectEgo(p.x,p.y,&p.route_s,&d,&link);
  return p;
}
bool PhysicalLineRouteS(const RouteManager& route,const sig::Geometry& g,
                        double hint,double& result) {
  int matches=0;double previous_s=0,previous_d=0;bool previous_valid=false;
  for(double s=std::max(0.,hint-10);s<=hint+10;s+=.1) {
    Point2D point;double yaw=0;
    if(!route.getRoutePose(s,&point,&yaw)) {previous_valid=false;continue;}
    const double d=(g.line.x-point.x)*g.approach.x+(g.line.y-point.y)*g.approach.y;
    if(previous_valid && previous_d>0 && d<=0) {
      double lo=previous_s,hi=s;
      for(int i=0;i<32;++i) {
        const double mid=(lo+hi)/2;
        route.getRoutePose(mid,&point,&yaw);
        const double md=(g.line.x-point.x)*g.approach.x+(g.line.y-point.y)*g.approach.y;
        if(md>0)lo=mid;else hi=mid;
      }
      result=(lo+hi)/2;++matches;
    }
    previous_valid=true;previous_s=s;previous_d=d;
  }
  return matches==1;
}
}  // namespace
SignalShadow::SignalShadow(ros::NodeHandle& nh,ros::NodeHandle& pnh,bool operational):operational_(operational) {
  std::string mode="shadow",geometry_path;
  pnh.param("signal_policy/mode",mode,mode);
  const bool bound=nh.param<bool>("/control_binding/enabled",false);
  if(mode!="shadow" && mode!="off" && mode!="active")throw std::runtime_error("invalid signal policy mode");
  if(mode=="active" && !bound)throw std::runtime_error("active signal policy requires Controller binding");
  enabled_=operational ? mode=="active" : mode!="off";
  pnh.param("signal_policy/source_clock_verified",source_clock_verified_,false);
  pnh.param("signal_policy/half_track_m",config_.half_track,config_.half_track);
  pnh.param("signal_policy/measured_max_delay_s",config_.measured_max_delay,0.0);
  pnh.param("signal_policy/recovery_speed_mps",config_.recovery_speed,config_.recovery_speed);
  pnh.param("signal_policy/freshness_s",config_.freshness,config_.freshness);
  pnh.param("signal_policy/observation_gap_s",config_.max_gap,config_.max_gap);
  pnh.param("signal_policy/position_interpolation_gap_s",config_.interpolation_gap,config_.interpolation_gap);
  pnh.param("signal_policy/position_error_m",config_.position_error,config_.position_error);
  pnh.param("signal_policy/entry_observation_error_m",config_.entry_observation_error,config_.entry_observation_error);
  pnh.param("signal_policy/entry_observation_speed_max_mps",config_.entry_observation_speed_max,config_.entry_observation_speed_max);
  pnh.param("signal_policy/front_clearance_m",config_.clearance,config_.clearance);
  pnh.param("signal_policy/front_extent_m",config_.front_extent,config_.front_extent);
  pnh.param("signal_policy/rear_extent_m",config_.rear_extent,config_.rear_extent);
  pnh.param("signal_policy/half_width_m",config_.half_width,config_.half_width);
  pnh.param("signal_policy/wheelbase_m",config_.wheelbase,config_.wheelbase);
  if(!std::isfinite(config_.interpolation_gap) || config_.interpolation_gap<=0 ||
     config_.interpolation_gap>config_.freshness ||
     !std::isfinite(config_.freshness) || config_.freshness<=0 ||
     !std::isfinite(config_.max_gap) || config_.max_gap<=0 || config_.max_gap>config_.freshness ||
     !std::isfinite(config_.position_error) || config_.position_error<0 ||
     !std::isfinite(config_.clearance) || config_.clearance<0)
    throw std::runtime_error("invalid signal freshness, gap or footprint uncertainty");
  const bool entry_unqualified=config_.entry_observation_error==-1 && config_.entry_observation_speed_max==0;
  if(!entry_unqualified && (!std::isfinite(config_.entry_observation_error) || config_.entry_observation_error<0 ||
      config_.entry_observation_error>config_.position_error || !std::isfinite(config_.entry_observation_speed_max) ||
      config_.entry_observation_speed_max<=0))throw std::runtime_error("invalid measured entry observation contract");
  if(!std::isfinite(config_.measured_max_delay) || config_.measured_max_delay<0)
    throw std::runtime_error("signal measured maximum delay must be finite and nonnegative");
  if(!std::isfinite(config_.recovery_speed) || config_.recovery_speed<=0 || config_.recovery_speed>25./3.6)
    throw std::runtime_error("signal recovery speed must be positive and within the approach cap");
  if(std::isfinite(config_.half_track) && (config_.half_track<=0 || config_.half_track>config_.half_width))
    throw std::runtime_error("signal half_track_m must be measured within the vehicle footprint");
  pnh.param("signal_policy/geometry_path",geometry_path,geometry_path);
  policy_=std::make_unique<sig::Policy>(config_);
  history_=std::make_unique<sig::PoseHistory>(config_);
  localization_=std::make_unique<sig::LocalizationContinuity>(config_);
  std::string localization_topic;
  pnh.param<std::string>("signal_policy/localization_history_topic",localization_topic,"");
  pnh.param<std::string>("signal_policy/localization_history_caller",localization_caller_,"");
  if(enabled_ && !localization_topic.empty()) {
    if(localization_caller_.empty())throw std::runtime_error("localization continuity requires a pinned producer");
    localization_sub_=nh.subscribe(localization_topic,128,&SignalShadow::LocalizationCallback,this);
  }
  if(!geometry_path.empty()) {
    const auto root=YAML::LoadFile(geometry_path);
    if(root["policy_route_span"]) {
      scope_first_link_=root["policy_route_span"]["first_link_id"].as<std::string>();
      scope_last_link_=root["policy_route_span"]["last_link_id"].as<std::string>();
      if(scope_first_link_.empty() || scope_last_link_.empty())
        throw std::runtime_error("invalid yellow policy route span");
    }
    const auto nodes=root["reviewed_bindings"];
    if(!nodes || !nodes.IsSequence()) throw std::runtime_error("reviewed_bindings must be a sequence");
    for(const auto& n:nodes) {
      if(!n["reviewed"].as<bool>(false)) continue;
      Binding b;
      b.node=n["stop_node_id"].as<std::string>();
      b.frame=n["camera_frame"].as<std::string>();
      b.movement=n["movement"].as<std::string>();
      b.geometry.line=ReadPoint(n["line_point"]);
      b.geometry.approach=ReadPoint(n["approach_direction"]);
      if(b.node.empty() || b.frame.empty() || b.movement.empty() ||
         std::abs(std::hypot(b.geometry.approach.x,b.geometry.approach.y)-1)>1e-5)
        throw std::runtime_error("invalid reviewed signal binding");
      b.geometry.reviewed=true;
      if(n["conflict_reviewed"].as<bool>(false)) {
        for(const auto& p:n["conflict_polygon"]) b.geometry.conflict_polygon.push_back(ReadPoint(p));
        b.geometry.exit=ReadPoint(n["exit_point"]);
        b.geometry.exit_direction=ReadPoint(n["exit_direction"]);
        if(b.geometry.conflict_polygon.size()<3 ||
           std::abs(std::hypot(b.geometry.exit_direction.x,b.geometry.exit_direction.y)-1)>1e-5)
          throw std::runtime_error("invalid reviewed conflict geometry");
        b.geometry.conflict_reviewed=true;
      }
      bindings_.push_back(std::move(b));
    }
  }
  if(operational_ && enabled_ &&
     (!source_clock_verified_ || !std::isfinite(config_.half_track) || bindings_.empty()))
    throw std::runtime_error("active signal policy requires reviewed geometry, source clock and measured front track");
  if(enabled_) publisher_=nh.advertise<std_msgs::String>(operational_?"/planning/signal_policy":"/planning/signal_policy_shadow",1);
}
void SignalShadow::LocalizationCallback(const ros::MessageEvent<asmc_msgs::VehicleState const>& event) {
  const auto message=event.getMessage();
  if(event.getPublisherName()!=localization_caller_ || !message ||
     message->header.frame_id!="map" || message->source!=asmc_msgs::VehicleState::SOURCE_GPS_IMU) {
    localization_->Clear();return;
  }
  sig::Pose pose;pose.stamp=message->header.stamp.toSec();pose.x=message->x;pose.y=message->y;
  pose.yaw=message->yaw;pose.speed=message->speed;pose.acceleration=message->acceleration;
  pose.valid=message->valid;
  localization_->Push(pose,ros::SteadyTime::now().toSec());
}
void SignalShadow::PushPose(const EgoState& e,const RouteManager& route) {
  if(!enabled_) return;
  // Store raw 100Hz positions cheaply. Project only source-frame positions
  // needed by the 10Hz observer, never scan the route in every ego callback.
  const auto p=PoseOf(e,route,false);
  if(route_generation_!=route.generation() ||
     (previous_pose_.valid && p.valid &&
      (p.stamp<previous_pose_.stamp ||
       std::hypot(p.x-previous_pose_.x,p.y-previous_pose_.y)>
          2*config_.position_error+std::max(p.speed,previous_pose_.speed)*
              std::max(0.,p.stamp-previous_pose_.stamp)))) {
    ++episode_generation_;route_generation_=route.generation();
    current_=Binding{};episode_activated_=false;history_->Clear();pending_.clear();
    localization_->Clear();
  }
  history_->Push(p);previous_pose_=p;
}
void SignalShadow::Update(const EgoState& ego,bool state_fresh,bool risks_fresh,
                         const UrbanFeature* f,const RouteManager& route,
                         RosTrafficLightProvider* provider) {
  if(!enabled_) return;
  in_scope_=false;recovery_candidate_=false;path_recovery_candidate_=false;path_recovery_requires_exit_=false;
  const double now=ros::Time::now().toSec(),steady=ros::SteadyTime::now().toSec();
  const bool clock_reset=last_steady_>0 &&
      std::abs((now-last_ros_)-(steady-last_steady_))>.1;
  last_ros_=now;last_steady_=steady;
  if(clock_reset) { ++source_epoch_;history_->Clear();localization_->Clear();pending_.clear();
  }
  bool overflow=false;
  if(provider) {
    auto incoming=provider->takeSignalObservations(overflow);
    for(auto& observation:incoming) pending_.push_back(std::move(observation));
  }
  if(pending_.size()>128) { pending_.clear();overflow=true; }
  const auto invalid_diagnostic=[&](const char* status) {
    if(operational_) {
      in_scope_=true;last_output_={};last_output_.severity=sig::Severity::EMERGENCY;
      last_output_.signal_requirement=sig::Severity::STOP;
      last_output_.actual_control_allowed=true;
    }
    std_msgs::String message;
    message.data=std::string("{\"mode\":\"")+(operational_?"active":"shadow")+
        "\",\"status\":\""+status+"\",\"severity\":2,\"actual_control_allowed\":"+
        (operational_?"true":"false")+"}";
    publisher_.publish(message);
  };
  auto pose=PoseOf(ego,route,false);
  if(f && f->ego.valid)pose.route_s=f->ego.s;
  else {
    double d=0;std::string link;
    pose.valid=pose.valid && route.projectEgo(pose.x,pose.y,&pose.route_s,&d,&link);
  }
  // Keep the same stop-line episode after rear-axle passage; absence of the
  // next stop line is not evidence that the current intersection was cleared.
  StopLineInfo stop;double remaining=0;
  bool have_stop=!current_.node.empty() && route.findStopLineByNode(current_.node,&stop);
  if(!have_stop || policy_->motion()==sig::Motion::EXITED) {
    unsigned occupied=0;std::string occupied_movement;
    // A restart/reposition inside a reviewed zone must not silently select
    // only the next intersection. It starts an uncertain recovery episode.
    for(const auto& b:bindings_) {
      StopLineInfo candidate;RouteSignalMovement movement;
      if(pose.valid && sig::BodyOccupiesConflict(pose,b.geometry,config_) &&
         route.findStopLineByNode(b.node,&candidate) &&
         route.findNextSignalMovement(std::max(0.,candidate.route_s-1),200,&movement) &&
         Movement(movement.movement)==b.movement) {
        stop=candidate;occupied_movement=b.movement;++occupied;
      }
    }
    if(occupied>1) {
      sig::Observation invalid;policy_->Observe(invalid,now,steady);
      pending_.clear();invalid_diagnostic("ambiguous_conflict_region");return;
    }
    have_stop=occupied==1 || (pose.valid &&
        route.findNextSignalizedStopLine(pose.route_s,200,&stop,&remaining));
    if(have_stop && !scope_first_link_.empty()) {
      bool inside=false;
      if(!route.stopInRouteSpan(stop.node_id,scope_first_link_,scope_last_link_,&inside)) {
        pending_.clear();invalid_diagnostic("unknown_policy_route_span");return;
      }
      if(!inside) {
        // This is applicability, never an UNKNOWN-to-green conversion. A
        // live intersection episode reaches this branch only after EXITED;
        // other Behavior/Controller safety and the existing LCS rules remain.
        have_stop=false;current_=Binding{};episode_activated_=false;
      }
    }
    if(have_stop) {
      current_=Binding{};episode_activated_=false;current_.node=stop.node_id;last_policy_pose_stamp_=0;
      const std::string movement=occupied==1 ? occupied_movement :
          (f ? Movement(f->signal.required_movement) : "");
      unsigned matches=0;
      for(const auto& b:bindings_) if(b.node==stop.node_id && b.movement==movement) {current_=b;++matches;}
      if(matches!=1) {current_=Binding{};current_.node=stop.node_id;current_.movement=movement;}
      current_.geometry.key={episode_generation_,stop.node_id,movement};
      current_.geometry.route_s=stop.route_s;
      if(current_.geometry.reviewed && !PhysicalLineRouteS(route,current_.geometry,
          stop.route_s,current_.geometry.route_s)) current_.geometry.reviewed=false;
      policy_->Bind(current_.geometry);
      selected_offset_=5.5;
      policy_->SetTarget(5.5,false);
    }
  }
  auto& geometry=current_.geometry;
  if(!have_stop) {
    pending_.clear();
    if(!state_fresh || !pose.valid || !std::isfinite(pose.route_s)) {
      invalid_diagnostic("unknown_input");return;
    }
    std_msgs::String message;
    message.data=std::string("{\"mode\":\"")+(operational_?"active":"shadow")+
        "\",\"status\":\"out_of_scope\",\"severity\":0,\"actual_control_allowed\":false}";
    publisher_.publish(message);return;
  }
  if(current_.movement=="right_unprotected") {
    pending_.clear();std_msgs::String message;
    message.data=std::string("{\"mode\":\"")+(operational_?"active":"shadow")+
        "\",\"status\":\"existing_unprotected_rule\",\"actual_control_allowed\":false}";
    current_=Binding{};publisher_.publish(message);return;
  }
  // Route association has a longer horizon than actuation. Do not strand a
  // stationary vehicle hundreds of metres before a signal that is not yet in
  // camera view. Enter the stopping episode before its conservative braking
  // distance, while the existing approach speed limiter remains in force.
  double activation=std::max(30.,5.5+5.+sig::MinimumStoppingDistance(
      pose.speed,pose.acceleration,config_.brake,sig::ResponseBudget(config_.measured_max_delay)));
  // AVOID can acquire an object inside or beyond the intersection before the
  // ordinary signal braking horizon. In that case the still-unconfirmed
  // signal line is the first constraint; enter its episode before AVOID can
  // commit to a path through the line. Keep this extension local to an
  // observed post-line object instead of stopping for every distant signal.
  if(f && f->risk.static_obstacle_ahead && f->signal.stop_line_valid &&
     std::isfinite(f->risk.nearest_obstacle_s) &&
     std::isfinite(f->risk.nearest_obstacle_length) &&
     f->risk.nearest_obstacle_s-
         .5*std::max(0.,f->risk.nearest_obstacle_length) >=
         f->signal.stop_line_s &&
     f->signal.stop_line_s-pose.route_s <= 60.) {
    activation=std::max(activation,60.);
  }
  if(operational_ && !episode_activated_ && current_.geometry.route_s-pose.route_s>activation &&
     !sig::BodyOccupiesConflict(pose,current_.geometry,config_)) {
    std_msgs::String message;message.data="{\"mode\":\"active\",\"status\":\"before_braking_horizon\",\"actual_control_allowed\":false}";
    publisher_.publish(message);pending_.clear();return;
  }
  in_scope_=true;episode_activated_=true;
  if(overflow || clock_reset) {
    sig::Observation invalid;invalid.source_epoch=source_epoch_;
    policy_->Observe(invalid,now,steady);
    pending_.clear();
  }
  std::deque<sig::Observation> waiting;
  std::ostringstream observation_log;bool first_observation=true;
  for(auto& o:pending_) {
    o.source_epoch=(source_epoch_<<32)|o.transport_epoch;
    o.clock_verified=source_clock_verified_ && !clock_reset;
    if(o.clock_verified && o.valid && !o.stale &&
       (o.label=="green" || o.label=="green_left" || o.label=="left") &&
       pose.stamp<o.stamp && now>=o.stamp && now-o.stamp<=config_.freshness) {
      waiting.push_back(o);continue;
    }
    sig::Pose at_capture;
    double capture_d=0;std::string capture_link;
    const bool capture_known=o.clock_verified && history_->At(o.stamp,at_capture) &&
        route.projectEgo(at_capture.x,at_capture.y,&at_capture.route_s,&capture_d,&capture_link);
    RouteSignalMovement capture_movement;
    const bool movement_known=capture_known &&
        route.findNextSignalMovement(at_capture.route_s,200,&capture_movement);
    const auto movement=movement_known ? capture_movement.movement : SignalMovement::UNKNOWN;
    const auto color=InterpretTrafficLightLabel(o.label,movement);
    if(color==TrafficLightState::GREEN)o.color=sig::Color::GREEN;
    else if(color==TrafficLightState::RED)o.color=sig::Color::RED;
    else if(color==TrafficLightState::YELLOW)o.color=sig::Color::YELLOW;
    else o.color=sig::Color::UNKNOWN;
    StopLineInfo capture_stop;double capture_distance=0;
    const bool same_line=capture_known &&
        ((route.findNextSignalizedStopLine(at_capture.route_s,200,&capture_stop,&capture_distance) &&
          capture_stop.node_id==current_.node) || sig::BodyOccupiesConflict(at_capture,geometry,config_));
    o.association_valid=geometry.reviewed && o.frame==current_.frame && same_line &&
        movement_known && Movement(movement)==current_.movement;
    o.key=geometry.key;
    policy_->Observe(o,now,steady,history_.get());
    if(!first_observation)observation_log<<",";
    first_observation=false;
    observation_log<<"{\"source_stamp\":"<<std::setprecision(12)<<o.stamp
        <<",\"source_stamp_ns\":"<<o.original_stamp_ns<<",\"frame\":"<<Quote(o.frame)
        <<",\"source_epoch\":"<<o.source_epoch<<",\"sequence\":"<<o.sequence
        <<",\"received_steady\":"<<o.received<<",\"color\":"<<static_cast<int>(o.color)
        <<",\"valid\":"<<(o.valid?"true":"false")<<",\"stale\":"<<(o.stale?"true":"false")
        <<",\"clock_verified\":"<<(o.clock_verified?"true":"false")
        <<",\"association_valid\":"<<(o.association_valid?"true":"false")<<"}";
  }
  pending_=std::move(waiting);
  const auto update_cause=[&](sig::Cause cause,bool danger,bool clear,bool fresh,
                              sig::Severity severity,double& since) {
    if(danger) {causes_.Set(cause,severity);since=-1;}
    else if(!fresh || !clear) since=-1;
    else if(causes_.Get(cause)!=sig::Severity::NONE) {
      if(since<0)since=steady;
      if(steady-since>=.5)causes_.Set(cause,sig::Severity::NONE);
    }
  };
  update_cause(sig::Cause::COLLISION,f && f->risk.emergency_risk,
      f && f->risk.min_ttc>=3,risks_fresh,sig::Severity::EMERGENCY,collision_safe_since_);
  update_cause(sig::Cause::PEDESTRIAN,f && f->risk.pedestrian_risk,
      f && !f->risk.pedestrian_risk,risks_fresh,sig::Severity::STOP,pedestrian_safe_since_);
  update_cause(sig::Cause::INPUT_INVALID,!state_fresh || !risks_fresh,
      state_fresh && risks_fresh,state_fresh && risks_fresh,sig::Severity::EMERGENCY,input_safe_since_);
  sig::Evidence evidence=operational_ ? execution_evidence_ : sig::Evidence{};
  evidence.position_valid=state_fresh && now>=pose.stamp && now-pose.stamp<=.3;
  evidence.state_age=now>=pose.stamp ? now-pose.stamp : 0;
  evidence.position_history_continuous=history_->Covers(last_policy_pose_stamp_,pose.stamp);
  evidence.localization_continuity_verified=
      last_localization_generation_==localization_->generation() &&
      localization_->CoversMatched(last_policy_pose_stamp_,pose,now,steady);
  last_localization_generation_=localization_->generation();
  last_policy_pose_stamp_=pose.stamp;
  evidence.green_capture_pose_valid=history_->At(policy_->gate().last_green(),evidence.green_capture_pose) &&
      history_->Covers(policy_->gate().last_green(),pose.stamp);
  evidence.risks_fresh=risks_fresh;
  evidence.conflict_known=geometry.conflict_reviewed;
  evidence.inside_conflict=sig::BodyDefinitelyOccupiesConflict(pose,geometry,config_);
  evidence.exited=sig::BodyOutsideExit(pose,geometry,config_);
  recovery_candidate_=evidence.position_valid && evidence.risks_fresh &&
      evidence.conflict_known && policy_->RecoveryPositionEligible(pose,evidence.position_history_continuous) &&
      causes_.OthersClear();
  path_recovery_candidate_=evidence.execution_failure_mask==2 && evidence.position_valid &&
      evidence.risks_fresh && pose.speed<=config_.standstill_speed && policy_->gate().authorized() &&
      causes_.OthersClear();
  // A stopped overshoot outside the conflict can receive a NEW green admission.
  // It cannot regain the old stop-goal proof: preview the entire exit instead.
  // This candidate still requires green; the red internal-recovery exception
  // remains exclusively recovery_candidate_. Neither preview releases brakes.
  path_recovery_requires_exit_=path_recovery_candidate_ && !sig::BodyBeforeLine(pose,geometry,config_);
  const auto pose_at_s=[&](double s,sig::Pose& goal) {
    Point2D point;double yaw=0;
    if(!route.getRoutePose(s,&point,&yaw))return false;
    goal=pose;goal.x=point.x;goal.y=point.y;goal.yaw=yaw;goal.speed=0;
    goal.acceleration=0;goal.route_s=s;return true;
  };
  sig::Pose goal;
  evidence.target_geometry_valid=pose_at_s(geometry.route_s-selected_offset_,goal) &&
      sig::BodyBeforeLine(goal,geometry,config_);
  const auto decision_state=sig::ProjectMeasuredState(pose.speed,pose.acceleration,evidence.state_age);
  const double available=geometry.route_s-pose.route_s-selected_offset_-decision_state.distance;
  const double budget=evidence.response_delay_known ? std::max(0.,evidence.response_delay_remaining) :
      sig::ResponseBudget(config_.measured_max_delay);
  const double boundary=sig::StopBoundaryAcceleration(decision_state.speed,pose.acceleration,config_.brake);
  const double required=std::max(
      sig::MinimumStoppingDistance(decision_state.speed,pose.acceleration,config_.brake,budget),
      sig::MinimumStoppingDistance(decision_state.speed,boundary,config_.brake,budget));
  // Near the endpoint, source-age and projection noise can make a few
  // centimetres of calculated deficit alternate sign. Do not move an active
  // braking goal for a deficit smaller than the existing 0.7 m arrival
  // tolerance; the new goal may consume physical-line clearance while the
  // original stop remains safely achievable. A material deficit still enters
  // the reviewed late-target path. Initial deadband: 0.2 m, below arrival
  // tolerance and measured from the 0.05 m red-stop oscillation.
  constexpr double kLateTargetSwitchDeficitM = 0.2;
  if(selected_offset_==5.5 && pose.route_s<geometry.route_s &&
     pose.speed>config_.standstill_speed && !policy_->gate().authorized() &&
     required>available+kLateTargetSwitchDeficitM) {
    const auto late=sig::SelectLateTarget(geometry,config_,pose_at_s);
    if(late.valid) {
      selected_offset_=late.offset;policy_->SetTarget(late.offset,true);
      evidence.target_geometry_valid=true;
    }
  }
  // Only the independent operational instance receives the explicit bound
  // Controller proof. Shadow computations retain independent virtual state.
  auto result=policy_->Step(pose,now,steady,evidence,causes_);
  path_recovery_requires_exit_ |= result.motion==sig::Motion::RELEASE_PENDING &&
      !sig::BodyBeforeLine(pose,geometry,config_);
  recovery_candidate_=recovery_candidate_ &&
      (result.motion==sig::Motion::RECOVERY_BRAKE || result.motion==sig::Motion::RECOVERY_CLEAR_PENDING ||
       result.motion==sig::Motion::RECOVERY_CLEAR);
  result.actual_control_allowed=operational_;
  last_output_=result;
  const auto front=sig::FrontWheelDistance(pose,geometry,config_);
  const char* band=!front.valid ? "UNKNOWN" : front.min>config_.crossing_band ? "BEFORE" :
      front.max < -config_.crossing_band ? "AFTER" : "BOUNDARY";
  std::ostringstream out;
  out<<std::setprecision(12)<<"{\"mode\":\""<<(operational_?"active":"shadow")
     <<"\",\"actual_control_allowed\":"<<(operational_?"true":"false")
     <<",\"events_are_counterfactual\":"<<(operational_?"false":"true")
     <<",\"motion\":\""<<sig::MotionName(result.motion)<<"\",\"severity\":"<<static_cast<int>(result.severity)
     <<",\"signal_requirement\":"<<static_cast<int>(result.signal_requirement)
     <<",\"causes\":{\"SIGNAL\":"<<static_cast<int>(result.causes[0])
     <<",\"COLLISION\":"<<static_cast<int>(result.causes[1])
     <<",\"PEDESTRIAN\":"<<static_cast<int>(result.causes[2])
     <<",\"INPUT_INVALID\":"<<static_cast<int>(result.causes[3])
     <<",\"EXECUTION_FAILURE\":"<<static_cast<int>(result.causes[4])<<"}"
     <<",\"release_barrier\":"<<(result.release_barrier?"true":"false")
     <<",\"execution_recovery_acknowledged\":"<<(result.execution_recovery_acknowledged?"true":"false")
     <<",\"virtual_command_id\":"<<result.virtual_command_id<<",\"target_revision\":"<<result.target_revision
     <<",\"entry_observation_error_m\":"<<result.entry_observation_error
     <<",\"target_reached\":"<<(result.target_reached?"true":"false")
     <<",\"stationary\":"<<(result.stationary?"true":"false")
     <<",\"position_history_continuous\":"<<(evidence.position_history_continuous?"true":"false")
     <<",\"localization_continuity_verified\":"<<(evidence.localization_continuity_verified?"true":"false")
     <<",\"entry_qualified\":"<<(result.entry_authorized?"true":"false")
     <<",\"entry_pending\":"<<(result.entry_pending?"true":"false")
     <<",\"green_count\":"<<policy_->gate().count()<<",\"reviewed_geometry\":"<<(geometry.reviewed?"true":"false")
     <<",\"route_generation\":"<<geometry.key.route_generation
     <<",\"stop_line\":"<<Quote(geometry.key.stop_line)<<",\"movement\":"<<Quote(geometry.key.movement)
     <<",\"target_offset_m\":"<<selected_offset_
     <<",\"physical_line_band\":\""<<band<<"\""
     <<",\"decision_ros\":"<<now<<",\"decision_steady\":"<<steady
     <<",\"vehicle\":{\"source_stamp\":"<<pose.stamp<<",\"x\":"<<pose.x
     <<",\"y\":"<<pose.y<<",\"yaw\":"<<pose.yaw<<",\"speed\":"<<pose.speed
     <<",\"acceleration\":";
  if(std::isfinite(pose.acceleration))out<<pose.acceleration;else out<<"null";
  out<<"}"
     <<",\"source_clock_verified\":"<<(source_clock_verified_?"true":"false")
     <<",\"front_track_verified\":"<<(std::isfinite(config_.half_track)?"true":"false")
     <<",\"model_feasible\":"<<(result.model_feasible?"true":"false")
     <<",\"minimum_distance_m\":";
  if(std::isfinite(result.minimum_distance))out<<result.minimum_distance;else out<<"null";
  out<<",\"connection\":\""<<(!operational_?"shadow":evidence.linked?"verified":"pending")
     <<"\",\"exit_path_evaluated\":"<<(operational_ && evidence.planning_checked?"true":"false")
     <<",\"actuation_response_evaluated\":"<<(evidence.response_delay_known?"true":"false")
     <<",\"physical_response_observed\":"<<(evidence.response_delay_known && evidence.response_observed?"true":"false")
     <<",\"observations\":["<<observation_log.str()<<"],\"new_events\":[";
  const auto& events=policy_->events();
  for(std::size_t i=published_events_;i<events.size();++i) {
    if(i>published_events_)out<<",";
    out<<"{\"id\":"<<i+1<<",\"route_generation\":"<<events[i].key.route_generation
       <<",\"stop_line\":"<<Quote(events[i].key.stop_line)
       <<",\"movement\":"<<Quote(events[i].key.movement)
       <<",\"kind\":"<<static_cast<int>(events[i].kind)<<",\"stamp\":"<<events[i].stamp
       <<",\"begin\":"<<events[i].interval_begin<<",\"end\":"<<events[i].interval_end<<"}";
  }
  out<<"]}";published_events_=events.size();
  std_msgs::String message;message.data=out.str();publisher_.publish(message);
}
}  // namespace behavior_planner
