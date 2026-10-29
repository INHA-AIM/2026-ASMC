#include "Planning/GpsModeSource.hpp"
#include "Planning/ShadowApproach.hpp"

#include <cstdio>
#include <string>
#include <utility>
#include <openssl/sha.h>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace asmc_planner {
namespace {
JammingDetectorConfig LoadConfig(const ros::NodeHandle& params) {
  JammingDetectorConfig config;
  params.param("gps_timeout_s", config.gps_timeout_s, config.gps_timeout_s);
  params.param("exit_hold_s", config.exit_hold_s, config.exit_hold_s);
  params.param("shadow_zone/enabled", config.zone_enabled, config.zone_enabled);
  return config;
}
double Now() { return ros::SteadyTime::now().toSec(); }
std::string Digest(const std::string& file){
  std::ifstream input(file,std::ios::binary);if(!input)throw std::runtime_error("cannot open jamming route contract file");
  SHA256_CTX context;SHA256_Init(&context);char block[8192];
  while(input){input.read(block,sizeof block);SHA256_Update(&context,block,input.gcount());}
  unsigned char result[SHA256_DIGEST_LENGTH];SHA256_Final(result,&context);
  std::ostringstream out;for(auto byte:result)out<<std::hex<<std::setw(2)<<std::setfill('0')<<unsigned(byte);return out.str();
}
}  // namespace

GpsModeSource::GpsModeSource(ros::NodeHandle nh)
    : nh_(std::move(nh)), detector_(LoadConfig(ros::NodeHandle(nh_, "planning_mode"))) {
  ros::NodeHandle params(nh_, "planning_mode");
  bool zone_enabled = true;
  std::string path_file;
  params.param("shadow_zone/enabled", zone_enabled, zone_enabled);
  params.param("shadow_zone/path_file", path_file, std::string());
  params.param("recovery_timeout_s", recovery_timeout_s_, recovery_timeout_s_);
  params.param("smooth_handoff",smooth_handoff_,false);
  params.param("approach_speed_kph",approach_speed_mps_,0.);
  if(!std::isfinite(approach_speed_mps_)||approach_speed_mps_<0||approach_speed_mps_>60.)
    throw std::runtime_error("invalid localization approach speed");
  approach_speed_mps_/=3.6;
  params.param<std::string>("jamming_controller",jamming_controller_,"/localization_controller");
  if(smooth_handoff_) {
    handoff_request_pub_=nh_.advertise<asmc_msgs::ControlHandoff>("/planning/jamming/handoff_request",1,true);
    prepare_pub_=nh_.advertise<std_msgs::Bool>("/planning/jamming/prepare",1,true);
    approach_pub_=nh_.advertise<std_msgs::Bool>("/planning/jamming/approach",1,true);
    candidate_sub_=nh_.subscribe("/planning/jamming/handoff_candidate",4,&GpsModeSource::candidateCallback,this);
  }
  std::string recovery_ready_topic;
  params.param<std::string>("recovery_ready_topic",recovery_ready_topic,"");
  require_recovery_ready_=!recovery_ready_topic.empty();
  if(require_recovery_ready_)recovery_ready_sub_=nh_.subscribe<std_msgs::Bool>(recovery_ready_topic,1,
    [this](const std_msgs::Bool::ConstPtr& message) {
      recovery_ready_=message->data;recovery_ready_received_=ros::SteadyTime::now();
      if(!recovery_ready_)ready_frames_=0;
    });
  // 단독 실행(시험 노드)처럼 경로가 주어지지 않으면 GPS 값 규칙만 쓴다.
  if (zone_enabled && !path_file.empty()) {
    double start_m = 0.0, end_m = 0.0, lateral_m = 10.0, heading_rad = 1.0;
    params.param("shadow_zone/station_start_m", start_m, start_m);
    params.param("shadow_zone/station_end_m", end_m, end_m);
    params.param("shadow_zone/lateral_tolerance_m", lateral_m, lateral_m);
    params.param("shadow_zone/heading_tolerance_rad", heading_rad, heading_rad);
    // 경로를 못 읽으면 조용히 끄지 않는다. 음영 진입을 놓치면 MPC가 GPS 없이 달린다.
    std::string path_sha,links_file,links_sha;
    if(params.getParam("path_sha256",path_sha)&&Digest(path_file)!=path_sha)throw std::runtime_error("jamming route SHA mismatch");
    if(params.getParam("links_sha256",links_sha)){
      if(!params.getParam("links_file",links_file)||Digest(links_file)!=links_sha)throw std::runtime_error("jamming link map SHA mismatch");
    }
    zone_.reset(new ShadowZone(path_file, start_m, end_m, lateral_m, heading_rad));
    std::string state_topic;
    params.param("shadow_zone/state_topic", state_topic, std::string("/planning/vehicle/state"));
    state_sub_ = nh_.subscribe(state_topic, 1, &GpsModeSource::stateCallback, this);
    ROS_INFO("Planning mode shadow zone: station %.1f-%.1f m (state %s)",
             start_m, end_m, state_topic.c_str());
  }
  mode_pub_ = nh_.advertise<std_msgs::UInt8>("/planning/mode", 1, true);
  status_pub_ = nh_.advertise<std_msgs::String>("/planning/mode/status", 1);
  // LiDAR는 재밍 원인이 사라지는 즉시 재개한다. NORMAL 복귀(exit_hold_s와 궤적 준비
  // 대기) 시점에 추적이 이미 데워져 있어야 판제가 장애물 입력 stale로 정지하지 않는다.
  lidar_pause_pub_ = nh_.advertise<std_msgs::Bool>("/planning/mode/lidar_pause", 1, true);
  // 재밍 원인이 사라졌지만 아직 JAMMING을 발행하는 동안 true. 재밍 제어기는 이때 바로
  // 진입 차로(GPS 기준 경로)로 돌아가 일반 플래너가 이어받을 수 있게 한다.
  gps_return_pub_ = nh_.advertise<std_msgs::Bool>("/planning/mode/gps_return", 1, true);
  gps_sub_ = nh_.subscribe("/gps", 1, &GpsModeSource::gpsCallback, this);
  feedback_sub_ = nh_.subscribe("/planning/frenet_feedback_v2", 1,
                                &GpsModeSource::feedbackCallback, this);
  std_msgs::UInt8 initial;
  initial.data = static_cast<std::uint8_t>(mode_);
  mode_pub_.publish(initial);
  publishStatus(Now());
  publishLidarPause();
  publishGpsReturn();
  timer_ = nh_.createSteadyTimer(ros::WallDuration(0.05), &GpsModeSource::timerCallback, this);
  ROS_INFO("Planning mode NORMAL: waiting for raw /gps");
}

void GpsModeSource::gpsCallback(const morai_msgs::GPSMessage::ConstPtr& message) {
  if (!message) return;
  const double now = Now();
  detector_.OnGps(*message, now);
  // 0 좌표·no-fix 진입은 타이머를 기다리지 않고 같은 메시지에서 반영한다.
  if (detector_.Update(now)) onDetectorChange();
  publishLidarPause();
  publishGpsReturn();
}

void GpsModeSource::stateCallback(const asmc_msgs::VehicleState::ConstPtr& message) {
  // blackout 동안의 추정 자세로 구간을 다시 판정하지 않는다. 마지막 판정을 유지한다.
  if (!message || !message->valid || detector_.gps_zero() || detector_.gps_no_fix() ||
      detector_.cause() == JammingReason::GPS_TIMEOUT) return;
  if(approach_speed_mps_>0) {
    const double age=(ros::Time::now()-message->header.stamp).toSec();
    if(message->header.frame_id!="map" || age<0 || age>.2 ||
       !std::isfinite(message->x) || !std::isfinite(message->y) ||
       !std::isfinite(message->yaw) || !std::isfinite(message->speed))return;
  }
  double station = -1.0;
  const bool inside = zone_->Contains(message->x, message->y, message->yaw, &station);
  if(smooth_handoff_) {
    const double lookahead=ShadowApproachDistance(message->speed,approach_speed_mps_,approach_distance_m_,message->acceleration);
    approaching_=inside||(approach_speed_mps_>0
        ?zone_->ApproachingAlongRoute(message->x,message->y,message->yaw,lookahead)
        :zone_->Approaching(message->x,message->y,message->yaw,lookahead));
    approach_distance_m_=approaching_?lookahead:0.;
    if(mode_==PlanningMode::NORMAL && approaching_ && !handoff_pending_ &&
       zone_->ApproachingAlongRoute(message->x,message->y,message->yaw,std::max(1.,std::abs(message->speed)*.35)))
      beginHandoff(PlanningMode::JAMMING);
    publishPrepare();
  }
  zone_station_m_ = inside ? station : -1.0;
  detector_.OnZone(inside);
}

void GpsModeSource::feedbackCallback(const asmc_msgs::FrenetFeedback::ConstPtr& message) {
  if (!message || !recovering_) return;
  if(!recoveryReady()) {ready_frames_=0;return;}
  const double age = (ros::Time::now() - message->header.stamp).toSec();
  if (!message->command_accepted || !message->plan_valid ||
      message->trajectory_source != asmc_msgs::FrenetFeedback::TRAJECTORY_NEW ||
      message->header.frame_id != "map" ||
      message->header.stamp <= recovery_source_time_ || age < 0.0 || age > 0.15) {
    ready_frames_ = 0;
    return;
  }
  if (message->header.stamp <= last_ready_stamp_) return;
  last_ready_stamp_ = message->header.stamp;
  if (++ready_frames_ >= 2) {
    recovering_ = false;
    if(smooth_handoff_)beginHandoff(PlanningMode::NORMAL);
    else publishMode(PlanningMode::NORMAL, "two fresh normal trajectories ready");
  }
}

bool GpsModeSource::recoveryReady() const {
  return !require_recovery_ready_ || (recovery_ready_ &&
      !recovery_ready_received_.isZero() &&
      (ros::SteadyTime::now()-recovery_ready_received_).toSec()<=.2);
}

void GpsModeSource::timerCallback(const ros::SteadyTimerEvent&) {
  const double now = Now();
  if (detector_.Update(now)) onDetectorChange();
  if(smooth_handoff_ && handoff_pending_ && detector_.mode()==PlanningMode::NORMAL &&
     (ros::SteadyTime::now()-handoff_started_).toSec()>.5){
    handoff_pending_=false;handoff_request_.valid=false;handoff_request_pub_.publish(handoff_request_);
  }
  if(smooth_handoff_ && handoff_pending_) {
    handoff_request_pub_.publish(handoff_request_);
    if(detector_.mode()==PlanningMode::JAMMING && (ros::SteadyTime::now()-handoff_started_).toSec()>.25)
    {publishMode(static_cast<PlanningMode>(handoff_request_.target_mode),"handoff preparation deadline; target safety checks required");
      handoff_pending_=false;handoff_request_.valid=false;handoff_request_pub_.publish(handoff_request_);}
  }
  publishPrepare();
  if (recovering_ && (ros::SteadyTime::now() - recovery_started_).toSec() >= recovery_timeout_s_) {
    if(require_recovery_ready_) {
      // A deadline must not bypass an invalid or unvalidated research anchor.
      // Keep the selected research path's stop/selector timeout authoritative;
      // return only after both recovery readiness and fresh NORMAL plans exist.
      ROS_WARN_THROTTLE(2.,"GPS handoff pending: validated shadow recovery and two fresh normal plans required");
    } else {
      recovering_ = false;
      // Do not drive indefinitely with the camera after a failed handoff.
      // The normal controller/selector's existing safe stop remains authoritative.
      publishMode(PlanningMode::NORMAL, "GPS recovery deadline; normal safety checks apply");
    }
  }
  publishLidarPause();
  publishGpsReturn();
  if (last_status_s_ < 0.0 || now - last_status_s_ >= 0.5) publishStatus(now);
}

void GpsModeSource::onDetectorChange() {
  if (detector_.mode() == PlanningMode::JAMMING) {
    // 복귀 대기 중 재진입하면 대기를 취소한다.
    recovering_ = false;
    ready_frames_ = 0;
    recovery_ready_=false;
    if(smooth_handoff_) {
      handoff_started_=ros::SteadyTime::now();
      if(mode_!=PlanningMode::JAMMING || !handoff_request_.valid ||
         (handoff_pending_ && handoff_request_.target_mode==static_cast<uint8_t>(PlanningMode::JAMMING)))
        beginHandoff(PlanningMode::JAMMING);
      else {handoff_pending_=false;handoff_request_.valid=false;handoff_request_.epoch=ros::SteadyTime::now().toNSec();handoff_request_pub_.publish(handoff_request_);}
    } else publishMode(PlanningMode::JAMMING, JammingReasonName(detector_.entry_reason()));
    return;
  }
  if(smooth_handoff_ && mode_==PlanningMode::NORMAL && handoff_pending_) {
    handoff_pending_=false;handoff_request_.valid=false;
    handoff_request_.epoch=ros::SteadyTime::now().toNSec();handoff_request_pub_.publish(handoff_request_);
  }
  if (mode_ != PlanningMode::JAMMING || recovering_) return;
  // GPS returning does not mean source-aligned pose, LiDAR objects and a
  // usable normal trajectory have already arrived. Keep the selected shadow
  // controller until handoff; the opt-in anchor gate cannot be timed out.
  recovering_ = true;
  recovery_started_ = ros::SteadyTime::now();
  recovery_source_time_ = ros::Time::now();
  last_ready_stamp_ = ros::Time();
  ready_frames_ = 0;
  if (require_recovery_ready_)
    ROS_INFO("Jamming cleared: waiting for validated anchor and fresh normal trajectories");
  else
    ROS_INFO("Jamming cleared: waiting for fresh normal trajectories (maximum %.1f s)", recovery_timeout_s_);
  publishStatus(Now());
}

void GpsModeSource::beginHandoff(PlanningMode target) {
  if(handoff_pending_ && handoff_request_.target_mode==static_cast<uint8_t>(target))return;
  handoff_request_={};handoff_request_.header.stamp=ros::Time::now();
  handoff_request_.epoch=ros::SteadyTime::now().toNSec();
  handoff_request_.target_mode=static_cast<uint8_t>(target);handoff_request_.valid=true;
  handoff_pending_=true;handoff_started_=ros::SteadyTime::now();
  handoff_request_pub_.publish(handoff_request_);publishPrepare();
}

void GpsModeSource::candidateCallback(const asmc_msgs::ControlHandoff::ConstPtr& m) {
  if(!handoff_pending_ || !asmc_msgs::handoff::Candidate(*m,handoff_request_,ros::Time::now()))return;
  if(m->header.frame_id!=(m->target_mode==0?"/mpc_controller":jamming_controller_))return;
  if(m->target_mode==0 && (!recoveryReady()||detector_.mode()!=PlanningMode::NORMAL))return;
  if(m->target_mode==1 && detector_.mode()!=PlanningMode::JAMMING)return;
  publishMode(static_cast<PlanningMode>(m->target_mode),"fresh prepared controller command ready");
  handoff_pending_=false;publishPrepare();
}

void GpsModeSource::publishPrepare() {
  if(!smooth_handoff_)return;
  std_msgs::Bool approach;approach.data=approaching_ && mode_==PlanningMode::NORMAL;approach_pub_.publish(approach);
  std_msgs::Bool prepare;prepare.data=approaching_ || mode_==PlanningMode::JAMMING ||
      (handoff_pending_&&handoff_request_.target_mode==1);
  prepare_pub_.publish(prepare);
}

void GpsModeSource::publishMode(PlanningMode next, const char* reason) {
  if (next == mode_) return;
  ROS_WARN("Planning mode %s -> %s: %s", PlanningModeName(mode_),
           PlanningModeName(next), reason);
  mode_ = next;
  std_msgs::UInt8 output;
  output.data = static_cast<std::uint8_t>(mode_);
  mode_pub_.publish(output);
  publishStatus(Now());
}

void GpsModeSource::publishLidarPause() {
  const bool pause = mode_ == PlanningMode::JAMMING && detector_.cause() != JammingReason::NONE;
  if (static_cast<int>(pause) == lidar_pause_) return;
  lidar_pause_ = pause;
  std_msgs::Bool output;
  output.data = pause;
  lidar_pause_pub_.publish(output);
}

void GpsModeSource::publishGpsReturn() {
  const bool gps_return = mode_ == PlanningMode::JAMMING && detector_.cause() == JammingReason::NONE;
  if (static_cast<int>(gps_return) == gps_return_) return;
  gps_return_ = gps_return;
  std_msgs::Bool output;
  output.data = gps_return;
  gps_return_pub_.publish(output);
}

void GpsModeSource::publishStatus(double now) {
  last_status_s_ = now;
  char text[320];
  std::snprintf(text, sizeof(text),
                "mode=%s detector=%s recovering=%d reason=%s cause=%s gps=%s gps_age=%.2f zone=%s station=%.1f clear=%.2f",
                PlanningModeName(mode_), PlanningModeName(detector_.mode()), recovering_ ? 1 : 0,
                JammingReasonName(detector_.entry_reason()),
                JammingReasonName(detector_.cause()),
                !detector_.has_gps() ? "none" : detector_.gps_zero() ? "zero"
                    : detector_.gps_no_fix() ? "no_fix" : "valid",
                detector_.gps_age(now),
                !zone_ ? "off" : detector_.in_zone() ? "in" : "out",
                zone_station_m_, detector_.clear_elapsed(now));
  std_msgs::String status;
  status.data = text;
  status_pub_.publish(status);
}
}  // namespace asmc_planner
