#include "Planning/GpsModeSource.hpp"

#include <cstdio>
#include <string>
#include <utility>

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
}  // namespace

GpsModeSource::GpsModeSource(ros::NodeHandle nh)
    : nh_(std::move(nh)), detector_(LoadConfig(ros::NodeHandle(nh_, "planning_mode"))) {
  ros::NodeHandle params(nh_, "planning_mode");
  bool zone_enabled = true;
  std::string path_file;
  params.param("shadow_zone/enabled", zone_enabled, zone_enabled);
  params.param("shadow_zone/path_file", path_file, std::string());
  // 단독 실행(시험 노드)처럼 경로가 주어지지 않으면 GPS 값 규칙만 쓴다.
  if (zone_enabled && !path_file.empty()) {
    double start_m = 0.0, end_m = 0.0, lateral_m = 10.0, heading_rad = 1.0;
    params.param("shadow_zone/station_start_m", start_m, start_m);
    params.param("shadow_zone/station_end_m", end_m, end_m);
    params.param("shadow_zone/lateral_tolerance_m", lateral_m, lateral_m);
    params.param("shadow_zone/heading_tolerance_rad", heading_rad, heading_rad);
    // 경로를 못 읽으면 조용히 끄지 않는다. 음영 진입을 놓치면 MPC가 GPS 없이 달린다.
    zone_.reset(new ShadowZone(path_file, start_m, end_m, lateral_m, heading_rad));
    std::string state_topic;
    params.param("shadow_zone/state_topic", state_topic, std::string("/planning/vehicle/state"));
    state_sub_ = nh_.subscribe(state_topic, 1, &GpsModeSource::stateCallback, this);
    ROS_INFO("Planning mode shadow zone: station %.1f-%.1f m (state %s)",
             start_m, end_m, state_topic.c_str());
  }
  mode_pub_ = nh_.advertise<std_msgs::UInt8>("/planning/mode", 1, true);
  status_pub_ = nh_.advertise<std_msgs::String>("/planning/mode/status", 1);
  gps_sub_ = nh_.subscribe("/gps", 1, &GpsModeSource::gpsCallback, this);
  feedback_sub_ = nh_.subscribe("/planning/frenet_feedback_v2", 1,
                                &GpsModeSource::feedbackCallback, this);
  std_msgs::UInt8 initial;
  initial.data = static_cast<std::uint8_t>(mode_);
  mode_pub_.publish(initial);
  publishStatus(Now());
  timer_ = nh_.createSteadyTimer(ros::WallDuration(0.05), &GpsModeSource::timerCallback, this);
  ROS_INFO("Planning mode NORMAL: waiting for raw /gps");
}

void GpsModeSource::gpsCallback(const morai_msgs::GPSMessage::ConstPtr& message) {
  if (!message) return;
  const double now = Now();
  detector_.OnGps(*message, now);
  // 0 좌표·no-fix 진입은 타이머를 기다리지 않고 같은 메시지에서 반영한다.
  if (detector_.Update(now)) onDetectorChange();
}

void GpsModeSource::stateCallback(const asmc_msgs::VehicleState::ConstPtr& message) {
  // blackout 동안의 추정 자세로 구간을 다시 판정하지 않는다. 마지막 판정을 유지한다.
  if (!message || !message->valid || detector_.gps_zero() || detector_.gps_no_fix() ||
      detector_.cause() == JammingReason::GPS_TIMEOUT) return;
  double station = -1.0;
  const bool inside = zone_->Contains(message->x, message->y, message->yaw, &station);
  zone_station_m_ = inside ? station : -1.0;
  detector_.OnZone(inside);
}

void GpsModeSource::feedbackCallback(const asmc_msgs::FrenetFeedback::ConstPtr& message) {
  if (!message || !recovering_) return;
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
    publishMode(PlanningMode::NORMAL, "two fresh normal trajectories ready");
  }
}

void GpsModeSource::timerCallback(const ros::SteadyTimerEvent&) {
  const double now = Now();
  if (detector_.Update(now)) onDetectorChange();
  if (recovering_ && (ros::SteadyTime::now() - recovery_started_).toSec() >= 5.0) {
    recovering_ = false;
    // Do not drive indefinitely with the camera after a failed handoff.
    // The normal controller/selector's existing safe stop remains authoritative.
    publishMode(PlanningMode::NORMAL, "GPS recovery deadline; normal safety checks apply");
  }
  if (last_status_s_ < 0.0 || now - last_status_s_ >= 0.5) publishStatus(now);
}

void GpsModeSource::onDetectorChange() {
  if (detector_.mode() == PlanningMode::JAMMING) {
    // 복귀 대기 중 재진입하면 대기를 취소한다.
    recovering_ = false;
    ready_frames_ = 0;
    publishMode(PlanningMode::JAMMING, JammingReasonName(detector_.entry_reason()));
    return;
  }
  if (mode_ != PlanningMode::JAMMING || recovering_) return;
  // GPS returning does not mean source-aligned pose, LiDAR objects and a
  // usable normal trajectory have already arrived. Keep the live camera
  // controller selected for this bounded handoff interval.
  recovering_ = true;
  recovery_started_ = ros::SteadyTime::now();
  recovery_source_time_ = ros::Time::now();
  last_ready_stamp_ = ros::Time();
  ready_frames_ = 0;
  ROS_INFO("Jamming cleared: waiting for fresh normal trajectories (maximum 5 s)");
  publishStatus(Now());
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
