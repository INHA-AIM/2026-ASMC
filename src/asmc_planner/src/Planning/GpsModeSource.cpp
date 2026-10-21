#include "Planning/GpsModeSource.hpp"

#include <utility>
#include <stdexcept>
#include <planning_inputs/gps_transform.hpp>

namespace asmc_planner {
GpsModeSource::GpsModeSource(ros::NodeHandle nh) : nh_(std::move(nh)) {
  ros::NodeHandle config("/asmc_planner/gps_recovery");
  config.param("enabled",recovery_enabled_,false);
  if(recovery_enabled_) {
    double residual=0;int confirmations=0;
    if(!config.getParam("maximum_position_residual_m",residual) ||
       !config.getParam("required_gps_samples",confirmations) ||
       !config.getParam("maximum_past_s",maximum_past_s_) ||
       !config.getParam("maximum_future_s",maximum_future_s_) ||
       !config.getParam("maximum_receive_age_s",maximum_receive_age_s_) ||
       !nh_.getParam("/planning_gps_imu_state_adapter/gps_sensor_offset/x",gps_offset_x_) ||
       !nh_.getParam("/planning_gps_imu_state_adapter/gps_sensor_offset/y",gps_offset_y_) ||
       !std::isfinite(residual)||residual<=0||confirmations<1 ||
       !std::isfinite(maximum_past_s_)||maximum_past_s_<=0 ||
       !std::isfinite(maximum_future_s_)||maximum_future_s_<0 ||
       !std::isfinite(maximum_receive_age_s_)||maximum_receive_age_s_<=0 ||
       !std::isfinite(gps_offset_x_)||!std::isfinite(gps_offset_y_))
      throw std::runtime_error("Missing/invalid GPS recovery or selected estimator mounting configuration");
    recovery_=GpsRecoveryGate(residual,static_cast<unsigned>(confirmations));
    std::string state_topic;
    nh_.param<std::string>("/behavior_planner/topics/ego",state_topic,"/planning/vehicle/state");
    state_sub_=nh_.subscribe(state_topic,1,&GpsModeSource::stateCallback,this);
  }
  mode_pub_ = nh_.advertise<std_msgs::UInt8>("/planning/mode", 1, true);
  gps_sub_ = nh_.subscribe("/gps", 1, &GpsModeSource::gpsCallback, this);
  std_msgs::UInt8 initial;
  initial.data = static_cast<std::uint8_t>(mode_);
  mode_pub_.publish(initial);
  ROS_INFO("Planning mode NORMAL: waiting for raw /gps");
}

void GpsModeSource::gpsCallback(const morai_msgs::GPSMessage::ConstPtr& message) {
  if (!message) return;
  const PlanningMode next = DetectPlanningMode(*message);
  gps_=*message;gps_received_=ros::SteadyTime::now();
  if(next==PlanningMode::JAMMING)recovery_.reset();
  if(recovery_enabled_ && mode_==PlanningMode::JAMMING && next==PlanningMode::NORMAL) {
    tryRecover();return;
  }
  if (next == mode_) return;
  ROS_WARN("Planning mode %s -> %s: raw GPS latitude/longitude/altitude %s",
           PlanningModeName(mode_), PlanningModeName(next),
           next == PlanningMode::JAMMING ? "all exactly zero" : "not all zero");
  publishMode(next);
}

void GpsModeSource::stateCallback(const asmc_msgs::VehicleState::ConstPtr& message) {
  if(!message)return;
  state_=*message;state_received_=ros::SteadyTime::now();tryRecover();
}

void GpsModeSource::tryRecover() {
  if(mode_!=PlanningMode::JAMMING || DetectPlanningMode(gps_)==PlanningMode::JAMMING)return;
  const auto now=ros::SteadyTime::now();
  const bool fresh=!gps_received_.isZero()&&!state_received_.isZero() &&
      (now-gps_received_).toSec()<=maximum_receive_age_s_ &&
      (now-state_received_).toSec()<=maximum_receive_age_s_;
  if(!fresh || !state_.valid || state_.header.frame_id!="map" ||
      state_.source!=asmc_msgs::VehicleState::SOURCE_GPS_IMU || gps_.status<=0 ||
      !std::isfinite(gps_.latitude)||!std::isfinite(gps_.longitude)||
      !std::isfinite(gps_.eastOffset)||!std::isfinite(gps_.northOffset)) {
    recovery_.observe(gps_.header.stamp.toSec(),{});return;
  }
  double east=0,north=0;GpsToUtm52N(gps_.latitude,gps_.longitude,east,north);
  const auto residual=GpsPoseResidual({state_.header.stamp.toSec(),state_.x,state_.y,
      state_.yaw,state_.speed,state_.yaw_rate},gps_.header.stamp.toSec(),
      east-gps_.eastOffset,north-gps_.northOffset,gps_offset_x_,gps_offset_y_,
      maximum_past_s_,maximum_future_s_);
  if(!recovery_.observe(gps_.header.stamp.toSec(),residual)) {
    ROS_WARN_THROTTLE(1.0,"GPS recovery pending: source-aligned residual=%.3f m, confirmations=%u",
        residual?*residual:-1.0,recovery_.confirmations());
    return;
  }
  ROS_INFO("GPS recovery ready: %u coherent source observations, residual=%.3f m",recovery_.confirmations(),*residual);
  publishMode(PlanningMode::NORMAL);
}

void GpsModeSource::publishMode(PlanningMode mode) {
  mode_ = mode;
  std_msgs::UInt8 output;
  output.data = static_cast<std::uint8_t>(mode_);
  mode_pub_.publish(output);
}
}  // namespace asmc_planner
