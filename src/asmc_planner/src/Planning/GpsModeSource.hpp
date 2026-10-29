#pragma once

#include <memory>

#include <asmc_msgs/FrenetFeedback.h>
#include <asmc_msgs/VehicleState.h>
#include <asmc_msgs/control_handoff.hpp>
#include <morai_msgs/GPSMessage.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>

#include "Planning/JammingDetector.hpp"
#include "Planning/PlanningMode.hpp"

namespace asmc_planner {
// /planning/mode의 단일 발행자. 판정 규칙은 JammingDetector가 소유한다.
// 파라미터는 /planning_mode/* 에서 읽는다(autonomy_no_bridges_v2.launch).
// 판정이 NORMAL로 돌아와도 정상 궤적이 준비될 때까지(최대 recovery_timeout_s) JAMMING 발행을 유지한다.
class GpsModeSource final {
 public:
  explicit GpsModeSource(ros::NodeHandle nh);
 private:
  void gpsCallback(const morai_msgs::GPSMessage::ConstPtr& message);
  void stateCallback(const asmc_msgs::VehicleState::ConstPtr& message);
  void feedbackCallback(const asmc_msgs::FrenetFeedback::ConstPtr& message);
  void timerCallback(const ros::SteadyTimerEvent&);
  void onDetectorChange();
  void publishMode(PlanningMode next, const char* reason);
  void publishStatus(double now);
  void publishLidarPause();
  void publishGpsReturn();
  ros::NodeHandle nh_;
  ros::Subscriber gps_sub_;
  ros::Subscriber state_sub_;
  ros::Subscriber feedback_sub_;
  ros::Subscriber recovery_ready_sub_;
  bool require_recovery_ready_=false,recovery_ready_=false;
  ros::SteadyTime recovery_ready_received_;
  bool recoveryReady() const;
  void beginHandoff(PlanningMode);
  void candidateCallback(const asmc_msgs::ControlHandoff::ConstPtr&);
  void publishPrepare();
  bool smooth_handoff_=false,handoff_pending_=false,approaching_=false;
  double approach_speed_mps_=0,approach_distance_m_=0;
  asmc_msgs::ControlHandoff handoff_request_;
  ros::SteadyTime handoff_started_;
  ros::Publisher handoff_request_pub_,prepare_pub_,approach_pub_;
  std::string jamming_controller_;
  ros::Subscriber candidate_sub_;
  ros::Publisher mode_pub_;
  ros::Publisher status_pub_;
  ros::Publisher lidar_pause_pub_;
  ros::Publisher gps_return_pub_;
  ros::SteadyTimer timer_;
  JammingDetector detector_;
  std::unique_ptr<ShadowZone> zone_;
  double zone_station_m_ = -1.0;
  double last_status_s_ = -1.0;
  PlanningMode mode_ = PlanningMode::NORMAL;  // 발행 중인 모드. detector_ 판정보다 늦게 NORMAL이 된다.
  bool recovering_ = false;
  ros::SteadyTime recovery_started_;
  ros::Time recovery_source_time_, last_ready_stamp_;
  unsigned ready_frames_ = 0;
  int lidar_pause_ = -1;  // 마지막 발행값, -1은 미발행.
  int gps_return_ = -1;   // 마지막 발행값, -1은 미발행.
  double recovery_timeout_s_ = 4.0;
};
}  // namespace asmc_planner
