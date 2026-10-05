#pragma once
#include "behavior_planner/context/behavior_context.hpp"
#include "behavior_planner/feature/urban_feature.hpp"
namespace behavior_planner {

struct TrafficStopConfig {
  // 신호 정지를 고려하기 시작하는 최소 거리. 저속에서도 지도/신호의 짧은
  // 순간 누락 때문에 STOP이 늦어지지 않도록 동적 거리의 하한으로 사용한다.
  double minimum_detection_distance = 30.0;
  double comfortable_decel = 2.0;
  double braking_margin = 3.0;
  double target_offset = 5.0;
  double planner_stop_buffer = 0.5;
  // Map stop-line approach policy. This is deliberately independent from the
  // camera observation and from the RED/YELLOW STOP braking policy above.
  double approach_speed_limit = 25.0 / 3.6;
  double approach_comfortable_decel = 2.0;
  // Complete the speed transition this far before the physical stop line.
  double approach_margin = 1.5;
  // Keep the same cap briefly after crossing the physical stop line.
  double post_stopline_distance = 5.0;
};

// Computes the forward distance at which the ordinary intersection speed cap
// should become active. This helper has no ROS or signal-detection dependency.
double ComputeIntersectionApproachDistance(
    double current_speed, double approach_speed,
    double comfortable_decel, double margin);

class HardRuleFilter {
 public:
  void setTrafficStopConfig(const TrafficStopConfig& config) {
    traffic_stop_config_ = config;
    approach_latched_ = false;
    latched_stop_line_s_ = 1e9;
  }
  HardConstraint apply(const UrbanFeature&, std::string*);

 private:
  TrafficStopConfig traffic_stop_config_;
  bool approach_latched_ = false;
  double latched_stop_line_s_ = 1e9;
};

}  // namespace behavior_planner
