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
  // A recognized route-associated signal uses an ordinary cruise-speed
  // ceiling. Frenet therefore generates a jerk/acceleration-constrained KEEP
  // trajectory instead of entering STOP or EMERGENCY_STOP for a green light.
  double visible_signal_speed_limit = 20.0 / 3.6;
};

class HardRuleFilter {
 public:
  void setTrafficStopConfig(const TrafficStopConfig& config) {
    traffic_stop_config_ = config;
  }
  HardConstraint apply(const UrbanFeature&, std::string*) const;

 private:
  TrafficStopConfig traffic_stop_config_;
};

}  // namespace behavior_planner
