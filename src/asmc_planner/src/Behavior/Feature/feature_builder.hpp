#pragma once
#include <unordered_map>
#include <functional>

#include "Behavior/Feature/object_projector.hpp"
#include "Parser/Traffic/traffic_light_provider.hpp"
namespace behavior_planner {
struct SafetyConfig {
  double front_min_gap = 8.0;
  double emergency_ttc_enter = 2.0;
  double emergency_predicted_ttc = 2.0;
  double emergency_required_decel = 5.0;
  double pedestrian_stop_distance = 15.0;
};
struct FollowTargetConfig {
  // Keep these acceptance limits no wider than Frenet's FOLLOW target
  // validation. Behavior owns target selection, so it must not select a
  // target which Frenet will immediately reject.
  double max_distance = 60.0;
  double max_lateral_offset = 1.75;
};
struct AvoidTargetConfig {
  double max_distance = 120.0;
  // Legacy acquisition ceiling. The actual AVOID corridor is narrower and is
  // derived from ego half width + path_margin_m.
  double max_lateral_offset = 1.75;
  double ego_width = 1.892;
  double path_margin_m = 0.4;
  // OBB가 실제 차체 swept corridor를 이 이상 침범하면 즉시 AVOID한다.
  // 10 cm voxel보다 작은 경계 접촉은 측정 잡음으로 취급한다.
  double min_physical_overlap_m = 0.10;
  // 실제 차체에는 닿지 않고 안전 margin에만 들어온 객체는 충분한 침범량과
  // 객체 폭 대비 비율이 연속 확인된 경우에만 AVOID한다.
  double min_margin_overlap_m = 0.20;
  double min_overlap_ratio = 0.20;
  // 실제 차체 주행 폭을 가리는 비율. 분모는 OBB 투영 폭과 ego 폭 중
  // 작은 값이라서, 작은 장애물도 충분히 안쪽에 있으면 100%로 평가된다.
  double immediate_blocking_ratio = 0.70;
  double blocking_enter_ratio = 0.35;
  double blocking_exit_ratio = 0.15;
  int overlap_confirm_cycles = 2;
  int overlap_release_cycles = 3;
  // LiDAR tracker ID가 짧게 끊기거나 바뀌어도 같은 위치의 단일 객체라면
  // overlap latch를 승계한다. 여러 객체가 동시에 매칭되는 경우에는
  // 잘못된 대상 고정을 피하기 위해 재연결하지 않는다.
  int reconnect_grace_cycles = 5;
  double reconnect_max_longitudinal_delta_m = 3.0;
  double reconnect_max_lateral_delta_m = 0.8;
  double reconnect_max_size_delta_m = 1.5;
};
class FeatureBuilder {
 public:
  void setRoundaboutMerge(bool enabled, const std::string& stop_node, const std::string& source, const std::string& target) {
    roundabout_enabled_=enabled;roundabout_stop_node_=stop_node;roundabout_source_=source;roundabout_target_=target;
  }
  void setStopTargetOffset(double offset_m) {
    stop_target_offset_m_ = offset_m > 0.0 ? offset_m : 0.0;
  }
  void setSafetyConfig(const SafetyConfig& config) { safety_ = config; }
  void setFollowTargetConfig(const FollowTargetConfig& config) {
    follow_target_ = config;
  }
  void setAvoidTargetConfig(const AvoidTargetConfig& config) {
    avoid_target_ = config;
  }
  UrbanFeature build(const EgoState&, const PerceptionObjects&,
                     const RouteManager&, const TrafficLightProvider&,
                     const std::function<bool(const ObjectFeature&)>& static_avoid_eligible = {},
                     const std::function<PerceptionObjects(UrbanFeature&,const PerceptionObjects&)>& object_policy = {});

 private:
  struct AvoidOverlapState {
    int enter_cycles = 0;
    int exit_cycles = 0;
    int missing_cycles = 0;
    bool latched = false;
    bool geometry_valid = false;
    double last_s = 0.0;
    double last_d = 0.0;
    double last_x = 0.0;
    double last_y = 0.0;
    double last_length = 0.0;
    double last_width = 0.0;
    double last_projected_longitudinal_length = 0.0;
  };

  // FSM이 FOLLOW/AVOID 전환을 결정할 수 있도록, 직전 선행 객체와 현재 정적
  // 관측이 같은 물리 객체인지에 대한 관측 사실만 만든다. 실제 maneuver 유지
  // 여부는 FeatureBuilder가 결정하지 않는다.
  struct RecentLeadState {
    bool geometry_valid = false;
    int id = -1;
    int missing_cycles = 0;
    double s = 0.0;
    double d = 0.0;
    double length = 0.0;
    double width = 0.0;
  };

  bool roundabout_enabled_=false;
  std::string roundabout_stop_node_,roundabout_source_,roundabout_target_;
  int frame_ = 0;
  int last_lead_vehicle_id_ = -1;
  double stop_target_offset_m_ = 0.0;
  SafetyConfig safety_;
  FollowTargetConfig follow_target_;
  AvoidTargetConfig avoid_target_;
  std::unordered_map<int, AvoidOverlapState> avoid_overlap_states_;
  RecentLeadState recent_lead_;
  ObjectProjector projector_;
};
}
