#pragma once
#include "Behavior/Feature/urban_feature.hpp"
#include "Behavior/Map/route_manager.hpp"
#include "Frenet/Map/AvoidRoadMap.hpp"
#include "Execution/PlannerFeedbackData.hpp"

namespace behavior_planner {
// Immutable references owned by the existing FrenetPlanner. No second map,
// tracking state, command lifecycle or hidden disappearance latch.
struct DynamicEnvironment {
  const AvoidRoadMap* road = nullptr;
  VehicleShape shape{};
  CollisionCheckConfig collision{};
  double lane_width = 3.5;
};
struct DynamicPolicyConfig {
  // Empty scope disables new actions; route ownership comes from configuration.
  std::vector<std::string> route_links;
  double section_speed_limit = 1e9;
  // Provisional simulation settings, not measured vehicle guarantees.
  double horizon_s = 4.0, step_s = .1, reaction_s = .3;
  double service_decel = 3.0, emergency_decel = 5.0;
  double avoid_max_speed = 1.0, restart_accel = 1.5;
};
class DynamicObjectPolicy {
 public:
  explicit DynamicObjectPolicy(DynamicEnvironment environment = {}, DynamicPolicyConfig config = {})
      : environment_(environment), config_(config) {}
  bool enabled() const { return environment_.road && environment_.road->enabled(); }
  bool inScope(const std::string& route_link) const;
  // Eligibility is checked before static AVOID overlap history is acquired.
  // Every canonical object remains in Frenet's collision input.
  bool allowsStaticAvoid(const ObjectFeature&, const RouteManager&) const;
  static PerceptionObjects legacyObjects(const PerceptionObjects&,bool scoped_dynamic_active,
      bool validated_native_avoid = false,const std::vector<int>& evaluated_provisional_ids = {},
      int nominated_in_lane_target = -1);
  static bool usesNativePathSafety(BehaviorType,PlannerExecutionState);
  DynamicDecision evaluate(const UrbanFeature&, const RouteManager&,
      const std::vector<::TrackedObject>&, double observation_age_s, bool ordinary_maneuver,
      bool previous_validated_stop = false) const;
 private:
  DynamicEnvironment environment_;
  DynamicPolicyConfig config_;
};
const char* DynamicActionName(DynamicAction);
}
