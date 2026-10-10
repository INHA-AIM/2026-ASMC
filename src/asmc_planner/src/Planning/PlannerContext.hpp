#pragma once
#include <optional>
#include <string>
#include "Global/Global.hpp"
#include "Execution/Acceptance/CommandExecutionManager.hpp"
namespace asmc_planner {
struct RouteMapFacts {
  const std::string& source_link_id;
  const std::string& target_link_id;
};
struct TrafficFacts {
  TrafficLightState light;
  SignalMovement movement;
};
// Immutable facts from the existing snapshot. The compatibility port currently
// supplies its exact vehicle/object value types, avoiding a second conversion.
// Semantic requested intent and planning results are separate from this context.
template<class Vehicle, class Objects, class PlanningTime>
struct PlannerContext {
  const Vehicle& ego;
  const Objects& perception;
  const RouteMapFacts route;
  const frenet_planner_ros::ExecutionSnapshot execution;
  const PlanningTime planning_time;
  const std::optional<TrafficFacts> traffic; // Not supplied by the legacy Frenet adapter.
  PlannerContext(const Vehicle& v, const Objects& o, RouteMapFacts r,
      frenet_planner_ros::ExecutionSnapshot e, const PlanningTime& t)
    : ego(v), perception(o), route(r), execution(e), planning_time(t), traffic(std::nullopt) {}
};
}
