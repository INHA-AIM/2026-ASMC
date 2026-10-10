#pragma once
#include "Behavior/Feature/feature_builder.hpp"
namespace asmc_planner {
// Immutable parser facts. Behavior and Frenet retain their deliberately
// different object representations and conversion rules.
struct BehaviorContext {
  const behavior_planner::EgoState& ego;
  const behavior_planner::PerceptionObjects& perception;
  const behavior_planner::RouteManager& route;
  const behavior_planner::TrafficLightProvider& traffic;
};
}
