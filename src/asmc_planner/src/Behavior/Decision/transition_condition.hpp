#pragma once
#include "Behavior/Context/behavior_types.hpp"
namespace behavior_planner {
inline bool isLaneChange(BehaviorType b) {
  return b == BehaviorType::LEFT_CHANGE || b == BehaviorType::RIGHT_CHANGE;
}
}
