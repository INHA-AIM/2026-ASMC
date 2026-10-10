#pragma once

#include <planning_inputs/DetectedObject.h>
#include <planning_inputs/DetectedObjectArray.h>

#include "Behavior/Feature/urban_feature.hpp"

namespace behavior_planner {

class DetectedMsgConverter {
 public:
  static PerceptionObjects toPerceptionObjects(
      const planning_inputs::DetectedObjectArray& message);

 private:
  static bool toTrackedObject(const planning_inputs::DetectedObject& message,
                              ObjectType type,
                              TrackedObject* object);
};

}  // namespace behavior_planner
