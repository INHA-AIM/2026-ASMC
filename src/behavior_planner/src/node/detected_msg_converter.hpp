#pragma once

#include <asmc_msgs/DetectedObject.h>
#include <asmc_msgs/DetectedObjectArray.h>

#include "behavior_planner/feature/urban_feature.hpp"

namespace behavior_planner {

class DetectedMsgConverter {
 public:
  static PerceptionObjects toPerceptionObjects(
      const asmc_msgs::DetectedObjectArray& message);

 private:
  static bool toTrackedObject(const asmc_msgs::DetectedObject& message,
                              ObjectType type,
                              TrackedObject* object);
};

}  // namespace behavior_planner
