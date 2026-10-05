#pragma once
#include <ros/ros.h>
#include <asmc_msgs/TrackedObjectArray.h>
#include <planning_inputs/DetectedObjectArray.h>
#include "planning_inputs/filters/DrivableBoundaryFilter.hpp"
#include "planning_inputs/filters/KnownStaticObjectFilter.hpp"
#include "planning_inputs/filters/StaticStructureExclusionFilter.hpp"

namespace planning_inputs {
// Apply the validated final-publication policy only to private planner inputs.
// No point-cloud processing, main publications or frozen schemas are changed.
class PlanningObjectFilter {
 public:
  explicit PlanningObjectFilter(ros::NodeHandle& parameters);
  void Apply(const asmc_msgs::TrackedObjectArray& source,
             DetectedObjectArray* output);
 private:
  DrivableBoundaryFilter road_;
  KnownStaticObjectFilter known_;
  StaticStructureExclusionFilter structure_;
  ros::Publisher debug_;
};
}
