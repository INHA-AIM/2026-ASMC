#pragma once

#include <cstddef>
#include <string>
#include <vector>
#include <planning_inputs/DetectedObjectArray.h>

namespace asmc_planner {
// Immutable HD-map mission scope, shared by Behavior and Frenet. Stations are
// measured on the metadata's official route, never a planner's active reference.
class StaticMissionPolicy {
 public:
  StaticMissionPolicy(const std::string& metadata_file, const std::string& data_root,
                      double lateral_tolerance_m = 10.0,
                      double heading_tolerance_rad = 1.0);
  bool contains(double x, double y, double* station = nullptr) const;
  bool containsEgo(double x, double y, double yaw, bool valid) const;
  // Call on a fresh copy of the original observation each cycle. Policy type
  // and effective motion policy change; raw velocity, labels, validity, IDs and
  // OBBs survive. This mission prior does not assert a measured zero velocity.
  std::size_t apply(double ego_x, double ego_y, double ego_yaw, bool ego_valid,
                    planning_inputs::DetectedObjectArray& objects) const;
  double start() const { return start_; }
  double end() const { return end_; }

  // Consumer-local cache: immutable ROS observations may be reused across
  // timer ticks, but ego scope must still be evaluated on every tick.
  class Observation {
   public:
    bool update(const StaticMissionPolicy& policy,
                const planning_inputs::DetectedObjectArray::ConstPtr& source,
                double x, double y, double yaw, bool valid);
    const planning_inputs::DetectedObjectArray& objects() const { return effective_; }
    bool egoInScope() const { return ego_in_scope_; }
    std::size_t remapped() const { return remapped_; }
   private:
    const StaticMissionPolicy* policy_ = nullptr;
    planning_inputs::DetectedObjectArray::ConstPtr source_;
    planning_inputs::DetectedObjectArray effective_;
    bool ego_in_scope_ = false;
    std::size_t remapped_ = 0;
  };

 private:
  struct Point { double x, y, s; };
  struct Bounds { double min_x, min_y, max_x, max_y; };
  struct Segment {
    Point a;
    double dx, dy, squared, ds, yaw;
    Bounds bounds;
    std::size_t order;
  };
  struct Node {
    Bounds bounds;
    std::size_t begin, end, left = 0, right = 0;
  };
  std::size_t buildIndex(std::size_t begin, std::size_t end);
  void nearest(std::size_t node, double x, double y, double& best,
               std::size_t& selected, double& station) const;
  std::size_t remapObjects(planning_inputs::DetectedObjectArray& objects) const;
  bool project(double x, double y, double& station, double& yaw) const;
  std::vector<Segment> segments_;
  std::vector<Node> nodes_;
  double start_ = 0, end_ = 0;
  double lateral_tolerance_, heading_tolerance_;
};
}  // namespace asmc_planner
