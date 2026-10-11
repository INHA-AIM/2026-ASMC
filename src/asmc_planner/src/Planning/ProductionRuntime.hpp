#pragma once
#include "Parser/BehaviorInput.hpp"
#include "Planning/PlanningPort.hpp"
#include "Planning/GpsModeSource.hpp"
#include <memory>
#include <ros/callback_queue.h>
namespace asmc_planner {
// All implementations and libraries belong to asmc_planner. The two adapter
// queues preserve independent legacy steady timers without an internal topic.
class ProductionRuntime final {
 public:
  ProductionRuntime();
  ~ProductionRuntime();
  void start();
 private:
  static ros::NodeHandle handle(const std::string&, ros::CallbackQueue&);
  ros::CallbackQueue planning_queue_, behavior_queue_;
  std::unique_ptr<frenet_planner_ros::PlanningPort> planning_;
  std::unique_ptr<behavior_planner::BehaviorInput> behavior_;
  std::unique_ptr<GpsModeSource> mode_source_;
  ros::AsyncSpinner planning_spinner_, behavior_spinner_;
};
}
