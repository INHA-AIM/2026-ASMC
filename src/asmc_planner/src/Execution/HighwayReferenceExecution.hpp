#pragma once
#include "Execution/LaneReferenceExecution.hpp"
#include "Execution/HighwayReferenceStatus.hpp"
#include <mutex>

namespace asmc_planner::lane_reference {
// In-process snapshot, shared by the existing Behavior and Planning queues.
// Mission station remains on the original route; local s/d belongs to lane.
class HighwayReferenceExecution {
 public:
  HighwayReferenceExecution(FrenetPlanner& mission,const std::string& config,
      execution::ManeuverState& execution_state);
  FrenetPlanner& planner() { return *active_; }
  const FrenetPlanner& planner() const { return *active_; }
  HighwayReferenceStatus status() const;
  const LaneReference& target(std::size_t zone) const { return *lanes_.at(zone); }
  bool begin(std::uint32_t command,std::size_t zone);
  bool beginLaneTransition(std::uint32_t command,const PlannerCommand&,
      const CartesianState&,bool optional,std::string&);
  const HighwayMergeRoute::Zone& transitionZone() const { return transition_zone_; }
  const LaneReference& transitionTarget() const { return *transition_target_; }
  bool hasLaneTransition(std::uint32_t id) const { return dynamic_&&command_==id; }
  // A preempted request loses command ownership, not the physical crossing
  // geometry. Recovery uses that geometry until source or target is settled.
  bool preserveInterruptedTransition(std::uint32_t next_command);
  bool recoveringTransition() const { return status().transition_interrupted; }
  const PlannerCommand& interruptedRequest() const { return transition_request_; }
  void finishInterruptedOnSource();
  FrenetPlanner& transitionPlanner() { return *transition_source_; }
  bool prepare(const ReferenceExecutionInput&,const CartesianPath&,double speed,std::string&);
  const ActiveReferenceFrame* preparedFrame() const;
  bool commit(const ReferenceExecutionInput&,const std::function<bool()>& complete,std::string&);
  // Restore the official reference only on the explicitly shared downstream
  // lane, after the same alignment/continuity/fresh-input certificate.
  bool prepareMissionReturn(const ReferenceExecutionInput&,const CartesianPath&,
      double speed,std::string&);
  bool commitMissionReturn(const ReferenceExecutionInput&,std::string&);
  void guardFiniteEnd(const CartesianState&,PlannerCommand&) const;
  bool validActivePath(const CartesianPath&) const;
  void observeMissionStation(double s);
 private:
  FrenetPlanner& mission_;
  execution::ManeuverState& execution_state_;
  FrenetPlanner* active_;
  std::shared_ptr<const HighwayMergeRoute> route_;
  std::vector<std::unique_ptr<FrenetPlanner>> cores_;
  std::vector<std::shared_ptr<const LaneReference>> lanes_;
  std::vector<std::unique_ptr<LaneReferenceExecution>> entries_,returns_;
  LaneReferenceOptions options_;
  HandoverOptions alignment_;
  HandoverContinuity continuity_;
  std::shared_ptr<HighwayLaneChangeNetwork> network_;
  std::vector<std::unique_ptr<FrenetPlanner>> lane_cores_;
  std::unique_ptr<LaneReferenceExecution> dynamic_entry_,dynamic_return_,staged_return_;
  LaneReferenceExecution* active_return_=nullptr;
  FrenetPlanner* transition_source_=nullptr;
  std::shared_ptr<const LaneReference> transition_target_,transition_arrival_;
  HighwayMergeRoute::Zone transition_zone_;
  PlannerCommand transition_request_;
  bool dynamic_=false,optional_=false;
  std::size_t zone_=0,active_zone_=0;
  std::uint32_t command_=0;
  double last_mission_s_=-1,end_guard_s_=0;
  bool pending_return_=false;
  mutable std::mutex status_mutex_;
  HighwayReferenceStatus status_;
};
}  // namespace asmc_planner::lane_reference
