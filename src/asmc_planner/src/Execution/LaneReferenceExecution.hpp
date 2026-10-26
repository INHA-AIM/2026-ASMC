#pragma once
#include "Execution/ReferenceHandover.hpp"
#include "Frenet/frenet_planner.hpp"
#include <functional>

namespace asmc_planner::lane_reference {
struct HandoverContinuity {
  double overlap_s=0, position_m=0, heading_rad=0, curvature_inv_m=0, speed_mps=0, acceleration_mps2=0;
};
struct ReferenceExecutionInput {
  HandoverObservation observation;
  std::vector<TrackedObject> objects;
  MapIdentity map;
  std::uint64_t sequence=0;
};
struct LaneObjectProjection { int id=-1; ReferenceProjection projection; };
struct ActiveReferenceFrame {
  std::uint64_t ref_epoch=1, input_sequence=0;
  double ego_stamp_s=0, objects_stamp_s=0, route_s=0;
  ReferenceProjection ego;
  std::vector<LaneObjectProjection> objects;
  int leader_id=-1;
  BehaviorState mode=LANE_KEEPING;
  CartesianPath path;
};

// One owner for core + reference + projections + leader + next trajectory.
// Construction (including both Init calls) is preparation work, not a 20Hz job.
// Shared by production and the opt-in ROS trial. It does not select/authorize
// highway gaps; that remains the existing Behavior/Frenet handshake's job.
class LaneReferenceExecution {
 public:
  LaneReferenceExecution(std::unique_ptr<FrenetPlanner> source,
      std::unique_ptr<FrenetPlanner> target,std::shared_ptr<const LaneReference> lane,
      HandoverOptions alignment,HandoverContinuity continuity,double end_guard_s,
      double arrival_s);
  // Production owns prebuilt cores for the lifetime of the runtime. Borrowed
  // cores share the same execution manager; only the active pointer changes.
  LaneReferenceExecution(FrenetPlanner& source,FrenetPlanner& target,
      std::shared_ptr<const LaneReference> lane,HandoverOptions alignment,
      HandoverContinuity continuity,double end_guard_s,double arrival_s,
      bool install_target_reference=true);
  void resetDormant();
  FrenetPlanner& planner() { return *current_; }
  std::uint64_t epoch() const { return epoch_; }
  bool adopted() const { return bool(frame_); }
  const ActiveReferenceFrame* frame() const { return frame_.get(); }
  const ActiveReferenceFrame* preparedFrame() const { return pending_.get(); }
  const LaneReference& target() const { return *lane_; }
  bool begin(std::uint32_t command_id,std::uint64_t source_epoch=0);
  bool prepare(const ReferenceExecutionInput& input,const CartesianPath& executing,
      double desired_speed,std::string& detail);
  // The callback performs the existing manager's completion transition. It
  // must return false without mutation on failure. No allocation/plan occurs
  // after success: owner swaps are noexcept on this single planning thread.
  bool commit(const ReferenceExecutionInput& latest,const std::function<bool()>& complete,
      std::string& detail);
  bool continueLane(const ReferenceExecutionInput& input,double desired_speed,
      std::string& detail);
 private:
  std::unique_ptr<FrenetPlanner> owned_source_, owned_target_;
  FrenetPlanner *current_=nullptr,*staged_=nullptr,*source_=nullptr,*target_=nullptr;
  RefLine mission_;
  std::shared_ptr<const LaneReference> lane_;
  ReferenceHandoverGate gate_;
  HandoverOptions alignment_;
  HandoverContinuity continuity_;
  double end_guard_s_=0, arrival_s_=0;
  bool finite_target_=true;
  std::uint32_t command_id_=0;
  std::uint64_t epoch_=1;
  std::unique_ptr<ActiveReferenceFrame> frame_, pending_;
  bool build(FrenetPlanner&,const ReferenceExecutionInput&,double,
      ActiveReferenceFrame&,std::string&);
  bool validInput(const ReferenceExecutionInput&) const;
  void initialize(bool install);
};
bool HandoverPathsContinuous(const CartesianPath& old_path,const CartesianPath& next,
    double dt,const HandoverContinuity& limits);
// After actual target-lane settling, two freshly planned trajectories may
// differ longitudinally (e.g. cruise -> FOLLOW). Compare steering geometry at
// equal physical distance and retain strict speed/acceleration at the boundary.
bool HandoverPathsGeometricallyContinuous(const CartesianPath&,const CartesianPath&,
    double dt,const HandoverContinuity&);
}  // namespace asmc_planner::lane_reference
