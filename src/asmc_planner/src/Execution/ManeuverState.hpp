#pragma once
#include "Global/Global.hpp"
#include "Execution/ManeuverFacts.hpp"
namespace asmc_planner::execution {
enum class AvoidPhase { TRACKING, SHIFT, PASS, RETURN };
enum class LaneChangePhase { IDLE, SHIFT, HOLD_TARGET_LANE };
enum class StopExecutionPhase { APPROACH, BRAKING, HOLD };
struct AvoidState {
  bool active = false;
  int command_obstacle_id = -1;
  int obstacle_id = -1;
  AvoidPhase phase = AvoidPhase::TRACKING;
  int missing_cycles = 0;
  LateralDirection requested_direction = LateralDirection::NONE;
  bool committed = false;
  bool completed = false;
};
struct LaneChangeState {
  bool active = false;
  bool committed = false;
  LateralDirection direction = LateralDirection::NONE;
  LaneChangePhase phase = LaneChangePhase::IDLE;
};
struct FollowState { bool active = false; int leader_id = -1; int missing_cycles = 0; };
struct MergeState { bool committed = false; bool waiting_latched = false; };
struct ManeuverState {
  AvoidState avoid;
  LaneChangeState lane_change;
  FollowState follow;
  MergeState merge;
  StopExecutionPhase stop_phase = StopExecutionPhase::APPROACH;
};
// Standalone/v1 calculations use a local Execution owner. The v2 node binds
// this view to the acceptance owner's storage before Init. Copies retain the
// old planner's independent value semantics, including probe/rollback snapshots.
class ManeuverExecution {
 public:
  ManeuverExecution() = default;
  ManeuverExecution(const ManeuverExecution& other) : owned_(*other.operator->()) {}
  ManeuverExecution& operator=(const ManeuverExecution& other) {
    if (this != &other) { owned_ = *other.operator->(); bound_ = nullptr; }
    return *this;
  }
  void bind(ManeuverState& state) { bound_ = &state; }
  const ManeuverState* operator->() const { return writable(); }
  bool canCommitAvoid(const CommitFacts& facts) const {
    return facts.candidate_valid && writable()->avoid.active &&
           !writable()->avoid.committed && facts.selected_target_finite;
  }
  bool canCommitLaneChange(const CommitFacts& facts) const {
    return facts.candidate_valid && writable()->lane_change.active &&
           !writable()->lane_change.committed && facts.selected_target_finite;
  }
  bool canCommitMerge(bool enabled, bool epoch_matches, bool candidate_valid,
      bool ready, bool previous_ready, bool has_path) const {
    return enabled && !writable()->merge.committed && epoch_matches &&
           candidate_valid && ready && previous_ready && has_path;
  }
  void resetAvoid() const { writable()->avoid = AvoidState{}; }
  void restoreAvoid(const AvoidState& snapshot) const { writable()->avoid = snapshot; }
  void resetLaneChange() const { writable()->lane_change = LaneChangeState{}; }
  void resetFollow() const { writable()->follow = FollowState{}; }
  void resetMerge() const { writable()->merge = MergeState{}; }
  void resetStopTarget() const { writable()->stop_phase = StopExecutionPhase::APPROACH; }
  void observeAvoidObject(int observation_id) const {
    auto& state = writable()->avoid;
    state.active = true; state.obstacle_id = observation_id;
  }
  void bindAvoidCommandTarget(int command_object_id) const {
    writable()->avoid.command_obstacle_id = command_object_id;
  }
  void clearAvoidBinding() const {
    auto& state = writable()->avoid;
    state.active = false; state.command_obstacle_id = -1;
    state.obstacle_id = -1; state.missing_cycles = 0;
  }
  void observeAvoidVisible() const { writable()->avoid.missing_cycles = 0; }
  bool observeAvoidMissing(int grace_cycles) const {
    return ++writable()->avoid.missing_cycles > grace_cycles;
  }
  bool observeCommittedAvoidTarget(bool visible, int grace_cycles) const {
    auto& state = writable()->avoid;
    if (state.phase == AvoidPhase::TRACKING) {
      if (visible) state.missing_cycles = 0;
      else if (++state.missing_cycles > grace_cycles) return false;
    } else state.missing_cycles = 0;
    return true;
  }
  void requestAvoidDirection(LateralDirection direction) const {
    writable()->avoid.requested_direction = direction;
  }
  void beginAvoidCommit() const {
    writable()->avoid.committed = true; writable()->avoid.completed = false;
  }
  void initializeAvoidEntry(const AvoidEntryFacts& facts) const {
    writable()->avoid.phase = facts.entry_reached ? AvoidPhase::SHIFT : AvoidPhase::TRACKING;
  }
  bool observeAvoidEntry(const AvoidEntryFacts& facts) const {
    auto& state = writable()->avoid;
    if (!state.completed && state.phase == AvoidPhase::TRACKING && facts.entry_reached) {
      state.phase = AvoidPhase::SHIFT; return true;
    }
    return false;
  }
  bool observeAvoidShift(const AvoidShiftFacts& facts) const {
    auto& state = writable()->avoid;
    if (!state.completed && state.phase == AvoidPhase::SHIFT && facts.longitudinally_cleared) {
      state.phase = AvoidPhase::RETURN;
    } else if (!state.completed && state.phase == AvoidPhase::SHIFT && facts.lateral_target_reached) {
      state.phase = AvoidPhase::PASS; return true;
    }
    return false;
  }
  void observeAvoidPass(bool cleared, bool entered_pass_this_cycle) const {
    auto& state = writable()->avoid;
    if (!state.completed && !entered_pass_this_cycle && state.phase == AvoidPhase::PASS && cleared)
      state.phase = AvoidPhase::RETURN;
  }
  void observeAvoidReturn(const AvoidReturnFacts& facts) const {
    auto& state = writable()->avoid;
    if (!state.completed && state.phase == AvoidPhase::RETURN && facts.lane_center_reached)
      state.completed = true;
  }
  bool observeEarlyReturnCandidate(bool selected) const {
    if (selected) writable()->avoid.phase = AvoidPhase::RETURN;
    return selected;
  }
  void beginAvoidShiftProbe() const { writable()->avoid.phase = AvoidPhase::SHIFT; }
  void retimeAvoidEntry(bool entry_ahead) const {
    writable()->avoid.phase = entry_ahead ? AvoidPhase::TRACKING : AvoidPhase::SHIFT;
  }
  void revalidateAvoidOffset(bool outside_tolerance) const {
    if (writable()->avoid.phase == AvoidPhase::PASS && outside_tolerance)
      writable()->avoid.phase = AvoidPhase::SHIFT;
  }
  void observeFollowTarget(int target_id) const {
    writable()->follow.active = true; writable()->follow.leader_id = target_id;
  }
  void observeFollowVisible() const { writable()->follow.missing_cycles = 0; }
  bool observeFollowMissing(int grace_cycles) const {
    return ++writable()->follow.missing_cycles > grace_cycles;
  }
  void requestLaneChangeDirection(LateralDirection direction) const {
    writable()->lane_change.direction = direction;
  }
  void beginLaneChange() const {
    writable()->lane_change.phase = LaneChangePhase::SHIFT;
    writable()->lane_change.active = true;
  }
  void beginLaneChangeCommit() const { writable()->lane_change.committed = true; }
  void initializeLaneChangeCommit() const { writable()->lane_change.phase = LaneChangePhase::SHIFT; }
  void observeLaneChange(const LaneChangeFacts& facts) const {
    if (writable()->lane_change.phase == LaneChangePhase::SHIFT &&
        facts.position_ok && facts.speed_ok && facts.heading_ok)
      writable()->lane_change.phase = LaneChangePhase::HOLD_TARGET_LANE;
  }
  void beginMergeCommit() const {
    writable()->merge.committed = true; writable()->merge.waiting_latched = false;
  }
  void observeMergeWaiting(bool stopped_at_frontier) const {
    if (stopped_at_frontier) writable()->merge.waiting_latched = true;
  }
  void observeStop(const StopFacts& facts) const {
    auto& phase = writable()->stop_phase;
    if (phase == StopExecutionPhase::APPROACH && facts.braking_trigger_reached)
      phase = StopExecutionPhase::BRAKING;
    if (facts.at_target && facts.low_speed) phase = StopExecutionPhase::HOLD;
  }

 private:
  ManeuverState* writable() const { return bound_ ? bound_ : &owned_; }
  mutable ManeuverState owned_;
  ManeuverState* bound_ = nullptr;
};
}
