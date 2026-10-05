#include <gtest/gtest.h>

#include "behavior_planner/decision/fsm.hpp"

namespace behavior_planner {
namespace {

PlannerFeedbackData Feedback(uint32_t id, PlannerExecutionState state) {
  PlannerFeedbackData feedback;
  feedback.valid = true;
  feedback.command_id = id;
  feedback.execution_state = state;
  return feedback;
}

void Prepare(Fsm& fsm, BehaviorType behavior, uint32_t id) {
  EXPECT_EQ(fsm.updateManeuver(behavior, {}, id - 1, false).phase,
            ManeuverPhase::PREPARE);
  EXPECT_EQ(fsm.updateManeuver(
                behavior, Feedback(id, PlannerExecutionState::WAITING), id,
                false).phase,
            ManeuverPhase::PREPARE);
}

TEST(FsmCommandBinding, NewPreparationAcceptsItsReadyForEachLateralManeuver) {
  for (const auto behavior : {BehaviorType::AVOID, BehaviorType::LEFT_CHANGE,
                             BehaviorType::RIGHT_CHANGE}) {
    Fsm fsm;
    Prepare(fsm, behavior, 19);
    const auto output = fsm.updateManeuver(
        behavior, Feedback(20, PlannerExecutionState::READY), 20, false);
    EXPECT_EQ(output.behavior, behavior);
    EXPECT_EQ(output.phase, ManeuverPhase::EXECUTE);
  }
}

TEST(FsmCommandBinding, ReplacementIgnoresOldReadyAndTerminalFeedback) {
  for (const auto state : {PlannerExecutionState::READY,
                           PlannerExecutionState::FAILED,
                           PlannerExecutionState::COMPLETED}) {
    Fsm fsm;
    Prepare(fsm, BehaviorType::AVOID, 19);
    EXPECT_EQ(fsm.updateManeuver(BehaviorType::AVOID, Feedback(19, state),
                                20, false).phase,
              ManeuverPhase::PREPARE);
    EXPECT_EQ(fsm.updateManeuver(
                  BehaviorType::AVOID,
                  Feedback(20, PlannerExecutionState::READY), 20, false).phase,
              ManeuverPhase::EXECUTE);
  }
}

TEST(FsmCommandBinding, ReplacementResetsReadyBeforeCommitment) {
  Fsm fsm;
  Prepare(fsm, BehaviorType::AVOID, 19);
  EXPECT_EQ(fsm.updateManeuver(
                BehaviorType::AVOID,
                Feedback(19, PlannerExecutionState::READY), 19, false).phase,
            ManeuverPhase::EXECUTE);
  EXPECT_EQ(fsm.updateManeuver(
                BehaviorType::AVOID,
                Feedback(19, PlannerExecutionState::READY), 20, false).phase,
            ManeuverPhase::PREPARE);
  EXPECT_EQ(fsm.updateManeuver(
                BehaviorType::AVOID,
                Feedback(20, PlannerExecutionState::WAITING), 20, false).phase,
            ManeuverPhase::PREPARE);
  EXPECT_EQ(fsm.updateManeuver(
                BehaviorType::AVOID,
                Feedback(20, PlannerExecutionState::READY), 20, false).phase,
            ManeuverPhase::EXECUTE);
}

TEST(FsmCommandBinding, FailedReplacementCanRestartAndExecute) {
  Fsm fsm;
  Prepare(fsm, BehaviorType::AVOID, 19);
  EXPECT_EQ(fsm.updateManeuver(
                BehaviorType::AVOID,
                Feedback(20, PlannerExecutionState::FAILED), 20, false).phase,
            ManeuverPhase::PREPARE);
  fsm.updateManeuver(BehaviorType::AVOID,
                     Feedback(20, PlannerExecutionState::FAILED), 21, false);
  EXPECT_EQ(fsm.updateManeuver(
                BehaviorType::AVOID,
                Feedback(23, PlannerExecutionState::READY), 23, false).phase,
            ManeuverPhase::EXECUTE);
}

TEST(FsmCommandBinding, StableTransactionStillExecutes) {
  Fsm fsm;
  Prepare(fsm, BehaviorType::AVOID, 19);
  EXPECT_EQ(fsm.updateManeuver(
                BehaviorType::AVOID,
                Feedback(19, PlannerExecutionState::READY), 19, false).phase,
            ManeuverPhase::EXECUTE);
}

TEST(FsmCommandBinding, CommittedTransactionRetainsBinding) {
  Fsm fsm;
  Prepare(fsm, BehaviorType::AVOID, 19);
  EXPECT_EQ(fsm.updateManeuver(
                BehaviorType::AVOID,
                Feedback(19, PlannerExecutionState::COMMITTED), 19, false).phase,
            ManeuverPhase::COMMITTED);
  EXPECT_EQ(fsm.updateManeuver(
                BehaviorType::KEEP,
                Feedback(20, PlannerExecutionState::READY), 20, false).phase,
            ManeuverPhase::COMMITTED);
  const auto completed = fsm.updateManeuver(
      BehaviorType::KEEP, Feedback(19, PlannerExecutionState::COMPLETED),
      20, false);
  EXPECT_EQ(completed.behavior, BehaviorType::KEEP);
  EXPECT_EQ(completed.phase, ManeuverPhase::NONE);
}

TEST(FsmCommandBinding, ZeroActiveIdDoesNotAcceptOtherReady) {
  Fsm fsm;
  Prepare(fsm, BehaviorType::AVOID, 19);
  EXPECT_EQ(fsm.updateManeuver(
                BehaviorType::AVOID,
                Feedback(20, PlannerExecutionState::READY), 0, false).phase,
            ManeuverPhase::PREPARE);
}

TEST(FsmCommandBinding, InvalidFeedbackDoesNotExecuteReplacement) {
  Fsm fsm;
  Prepare(fsm, BehaviorType::AVOID, 19);
  auto feedback = Feedback(20, PlannerExecutionState::READY);
  feedback.valid = false;
  EXPECT_EQ(fsm.updateManeuver(BehaviorType::AVOID, feedback, 20, false).phase,
            ManeuverPhase::PREPARE);
}

}  // namespace
}  // namespace behavior_planner
