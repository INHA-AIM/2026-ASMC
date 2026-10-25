#include <gtest/gtest.h>
#include <asmc_msgs/control_binding.hpp>
#include "Util/BrakeResponseWatchdog.hpp"
#include "Util/RecoverySpeed.hpp"
namespace b=asmc_msgs::binding;
namespace {
const std::string controller(32,'c'),behavior(32,'b'),planner(32,'p');
b::Request Request(unsigned revision=1) {
  b::Request r;r.token={controller,behavior,revision};r.command.command_id=revision;
  r.command.maneuver_type=asmc_msgs::BehaviorCommand::KEEP;
  r.command.execution_request=asmc_msgs::BehaviorCommand::EXECUTE;r.command.desired_speed_mps=4;
  r.command.header.frame_id="map";r.command.header.stamp=ros::Time(100);r.command.valid_until=ros::Time(101);
  return r;
}
b::Plan Plan(const b::Request& r,unsigned sequence=1) {
  b::Plan p;p.request=r;p.planner_epoch=planner;p.sequence=sequence;
  p.feedback.command_id=r.command.command_id;p.feedback.command_accepted=true;
  p.feedback.active_maneuver=r.command.maneuver_type;p.feedback.plan_valid=true;
  p.feedback.trajectory_source=asmc_msgs::FrenetFeedback::TRAJECTORY_NEW;
  p.trajectory.valid=true;p.trajectory.header.stamp=ros::Time(100);
  p.trajectory.points.resize(3);return p;
}
}
TEST(ControlBinding,OldCommandWithNewStateTimestampCannotRelease) {
 b::Gate gate(controller);auto old=Request(),current=Request(2);
 ASSERT_TRUE(gate.RequestReceived(current));auto p=Plan(old);
 p.trajectory.header.stamp=ros::Time(999);p.trajectory.ego_at_plan.header.stamp=ros::Time(999);
 EXPECT_FALSE(gate.PlanReceived(p));EXPECT_FALSE(gate.linked());
 EXPECT_TRUE(gate.PlanReceived(Plan(current,2)));EXPECT_TRUE(gate.linked());
}
TEST(RecoverySpeed,MeasuredAccelerationAndOverspeedCannotReinforceLaunchAcceleration) {
 EXPECT_GT(RecoveryAccelerationLimit(2,0,0,.5,-6,2),0);
 EXPECT_LE(RecoveryAccelerationLimit(2,1.5,1,.5,-6,2),0);
 EXPECT_LT(RecoveryAccelerationLimit(2,2.2,0,.5,-6,2),0);
 EXPECT_DOUBLE_EQ(RecoveryAccelerationLimit(2,10,0,.5,-6,2),-6);
 EXPECT_LT(RecoveryAccelerationLimit(NAN,0,0,.5,-6,2),0);
}
TEST(ControlBinding,CertifiedRoundaboutPrepareApproachIsNotDeparturePermission) {
 auto r=Request();r.command.maneuver_type=asmc_msgs::BehaviorCommand::MERGE;
 r.command.merge_type=asmc_msgs::BehaviorCommand::MERGE_ROUNDABOUT;
 r.command.execution_request=asmc_msgs::BehaviorCommand::PREPARE;
 auto p=Plan(r);p.feedback.plan_valid=false;p.feedback.execution_state=asmc_msgs::FrenetFeedback::WAITING;
 p.merge_waiting_path_valid=true;p.trajectory.points[1].velocity=2;
 b::Plan decoded;ASSERT_TRUE(b::Decode(b::Encode(p),decoded));
 EXPECT_TRUE(decoded.merge_waiting_path_valid);EXPECT_FALSE(decoded.feedback.plan_valid);
 b::Gate gate(controller);ASSERT_TRUE(gate.RequestReceived(r));EXPECT_TRUE(gate.PlanReceived(decoded));
 EXPECT_EQ(gate.request().command.execution_request,asmc_msgs::BehaviorCommand::PREPARE);
 EXPECT_EQ(gate.plan().feedback.execution_state,asmc_msgs::FrenetFeedback::WAITING);
}
TEST(ControlBinding,WaitingCertificateCannotBypassSignalOrCommitOrStopTail) {
 auto r=Request();r.command.maneuver_type=asmc_msgs::BehaviorCommand::MERGE;
 r.command.merge_type=asmc_msgs::BehaviorCommand::MERGE_ROUNDABOUT;
 r.command.execution_request=asmc_msgs::BehaviorCommand::PREPARE;
 const auto base=Plan(r);
 for(int variant=0;variant<7;++variant){
  auto p=base;p.feedback.plan_valid=false;p.feedback.execution_state=asmc_msgs::FrenetFeedback::WAITING;
  p.merge_waiting_path_valid=true;
  if(variant==0)p.merge_waiting_path_valid=false;
  if(variant==1)p.request.signal.enabled=true;
  if(variant==2)p.request.command.execution_request=asmc_msgs::BehaviorCommand::EXECUTE;
  if(variant==3)p.feedback.execution_state=asmc_msgs::FrenetFeedback::COMMITTED;
  if(variant==4)p.trajectory.points.back().velocity=.2;
  if(variant==5)p.feedback.trajectory_source=asmc_msgs::FrenetFeedback::TRAJECTORY_SAFE_STOP;
  if(variant==6)p.request.command.merge_type=asmc_msgs::BehaviorCommand::MERGE_HIGHWAY;
  b::Gate gate(controller);ASSERT_TRUE(gate.RequestReceived(p.request));EXPECT_FALSE(gate.PlanReceived(p))<<variant;
 }
}
TEST(ControlBinding,LegacyWireVersionCannotReleaseAnActiveVersionTwoConsumer) {
 auto packet=b::Encode(Plan(Request()));packet.data[4]=1;
 b::Plan decoded;EXPECT_FALSE(b::Decode(packet,decoded));
}
TEST(ControlBinding,PlanMayArriveBeforeRequestWithoutAssumingOwnership) {
 b::Gate gate(controller);auto r=Request();EXPECT_FALSE(gate.PlanReceived(Plan(r)));
 EXPECT_FALSE(gate.linked());EXPECT_TRUE(gate.RequestReceived(r));EXPECT_TRUE(gate.linked());
}
TEST(ControlBinding,HeartbeatDoesNotResetLinkAndOldResultsCannotReplaceCurrent) {
 b::Gate gate(controller);auto r=Request(2);ASSERT_TRUE(gate.RequestReceived(r));ASSERT_TRUE(gate.PlanReceived(Plan(r,5)));
 r.command.header.stamp=ros::Time(101);r.command.valid_until=ros::Time(102);
 EXPECT_TRUE(gate.RequestReceived(r));EXPECT_TRUE(gate.linked());EXPECT_EQ(gate.plan().sequence,5u);
 EXPECT_FALSE(gate.RequestReceived(Request(1)));EXPECT_TRUE(gate.linked());
 EXPECT_FALSE(gate.PlanReceived(Plan(r,4)));EXPECT_EQ(gate.plan().sequence,5u);
}
TEST(ControlBinding,SameTokenCannotChangeTargetOrExecution) {
 b::Gate gate(controller);auto r=Request();ASSERT_TRUE(gate.RequestReceived(r));
 auto changed=r;changed.command.execution_request=asmc_msgs::BehaviorCommand::PREPARE;
 EXPECT_FALSE(gate.RequestReceived(changed));EXPECT_FALSE(gate.PlanReceived(Plan(changed)));
 changed=r;changed.signal.target_revision=2;EXPECT_FALSE(gate.PlanReceived(Plan(changed,2)));
 EXPECT_FALSE(gate.linked());
}
TEST(ControlBinding,ControllerAndBehaviorRestartInvalidateOldResults) {
 b::Gate gate(controller);auto r=Request();ASSERT_TRUE(gate.RequestReceived(r));ASSERT_TRUE(gate.PlanReceived(Plan(r)));
 auto next=r;next.token.behavior_epoch=std::string(32,'n');next.token.revision=1;
 EXPECT_TRUE(gate.RequestReceived(next));EXPECT_FALSE(gate.linked());
 EXPECT_FALSE(gate.RequestReceived(r));EXPECT_FALSE(gate.PlanReceived(Plan(r,2)));EXPECT_FALSE(gate.linked());
 b::Gate restarted(std::string(32,'r'));EXPECT_FALSE(restarted.RequestReceived(next));EXPECT_FALSE(restarted.PlanReceived(Plan(next)));
}
TEST(ControlBinding,RejectedFailedAndFallbackPlansDoNotRelease) {
 b::Gate gate(controller);auto r=Request();ASSERT_TRUE(gate.RequestReceived(r));
 auto p=Plan(r);p.feedback.command_accepted=false;EXPECT_FALSE(gate.PlanReceived(p));
 p=Plan(r,2);p.feedback.plan_valid=false;EXPECT_FALSE(gate.PlanReceived(p));
 p=Plan(r,3);p.feedback.trajectory_source=asmc_msgs::FrenetFeedback::TRAJECTORY_REUSED;EXPECT_FALSE(gate.PlanReceived(p));
 p=Plan(r,4);p.feedback.active_maneuver=asmc_msgs::BehaviorCommand::FOLLOW;EXPECT_FALSE(gate.PlanReceived(p));
 EXPECT_TRUE(gate.PlanReceived(Plan(r,5)));
 p=Plan(r,6);p.trajectory.valid=false;EXPECT_FALSE(gate.PlanReceived(p));EXPECT_FALSE(gate.linked());
}
TEST(ControlBinding,AtomicSerializationPreservesRequestAndTrajectory) {
 auto r=Request();r.signal.enabled=true;r.signal.target_revision=7;r.signal.stop_line="line";
 auto p=Plan(r);p.trajectory.points[2].velocity=1.23;p.goal_satisfied=true;
 b::Plan copy;ASSERT_TRUE(b::Decode(b::Encode(p),copy));EXPECT_TRUE(b::SameRequest(r,copy.request));
 EXPECT_DOUBLE_EQ(copy.trajectory.points[2].velocity,1.23);EXPECT_TRUE(copy.goal_satisfied);
 b::Request wrong;EXPECT_FALSE(b::Decode(b::Encode(p),wrong));
 auto truncated=b::Encode(p);truncated.data.pop_back();EXPECT_FALSE(b::Decode(truncated,copy));
 auto trailing=b::Encode(p);trailing.data.push_back(0);EXPECT_FALSE(b::Decode(trailing,copy));
}
TEST(BrakeWatchdog,HeartbeatAndGoalRevisionDoNotRestartDecisionDeadline) {
 BrakeResponseWatchdog w;auto r=Request();r.brake_generation=1;r.brake_level=1;
 r.brake_decision_ros=100;r.brake_speed=5;r.brake_state_stamp=100;r.brake_budget=.5;
 w.Request(r,100.1,10.1);r.token.revision=2;r.signal.target_revision=2;
 w.Request(r,100.45,10.45);EXPECT_FALSE(w.Observe(10.49,100.49,5,0,true));
 EXPECT_TRUE(w.Observe(10.51,100.51,5,0,true));
 w.Released();EXPECT_TRUE(w.failed());
}
TEST(BrakeWatchdog,PhysicalResponseRequiredAndEscalationNeedsStrongerResponse) {
 BrakeResponseWatchdog w;auto r=Request();r.brake_generation=1;r.brake_level=1;
 r.brake_decision_ros=100;r.brake_speed=5;r.brake_state_stamp=100;
 w.Request(r,100.05,10.05);
 EXPECT_FALSE(w.Observe(10.1,100.1,4.9,-1,true));EXPECT_TRUE(w.observed());
 r.brake_generation=2;r.brake_level=2;r.brake_decision_ros=100.2;r.brake_speed=4.8;
 r.brake_acceleration=-1;r.brake_state_stamp=100.2;w.Request(r,100.25,10.25);
 EXPECT_FALSE(w.Observe(10.5,100.5,4.5,-1,true));EXPECT_FALSE(w.observed());
 EXPECT_TRUE(w.Observe(10.71,100.71,4.4,-1,true));
}
TEST(BrakeWatchdog,StationaryHoldHasNoOnsetTimeoutButUnexpectedMotionFails) {
 BrakeResponseWatchdog w;auto r=Request();r.brake_generation=1;r.brake_level=1;
 r.brake_decision_ros=100;r.brake_state_stamp=100;r.brake_speed=0;
 w.Request(r,100.05,10.05);EXPECT_FALSE(w.Observe(20,110,0,0,true));
 EXPECT_TRUE(w.Observe(20.1,110.1,.4,.5,true));
}
TEST(BrakeWatchdog,OrdinaryStopCanContinueMeasuredBrakingButEscalationNeedsMore) {
 BrakeResponseWatchdog w;auto r=Request();r.brake_generation=1;r.brake_level=1;
 r.brake_decision_ros=100;r.brake_speed=3.4;r.brake_acceleration=-.8;r.brake_state_stamp=100;
 w.Request(r,100.01,10.01);
 EXPECT_FALSE(w.Observe(10.1,100,3.3,-.8,true));EXPECT_FALSE(w.observed());
 EXPECT_FALSE(w.Observe(10.2,100.2,3.4,-.8,true));EXPECT_FALSE(w.observed());
 EXPECT_FALSE(w.Observe(10.3,100.3,3.2,0,true));EXPECT_FALSE(w.observed());
 EXPECT_FALSE(w.Observe(10.4,100.4,3.1,-.78,true));EXPECT_TRUE(w.observed());
 r.brake_generation=2;r.brake_level=2;r.brake_decision_ros=100.4;r.brake_speed=3.1;
 r.brake_acceleration=-.78;r.brake_state_stamp=100.4;w.Request(r,100.41,10.41);
 EXPECT_FALSE(w.Observe(10.5,100.5,3.,-.78,true));EXPECT_FALSE(w.observed());
 EXPECT_FALSE(w.Observe(10.6,100.6,2.9,-2.,true));EXPECT_TRUE(w.observed());
}
TEST(BrakeWatchdog,OwnerAcknowledgmentOnlyClearsValidatedPathFailure) {
 BrakeResponseWatchdog path;path.Fail(2);
 EXPECT_FALSE(path.AcknowledgePathRecovery(false));EXPECT_TRUE(path.failed());
 EXPECT_TRUE(path.AcknowledgePathRecovery(true));EXPECT_FALSE(path.failed());EXPECT_TRUE(path.ever_failed());
 for(unsigned mask:{1u,3u,4u,8u}) {
   BrakeResponseWatchdog failed;failed.Fail(mask);
   EXPECT_FALSE(failed.AcknowledgePathRecovery(true));EXPECT_TRUE(failed.failed());
 }
}
TEST(BrakeWatchdog,NoPhysicalResponseAtMaximumBrakeRemainsFailed) {
 BrakeResponseWatchdog w;auto r=Request();r.brake_generation=1;r.brake_level=2;
 r.brake_decision_ros=100;r.brake_state_stamp=100;r.brake_speed=6;r.brake_budget=.5;
 w.Request(r,100.1,10.1);EXPECT_TRUE(w.Observe(10.51,100.51,6,0,true));
 r.token.revision=8;w.Request(r,100.8,10.8);
 EXPECT_TRUE(w.Observe(11,101,5,-2,true));EXPECT_FALSE(w.AcknowledgePathRecovery(true));
 EXPECT_EQ(w.failure_mask(),1u);
}
TEST(BrakeWatchdog,MeasuredSceneDelayUsesTheSameDecisionToObservedResponseBudget) {
 // Full-scene bag: tD=1792652475.29674, first valid response received at
 // +0.715s; callback trace bounds it by +0.736s. Earlier fresh frames still
 // showed acceleration and the intervening frames were invalid.
 auto r=Request();r.brake_generation=2;r.brake_level=2;
 r.brake_decision_ros=100;r.brake_state_stamp=99.8666;
 r.brake_speed=1.7518;r.brake_acceleration=2.8794;r.brake_budget=.736+.1;
 BrakeResponseWatchdog observed;observed.Request(r,100.02,10.02);
 EXPECT_FALSE(observed.Observe(10.395,100.265,1.8707,3.1559,true));
 EXPECT_FALSE(observed.observed());
 EXPECT_FALSE(observed.Observe(10.55,100.265,1.2032,3.1559,false));
 EXPECT_FALSE(observed.observed());
 EXPECT_FALSE(observed.Observe(10.736,100.709,0,-4.9203,true));
 EXPECT_TRUE(observed.observed());
 BrakeResponseWatchdog missing;missing.Request(r,100.02,10.02);
 EXPECT_TRUE(missing.Observe(10.837,100.837,1.7518,2.8794,false));
 EXPECT_EQ(missing.failure_mask(),1u);EXPECT_FALSE(missing.observed());
}
TEST(BrakeWatchdog,PathRecoveryRetiresOldDecisionButKeepsStationaryGuardAndNextRequest) {
 BrakeResponseWatchdog w;auto r=Request();r.brake_generation=2;r.brake_level=2;
 r.brake_decision_ros=100;r.brake_state_stamp=100;r.brake_speed=5;
 w.Request(r,100.1,10.1);EXPECT_FALSE(w.Observe(10.2,100.2,4,-5,true));
 w.Fail(2);ASSERT_TRUE(w.AcknowledgePathRecovery(true));
 r.token.revision=8;w.Request(r,101.5,11.5);
 EXPECT_FALSE(w.Observe(11.6,101.6,0,0,true)); // Old tD must not time out again.
 EXPECT_TRUE(w.ever_failed());EXPECT_FALSE(w.failed());
 r.brake_generation=3;r.brake_decision_ros=102;r.brake_state_stamp=102;r.brake_speed=2;
 w.Request(r,102.1,12.1);EXPECT_TRUE(w.Observe(12.51,102.51,2,0,true));
 EXPECT_EQ(w.failure_mask(),1u);

 BrakeResponseWatchdog held;held.Request(r,102.1,12.1);held.Fail(2);
 ASSERT_TRUE(held.AcknowledgePathRecovery(true));
 EXPECT_TRUE(held.Observe(13,103,.4,.5,true));EXPECT_EQ(held.failure_mask(),4u);
}
TEST(BrakeWatchdog,RetiredGenerationDoesNotSuppressNewBehaviorEpoch) {
 BrakeResponseWatchdog w;auto r=Request();r.brake_generation=1;r.brake_level=1;
 r.brake_decision_ros=100;r.brake_state_stamp=100;r.brake_speed=5;
 w.Request(r,100.1,10.1);EXPECT_FALSE(w.Observe(10.2,100.2,4,-2,true));w.Released();
 r.token.behavior_epoch=std::string(32,'n');r.brake_decision_ros=102;r.brake_state_stamp=102;
 w.Request(r,102.1,12.1);EXPECT_TRUE(w.Observe(12.51,102.51,5,0,true));
}
