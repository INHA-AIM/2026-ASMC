#include <gtest/gtest.h>
#include "Signal/SignalPolicy.hpp"
#include "Signal/ReleaseCommit.hpp"
#include "Signal/ExecutionRecovery.hpp"
#include "Signal/LocalizationContinuity.hpp"
#include "Frenet/Generator/SpatialStop.hpp"

namespace sig=asmc_planner::signal;
namespace {
sig::Key K{1,"stop-A","straight"};
sig::Config Settings() {sig::Config c;c.position_error=0;c.half_track=.8;return c;}
sig::Geometry Geometry() {
  sig::Geometry g;g.key=K;g.line={20,0};g.approach={1,0};g.reviewed=true;g.route_s=20;
  g.conflict_reviewed=true;g.conflict_polygon={{21,-10},{30,-10},{30,10},{21,10}};
  g.exit={30,0};g.exit_direction={1,0};return g;
}
sig::Pose Pose(double stamp,double x,double v=5,double a=0) {
  sig::Pose p;p.stamp=stamp;p.x=x;p.speed=v;p.acceleration=a;p.route_s=x;p.valid=true;return p;
}
sig::Observation Frame(double stamp,sig::Color color=sig::Color::GREEN) {
  sig::Observation o;o.source_epoch=1;o.stamp=stamp;o.received=stamp+10;
  o.frame="front";o.color=color;o.valid=true;o.stale=false;o.clock_verified=true;
  o.association_valid=true;o.key=K;return o;
}
void Deliver(sig::GreenGate& gate,sig::Observation o) {gate.Observe(o,o.stamp,o.received);}
void Greens(sig::GreenGate& gate,double start=1) {
  for(int i=0;i<3;++i)Deliver(gate,Frame(start+i*.05));
}
void Greens(sig::Policy& policy,double start=1) {
  for(int i=0;i<3;++i) {auto f=Frame(start+i*.05);policy.Observe(f,f.stamp,f.received);}
}
sig::Evidence Evidence() {
  sig::Evidence e;e.position_valid=e.risks_fresh=e.conflict_known=true;return e;
}
void Connect(sig::Evidence& e) {
  e.linked=e.command_accepted=e.trajectory_belongs_to_command=true;e.command_id=10;
}
}  // namespace

TEST(SignalEntryMeasurement,QualifiedWheelEvidenceDoesNotShrinkBodyClearance) {
 auto c=Settings();c.position_error=.7;c.entry_observation_error=.5;c.entry_observation_speed_max=7.;
 sig::Policy p(c);p.Bind(Geometry());p.SetTarget(5.5,false);auto e=Evidence();e.position_history_continuous=true;
 Greens(p);p.Step(Pose(1.1,16,6.9),1.1,11.1,e,{});
 p.Step(Pose(1.2,16.69,6.9),1.2,11.2,e,{});
 p.Observe(Frame(1.3),1.3,11.3);
 ASSERT_EQ(p.Step(Pose(1.3,17.38,6.9),1.3,11.3,e,{}).motion,sig::Motion::GREEN_CROSSING_CLEAR);
 p.Observe(Frame(1.332),1.332,11.332);
 const auto out=p.Step(Pose(1.332,17.6,6.9),1.332,11.332,e,{});
 EXPECT_EQ(out.motion,sig::Motion::NORMAL_CLEAR);EXPECT_DOUBLE_EQ(out.entry_observation_error,.5);
 EXPECT_FALSE(sig::BodyBeforeLine(Pose(1.332,15.1,0),Geometry(),c));
}
TEST(SignalEntryMeasurement,UnqualifiedObservationUsesTheWholeBodyUncertainty) {
 auto c=Settings();c.position_error=.7;sig::Policy p(c);p.Bind(Geometry());p.SetTarget(5.5,false);
 auto e=Evidence();e.position_history_continuous=true;Greens(p);p.Step(Pose(1.1,16,6.9),1.1,11.1,e,{});
 p.Step(Pose(1.2,16.69,6.9),1.2,11.2,e,{});p.Observe(Frame(1.3),1.3,11.3);
 p.Step(Pose(1.3,17.38,6.9),1.3,11.3,e,{});p.Observe(Frame(1.332),1.332,11.332);
 const auto out=p.Step(Pose(1.332,17.6,6.9),1.332,11.332,e,{});
 EXPECT_EQ(out.motion,sig::Motion::GREEN_CROSSING_CLEAR);EXPECT_DOUBLE_EQ(out.entry_observation_error,.7);
}
TEST(SignalEntryMeasurement,SlowingAfterCrossingCannotShrinkTheRecordedCrossingWindow) {
 auto c=Settings();c.position_error=.7;c.entry_observation_error=.5;c.entry_observation_speed_max=7.;
 sig::Policy p(c);p.Bind(Geometry());p.SetTarget(5.5,false);auto e=Evidence();e.position_history_continuous=true;
 Greens(p);p.Step(Pose(1.1,16,8.2),1.1,11.1,e,{});
 p.Step(Pose(1.2,16.82,8.2),1.2,11.2,e,{});p.Observe(Frame(1.25),1.25,11.25);
 p.Step(Pose(1.25,17.23,8.2),1.25,11.25,e,{});p.Observe(Frame(1.3),1.3,11.3);
 const auto out=p.Step(Pose(1.3,17.6,6.9),1.3,11.3,e,{});
 EXPECT_EQ(out.motion,sig::Motion::GREEN_CROSSING_CLEAR);EXPECT_DOUBLE_EQ(out.entry_observation_error,.7);
 p.Observe(Frame(1.31,sig::Color::YELLOW),1.31,11.31);
 p.Step(Pose(1.32,17.8,6.9),1.32,11.32,e,{});
 for(const auto& event:p.events())EXPECT_NE(event.kind,sig::EventKind::NORMAL_ENTRY_CONFIRMED);
}
TEST(SignalEntryMeasurement,PositiveBoundaryAccelerationCanExceedTheQualifiedSpeedRange) {
 auto c=Settings();c.position_error=.7;c.entry_observation_error=.5;c.entry_observation_speed_max=7.;
 sig::Policy p(c);p.Bind(Geometry());p.SetTarget(5.5,false);auto e=Evidence();Greens(p);
 EXPECT_DOUBLE_EQ(p.Step(Pose(1.1,16,6.9,2),1.1,11.1,e,{}).entry_observation_error,.7);
}
TEST(SignalEntryMeasurement,SlowingWithinPossibleCrossingBeforeNominalZeroCannotNarrowEvidence) {
 auto c=Settings();c.position_error=.7;c.entry_observation_error=.5;c.entry_observation_speed_max=7.;
 sig::Policy p(c);p.Bind(Geometry());p.SetTarget(5.5,false);auto e=Evidence();e.position_history_continuous=true;
 Greens(p);p.Step(Pose(1.1,16,8.2),1.1,11.1,e,{});
 p.Step(Pose(1.15,16.5,8.2),1.15,11.15,e,{});
 p.Step(Pose(1.2,16.9,6.9),1.2,11.2,e,{});
 p.Observe(Frame(1.25),1.25,11.25);p.Step(Pose(1.25,17.3,6.9),1.25,11.25,e,{});
 p.Observe(Frame(1.3),1.3,11.3);
 const auto out=p.Step(Pose(1.3,17.6,6.9),1.3,11.3,e,{});
 EXPECT_DOUBLE_EQ(out.entry_observation_error,.7);EXPECT_EQ(out.motion,sig::Motion::GREEN_CROSSING_CLEAR);
}
TEST(SignalPolicy,UnknownCanApproachTargetButBrakingAndHoldCannotReapproach) {
 sig::Policy p(Settings());p.Bind(Geometry());p.SetTarget(5.5,false);
 auto e=Evidence();
 auto out=p.Step(Pose(1,-20,1),1,11,e,{});
 EXPECT_EQ(out.motion,sig::Motion::APPROACH);EXPECT_EQ(out.severity,sig::Severity::STOP);
 EXPECT_FALSE(out.entry_authorized);
 out=p.Step(Pose(1.1,11,1),1.1,11.1,e,{});
 EXPECT_EQ(out.motion,sig::Motion::BRAKING);
 p.SetTarget(4.345,true);
 out=p.Step(Pose(1.2,11.1,.5),1.2,11.2,e,{});
 EXPECT_EQ(out.motion,sig::Motion::BRAKING);
 out=p.Step(Pose(1.3,11.2,0,-.1),1.3,11.3,e,{});
 EXPECT_EQ(out.motion,sig::Motion::STATIONARY_HOLD);EXPECT_FALSE(out.target_reached);
 EXPECT_DOUBLE_EQ(out.minimum_distance,0);
 p.SetTarget(4.4,true);
 out=p.Step(Pose(1.4,11.2,0,-.1),1.4,11.4,e,{});
 EXPECT_EQ(out.motion,sig::Motion::STATIONARY_HOLD);
}
TEST(SignalExecutionRecovery,RawSafeStatusCannotClearOwnerDuringInvalidInputsOrAnAckInFlight) {
 sig::Policy p(Settings());p.Bind(Geometry());p.SetTarget(5.5,false);auto e=Evidence();
 sig::ExecutionRecovery owner;owner.Observe(true,2,true,false);
 e.execution_failed=true;e.execution_failure_mask=2;e.planning_checked=true;
 e.stop_path_valid=e.path_goal_satisfied=false;Connect(e);
 p.Step(Pose(1,10,1),1,11,e,{});
 e.execution_recovery_acknowledged=true;e.stop_path_valid=e.path_goal_satisfied=true;
 e.risks_fresh=false;
 auto out=p.Step(Pose(1.1,10.1,0),1.1,11.1,e,{});owner.Authorize(out.execution_recovery_acknowledged);
 EXPECT_FALSE(owner.pending());EXPECT_TRUE(owner.failed());
 owner.Observe(false,0,true,false);EXPECT_TRUE(owner.failed()); // An unsolicited Controller clear is insufficient.
 e.risks_fresh=true;sig::StopCauses pedestrian;pedestrian.Set(sig::Cause::PEDESTRIAN,sig::Severity::STOP);
 out=p.Step(Pose(1.4,10.1,0),1.4,11.4,e,pedestrian);
 ASSERT_TRUE(out.execution_recovery_acknowledged);EXPECT_NE(out.severity,sig::Severity::NONE);
 EXPECT_EQ(out.causes[static_cast<unsigned>(sig::Cause::PEDESTRIAN)],sig::Severity::STOP);
 owner.Authorize(out.execution_recovery_acknowledged);ASSERT_TRUE(owner.pending());
 owner.Observe(true,2,false,true);EXPECT_TRUE(owner.pending()); // ACK's new trajectory has not arrived yet.
 owner.Observe(false,0,true,true);EXPECT_FALSE(owner.failed());EXPECT_FALSE(owner.pending());
 owner.Observe(true,2,true,false);EXPECT_TRUE(owner.failed());
 owner.Authorize(true);owner.Observe(true,1,true,true);EXPECT_FALSE(owner.pending());EXPECT_TRUE(owner.failed());
}
TEST(SignalPolicy,CommittedRecoveryContinuesThroughExitUncertaintyBandAndRetainsOtherStops) {
 auto c=Settings();c.position_error=.5;sig::Policy p(c);p.Bind(Geometry());p.SetTarget(5.5,false);
 auto e=Evidence();e.position_history_continuous=true;e.inside_conflict=true;e.exit_path_valid=true;Connect(e);
 auto out=p.Step(Pose(1,25,1),1,11,e,{});ASSERT_EQ(out.motion,sig::Motion::RECOVERY_CLEAR);
 const auto records=p.events().size();auto boundary=Pose(1.05,30.9,1);e.inside_conflict=false;
 ASSERT_FALSE(sig::BodyDefinitelyOccupiesConflict(boundary,Geometry(),c));
 ASSERT_TRUE(sig::BodyOccupiesConflict(boundary,Geometry(),c));
 ASSERT_FALSE(sig::BodyOutsideExit(boundary,Geometry(),c));
 out=p.Step(boundary,1.05,11.05,e,{});EXPECT_EQ(out.motion,sig::Motion::RECOVERY_CLEAR);
 sig::StopCauses hazard;hazard.Set(sig::Cause::COLLISION,sig::Severity::EMERGENCY);
 boundary.stamp=1.1;out=p.Step(boundary,1.1,11.1,e,hazard);
 EXPECT_EQ(out.severity,sig::Severity::EMERGENCY);EXPECT_EQ(out.motion,sig::Motion::RECOVERY_BRAKE);
 boundary.stamp=1.15;out=p.Step(boundary,1.15,11.15,e,{});
 EXPECT_EQ(out.motion,sig::Motion::RECOVERY_CLEAR);EXPECT_EQ(p.events().size(),records);
 e.exited=true;out=p.Step(Pose(1.2,31.5,1),1.2,11.2,e,{});EXPECT_EQ(out.motion,sig::Motion::EXITED);
}
TEST(SignalPolicy,UnprovenOrDiscontinuousRecoveryCannotStartInExitBand) {
 auto c=Settings();c.position_error=.5;auto e=Evidence();e.position_history_continuous=true;e.exit_path_valid=true;Connect(e);
 sig::Policy fresh(c);fresh.Bind(Geometry());fresh.SetTarget(5.5,false);
 EXPECT_EQ(fresh.Step(Pose(1,30.9,0),1,11,e,{}).motion,sig::Motion::RECOVERY_BRAKE);
 sig::Policy p(c);p.Bind(Geometry());p.SetTarget(5.5,false);e.inside_conflict=true;
 ASSERT_EQ(p.Step(Pose(1,25,1),1,11,e,{}).motion,sig::Motion::RECOVERY_CLEAR);
 e.inside_conflict=false;e.position_history_continuous=false;
 EXPECT_EQ(p.Step(Pose(2,30.9,0),2,12,e,{}).motion,sig::Motion::RECOVERY_BRAKE);
 e.position_history_continuous=true;
 EXPECT_EQ(p.Step(Pose(2.05,30.9,0),2.05,12.05,e,{}).motion,sig::Motion::RECOVERY_BRAKE);
}
TEST(SignalPolicy,MissingSpeedBrakesButIndependentPositionsPreserveOnlyThePriorRecoveryFact) {
 auto c=Settings();c.position_error=.5;sig::Policy p(c);p.Bind(Geometry());p.SetTarget(5.5,false);
 auto e=Evidence();e.position_history_continuous=true;e.inside_conflict=true;e.exit_path_valid=true;Connect(e);
 ASSERT_EQ(p.Step(Pose(1,25,1),1,11,e,{}).motion,sig::Motion::RECOVERY_CLEAR);
 auto boundary=Pose(1.1,30.9,0);boundary.valid=false;
 e.inside_conflict=false;e.position_history_continuous=false;e.localization_continuity_verified=true;
 auto out=p.Step(boundary,1.1,11.1,e,{});
 EXPECT_EQ(out.motion,sig::Motion::RECOVERY_BRAKE);EXPECT_EQ(out.severity,sig::Severity::EMERGENCY);
 EXPECT_EQ(out.causes[static_cast<size_t>(sig::Cause::INPUT_INVALID)],sig::Severity::EMERGENCY);
 boundary.valid=true;boundary.stamp=1.2;e.position_history_continuous=true;
 e.exit_path_valid=false;
 EXPECT_EQ(p.Step(boundary,1.2,11.2,e,{}).motion,sig::Motion::RECOVERY_BRAKE);
 e.exit_path_valid=true;boundary.stamp=1.25;
 EXPECT_EQ(p.Step(boundary,1.25,11.25,e,{}).motion,sig::Motion::RECOVERY_CLEAR);
 sig::StopCauses collision;collision.Set(sig::Cause::COLLISION,sig::Severity::EMERGENCY);
 EXPECT_EQ(p.Step(boundary,1.25,11.25,e,collision).severity,sig::Severity::EMERGENCY);
 sig::Policy fresh(c);fresh.Bind(Geometry());fresh.SetTarget(5.5,false);
 EXPECT_EQ(fresh.Step(boundary,1.25,11.25,e,{}).motion,sig::Motion::RECOVERY_BRAKE);
 for(const auto& event:p.events())EXPECT_NE(event.kind,sig::EventKind::NORMAL_ENTRY_CONFIRMED);
}
TEST(SignalLocalization,MatchesOnlyFreshContinuousOriginalPositionsAndNeverRenewsDuplicates) {
 auto c=Settings();c.position_error=.5;sig::LocalizationContinuity history(c);
 for(int i=0;i<5;++i)history.Push(Pose(1+i*.02,25,1),11+i*.02);
 auto state=Pose(1.08,25,0);state.valid=false;state.speed=NAN;
 EXPECT_TRUE(history.CoversMatched(1,state,1.09,11.09)); // Position proof only.
 auto wrong=state;wrong.x+=.01;EXPECT_FALSE(history.CoversMatched(1,wrong,1.09,11.09));
 history.Push(Pose(1.08,25,1),11.4);
 EXPECT_FALSE(history.CoversMatched(1,state,1.4,11.4));
 const auto generation=history.generation();history.Push(Pose(1.3,25,1),11.3);
 EXPECT_NE(generation,history.generation());EXPECT_FALSE(history.CoversMatched(1,Pose(1.3,25),1.3,11.3));
 const auto next=history.generation();auto invalid=Pose(1.31,25);invalid.valid=false;
 history.Push(invalid,11.31);EXPECT_NE(next,history.generation());
 EXPECT_FALSE(history.CoversMatched(1.3,Pose(1.31,25),1.31,11.31));
 history.Push(Pose(2,25),12);const auto prior=history.generation();
 history.Push(Pose(2.01,100),12.01);EXPECT_NE(prior,history.generation());
 EXPECT_FALSE(history.CoversMatched(2,Pose(2.01,100),2.01,12.01));
}
TEST(SignalPolicy,CommittedRecoveryCoversVerifiedCorridorBetweenConflictEndAndExit) {
 auto c=Settings();c.position_error=.5;auto g=Geometry();
 g.conflict_polygon={{21,-10},{27,-10},{27,10},{21,10}};
 sig::Policy p(c);p.Bind(g);p.SetTarget(5.5,false);auto e=Evidence();
 e.position_history_continuous=true;e.inside_conflict=true;e.exit_path_valid=true;Connect(e);
 ASSERT_EQ(p.Step(Pose(1,25,1),1,11,e,{}).motion,sig::Motion::RECOVERY_CLEAR);
 auto corridor=Pose(1.05,29.5,1);e.inside_conflict=false;
 ASSERT_FALSE(sig::BodyOccupiesConflict(corridor,g,c));ASSERT_FALSE(sig::BodyOutsideExit(corridor,g,c));
 EXPECT_EQ(p.Step(corridor,1.05,11.05,e,{}).motion,sig::Motion::RECOVERY_CLEAR);
}
TEST(SignalBraking,ResidualDecelerationAtRestDoesNotInventStoppingDistance) {
 sig::BrakeLimits limits;
 EXPECT_DOUBLE_EQ(sig::MinimumStoppingDistance(0,-.1,limits,0),0);
 EXPECT_DOUBLE_EQ(sig::MinimumStoppingDistance(0,-9,limits,0),0);
 EXPECT_GT(sig::MinimumStoppingDistance(0,1,limits,.5),0);
}
TEST(SignalBraking,StrongerMeasuredBrakingUsesConservativeBoundaryInsteadOfInventingFailure) {
 sig::BrakeLimits limits;
 EXPECT_DOUBLE_EQ(sig::MinimumStoppingDistance(4.6,-4.06,limits),sig::MinimumStoppingDistance(4.6,-4,limits));
 EXPECT_TRUE(std::isfinite(sig::MinimumStoppingDistance(.01,-2,limits)));
 EXPECT_FALSE(std::isfinite(sig::MinimumStoppingDistance(4.6,5,limits)));
}
TEST(SignalRelease,OwnerPermitWaitsForCurrentPlanWithoutHeartbeatRestartOrAlternatingTokens) {
 sig::ReleaseCommit permit;
 EXPECT_TRUE(permit.Authorize(K,true,true,true,false,1));
 EXPECT_TRUE(permit.Authorize(K,true,true,false,true,1.1));
 EXPECT_TRUE(permit.Authorize(K,true,true,false,true,1.4));
 EXPECT_FALSE(permit.Authorize(K,true,true,false,true,1.51));
 EXPECT_TRUE(permit.Authorize(K,true,true,true,false,2));
 EXPECT_TRUE(permit.Authorize(K,true,true,true,true,5));
 EXPECT_FALSE(permit.Authorize(K,true,false,true,true,5.01));
 auto other=K;other.route_generation++;
 EXPECT_FALSE(permit.Authorize(other,true,true,false,true,5.02));
}

TEST(SignalObservation, ThreeDistinctOriginalsAndDuplicatesDoNotRenew) {
  sig::GreenGate gate;gate.Bind(K);
  auto f=Frame(1);Deliver(gate,f);f.sequence=99;f.received=11.2;
  gate.Observe(f,1.2,11.2);EXPECT_EQ(gate.count(),1);
  gate.Tick(1.31,11.31);EXPECT_EQ(gate.count(),0);
  Greens(gate,2);EXPECT_TRUE(gate.authorized());
}
TEST(SignalObservation, SameHeaderStaleTombstoneWinsEvenBeforeFirstValid) {
  for(bool prior_valid:{false,true}) {
    sig::GreenGate gate;gate.Bind(K);auto f=Frame(1);
    if(prior_valid)Deliver(gate,f);
    f.stale=true;Deliver(gate,f);f.stale=false;Deliver(gate,f);
    EXPECT_EQ(gate.count(),0);
    Greens(gate,1.1);EXPECT_TRUE(gate.authorized());
  }
}
TEST(SignalObservation, StaleFirstMessageOfNewSourceAlsoLeavesTombstone) {
  sig::GreenGate gate;gate.Bind(K);Greens(gate);
  auto f=Frame(2);f.source_epoch=2;f.stale=true;Deliver(gate,f);
  f.stale=false;Deliver(gate,f);EXPECT_EQ(gate.count(),0);
}
TEST(SignalObservation, OldDuplicateDoesNotRevokeOrRenewNewerEvidence) {
  sig::GreenGate gate;gate.Bind(K);Greens(gate);
  auto old=Frame(1);old.received=11.35;gate.Observe(old,1.35,11.35);
  EXPECT_EQ(gate.count(),3);gate.Tick(1.41,11.41);EXPECT_EQ(gate.count(),0);
}
TEST(SignalObservation, TwoThenGapThenOneStartsAtOne) {
  sig::GreenGate gate;gate.Bind(K);Deliver(gate,Frame(1));Deliver(gate,Frame(1.05));
  gate.Tick(1.36,11.36);Deliver(gate,Frame(1.4));EXPECT_EQ(gate.count(),1);
}
TEST(SignalObservation, RedBetweenBehaviorCyclesCannotDisappear) {
  sig::GreenGate gate;gate.Bind(K);Greens(gate);
  Deliver(gate,Frame(1.15,sig::Color::RED));Deliver(gate,Frame(1.2));
  EXPECT_EQ(gate.count(),1);EXPECT_FALSE(gate.authorized());
}
TEST(SignalObservation, ReceiveGapAndSourceAgeAreIndependentlyChecked) {
  sig::GreenGate gate;gate.Bind(K);Deliver(gate,Frame(1));
  auto f=Frame(1.05);f.received=11.4;gate.Observe(f,1.05,11.4);
  EXPECT_EQ(gate.count(),1);
  gate.Tick(1.36,11.41);EXPECT_FALSE(gate.authorized());EXPECT_EQ(gate.count(),0);
}
TEST(SignalObservation, UnknownClockAssociationLegacyAndFutureNeverAuthorize) {
  for(int mode=0;mode<4;++mode) {
    sig::GreenGate gate;gate.Bind(K);
    for(int i=0;i<3;++i) {
      auto f=Frame(1+i*.05);
      if(mode==0)f.clock_verified=false;
      if(mode==1)f.association_valid=false;
      if(mode==2)f.stamp=0;
      gate.Observe(f,mode==3 ? .9 : 1.2,f.received);
    }
    EXPECT_FALSE(gate.authorized());
  }
}
TEST(SignalObservation, OldSourceEpochAndDifferentMovementCannotRestorePermission) {
  sig::GreenGate gate;gate.Bind(K);Greens(gate);
  auto f=Frame(2);f.source_epoch=2;Deliver(gate,f);EXPECT_EQ(gate.count(),1);
  f=Frame(2.1);Deliver(gate,f);EXPECT_EQ(gate.count(),1);  // Old epoch cannot replace current evidence.
  f=Frame(2.2);f.source_epoch=2;f.key.movement="left";Deliver(gate,f);
  EXPECT_EQ(gate.count(),0);
}
TEST(SignalHistory, RequiresBracketAndBoundedInterpolationGap) {
  sig::PoseHistory history(Settings());history.Push(Pose(1,0));history.Push(Pose(1.05,1));
  sig::Pose p;ASSERT_TRUE(history.At(1.025,p));EXPECT_NEAR(p.x,.5,1e-10);
  EXPECT_FALSE(history.At(.9,p));EXPECT_FALSE(history.At(1.06,p));
  history.Push(Pose(1.3,2));EXPECT_FALSE(history.At(1.2,p));
  history.Push(Pose(.5,4));EXPECT_FALSE(history.At(1,p));
}
TEST(SignalGeometry, PhysicalCrossingZeroAndHalfMeterConfirmationAreDistinct) {
  auto c=Settings();auto g=Geometry();
  auto p=Pose(1,17.1);auto d=sig::FrontWheelDistance(p,g,c);
  ASSERT_TRUE(d.valid);EXPECT_LT(d.min,0);EXPECT_GT(d.max,-c.crossing_band);
  p.x=17.6;d=sig::FrontWheelDistance(p,g,c);EXPECT_LT(d.max,-c.crossing_band);
  c.half_track=NAN;EXPECT_FALSE(sig::FrontWheelDistance(p,g,c).valid);
}
TEST(SignalGeometry, LateTargetUsesCornersAndErrorsNotFixedOffset) {
  auto c=Settings();auto g=Geometry();
  auto straight=[](double s,sig::Pose& p){p=Pose(1,s,0);return true;};
  const auto late=sig::SelectLateTarget(g,c,straight);
  ASSERT_TRUE(late.valid);EXPECT_NEAR(late.offset,4.345,1e-6);
  EXPECT_NEAR(5.5-late.offset,1.155,1e-6);
  c.position_error=.5;
  const auto uncertain=sig::SelectLateTarget(g,c,straight);
  ASSERT_TRUE(uncertain.valid);EXPECT_GE(uncertain.offset,4.84);
  auto oblique=[](double s,sig::Pose& p){p=Pose(1,s,0);p.yaw=.25;return true;};
  c.position_error=0;const auto angled=sig::SelectLateTarget(g,c,oblique);
  ASSERT_TRUE(angled.valid);EXPECT_TRUE(sig::BodyBeforeLine(angled.pose,g,c));
}
TEST(SignalGeometry, BodyOccupancyAndWholeBodyExitUseDifferentBoundaries) {
  const auto c=Settings();const auto g=Geometry();
  EXPECT_FALSE(sig::BodyOccupiesConflict(Pose(1,17),g,c));
  EXPECT_TRUE(sig::BodyOccupiesConflict(Pose(1,18),g,c));
  EXPECT_FALSE(sig::BodyOutsideExit(Pose(1,30),g,c));
  EXPECT_TRUE(sig::BodyOutsideExit(Pose(1,31),g,c));
}
TEST(SignalGeometry, InvalidShapeOrExitNeverPassesClearance) {
  auto c=Settings();auto g=Geometry();c.position_error=NAN;
  EXPECT_FALSE(sig::BodyBeforeLine(Pose(1,0),g,c));
  EXPECT_FALSE(sig::BodyOutsideExit(Pose(1,100),g,c));
  c=Settings();g.exit_direction.x=NAN;
  EXPECT_FALSE(sig::BodyOutsideExit(Pose(1,100),g,c));
}
TEST(SignalPolicy, UncertainCrossingOutsideBrakesAndInsideMayRecoverOnRed) {
  for(bool inside:{false,true}) for(bool safe:{false,true}) for(bool collision:{false,true}) {
    sig::Policy p(Settings());p.Bind(Geometry());p.SetTarget(5.5,false);Greens(p);
    auto e=Evidence();p.Step(Pose(1.1,16.9),1.1,11.1,e,{});
    auto yellow=Frame(1.15,sig::Color::YELLOW);p.Observe(yellow,1.15,11.15);
    auto red=Frame(1.2,sig::Color::RED);p.Observe(red,1.2,11.2);
    e.inside_conflict=inside;e.exit_path_valid=safe;Connect(e);
    sig::StopCauses causes;if(collision)causes.Set(sig::Cause::COLLISION,sig::Severity::EMERGENCY);
    auto out=p.Step(Pose(1.2,inside ? 18 : 17.1),1.2,11.2,e,causes);
    EXPECT_EQ(out.motion,inside && safe && !collision ? sig::Motion::RECOVERY_CLEAR : sig::Motion::RECOVERY_BRAKE);
    if(inside && safe && !collision)EXPECT_EQ(out.severity,sig::Severity::NONE);
    else EXPECT_NE(out.severity,sig::Severity::NONE);
    ASSERT_FALSE(p.events().empty());EXPECT_EQ(p.events().front().kind,sig::EventKind::ENTRY_UNCERTAIN);
    EXPECT_FALSE(out.actual_control_allowed);
    const auto n=p.events().size();e.exited=true;
    p.Step(Pose(1.25,32),1.25,11.25,e,causes);EXPECT_GE(p.events().size(),n);
    EXPECT_EQ(p.events().front().kind,sig::EventKind::ENTRY_UNCERTAIN);
  }
}
TEST(SignalPolicy, UnknownSignalDoesNotInvalidateFreshRecoveryPositionOrRisks) {
  sig::Policy p(Settings());p.Bind(Geometry());p.SetTarget(5.5,false);
  auto e=Evidence();e.inside_conflict=e.exit_path_valid=true;Connect(e);
  EXPECT_EQ(p.Step(Pose(1,18),1,11,e,{}).motion,sig::Motion::RECOVERY_CLEAR);
  e.risks_fresh=false;auto out=p.Step(Pose(1.05,18),1.05,11.05,e,{});
  EXPECT_EQ(out.motion,sig::Motion::RECOVERY_BRAKE);EXPECT_EQ(out.severity,sig::Severity::EMERGENCY);
}
TEST(SignalPolicy, RecoveryWaitsForBothAcceptanceAndActualTrajectoryMembership) {
  sig::Policy p(Settings());p.Bind(Geometry());p.SetTarget(5.5,false);
  auto e=Evidence();e.inside_conflict=e.exit_path_valid=e.linked=true;e.command_id=12;
  e.command_accepted=true;e.trajectory_belongs_to_command=false;
  auto out=p.Step(Pose(2,18),2,12,e,{});
  EXPECT_EQ(out.motion,sig::Motion::RECOVERY_CLEAR_PENDING);EXPECT_NE(out.severity,sig::Severity::NONE);
}
TEST(SignalPolicy, BrakingOvershootNeverBecomesNormalAfterGreenOrRebind) {
  sig::Policy p(Settings());p.Bind(Geometry());p.SetTarget(5.5,false);auto e=Evidence();
  p.Step(Pose(1,15),1,11,e,{});p.Step(Pose(1.05,18),1.05,11.05,e,{});
  Greens(p,1.1);e.inside_conflict=e.exit_path_valid=true;Connect(e);
  p.Step(Pose(1.2,19),1.2,11.2,e,{});
  bool braking_cross=false;
  for(const auto& event:p.events()) {
    EXPECT_NE(event.kind,sig::EventKind::NORMAL_ENTRY_CONFIRMED);
    braking_cross |= event.kind==sig::EventKind::STOP_LINE_CROSS_DURING_BRAKING;
  }
  EXPECT_TRUE(braking_cross);const auto size=p.events().size();
  auto g=Geometry();g.key.route_generation=2;p.Bind(g);
  EXPECT_EQ(p.events().size(),size);EXPECT_FALSE(p.gate().authorized());
}
TEST(SignalPolicy, ContinuousPermissionAndPostCrossGreenCanConfirmNormalEntry) {
  sig::Policy p(Settings());p.Bind(Geometry());p.SetTarget(5.5,false);Greens(p);
  auto e=Evidence();p.Step(Pose(1.1,16.9),1.1,11.1,e,{});
  auto f=Frame(1.15);p.Observe(f,1.15,11.15);
  auto out=p.Step(Pose(1.15,17.1),1.15,11.15,e,{});
  EXPECT_EQ(out.motion,sig::Motion::NORMAL_CLEAR);EXPECT_EQ(out.severity,sig::Severity::NONE);
}
TEST(SignalPolicy,PassedNormalSignalCannotReapplyBrakingOrEraseOtherHazards) {
  sig::Policy p(Settings());p.Bind(Geometry());p.SetTarget(5.5,false);Greens(p);
  auto e=Evidence();p.Step(Pose(1.1,16.9),1.1,11.1,e,{});
  p.Observe(Frame(1.15),1.15,11.15);
  ASSERT_EQ(p.Step(Pose(1.15,17.1),1.15,11.15,e,{}).motion,sig::Motion::NORMAL_CLEAR);
  auto missing=Frame(1.2,sig::Color::UNKNOWN);missing.valid=false;
  p.Observe(missing,1.2,11.2);
  auto out=p.Step(Pose(1.5,24),1.5,11.5,e,{});
  EXPECT_EQ(out.motion,sig::Motion::NORMAL_CLEAR);
  EXPECT_EQ(out.signal_requirement,sig::Severity::NONE);EXPECT_EQ(out.severity,sig::Severity::NONE);
  e.exited=true;out=p.Step(Pose(1.6,32),1.6,11.6,e,{});
  EXPECT_EQ(out.motion,sig::Motion::EXITED);EXPECT_EQ(out.severity,sig::Severity::NONE);
  p.Observe(Frame(1.7,sig::Color::RED),1.7,11.7);
  out=p.Step(Pose(1.7,33),1.7,11.7,e,{});
  EXPECT_EQ(out.motion,sig::Motion::EXITED);EXPECT_EQ(out.signal_requirement,sig::Severity::NONE);
  sig::StopCauses collision;collision.Set(sig::Cause::COLLISION,sig::Severity::EMERGENCY);
  out=p.Step(Pose(1.75,33.2),1.75,11.75,e,collision);
  EXPECT_EQ(out.signal_requirement,sig::Severity::NONE);EXPECT_EQ(out.severity,sig::Severity::EMERGENCY);
  ASSERT_EQ(p.events().size(),1u);EXPECT_EQ(p.events().front().kind,sig::EventKind::NORMAL_ENTRY_CONFIRMED);
}
TEST(SignalPolicy,GreenDepartureKeepsClearingAfterCameraLosesSignalPastTheLine) {
  sig::Policy p(Settings());p.Bind(Geometry());p.SetTarget(5.5,false);
  auto e=Evidence();e.position_history_continuous=true;Connect(e);
  EXPECT_EQ(p.Step(Pose(1,14.5,0),1,11,e,{}).motion,sig::Motion::STATIONARY_HOLD);
  Greens(p,1.1);
  EXPECT_EQ(p.Step(Pose(1.21,14.5,0),1.21,11.21,e,{}).severity,sig::Severity::NONE);
  p.Step(Pose(1.26,16.8,3),1.26,11.26,e,{});
  auto out=p.Step(Pose(1.31,17.1,3),1.31,11.31,e,{});
  ASSERT_EQ(out.motion,sig::Motion::GREEN_CROSSING_CLEAR);
  auto lost=Frame(1.32,sig::Color::UNKNOWN);lost.valid=false;
  p.Observe(lost,1.32,11.32);
  out=p.Step(Pose(1.36,17.3,3),1.36,11.36,e,{});
  EXPECT_EQ(out.motion,sig::Motion::GREEN_CROSSING_CLEAR);
  EXPECT_EQ(out.signal_requirement,sig::Severity::NONE);
  EXPECT_EQ(out.severity,sig::Severity::NONE);
  EXPECT_FALSE(out.entry_authorized); // No new admission on an unknown signal.
  ASSERT_EQ(p.events().size(),1u);
  EXPECT_EQ(p.events()[0].kind,sig::EventKind::ENTRY_UNCERTAIN);
  sig::StopCauses collision;collision.Set(sig::Cause::COLLISION,sig::Severity::EMERGENCY);
  out=p.Step(Pose(1.41,17.4,3),1.41,11.41,e,collision);
  EXPECT_EQ(out.signal_requirement,sig::Severity::NONE);
  EXPECT_EQ(out.severity,sig::Severity::EMERGENCY);
  e.inside_conflict=sig::BodyDefinitelyOccupiesConflict(Pose(1.46,17.5,1),Geometry(),Settings());
  e.exit_path_valid=true;
  out=p.Step(Pose(1.46,17.5,1),1.46,11.46,e,{});
  EXPECT_EQ(out.motion,sig::Motion::RECOVERY_CLEAR);
  EXPECT_EQ(out.severity,sig::Severity::NONE);
  EXPECT_EQ(p.events()[0].kind,sig::EventKind::ENTRY_UNCERTAIN);
}
TEST(SignalPolicy,RedAfterGreenCrossingKeepsClearingButRedBeforeCrossingDoesNot) {
  for(bool red_before_crossing:{false,true}) {
    sig::Policy p(Settings());p.Bind(Geometry());p.SetTarget(5.5,false);
    auto e=Evidence();e.position_history_continuous=true;Greens(p);
    p.Step(Pose(1.1,16.8,3),1.1,11.1,e,{});
    if(red_before_crossing)p.Observe(Frame(1.12,sig::Color::RED),1.12,11.12);
    auto out=p.Step(Pose(1.15,17.1,3),1.15,11.15,e,{});
    if(!red_before_crossing)p.Observe(Frame(1.16,sig::Color::RED),1.16,11.16);
    out=p.Step(Pose(1.2,17.3,3),1.2,11.2,e,{});
    EXPECT_EQ(out.signal_requirement,red_before_crossing ? sig::Severity::EMERGENCY : sig::Severity::NONE);
    EXPECT_EQ(out.motion,red_before_crossing ? sig::Motion::RECOVERY_BRAKE :
        sig::Motion::GREEN_CROSSING_CLEAR);
    for(const auto& event:p.events())EXPECT_NE(event.kind,sig::EventKind::NORMAL_ENTRY_CONFIRMED);
  }
}
TEST(SignalPolicy, InferenceLagDefersClassificationWithoutInventingPermanentUncertainty) {
  auto config=Settings();config.position_error=.1;
  sig::Policy p(config);p.Bind(Geometry());p.SetTarget(5.5,false);Greens(p);
  auto e=Evidence();p.Step(Pose(1.1,16.8),1.1,11.1,e,{});
  auto out=p.Step(Pose(1.15,17.2),1.15,11.15,e,{});
  EXPECT_TRUE(out.entry_pending);EXPECT_TRUE(p.events().empty());
  p.Observe(Frame(1.16),1.17,11.17);
  out=p.Step(Pose(1.2,17.3),1.2,11.2,e,{});
  EXPECT_FALSE(out.entry_pending);EXPECT_EQ(out.motion,sig::Motion::NORMAL_CLEAR);
  ASSERT_EQ(p.events().size(),1u);EXPECT_EQ(p.events()[0].kind,sig::EventKind::NORMAL_ENTRY_CONFIRMED);
}
TEST(SignalPolicy, OriginalGreenCapturePoseCanProveEntryBeforeLatestVehicleSample) {
  auto c=Settings();c.position_error=.5;
  sig::Policy p(c);p.Bind(Geometry());p.SetTarget(5.5,false);Greens(p);
  auto e=Evidence();e.position_history_continuous=true;
  p.Step(Pose(1.1,16.4),1.1,11.1,e,{});
  p.Observe(Frame(1.2),1.21,11.21);
  EXPECT_TRUE(p.Step(Pose(1.22,17.2),1.22,11.22,e,{}).entry_pending);
  p.Observe(Frame(1.27),1.29,11.29);
  e.green_capture_pose_valid=true;e.green_capture_pose=Pose(1.27,17.6);
  const auto out=p.Step(Pose(1.3,17.8),1.3,11.3,e,{});
  EXPECT_EQ(out.motion,sig::Motion::NORMAL_CLEAR);
  ASSERT_EQ(p.events().size(),1u);EXPECT_DOUBLE_EQ(p.events()[0].interval_end,1.27);
}
TEST(SignalPolicy, UnorderedCapturePoseCannotBypassRevocationOrUnknownHistoryOrImageIdentity) {
  for(int invalid=0;invalid<5;++invalid) {
    auto c=Settings();c.position_error=.5;
    sig::Policy p(c);p.Bind(Geometry());p.SetTarget(5.5,false);Greens(p);
    auto e=Evidence();e.position_history_continuous=true;
    p.Step(Pose(1.1,16.4),1.1,11.1,e,{});
    p.Observe(Frame(1.2),1.21,11.21);p.Step(Pose(1.22,17.2),1.22,11.22,e,{});
    p.Observe(Frame(1.27),1.29,11.29);
    e.green_capture_pose_valid=true;e.green_capture_pose=Pose(1.27,17.6);
    if(invalid==0)p.Observe(Frame(1.28,sig::Color::YELLOW),1.29,11.29);
    if(invalid==1)e.position_history_continuous=false;
    if(invalid==2)e.green_capture_pose.stamp=1.26;
    if(invalid==3)e.green_capture_pose_valid=false;
    if(invalid==4)e.position_valid=false;
    const auto out=p.Step(Pose(1.3,17.8),1.3,11.3,e,{});
    EXPECT_NE(out.motion,sig::Motion::NORMAL_CLEAR);
    for(const auto& event:p.events())EXPECT_NE(event.kind,sig::EventKind::NORMAL_ENTRY_CONFIRMED);
  }
}
TEST(SignalPolicy, OrderedOriginalGreenProofSurvivesLaterRedInTheSameBehaviorBatch) {
  auto c=Settings();c.position_error=.5;sig::Policy p(c);p.Bind(Geometry());p.SetTarget(5.5,false);
  Greens(p);p.Step(Pose(1.1,16.4,6),1.1,11.1,Evidence(),{});
  sig::PoseHistory history(c);
  for(int i=0;i<=6;++i)history.Push(Pose(1.1+i*.05,16.4+i*.3,6));
  p.Observe(Frame(1.3),1.4,11.4,&history);
  p.Observe(Frame(1.35,sig::Color::RED),1.4,11.4,&history);
  const auto out=p.Step(Pose(1.4,18.2,6),1.4,11.4,Evidence(),{});
  EXPECT_EQ(out.motion,sig::Motion::NORMAL_CLEAR);EXPECT_EQ(out.signal_requirement,sig::Severity::NONE);
  ASSERT_EQ(p.events().size(),1u);EXPECT_EQ(p.events()[0].kind,sig::EventKind::NORMAL_ENTRY_CONFIRMED);
  EXPECT_DOUBLE_EQ(p.events()[0].interval_end,1.3);EXPECT_FALSE(p.gate().authorized());
}
TEST(SignalPolicy, GreenBeforePassageThenRedInTheSameBatchRemainsUncertain) {
  auto c=Settings();c.position_error=.5;sig::Policy p(c);p.Bind(Geometry());p.SetTarget(5.5,false);
  Greens(p);p.Step(Pose(1.1,16.4,6),1.1,11.1,Evidence(),{});
  sig::PoseHistory history(c);
  history.Push(Pose(1.1,16.4,6));history.Push(Pose(1.2,16.7,6));history.Push(Pose(1.3,17.3,6));
  p.Observe(Frame(1.2),1.3,11.3,&history);
  p.Observe(Frame(1.25,sig::Color::RED),1.3,11.3,&history);
  p.Step(Pose(1.3,17.3,6),1.3,11.3,Evidence(),{});
  ASSERT_FALSE(p.events().empty());
  for(const auto& event:p.events())EXPECT_NE(event.kind,sig::EventKind::NORMAL_ENTRY_CONFIRMED);
  EXPECT_EQ(p.events()[0].kind,sig::EventKind::ENTRY_UNCERTAIN);
}
TEST(SignalPolicy, RedBeforePostCrossGreenProofBreaksTheHistoricalPermissionChain) {
  auto c=Settings();c.position_error=.5;sig::Policy p(c);p.Bind(Geometry());p.SetTarget(5.5,false);
  Greens(p);p.Step(Pose(1.1,16.4,6),1.1,11.1,Evidence(),{});
  sig::PoseHistory history(c);
  for(int i=0;i<=6;++i)history.Push(Pose(1.1+i*.05,16.4+i*.3,6));
  p.Observe(Frame(1.15,sig::Color::RED),1.4,11.4,&history);
  for(double t:{1.2,1.25,1.3})p.Observe(Frame(t),1.4,11.4,&history);
  p.Step(Pose(1.4,18.2,6),1.4,11.4,Evidence(),{});
  ASSERT_FALSE(p.events().empty());
  for(const auto& event:p.events())EXPECT_NE(event.kind,sig::EventKind::NORMAL_ENTRY_CONFIRMED);
}
TEST(SignalPolicy, OrderedProofNeedsContinuousHistoryAndTheLargestCrossingWindowError) {
  for(bool gap:{true,false}) {
    auto c=Settings();c.position_error=.7;c.entry_observation_error=.5;c.entry_observation_speed_max=7.;
    sig::Policy p(c);p.Bind(Geometry());p.SetTarget(5.5,false);Greens(p);
    p.Step(Pose(1.1,16.2,6),1.1,11.1,Evidence(),{});
    sig::PoseHistory history(c);history.Push(Pose(1.1,16.2,6));
    if(!gap)history.Push(Pose(1.15,16.5,8.2));
    if(!gap)history.Push(Pose(1.2,16.8,6));
    if(!gap)history.Push(Pose(1.25,17.2,6));
    history.Push(Pose(1.3,17.6,6));
    p.Observe(Frame(1.3),1.35,11.35,&history);
    p.Observe(Frame(1.31,sig::Color::YELLOW),1.35,11.35,&history);
    p.Step(Pose(1.35,17.9,6),1.35,11.35,Evidence(),{});
    ASSERT_FALSE(p.events().empty());
    for(const auto& event:p.events())EXPECT_NE(event.kind,sig::EventKind::NORMAL_ENTRY_CONFIRMED);
  }
}
TEST(SignalGeometry, PossibleBodyOverlapIsNotProofOfEntryForRecovery) {
  auto c=Settings();c.position_error=.5;const auto g=Geometry();
  const auto p=Pose(1,17.2);
  EXPECT_TRUE(sig::BodyOccupiesConflict(p,g,c));
  EXPECT_FALSE(sig::BodyDefinitelyOccupiesConflict(p,g,c));
}
TEST(SignalPolicy, GreenNeverClearsCollisionPedestrianOrExecutionFailure) {
  for(auto cause:{sig::Cause::COLLISION,sig::Cause::PEDESTRIAN,sig::Cause::EXECUTION_FAILURE}) {
    sig::Policy p(Settings());p.Bind(Geometry());p.SetTarget(5.5,false);Greens(p);
    sig::StopCauses causes;causes.Set(cause,sig::Severity::EMERGENCY);
    const auto out=p.Step(Pose(1.1,10),1.1,11.1,Evidence(),causes);
    EXPECT_EQ(out.severity,sig::Severity::EMERGENCY);
    EXPECT_EQ(out.signal_requirement,sig::Severity::NONE);
  }
}
TEST(SignalPolicy, OtherCauseReleaseStillRequiresTrajectoryBinding) {
  sig::Policy p(Settings());p.Bind(Geometry());p.SetTarget(5.5,false);Greens(p);
  sig::StopCauses causes;causes.Set(sig::Cause::PEDESTRIAN,sig::Severity::STOP);
  p.Step(Pose(1.1,10),1.1,11.1,Evidence(),causes);
  auto out=p.Step(Pose(1.15,10),1.15,11.15,Evidence(),{});
  EXPECT_EQ(out.signal_requirement,sig::Severity::NONE);EXPECT_EQ(out.severity,sig::Severity::STOP);
  auto e=Evidence();Connect(e);
  out=p.Step(Pose(1.2,10),1.2,11.2,e,{});EXPECT_EQ(out.severity,sig::Severity::NONE);
}
TEST(SignalPolicy, GreenCannotDowngradeLatchedEmergencyBeforeStopAndBinding) {
  sig::Policy p(Settings());p.Bind(Geometry());p.SetTarget(5.5,false);
  p.Step(Pose(1,14,8),1,11,Evidence(),{});Greens(p,1.1);
  auto e=Evidence();Connect(e);
  EXPECT_EQ(p.Step(Pose(1.2,14,0),1.2,11.2,e,{}).severity,sig::Severity::EMERGENCY);
}
TEST(SignalPolicy, HoldingRevisionUpdatesGoalWithoutForwardMotionOrOldCompletion) {
  sig::Policy p(Settings());p.Bind(Geometry());p.SetTarget(5.5,false);auto e=Evidence();
  auto out=p.Step(Pose(1,14.5,0),1,11,e,{});ASSERT_TRUE(out.target_reached);
  p.SetTarget(4.345,true);out=p.Step(Pose(1.05,14.5,0),1.05,11.05,e,{});
  EXPECT_EQ(out.motion,sig::Motion::STATIONARY_HOLD);EXPECT_EQ(out.target_revision,2u);
  EXPECT_FALSE(out.target_reached);EXPECT_TRUE(out.stationary);
}
TEST(SignalPolicy, UnreachedStationaryHoldAlsoStaysHeldOnRevision) {
  sig::Policy p(Settings());p.Bind(Geometry());p.SetTarget(5.5,false);
  auto out=p.Step(Pose(1,10,0),1,11,Evidence(),{});
  ASSERT_EQ(out.motion,sig::Motion::STATIONARY_HOLD);ASSERT_FALSE(out.target_reached);
  p.SetTarget(4.345,true);out=p.Step(Pose(1.05,10,0),1.05,11.05,Evidence(),{});
  EXPECT_EQ(out.motion,sig::Motion::STATIONARY_HOLD);EXPECT_FALSE(out.target_reached);
  EXPECT_EQ(out.target_revision,2u);
}
TEST(SignalPolicy, OutsideUncertainRecoveryRequiresNewGreenAndKeepsItsRecord) {
  sig::Policy p(Settings());p.Bind(Geometry());p.SetTarget(5.5,false);
  auto e=Evidence();Connect(e);
  EXPECT_EQ(p.Step(Pose(1,17.1,0),1,11,e,{}).motion,sig::Motion::RECOVERY_BRAKE);
  Greens(p,2);
  EXPECT_EQ(p.Step(Pose(2.1,17.1,0),2.1,12.1,e,{}).motion,sig::Motion::RELEASE_PENDING);
  ASSERT_FALSE(p.events().empty());EXPECT_EQ(p.events().front().kind,sig::EventKind::ENTRY_UNCERTAIN);
}
TEST(SignalPolicy,GreenReentryAfterOvershootNeedsFullPathAndDoesNotCreepBackIntoBraking) {
  sig::Policy p(Settings());p.Bind(Geometry());p.SetTarget(5.5,false);
  auto e=Evidence();Connect(e);e.position_history_continuous=true;
  auto out=p.Step(Pose(1,17.1,0),1,11,e,{});
  ASSERT_EQ(out.motion,sig::Motion::RECOVERY_BRAKE);
  Greens(p,2);
  out=p.Step(Pose(2.1,17.1,0),2.1,12.1,e,{});
  ASSERT_EQ(out.motion,sig::Motion::RELEASE_PENDING);
  EXPECT_NE(out.severity,sig::Severity::NONE); // An ordinary short KEEP is insufficient.
  e.exit_path_valid=true;
  out=p.Step(Pose(2.15,17.1,0),2.15,12.15,e,{});
  ASSERT_EQ(out.severity,sig::Severity::NONE);
  out=p.Step(Pose(2.2,17.12,.5),2.2,12.2,e,{});
  EXPECT_EQ(out.motion,sig::Motion::RELEASE_PENDING);EXPECT_EQ(out.severity,sig::Severity::NONE);
  for(const auto& event:p.events())EXPECT_NE(event.kind,sig::EventKind::NORMAL_ENTRY_CONFIRMED);
  p.Observe(Frame(2.22,sig::Color::YELLOW),2.22,12.22);
  out=p.Step(Pose(2.23,17.13,.5),2.23,12.23,e,{});
  EXPECT_EQ(out.motion,sig::Motion::RECOVERY_BRAKE);EXPECT_NE(out.severity,sig::Severity::NONE);
}
TEST(SignalPolicy,GreenReentryCannotReuseItsPermitAfterPositionContinuityIsLost) {
  sig::Policy p(Settings());p.Bind(Geometry());p.SetTarget(5.5,false);
  auto e=Evidence();Connect(e);e.position_history_continuous=e.exit_path_valid=true;
  p.Step(Pose(1,17.1,0),1,11,e,{});Greens(p,2);
  ASSERT_EQ(p.Step(Pose(2.1,17.1,0),2.1,12.1,e,{}).severity,sig::Severity::NONE);
  e.position_history_continuous=false;
  EXPECT_EQ(p.Step(Pose(2.15,17.12,.5),2.15,12.15,e,{}).motion,sig::Motion::RECOVERY_BRAKE);
}
TEST(SignalPolicy, ValidFallbackPathDoesNotProveStopGoalAndGreenDoesNotEraseFailure) {
  sig::Policy p(Settings());p.Bind(Geometry());p.SetTarget(5.5,false);
  auto e=Evidence();e.planning_checked=e.stop_path_valid=true;e.path_goal_satisfied=false;
  EXPECT_EQ(p.Step(Pose(1,5),1,11,e,{}).severity,sig::Severity::EMERGENCY);
  Greens(p,1.1);e.planning_checked=false;
  EXPECT_EQ(p.Step(Pose(1.2,5),1.2,11.2,e,{}).severity,sig::Severity::EMERGENCY);
  bool recorded=false;for(const auto& event:p.events())recorded|=event.kind==sig::EventKind::PATH_FAILURE;
  EXPECT_TRUE(recorded);
}
TEST(SignalBraking, VehicleAgeAndPendingCommandDelayAreSeparateIntervals) {
  const auto projected=sig::ProjectMeasuredState(5,1,.1);
  ASSERT_TRUE(projected.valid);EXPECT_NEAR(projected.distance,.505,1e-12);
  EXPECT_NEAR(projected.speed,5.1,1e-12);
  sig::Policy p(Settings());p.Bind(Geometry());p.SetTarget(5.5,false);
  auto e=Evidence();e.state_age=.1;
  const auto out=p.Step(Pose(1,5,5,1),1.1,11.1,e,{});
  EXPECT_NEAR(out.minimum_distance,sig::MinimumStoppingDistance(5.1,1,Settings().brake,.5),1e-10);
  EXPECT_FALSE(sig::ProjectMeasuredState(5,1,-.1).valid);
}
TEST(SignalBraking, DelayBudgetAndLateResponseAreNotOptimistic) {
  EXPECT_DOUBLE_EQ(sig::ResponseBudget(.2),.5);EXPECT_DOUBLE_EQ(sig::ResponseBudget(.7),.8);
  sig::ResponseMonitor m;m.Begin(1,10,5,0,.5,1);
  m.Begin(1,10.4,5,0,.5,1.4);  // heartbeat cannot postpone deadline
  EXPECT_EQ(m.Observe(10.51,1.51,4,-1,true),sig::BrakeResponse::FAILED);
  EXPECT_TRUE(m.failed());m.Begin(2,10.6,4,-1,.5,1.6);
  EXPECT_EQ(m.Observe(11.11,2.11,3.5,-1,true),sig::BrakeResponse::FAILED);
  EXPECT_TRUE(m.failed());
}
TEST(SignalBraking, ValidResponseRequiresNewStateAndExtraResponseAfterEscalation) {
  sig::ResponseMonitor m;m.Begin(1,10,5,0,.5,1);
  EXPECT_EQ(m.Observe(10.1,1,4,-1,true),sig::BrakeResponse::WAITING);
  EXPECT_EQ(m.Observe(10.2,1.2,4.9,-1,true),sig::BrakeResponse::OBSERVED);
  m.Begin(2,10.3,4.8,-1,.5,1.3);
  EXPECT_EQ(m.Observe(10.4,1.4,4.7,-1,true),sig::BrakeResponse::WAITING);
}
TEST(SignalBraking, StationaryHoldDoesNotInventResponseOrFailure) {
  sig::ResponseMonitor m;m.Begin(2,10,0,0,.5,1);
  EXPECT_EQ(m.Observe(11,2,0,0,true),sig::BrakeResponse::IDLE);
  EXPECT_FALSE(m.observed());EXPECT_FALSE(m.failed());
  m.Begin(2,11,1,0,.5,2);  // A later moving request still starts monitoring.
  EXPECT_EQ(m.Observe(11.6,2.6,1,0,true),sig::BrakeResponse::FAILED);
}
TEST(SignalBraking,OrdinaryResponseCanMaintainExistingMeasuredDeceleration) {
 sig::ResponseMonitor m;m.Begin(1,10,3.4,-.8,.5,1);
 EXPECT_EQ(m.Observe(10.1,1,3.3,-.8,true),sig::BrakeResponse::WAITING);
 EXPECT_EQ(m.Observe(10.2,1.2,3.4,-.8,true),sig::BrakeResponse::WAITING);
 EXPECT_EQ(m.Observe(10.3,1.3,3.2,0,true),sig::BrakeResponse::WAITING);
 EXPECT_EQ(m.Observe(10.4,1.4,3.1,-.78,true),sig::BrakeResponse::OBSERVED);
 m.Begin(2,10.4,3.1,-.78,.5,1.4);
 EXPECT_EQ(m.Observe(10.5,1.5,3.,-.78,true),sig::BrakeResponse::WAITING);
 EXPECT_EQ(m.Observe(10.6,1.6,2.9,-2.,true),sig::BrakeResponse::OBSERVED);
}
TEST(SignalBraking, SharedDistanceMatchesSpatialGeneratorAndPreservesActualSpeed) {
  sig::BrakeLimits limits;PathGeneratorConfig cfg;cfg.dt=.05;
  cfg.delta_s={0,0,0};cfg.time_horizon={1,12,1};KinematicLimits kl;
  kl.max_longitudinal_accel=5;kl.max_longitudinal_jerk=6;kl.boundary_condition_accel_fraction=.8;
  for(double v:{25/3.6,40/3.6}) for(double a:{-3.5,0.,2.}) {
    FrenetState start{};start.s_d=v;start.s_dd=a;
    const double d=sig::MinimumStoppingDistance(v,a,limits);
    ASSERT_TRUE(std::isfinite(d));
    EXPECT_TRUE(GenerateSpatialStoppingCandidates(start,d-.1, v,1.5,cfg,kl).empty());
    EXPECT_FALSE(GenerateSpatialStoppingCandidates(start,d+.1,v,1.5,cfg,kl).empty());
    EXPECT_GT(sig::MinimumStoppingDistance(v,a,limits,.5),d);
  }
}
TEST(SignalBraking, BrakingOnlyRevisionDoesNotCreatePositiveAcceleration) {
  FrenetState start{};start.s_d=5;start.s_dd=-1;
  PathGeneratorConfig cfg;cfg.dt=.05;cfg.delta_s={0,0,0};cfg.time_horizon={1,12,1};
  KinematicLimits l;l.max_longitudinal_accel=5;l.max_longitudinal_jerk=6;l.boundary_condition_accel_fraction=.8;
  auto paths=GenerateSpatialStoppingCandidates(start,10,13.8,1.5,cfg,l,.5,true);
  ASSERT_FALSE(paths.empty());for(const auto& p:paths)for(double a:p.s_dd)EXPECT_LE(a,1e-6);
}

TEST(SignalHistory, MeasuredInterpolationBoundStillRejectsLongerSourceOutage) {
 auto c=Settings();c.interpolation_gap=.17;
 sig::PoseHistory h(c);sig::Pose at;
 h.Push(Pose(10,0,6));h.Push(Pose(10.16,.96,6));
 ASSERT_TRUE(h.At(10.08,at));
 EXPECT_NEAR(at.x,.48,1e-8);
 EXPECT_TRUE(h.Covers(10,10.16));
 h.Push(Pose(10.34,2.04,6));
 EXPECT_FALSE(h.At(10.25,at));
 EXPECT_FALSE(h.Covers(10.16,10.34));
}
