#include <gtest/gtest.h>
#include "Planning/GpsRecoveryGate.hpp"
using namespace asmc_planner;
TEST(GpsRecoveryGate, RequiresDistinctCoherentGpsObservations) {
  GpsRecoveryGate gate(.3,3);
  EXPECT_FALSE(gate.observe(10,.1));
  EXPECT_FALSE(gate.observe(10,.1));
  EXPECT_FALSE(gate.observe(10,.1));
  EXPECT_EQ(1u,gate.confirmations());
  EXPECT_FALSE(gate.observe(10.2,.1));
  EXPECT_TRUE(gate.observe(10.4,.1));
}
TEST(GpsRecoveryGate, LargeCorrectionAndMissingInputResetConfirmation) {
  GpsRecoveryGate gate(.3,3);
  gate.observe(10,.1);gate.observe(10.2,.1);
  EXPECT_FALSE(gate.observe(10.4,6.7));
  EXPECT_FALSE(gate.observe(10.4,.1));
  EXPECT_FALSE(gate.observe(10.6,{}));
  EXPECT_EQ(0u,gate.confirmations());
}
TEST(GpsRecoveryGate, RestartDoesNotReusePriorConfirmations) {
  GpsRecoveryGate gate(.3,3);
  gate.observe(10,.1);gate.observe(10.2,.1);
  EXPECT_FALSE(gate.observe(1,.1));
  EXPECT_EQ(1u,gate.confirmations());
  EXPECT_FALSE(gate.observe(1.2,.1));
  EXPECT_TRUE(gate.observe(1.4,.1));
}
TEST(GpsRecoveryGate, SourceTimeAndMountingAreAlignedBeforeComparison) {
  RecoveryPose pose{10,100,200,0,5,0};
  auto residual=GpsPoseResidual(pose,9.8,99.35,200,.35,0,.3,.1);
  ASSERT_TRUE(residual);EXPECT_NEAR(0,*residual,1e-10);
  // A source-time gap cannot be treated as an instantaneous GPS correction.
  EXPECT_FALSE(GpsPoseResidual(pose,9,99.35,200,.35,0,.3,.1));
  EXPECT_FALSE(GpsPoseResidual(pose,11,99.35,200,.35,0,.3,.1));
}
TEST(GpsRecoveryGate, NonfiniteOrMissingStateCannotAuthorizeRecovery) {
  RecoveryPose pose{10,NAN,200,0,5,0};
  EXPECT_FALSE(GpsPoseResidual(pose,10,100,200,.35,0,.3,.1));
  GpsRecoveryGate gate(.3,3);
  EXPECT_FALSE(gate.observe(10,NAN));
  EXPECT_FALSE(gate.observe(10,-1));
  EXPECT_FALSE(gate.observe(0,.1));
}
