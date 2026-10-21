#include <gtest/gtest.h>
#include "Planning/CompetitionSpeedAlignment.hpp"
using planning::CompetitionSpeedAlignment;
TEST(CompetitionSpeedAlignment, PreservesPhysicalSpeedAndAlignsWithImuAcceleration) {
  CompetitionSpeedAlignment alignment;
  ASSERT_TRUE(alignment.observe(10,100,15));
  ASSERT_TRUE(alignment.align(10.02,2,100.03));
  EXPECT_NEAR(15.04,*alignment.align(10.02,2,100.03),1e-8);
}
TEST(CompetitionSpeedAlignment, MissingFutureDelayedAndReceiveStaleNeverBecomeValid) {
  CompetitionSpeedAlignment alignment;
  EXPECT_FALSE(alignment.align(10,0,100));
  alignment.observe(10.1,100,15);
  EXPECT_FALSE(alignment.align(10,0,100.01));
  EXPECT_FALSE(alignment.align(10.3,0,100.02));
  EXPECT_FALSE(alignment.align(10.12,0,100.3));
}
TEST(CompetitionSpeedAlignment, OutOfOrderAndDuplicatesDoNotReplaceCurrentMeasurement) {
  CompetitionSpeedAlignment alignment;
  alignment.observe(10,100,15);
  EXPECT_FALSE(alignment.observe(9.99,100.01,0));
  EXPECT_FALSE(alignment.observe(10,100.02,0));
  EXPECT_DOUBLE_EQ(15,*alignment.align(10.02,0,100.03));
}
TEST(CompetitionSpeedAlignment, RestartCannotJoinDifferentTimeEpochs) {
  CompetitionSpeedAlignment alignment;
  alignment.observe(100,10,15);
  EXPECT_TRUE(alignment.align(100.01,0,10.01));
  EXPECT_FALSE(alignment.align(1,0,10.02));
  alignment.observe(1.01,10.03,0);
  EXPECT_DOUBLE_EQ(0,*alignment.align(1.02,0,10.04));
}
TEST(CompetitionSpeedAlignment, SpeedRestartBeforePoseAlsoClearsOldMeasurements) {
  CompetitionSpeedAlignment alignment;
  alignment.observe(100,10,15);alignment.align(100,0,10);
  alignment.observe(1,10.01,0);
  EXPECT_DOUBLE_EQ(0,*alignment.align(1.02,0,10.02));
}
TEST(CompetitionSpeedAlignment, InvalidAndReverseMeasurementsAreRejected) {
  CompetitionSpeedAlignment alignment;
  EXPECT_FALSE(alignment.observe(10,100,NAN));
  EXPECT_FALSE(alignment.observe(10,100,-1));
  EXPECT_FALSE(alignment.observe(0,100,10));
  EXPECT_FALSE(alignment.align(NAN,0,100));
}
TEST(CompetitionSpeedAlignment, SignedStationaryJitterNeverBecomesMissingSpeed) {
  CompetitionSpeedAlignment alignment;
  for(int i=0;i<100;++i) {
    const double t=10+i*.02;
    ASSERT_TRUE(alignment.observe(t,100+i*.02,i%2?-.04:.02));
    const auto speed=alignment.align(t,0,100+i*.02+.01);
    ASSERT_TRUE(speed);EXPECT_DOUBLE_EQ(0,*speed);
  }
  EXPECT_FALSE(alignment.observe(12.1,102.1,-.1));
}
