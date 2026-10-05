#include <gtest/gtest.h>
#include "control/braking_steering.hpp"
TEST(BrakingSteering, MovingStopDecaysAtRateWithoutSignChange) {
  EXPECT_DOUBLE_EQ(BrakingSteering(.3,0,8,1),.3);
  EXPECT_NEAR(BrakingSteering(.3,.05,8,1),.25,1e-12);
  EXPECT_NEAR(BrakingSteering(-.3,.05,8,1),-.25,1e-12);
  EXPECT_DOUBLE_EQ(BrakingSteering(.3,1,8,1),0);
}
TEST(BrakingSteering, StationaryAndInvalidHistoryClearSteering) {
  EXPECT_DOUBLE_EQ(BrakingSteering(.3,0,.1,1),0);
  EXPECT_DOUBLE_EQ(BrakingSteering(.3,-1,8,1),0);
  EXPECT_DOUBLE_EQ(BrakingSteering(NAN,0,8,1),0);
}
