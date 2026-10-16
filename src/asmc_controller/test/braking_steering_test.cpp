#include <gtest/gtest.h>
#include "Util/BrakingSteering.hpp"
TEST(BrakingSteering, MovingStopDecaysAtRateWithoutSignChange) {
  MPCParams p;
  EXPECT_DOUBLE_EQ(BrakingSteering(.3,0,8,p),.3);
  EXPECT_NEAR(BrakingSteering(.3,.05,8,p),.25,1e-12);
  EXPECT_NEAR(BrakingSteering(-.3,.05,8,p),-.25,1e-12);
  EXPECT_DOUBLE_EQ(BrakingSteering(.3,1,8,p),0);
}
TEST(BrakingSteering, StationaryAndInvalidHistoryClearSteering) {
  MPCParams p;
  EXPECT_DOUBLE_EQ(BrakingSteering(.3,0,.1,p),0);
  EXPECT_DOUBLE_EQ(BrakingSteering(.3,-1,8,p),0);
  EXPECT_DOUBLE_EQ(BrakingSteering(NAN,0,8,p),0);
}

TEST(BrakingSteering, ShortHighSpeedStopDoesNotSnapSteeringToZero) {
  MPCParams p;
  p.steering_rate_reference_speed_mps = 5.0;
  p.steering_rate_min = 0.1;
  // Recorded failure: a short collision STOP centered the wheels while the
  // vehicle was still following a bend. The normal solver permits only 0.005
  // rad per tick here, whereas the old fixed 1 rad/s decay removed all 0.04.
  for (double sign : {-1.0, 1.0}) {
    EXPECT_NEAR(BrakingSteering(sign*.04, .05, 58.0/3.6, p), sign*.035, 1e-12);
    double steering = sign*.1;
    for (double speed : {16.0, 14.0, 12.0, 10.0, 8.0}) {
      const double next = BrakingSteering(steering, .05, speed, p);
      EXPECT_LE(std::fabs(next-steering), SteeringRateLimit(speed,p)*.05+1e-12);
      EXPECT_GE(sign*next, 0.0);
      steering = next;
    }
  }
}
