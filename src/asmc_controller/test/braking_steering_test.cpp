#include <gtest/gtest.h>
#include "Util/BrakingSteering.hpp"
#include "Util/EmergencySteering.hpp"
#include "MPC/Reference/braking_reference.hpp"
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

TEST(EmergencySteering, InitialOutputAndRepeatedTicksShareThePublishedHistory) {
  MPCParams p;EmergencySteeringState state;
  state.Capture(.3,10,true,1,100);
  EXPECT_DOUBLE_EQ(state.Fallback(10,8,true,p),.3);
  const double next=state.Fallback(10.05,8,true,p);
  EXPECT_LE(std::abs(next-.3),SteeringRateLimit(8,p)*.05+1e-9);
  state.Published(next,10.05);
  state.Capture(-.4,10.1,true,2,200);  // heartbeat cannot recapture/reset
  EXPECT_EQ(state.generation,1u);EXPECT_DOUBLE_EQ(state.geometry_stamp,100);
  EXPECT_DOUBLE_EQ(state.Fallback(10.1,8,false,p),0);
}
TEST(EmergencySteering, MaximumBrakeDoesNotExtrapolatePartialPedalCalibration) {
  MPCParams p;p.use_calibrated_pedal_mapping=true;p.brake_full_decel=8.9;
  p.brake_pedal_lut={0,.5,.7};p.brake_response_lut={0,9.5,10.3};
  EXPECT_DOUBLE_EQ(BrakePedalDeceleration(1,p),8.9);
  EXPECT_DOUBLE_EQ(BrakePedalDeceleration(.7,p),10.3);
}
TEST(EmergencySteering, InsufficientGeometryCannotBeExtendedForSignalEmergency) {
  MPCParams p;p.horizon=20;p.dt=.1;ReferencePath r;
  for(double x:{0.,.1,.2}) {
    r.x_ref.push_back(x);r.y_ref.push_back(0);r.yaw_ref.push_back(0);
    r.k_ref.push_back(0);r.v_ref.push_back(5);r.a_ref.push_back(0);
  }
  EXPECT_TRUE(MakeBrakingReference(r,MPCState{0,0,0,5},8.9,p,false).empty());
}
