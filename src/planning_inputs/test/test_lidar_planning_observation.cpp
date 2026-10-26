#include <gtest/gtest.h>
#include <planning_inputs/lidar_observation_gate.hpp>

namespace {
using Packet = asmc_msgs::LidarPlanningObservation;
Packet Frame(double stamp = 10.) {
  Packet p;
  auto& o = p.observation;
  o.producer_session_id = "session-a"; o.config_id = "config"; o.calibration_id = "calib";
  o.generation = 1; o.scan_sequence = 1;
  auto& a = o.objects;
  a.header.stamp.fromSec(stamp); a.header.frame_id = "map"; a.status = a.OBSERVED;
  asmc_msgs::LidarObject b;
  b.track_id = 42; b.observation_state = b.OBSERVED; b.age = 5;
  b.center.x = 100.; b.center.y = 100.; b.center.z = .5;
  b.width = 2.; b.length = 4.; b.height = 1.; b.classification = "vehicle"; b.camera_class = "car";
  b.absolute_velocity_valid = true; b.absolute_velocity_mps.x = 3.;
  for (unsigned i = 0; i < 8; ++i) {
    b.obb_corners[i].x = i & 1 ? 102. : 98.;
    b.obb_corners[i].y = i & 2 ? 101. : 99.;
    b.obb_corners[i].z = i & 4 ? 1. : 0.;
  }
  a.objects.push_back(b); p.motion_states.push_back(p.UNKNOWN);
  return p;
}
TEST(LidarObservation, FourStatesPreserveMeasuredSpeedAndOnlyLowThreatStopsPrediction) {
  for (uint8_t state = 0; state < 4; ++state) {
    planning_inputs::LidarObservationGate gate;
    auto p = Frame(); p.motion_states[0] = state;
    p.observation.objects.objects[0].dynamic_valid = state == p.LOW_THREAT || state == p.DYNAMIC;
    p.observation.objects.objects[0].is_dynamic = state == p.DYNAMIC;
    planning_inputs::DetectedObjectArray out; std::string reason;
    ASSERT_TRUE(gate.Convert(p, 10.1, &out, &reason)) << reason;
    ASSERT_EQ(out.objects.size(), 1u);
    EXPECT_EQ(out.objects[0].motion_state, state);
    EXPECT_DOUBLE_EQ(out.objects[0].vx, state == p.LOW_THREAT ? 0. : 3.);
    EXPECT_EQ(out.header, p.observation.objects.header);
  }
}
TEST(LidarObservation, InvalidCannotBecomeEmptyOrMeasuredZero) {
  for (int variant = 0; variant < 10; ++variant) {
    planning_inputs::LidarObservationGate gate; auto p = Frame();
    auto& a = p.observation.objects; auto& b = a.objects[0];
    if (variant == 0) { b.absolute_velocity_valid = false; p.motion_states[0] = p.DYNAMIC_PENDING; }
    if (variant == 1) a.status = a.POSE_UNAVAILABLE;
    if (variant == 2) a.header.frame_id = "base_link";
    if (variant == 3) p.motion_states.clear();
    if (variant == 4) b.width = -1.;
    if (variant == 5) b.absolute_velocity_mps.y = NAN;
    if (variant == 6) b.miss_count = 1;
    if (variant == 7) a.status = a.OBSERVED_EMPTY;
    if (variant == 8) p.motion_states[0] = p.DYNAMIC;
    if (variant == 9) p.observation.producer_session_id.clear();
    planning_inputs::DetectedObjectArray out; out.header.frame_id = "untouched"; std::string reason;
    EXPECT_FALSE(gate.Convert(p, 10.1, &out, &reason)) << variant;
    EXPECT_EQ(out.header.frame_id, "untouched");
  }
}
TEST(LidarObservation, UnmeasuredVelocityRetainsGeometryWithoutInventingStationaryMotion) {
  planning_inputs::LidarObservationGate gate; auto p = Frame();
  p.observation.objects.objects[0].absolute_velocity_valid = false;
  p.observation.objects.objects[0].absolute_velocity_mps.x = NAN;  // invalid storage is never read
  planning_inputs::DetectedObjectArray out; std::string reason;
  ASSERT_TRUE(gate.Convert(p, 10.1, &out, &reason)) << reason;
  ASSERT_EQ(out.objects.size(), 1u);
  EXPECT_TRUE(out.objects[0].velocity_unavailable);
  EXPECT_EQ(out.objects[0].motion_state, out.objects[0].MOTION_UNKNOWN);
  EXPECT_EQ(out.objects[0].obb_corners, p.observation.objects.objects[0].obb_corners);
  EXPECT_DOUBLE_EQ(out.objects[0].vx, 0.);  // explicitly unmeasured occupancy placeholder
  EXPECT_FALSE(p.observation.objects.objects[0].absolute_velocity_valid);
}
TEST(LidarObservation, ValidEmptyAndSourceAge) {
  auto p = Frame(); p.observation.objects.objects.clear(); p.motion_states.clear();
  p.observation.objects.status = p.observation.objects.OBSERVED_EMPTY;
  planning_inputs::DetectedObjectArray out; std::string reason;
  planning_inputs::LidarObservationGate gate;
  EXPECT_TRUE(gate.Convert(p, 10.1, &out, &reason)); EXPECT_TRUE(out.objects.empty());
  for (double now : {9., 11., double(NAN)}) {
    planning_inputs::LidarObservationGate fresh;
    EXPECT_FALSE(fresh.Convert(p, now, &out, &reason));
  }
}
TEST(LidarObservation, EpochsDoNotReusePrivateIdsOrAcceptRetiredPackets) {
  planning_inputs::LidarObservationGate gate; planning_inputs::DetectedObjectArray out; std::string reason;
  auto a = Frame(); ASSERT_TRUE(gate.Convert(a, 10., &out, &reason)); int first = out.objects[0].id;
  EXPECT_FALSE(gate.Convert(a, 10., &out, &reason));
  a.observation.objects.header.stamp.fromSec(10.1);
  EXPECT_FALSE(gate.Convert(a, 10.1, &out, &reason));  // same scan
  a.observation.scan_sequence = 2;
  ASSERT_TRUE(gate.Convert(a, 10.1, &out, &reason)); EXPECT_EQ(first, out.objects[0].id);
  a.observation.generation++; a.observation.scan_sequence = 1; a.observation.objects.header.stamp.fromSec(10.2);
  ASSERT_TRUE(gate.Convert(a, 10.2, &out, &reason)); EXPECT_NE(first, out.objects[0].id);
  int second = out.objects[0].id;
  a.observation.producer_session_id = "session-b"; a.observation.objects.header.stamp.fromSec(10.3);
  ASSERT_TRUE(gate.Convert(a, 10.3, &out, &reason)); EXPECT_NE(second, out.objects[0].id);
  a.observation.producer_session_id = "session-a"; a.observation.objects.header.stamp.fromSec(10.4);
  EXPECT_FALSE(gate.Convert(a, 10.4, &out, &reason)); EXPECT_EQ(reason, "retired_session");
}
}  // namespace
