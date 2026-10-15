// B0 legacy compatibility in common geometry (main_integration.launch): the legacy outputs
// /last_lidar, /lidar/tracked_objects_map, /lidar/motion_debug and the planner UDP sender all carry
// MakePlannerObjects() and are driven by ONE boolean, output_gate::PlanLegacy(...).publish
// (main.cpp: PlannerObjectRosPublisher::Publish and PlannerObjectSender::Send). The legacy wire has
// no motion field, so a warm-up track would look like a stationary object; the frame-wide
// LegacyMotionReady gate must therefore withhold the whole legacy frame.
// ROS-free: no master, no NodeHandle, no socket. Seam: frame_boundary::ConfigureForTest.
// Production main.launch (common geometry OFF) is NOT covered here: there LegacyMotionReady is not
// applied and warm-up objects still reach /last_lidar and UDP as before B0 (known B1 gap; nothing
// in this file asserts that behaviour).
#include <gtest/gtest.h>

#include <Geometry/FrameBoundary.hpp>
#include <Geometry/FrameBoundaryPublisher.hpp>
#include <Geometry/OutputGate.hpp>
#include <Lidar/Tracking/MotionClassifier.hpp>
#include <PlannerInterface/PlannerObjectAdapter.hpp>
#include <asmc_lidar/projection.hpp>

#include <Msg/TrackedObject.h>
#include <Msg/TrackedObjectArray.h>
#include <asmc_msgs/TrackedObject.h>
#include <asmc_msgs/TrackedObjectArray.h>

#include <cmath>
#include <iostream>
#include <limits>
#include <set>
#include <string>
#include <utility>
#include <vector>

extern VehicleSpec st_VehicleSpec;

namespace {
using output_gate::Mode;
constexpr std::int64_t kMs = 1000000LL;
constexpr std::int64_t kSec = 1000000000LL;
constexpr double kRoll = 0.02, kPitch = -0.01, kYaw = 0.3;
constexpr double kMpsToKph = 3.6;

void UseCommonGeometry() {
  frame_boundary::ConfigureForTest(true, true, "unit_test_calibration");
  st_VehicleSpec = VehicleSpec{};
  st_VehicleSpec.st_LiDARPos = {0.80f, 0.0f, 1.45f};
  st_VehicleSpec.st_LiDARRotationDeg = {0.f, 0.f, 0.f};
  st_VehicleSpec.st_GPSPos = {0.35f, 0.0f, 1.25f};
}
GPS_IMU Local() {
  GPS_IMU out{};
  out.st_IMU.b_ImuReady = true;
  out.st_IMU.f32_Roll_rad = float(kRoll); out.st_IMU.f32_Pitch_rad = float(kPitch);
  out.st_IMU.f32_Yaw_rad = float(kYaw);
  out.st_CAN.f32_CurrentVel_MPS = 0.f; out.st_CAN.f32_CurrentVel_KPH = 0.f;
  return out;
}
asmc::lidar::Observation Time(std::int64_t ns) { return {ns, ns, ns, "ros", "bridge_receive_ns", ""}; }
asmc::lidar::PoseSelection MapPose(std::int64_t ns, bool valid, const std::string& reason) {
  asmc::lidar::PoseSelection map;
  map.valid = valid; map.reason = reason; map.query_stamp_ns = ns; map.clock = "ros";
  map.map_from_base.linear() = frame_boundary::MapRotation(Local());
  map.map_from_base.translation() = asmc::lidar::Vec(100.0, 200.0, 30.0);
  return map;
}
// One detection (OBB in the leveled sensor frame) moving with (vx, 0) m/s in map E.
LiDARObjectInfo Detection(int id, double x, double y, double vx_mps, double map_e, double map_n) {
  LiDARObjectInfo out{};
  out.s32_Id = id; out.u16_Idx = 5; out.s32_Age = 4; out.s32_MissCount = 0;
  out.str_ObjectClass = "car"; out.str_YoloClass = "car";
  out.f32_OBB_CenterX = float(x); out.f32_OBB_CenterY = float(y); out.f32_OBB_CenterZ = -0.2f;
  out.f32_CoreX = float(x); out.f32_CoreY = float(y); out.f32_CoreZ = -0.2f;
  out.f32_OBB_Width = 4.f; out.f32_OBB_Height = 1.8f; out.f32_OBB_ZMin = -0.7f; out.f32_OBB_ZMax = 0.3f;
  out.f32_OBB_YawRad = 0.f;
  for (unsigned i = 0; i < 8; ++i) {
    const double u = ((i & 1) ? 2.0 : -2.0), v = ((i & 2) ? 0.9 : -0.9);
    out.OBB_Corners_Body[i] = {float(x + u), float(y + v), (i & 4) ? 0.3f : -0.7f};
    pcl::PointXYZI p; p.x = out.OBB_Corners_Body[i].x; p.y = out.OBB_Corners_Body[i].y;
    p.z = out.OBB_Corners_Body[i].z; p.intensity = 1.f; out.pcl_ClusterPoints.push_back(p);
  }
  out.f32_AbsVx = float(vx_mps); out.f32_AbsVy = 0.f; out.f32_Velocity = float(std::abs(vx_mps));
  out.f32_CoreE = float(map_e); out.f32_CoreN = float(map_n);
  out.motion_valid = false;
  return out;
}
// Scan context exactly as main.cpp/LidarProcess build it: PrepareScan, one StoreLeveling, copy.
asmc::lidar::Context Prepare(std::int64_t ns, bool map_valid, std::uint64_t generation) {
  frame_boundary::PrepareScan(Time(ns), Local(), st_VehicleSpec,
                              MapPose(ns, map_valid, map_valid ? "gps_observed" : "gps_invalid"), generation);
  frame_boundary::StoreLeveling(asmc::lidar::leveling_rotation(
      kRoll, kPitch, frame_boundary::SensorMount(st_VehicleSpec).rotation(), true, true));
  return frame_boundary::Current();
}
bool Publishable(const LiDARObjectInfo& o) {     // FrameBoundary.cpp LegacyMotionReady / V2 predicate
  return o.s32_Id >= 0 && o.s32_MissCount == 0 && !o.pcl_ClusterPoints.empty() &&
         o.str_ObjectClass != "noise" && o.str_ObjectClass != "지면노이즈";
}
struct LegacyFrame {
  output_gate::Decision decision;
  output_gate::LegacyPlan plan;
  bool any_warmup = false;
};
// main.cpp B0 gate order: GateInputs -> Decide -> (PublishV2: BuildV2, Downgrade on rejection)
// -> LegacyMotionReady -> PlanLegacy. plan.publish drives Publish(/last_lidar,...) AND Send(UDP).
LegacyFrame Gate(const LiDAR& lidar, std::uint64_t generation) {
  LegacyFrame out;
  const auto in = frame_boundary::GateInputs(true, "local_ready", true, lidar, generation);
  out.decision = output_gate::Decide(in);
  if (out.decision.v2 != Mode::kNone) {
    const auto frame = frame_boundary::BuildV2(lidar, out.decision.v2);
    if (!frame.valid) out.decision = output_gate::Downgrade(out.decision, frame.reason);
  }
  out.plan = output_gate::PlanLegacy(out.decision, in, frame_boundary::LegacyMotionReady(lidar));
  for (const auto& o : lidar.vec_Objects) out.any_warmup |= Publishable(o) && !o.motion_valid;
  return out;
}
}  // namespace

// LegacyMotionReady is frame-wide: one publishable warm-up object blocks the frame; warm-up objects
// that are never published (missed, noise, ground noise, negative id, no points) do not.
TEST(LegacyCompat, LegacyMotionReadyIsFrameWide) {
  UseCommonGeometry();
  LiDAR lidar;
  EXPECT_TRUE(frame_boundary::LegacyMotionReady(lidar));            // empty frame
  for (int i = 0; i < 5; ++i) {
    auto o = Detection(i + 1, 10.0 + 3 * i, 2.0, 0.0, 0.0, 0.0);
    o.motion_valid = true; o.s32_MotionState = static_cast<int>(MotionState::LowThreat);
    lidar.vec_Objects.push_back(o);
  }
  EXPECT_TRUE(frame_boundary::LegacyMotionReady(lidar));
  auto warmup = Detection(9, 30.0, -2.0, 4.0, 0.0, 0.0);              // motion_valid=false
  lidar.vec_Objects.push_back(warmup);
  EXPECT_FALSE(frame_boundary::LegacyMotionReady(lidar)) << "one warm-up object must block the frame";
  lidar.vec_Objects.pop_back();
  std::vector<LiDARObjectInfo> ignored;
  { auto o = warmup; o.s32_MissCount = 1; ignored.push_back(o); }
  { auto o = warmup; o.str_ObjectClass = "noise"; ignored.push_back(o); }
  { auto o = warmup; o.str_ObjectClass = "지면노이즈"; ignored.push_back(o); }
  { auto o = warmup; o.s32_Id = -1; ignored.push_back(o); }
  { auto o = warmup; o.pcl_ClusterPoints.clear(); ignored.push_back(o); }
  for (const auto& o : ignored) {
    lidar.vec_Objects.push_back(o);
    EXPECT_TRUE(frame_boundary::LegacyMotionReady(lidar)) << "unpublished warm-up object must not block";
    lidar.vec_Objects.pop_back();
  }
}

// Hazard premise: during warm-up (the first motion_warmup_observations=3 observations) the real
// MotionClassifier leaves a track at MotionState::Unknown with planner velocity 0 and
// b_IsDynamic=false. The legacy conversion (MakePlannerObjects -> TrackedObject / UDP slot) then
// carries a 15 km/h object as speed 0, is_dynamic=false: indistinguishable from a stationary object.
// V2 keeps it as MOTION_UNKNOWN; the legacy frame must be withheld.
TEST(LegacyCompat, WarmupTrackWouldLeaveAsStationaryOnTheLegacyWire) {
  UseCommonGeometry();
  MotionClassifier classifier;
  const double vx = 15.0 / kMpsToKph;
  for (int scan = 0; scan < 3; ++scan) {
    const std::int64_t ns = 50 * kSec + scan * 100 * kMs;
    std::vector<LiDARObjectInfo> objects{Detection(7, 12.0 + vx * 0.1 * scan, -2.0, vx, 100.0 + vx * 0.1 * scan, 200.0)};
    classifier.Update(objects, ros::Time().fromNSec(std::uint64_t(ns)));
    LiDAR lidar;
    lidar.frame_context = Prepare(ns, true, std::uint64_t(scan + 1));
    lidar.vec_Objects = objects;
    const auto& o = lidar.vec_Objects.front();
    SCOPED_TRACE("warm-up observation " + std::to_string(scan + 1));
    EXPECT_FALSE(o.motion_valid);
    EXPECT_EQ(o.s32_MotionState, static_cast<int>(MotionState::Unknown));
    EXPECT_FALSE(o.b_IsDynamic);
    EXPECT_EQ(o.f32_PlannerVxKPH, 0.f);
    const auto planner = MakePlannerObjects(lidar, Local(), st_VehicleSpec, 20);
    ASSERT_EQ(planner.size(), 1U);
    EXPECT_EQ(planner[0].speed_kph, 0.0);                    // TrackedObject.speed_kph, UDP km/h slots
    EXPECT_EQ(planner[0].velocity_map_mps.x, 0.0);
    EXPECT_EQ(planner[0].velocity_map_mps.y, 0.0);
    EXPECT_FALSE(planner[0].is_dynamic);
    EXPECT_TRUE(std::isfinite(planner[0].obb_center_map.x)); // a real, placeable object
    asmc_msgs::LidarObjectV2 v2;
    frame_boundary::FillMotionV2(o, &v2);
    EXPECT_FALSE(v2.motion_valid);
    EXPECT_EQ(int(v2.motion_state), int(asmc_msgs::LidarObjectV2::MOTION_UNKNOWN));
    const auto frame = Gate(lidar, std::uint64_t(scan + 1));
    EXPECT_EQ(frame.decision.v2, Mode::kMap);                // V2 carries it, marked unknown
    EXPECT_FALSE(frame.plan.publish);                         // no /last_lidar, no tracked_objects_map, no UDP
    EXPECT_TRUE(frame.plan.suppress_markers);
    EXPECT_EQ(frame.plan.reason, "motion_state_unrepresentable");
  }
}

// Scan sequence with the real MotionClassifier exactly as LidarProcess drives it in common geometry
// (Update while the map pose is valid, InvalidateMapHistory while it is not): A is stationary from
// scan 0, B (15 km/h) appears at scan 25, GPS is denied at scans 40-44. For every scan the legacy
// frame is published iff map_v2 is published and no publishable object is warming up; a published
// legacy frame never contains a warm-up object, and a withheld frame withholds every object.
TEST(LegacyCompat, CommonModeWithholdsTheWholeLegacyFrameWhileAnyObjectWarmsUp) {
  UseCommonGeometry();
  MotionClassifier classifier;
  const double vb = 15.0 / kMpsToKph;
  std::string pattern;
  int published = 0, frame_wide_withheld = 0, warmup_withheld = 0;
  for (int scan = 0; scan < 60; ++scan) {
    SCOPED_TRACE("scan " + std::to_string(scan));
    const std::int64_t ns = 60 * kSec + scan * 100 * kMs;
    const bool map_valid = scan < 40 || scan >= 45;
    std::vector<LiDARObjectInfo> objects{Detection(1, 20.0, 3.4, 0.0, 120.0, 203.4)};
    if (scan >= 25) {
      const double t = (scan - 25) * 0.1;
      objects.push_back(Detection(2, 10.0 + vb * t, 0.0, vb, 110.0 + vb * t, 200.0));
    }
    LiDAR lidar;
    lidar.frame_context = Prepare(ns, map_valid, std::uint64_t(scan + 1));
    if (!lidar.frame_context.map_pose.valid) classifier.InvalidateMapHistory(objects);   // LidarProcess.cpp
    else classifier.Update(objects, ros::Time().fromNSec(std::uint64_t(ns)));
    lidar.vec_Objects = objects;
    const auto frame = Gate(lidar, std::uint64_t(scan + 1));
    const bool expect_publish = map_valid && !frame.any_warmup;
    EXPECT_EQ(frame.decision.v2, map_valid ? Mode::kMap : Mode::kLocal);
    EXPECT_EQ(frame.plan.publish, expect_publish);
    EXPECT_EQ(frame.plan.suppress_markers, !frame.plan.publish);    // DELETEALL whenever withheld
    if (frame.plan.publish) {
      ++published;
      // What Publish()/Send() would put on the wire: no warm-up track among them.
      std::set<int> ready;
      for (const auto& o : lidar.vec_Objects) if (o.motion_valid) ready.insert(o.s32_Id);
      for (const auto& p : MakePlannerObjects(lidar, Local(), st_VehicleSpec, 20))
        EXPECT_TRUE(ready.count(p.id)) << "warm-up track " << p.id << " on the legacy wire";
    } else if (frame.any_warmup && map_valid) {
      ++warmup_withheld;
      bool some_ready = false;
      for (const auto& o : lidar.vec_Objects) some_ready |= Publishable(o) && o.motion_valid;
      frame_wide_withheld += some_ready;      // a motion-ready object is withheld with the frame
      EXPECT_EQ(frame.plan.reason, "motion_state_unrepresentable");
    } else if (!map_valid) {
      EXPECT_EQ(frame.plan.reason, "pose_unavailable");
    }
    pattern += frame.plan.publish ? 'P' : (map_valid ? 'w' : 'd');
  }
  ::testing::Test::RecordProperty("legacy_pattern", pattern);
  std::cout << "legacy pattern (P published, w warm-up withheld, d GPS denied): " << pattern << std::endl;
  EXPECT_GT(published, 0) << pattern;
  EXPECT_GT(frame_wide_withheld, 0) << "B warming up next to a ready A must withhold A too: " << pattern;
  EXPECT_GE(warmup_withheld, 3) << pattern;
  EXPECT_EQ(pattern.substr(0, 3), "www");                // startup warm-up
  EXPECT_EQ(pattern.substr(25, 3), "www");               // B appears: whole frame withheld
  EXPECT_EQ(pattern.substr(40, 5), "ddddd");             // GPS denial: never legacy
  EXPECT_EQ(pattern.substr(45, 3), "www");               // tracks reset by the denial: warm-up again
}

// Without map_v2 there is no legacy frame at all, even when LegacyMotionReady holds: an empty scene
// (vacuously motion-ready) or motion-ready objects during GPS denial/recovery, a non-GPS map
// invalidity or a rejected object geometry never reach /last_lidar, /lidar/tracked_objects_map or UDP.
TEST(LegacyCompat, LegacyIsNeverPublishedWithoutMapV2EvenWhenMotionReady) {
  UseCommonGeometry();
  auto ready = Detection(3, 15.0, 1.0, 0.0, 115.0, 201.0);
  ready.motion_valid = true; ready.s32_MotionState = static_cast<int>(MotionState::LowThreat);
  auto broken = ready; broken.f32_OBB_Width = std::numeric_limits<float>::quiet_NaN();
  struct Case { const char* name; bool map_valid; std::string reason; std::vector<LiDARObjectInfo> objects; Mode v2; };
  const std::vector<Case> cases{
      {"empty_scene_gps_invalid", false, "gps_invalid", {}, Mode::kLocal},
      {"ready_objects_gps_stale", false, "gps_stale", {ready}, Mode::kLocal},
      {"ready_objects_gps_recovering", false, "gps_recovering", {ready}, Mode::kLocal},
      {"empty_scene_gps_missing", false, "gps_missing", {}, Mode::kLocal},
      {"ready_objects_can_speed_invalid", false, "map_speed_invalid", {ready}, Mode::kNone},
      {"empty_scene_can_speed_invalid", false, "map_speed_invalid", {}, Mode::kNone},
      {"invalid_object_geometry_with_map", true, "gps_observed", {ready, broken}, Mode::kNone},
  };
  std::uint64_t generation = 100;
  for (const auto& c : cases) {
    SCOPED_TRACE(c.name);
    const std::int64_t ns = 70 * kSec + std::int64_t(generation) * 100 * kMs;
    ++generation;
    frame_boundary::PrepareScan(Time(ns), Local(), st_VehicleSpec, MapPose(ns, c.map_valid, c.reason), generation);
    frame_boundary::StoreLeveling(asmc::lidar::leveling_rotation(
        kRoll, kPitch, frame_boundary::SensorMount(st_VehicleSpec).rotation(), true, true));
    LiDAR lidar;
    lidar.frame_context = frame_boundary::Current();
    lidar.vec_Objects = c.objects;
    EXPECT_TRUE(frame_boundary::LegacyMotionReady(lidar));   // the frame-wide motion gate alone would pass
    const auto frame = Gate(lidar, generation);
    EXPECT_EQ(frame.decision.v2, c.v2);
    EXPECT_FALSE(frame.plan.publish);              // no Publish(), no Send()
    EXPECT_TRUE(frame.plan.suppress_markers);      // legacy map markers DELETEALL instead
  }
  // Control: the same motion-ready frame with a valid map pose is published.
  const std::int64_t ns = 80 * kSec;
  frame_boundary::PrepareScan(Time(ns), Local(), st_VehicleSpec, MapPose(ns, true, "gps_observed"), ++generation);
  frame_boundary::StoreLeveling(asmc::lidar::leveling_rotation(
      kRoll, kPitch, frame_boundary::SensorMount(st_VehicleSpec).rotation(), true, true));
  LiDAR lidar;
  lidar.frame_context = frame_boundary::Current();
  lidar.vec_Objects = {ready};
  EXPECT_TRUE(Gate(lidar, generation).plan.publish);
}

// planning_inputs bridges /last_lidar to asmc_msgs by MD5 identity (tracked_wire_bridge.py); B0 must
// keep the legacy wire types byte-identical, and that wire has no motion state at all.
TEST(LegacyCompat, PlanningBridgeMessageMd5IdentityAndNoMotionFieldOnTheLegacyWire) {
  EXPECT_STREQ(ros::message_traits::MD5Sum<Msg::TrackedObjectArray>::value(), "e127b2aabd7494ad01a5a56b005bf07e");
  EXPECT_STREQ(ros::message_traits::MD5Sum<asmc_msgs::TrackedObjectArray>::value(), "e127b2aabd7494ad01a5a56b005bf07e");
  EXPECT_STREQ(ros::message_traits::MD5Sum<Msg::TrackedObject>::value(), "6c96135eb03a3ee93bba474fa835f57d");
  EXPECT_STREQ(ros::message_traits::MD5Sum<asmc_msgs::TrackedObject>::value(), "6c96135eb03a3ee93bba474fa835f57d");
  const std::string definition = ros::message_traits::Definition<Msg::TrackedObject>::value();
  // Comments may describe motion semantics without changing the frozen wire.
  // Assert the absence of the actual v2 fields instead of banning that word.
  EXPECT_EQ(definition.find("bool motion_valid"), std::string::npos);
  EXPECT_EQ(definition.find("uint8 motion_state"), std::string::npos);
  EXPECT_NE(definition.find("bool is_dynamic"), std::string::npos);
  EXPECT_NE(definition.find("float64 speed_kph"), std::string::npos);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
