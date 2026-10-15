// LiDAR V2 builder, motion mapping and gate-input tests (Geometry/FrameBoundaryPublisher.hpp).
// ROS-free: no ros::init, no master, no NodeHandle, no ros::Time::now().
// Only test seam used: frame_boundary::ConfigureForTest(enabled,use_roll,calibration_id).
#include <gtest/gtest.h>

#include <Geometry/FrameBoundaryPublisher.hpp>
#include <Geometry/OutputGate.hpp>
#include <Lidar/Tracking/MotionClassifier.hpp>
#include <asmc_lidar/projection.hpp>

#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

extern VehicleSpec st_VehicleSpec;

namespace {
using asmc::lidar::Rot;
using asmc::lidar::Vec;
using ObjectV2 = asmc_msgs::LidarObjectV2;
using ArrayV2 = asmc_msgs::LidarObjectArrayV2;
using output_gate::Mode;

constexpr std::int64_t kSec = 1000000000LL;
constexpr double kRoll = 0.04, kPitch = -0.03, kYaw = 0.7;

void UseCommonGeometry() {
  frame_boundary::ConfigureForTest(true, true, "test_calibration");
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
  out.st_CAN.f32_CurrentVel_MPS = 2.f; out.st_CAN.f32_CurrentVel_KPH = 7.2f;
  return out;
}
asmc::lidar::Observation Time(std::int64_t ns) { return {ns, ns, ns, "ros", "bridge_receive_ns", ""}; }
asmc::lidar::PoseSelection MapPose(std::int64_t ns, bool valid, const std::string& reason) {
  asmc::lidar::PoseSelection map;
  map.valid = valid; map.reason = reason; map.query_stamp_ns = ns; map.clock = "ros";
  map.map_from_base.linear() = frame_boundary::MapRotation(Local());
  map.map_from_base.translation() = Vec(100.0, 200.0, 30.0);
  return map;
}
// One scan exactly as main.cpp/LidarProcess run it: PrepareScan, one StoreLeveling (roll+pitch),
// then LidarProcess copies Current() into the scan's frame_context.
LiDAR Scan(std::int64_t ns, const asmc::lidar::PoseSelection& map, std::vector<LiDARObjectInfo> objects,
           std::uint64_t generation = 1, bool store_leveling = true) {
  frame_boundary::PrepareScan(Time(ns), Local(), st_VehicleSpec, map, generation);
  if (store_leveling) {
    frame_boundary::StoreLeveling(asmc::lidar::leveling_rotation(
        kRoll, kPitch, frame_boundary::SensorMount(st_VehicleSpec).rotation()));
  }
  LiDAR lidar;
  lidar.frame_context = frame_boundary::Current();
  lidar.vec_Objects = std::move(objects);
  return lidar;
}
LiDARObjectInfo Detection(int id, double x, double y, double yaw = 0.3) {
  LiDARObjectInfo out{};
  out.s32_Id = id; out.u16_Idx = 5; out.s32_Age = 4; out.s32_MissCount = 0;
  out.str_ObjectClass = "car"; out.str_YoloClass = "truck";
  out.f32_OBB_CenterX = float(x); out.f32_OBB_CenterY = float(y); out.f32_OBB_CenterZ = -0.2f;
  out.f32_OBB_Width = 4.f; out.f32_OBB_Height = 1.8f; out.f32_OBB_ZMin = -0.7f; out.f32_OBB_ZMax = 0.3f;
  out.f32_OBB_YawRad = float(yaw);
  const double c = std::cos(yaw), s = std::sin(yaw);
  for (unsigned i = 0; i < 8; ++i) {
    const double u = ((i & 1) ? 2.0 : -2.0), v = ((i & 2) ? 0.9 : -0.9);
    out.OBB_Corners_Body[i] = {float(x + u * c - v * s), float(y + u * s + v * c), (i & 4) ? 0.3f : -0.7f};
    pcl::PointXYZI p; p.x = out.OBB_Corners_Body[i].x; p.y = out.OBB_Corners_Body[i].y;
    p.z = out.OBB_Corners_Body[i].z; p.intensity = 1.f; out.pcl_ClusterPoints.push_back(p);
  }
  out.motion_valid = true; out.s32_MotionState = static_cast<int>(MotionState::LowThreat);
  return out;
}
asmc::lidar::Result Expected(const LiDARObjectInfo& o, const asmc::lidar::Context& context) {
  asmc::lidar::Object object; object.id = o.s32_Id; object.classification = o.str_ObjectClass;
  object.center = Vec(o.f32_OBB_CenterX, o.f32_OBB_CenterY, o.f32_OBB_CenterZ);
  object.long_axis = Vec(std::cos(o.f32_OBB_YawRad), std::sin(o.f32_OBB_YawRad), 0);
  for (unsigned i = 0; i < 8; ++i)
    object.corners[i] = Vec(o.OBB_Corners_Body[i].x, o.OBB_Corners_Body[i].y, o.OBB_Corners_Body[i].z);
  return asmc::lidar::transform(object, context);
}
Vec P(const geometry_msgs::Point& p) { return Vec(p.x, p.y, p.z); }
ObjectV2 Fill(bool motion_valid, int state, bool merged_flag = true) {
  LiDARObjectInfo in{};
  in.motion_valid = motion_valid; in.s32_MotionState = state;
  in.b_IsDynamic = merged_flag;   // legacy PENDING|DYNAMIC merge: must never leak into V2
  ObjectV2 out;
  frame_boundary::FillMotionV2(in, &out);
  return out;
}
}  // namespace

TEST(LidarObjectV2Builder, FillMotionMapsFourValidStatesOneToOne) {
  struct Row { int state; bool valid; int out; bool dynamic; };
  const Row rows[] = {
      {static_cast<int>(MotionState::Unknown), false, int(ObjectV2::MOTION_UNKNOWN), false},
      {static_cast<int>(MotionState::LowThreat), true, int(ObjectV2::MOTION_LOW_THREAT), false},
      {static_cast<int>(MotionState::DynamicPending), true, int(ObjectV2::MOTION_DYNAMIC_PENDING), false},
      {static_cast<int>(MotionState::Dynamic), true, int(ObjectV2::MOTION_DYNAMIC), true}};
  std::vector<int> seen;
  for (const auto& row : rows) for (bool merged : {false, true}) {
    const auto out = Fill(true, row.state, merged);
    EXPECT_EQ(out.motion_valid, row.valid) << row.state;
    EXPECT_EQ(int(out.motion_state), row.out) << row.state;
    EXPECT_EQ(out.dynamic_valid, out.motion_valid) << row.state;
    EXPECT_EQ(out.is_dynamic, row.dynamic) << row.state << " merged=" << merged;
    if (merged) seen.push_back(int(out.motion_state));
  }
  // Four producer states stay four distinct wire values: nothing merged.
  EXPECT_EQ(seen, std::vector<int>({0, 1, 2, 3}));
}

TEST(LidarObjectV2Builder, FillMotionInvalidOrOutOfRangeIsUnknownInvalid) {
  const std::pair<bool, int> rows[] = {{false, 0}, {false, 1}, {false, 2}, {false, 3},
                                       {true, 0}, {true, 4}, {true, -1}, {false, -7}, {true, 255}};
  for (const auto& row : rows) {
    const auto out = Fill(row.first, row.second);
    EXPECT_FALSE(out.motion_valid) << row.first << "," << row.second;
    EXPECT_EQ(int(out.motion_state), int(ObjectV2::MOTION_UNKNOWN)) << row.first << "," << row.second;
    EXPECT_FALSE(out.dynamic_valid);
    EXPECT_FALSE(out.is_dynamic);
  }
}

TEST(LidarObjectV2Builder, LocalModeUsesBaseGeometryAndCopiesEveryV1Field) {
  UseCommonGeometry();
  auto object = Detection(7, 12., -3.);
  object.s32_MotionState = static_cast<int>(MotionState::DynamicPending);
  const auto lidar = Scan(10 * kSec, MapPose(10 * kSec, true, "gps_observed"), {object});
  const auto frame = frame_boundary::BuildV2(lidar, Mode::kLocal);
  ASSERT_TRUE(frame.valid) << frame.reason;
  EXPECT_EQ(frame.array.header.frame_id, "base_link");
  EXPECT_EQ(frame.array.header.stamp.toNSec(), static_cast<std::uint64_t>(10 * kSec));
  EXPECT_EQ(int(frame.array.status), int(ArrayV2::OBSERVED));
  EXPECT_EQ(frame.array.reason, "observed");
  EXPECT_EQ(frame.array.stamp_provenance, "bridge_receive_ns");
  ASSERT_EQ(frame.array.objects.size(), 1U);
  const auto& o = frame.array.objects[0];
  const auto expected = Expected(object, lidar.frame_context);
  ASSERT_TRUE(expected.valid);
  EXPECT_LT((P(o.center) - expected.base.center).norm(), 1e-9);      // base geometry, not map
  for (unsigned i = 0; i < 8; ++i) EXPECT_LT((P(o.obb_corners[i]) - expected.base.corners[i]).norm(), 1e-9);
  EXPECT_NEAR(o.yaw, *expected.base.yaw, 1e-12);
  EXPECT_EQ(o.track_id, 7); EXPECT_EQ(o.detection_index, 5U);
  EXPECT_EQ(int(o.observation_state), int(ObjectV2::OBSERVED)); EXPECT_EQ(o.age, 4U); EXPECT_EQ(o.miss_count, 0U);
  EXPECT_EQ(o.classification, "car"); EXPECT_EQ(o.camera_class, "truck");
  EXPECT_FALSE(o.yaw_rate_valid); EXPECT_EQ(o.yaw_rate_rad_s, 0.);
  EXPECT_DOUBLE_EQ(o.length, 4.); EXPECT_FLOAT_EQ(float(o.width), 1.8f); EXPECT_FLOAT_EQ(float(o.height), 1.f);
  EXPECT_FALSE(o.relative_velocity_valid); EXPECT_FALSE(o.absolute_velocity_valid);   // never a valid zero
  EXPECT_EQ(o.source_point_count, 8U);
  EXPECT_TRUE(o.motion_valid); EXPECT_EQ(int(o.motion_state), int(ObjectV2::MOTION_DYNAMIC_PENDING));
  EXPECT_TRUE(o.dynamic_valid); EXPECT_FALSE(o.is_dynamic);                 // PENDING is not DYNAMIC
}

TEST(LidarObjectV2Builder, MapModeEqualsMapPoseTimesLocalFromTheSameScan) {
  UseCommonGeometry();
  const auto lidar = Scan(11 * kSec, MapPose(11 * kSec, true, "gps_bounded_prediction"),
                          {Detection(1, 15., 2., .2), Detection(2, 25., -4., -1.1)});
  const auto local = frame_boundary::BuildV2(lidar, Mode::kLocal);
  const auto map = frame_boundary::BuildV2(lidar, Mode::kMap);
  ASSERT_TRUE(local.valid && map.valid);
  EXPECT_EQ(map.array.header.frame_id, "map");
  EXPECT_EQ(int(map.array.status), int(ArrayV2::OBSERVED));
  EXPECT_EQ(map.array.reason, "gps_bounded_prediction");
  ASSERT_EQ(map.array.objects.size(), 2U); ASSERT_EQ(local.array.objects.size(), 2U);
  const auto& map_from_base = lidar.frame_context.map_pose.map_from_base;
  for (unsigned k = 0; k < 2; ++k) {
    const auto& m = map.array.objects[k]; const auto& l = local.array.objects[k];
    EXPECT_LT((P(m.center) - map_from_base * P(l.center)).norm(), 1e-9);
    for (unsigned i = 0; i < 8; ++i)
      EXPECT_LT((P(m.obb_corners[i]) - map_from_base * P(l.obb_corners[i])).norm(), 1e-9);
    const Vec edge = P(m.obb_corners[1]) - P(m.obb_corners[0]);   // long edge in map frame
    EXPECT_LT(std::fabs(std::remainder(m.yaw - std::atan2(edge.y(), edge.x()), M_PI)), 1e-5);
    EXPECT_EQ(m.track_id, l.track_id); EXPECT_EQ(m.motion_state, l.motion_state);
  }
  // Roll and pitch: the base geometry undoes exactly the one stored leveling rotation.
  const auto base = asmc::lidar::base_from_input(lidar.frame_context);
  ASSERT_TRUE(base.has_value());
  const Vec raw_center(15., 2., -.2);
  EXPECT_LT((P(local.array.objects[0].center) - (*base) * raw_center).norm(), 1e-6);
}

TEST(LidarObjectV2Builder, InvalidObjectGeometryRejectsWholeFrame) {
  UseCommonGeometry();
  const auto map = MapPose(12 * kSec, true, "gps_observed");
  struct Case { const char* name; void (*mutate)(LiDARObjectInfo&); const char* reason; };
  const Case cases[] = {
      {"nan_width", [](LiDARObjectInfo& o) { o.f32_OBB_Width = NAN; }, "invalid_object_dimensions"},
      {"negative_width", [](LiDARObjectInfo& o) { o.f32_OBB_Height = -1.f; }, "invalid_object_dimensions"},
      {"inverted_z", [](LiDARObjectInfo& o) { o.f32_OBB_ZMax = o.f32_OBB_ZMin - 1.f; }, "invalid_object_dimensions"},
      {"nonfinite_corner", [](LiDARObjectInfo& o) { o.OBB_Corners_Body[3].x = INFINITY; }, "nonfinite_or_invalid_geometry"},
      {"nan_center", [](LiDARObjectInfo& o) { o.f32_OBB_CenterY = NAN; }, "nonfinite_or_invalid_geometry"},
  };
  std::int64_t t = 12 * kSec;
  for (const auto& c : cases) for (Mode mode : {Mode::kLocal, Mode::kMap}) {
    t += kSec / 10;
    auto bad = Detection(2, 14., 1.); c.mutate(bad);
    auto scan_map = map; scan_map.query_stamp_ns = t;
    const auto lidar = Scan(t, scan_map, {Detection(1, 10., 0.), bad});
    const auto frame = frame_boundary::BuildV2(lidar, mode);
    EXPECT_FALSE(frame.valid) << c.name;
    EXPECT_EQ(frame.reason, c.reason) << c.name;
    EXPECT_TRUE(frame.array.objects.empty()) << c.name;   // no partial frame
  }
}

TEST(LidarObjectV2Builder, UnpublishableObjectsAreSkippedNeverZeroFilled) {
  UseCommonGeometry();
  auto unassigned = Detection(-1, 9., 0.);
  auto missed = Detection(3, 9., 2.); missed.s32_MissCount = 1;
  auto no_points = Detection(4, 9., 4.); no_points.pcl_ClusterPoints.clear();
  auto noise = Detection(5, 9., 6.); noise.str_ObjectClass = "noise";
  auto ground = Detection(6, 9., 8.); ground.str_ObjectClass = "지면노이즈";
  const auto real = Detection(8, 20., -1.);
  const auto map = MapPose(13 * kSec, true, "gps_observed");
  const auto lidar = Scan(13 * kSec, map, {unassigned, missed, no_points, noise, ground, real});
  for (Mode mode : {Mode::kLocal, Mode::kMap}) {
    const auto frame = frame_boundary::BuildV2(lidar, mode);
    ASSERT_TRUE(frame.valid);
    ASSERT_EQ(frame.array.objects.size(), 1U);
    EXPECT_EQ(frame.array.objects[0].track_id, 8);
    EXPECT_GT(P(frame.array.objects[0].center).norm(), 1.);   // a real object, not a zero dummy
  }
  // Only unpublishable objects: an empty observation, never a zero-coordinate dummy object.
  const auto empty = Scan(13 * kSec + kSec / 10, MapPose(13 * kSec + kSec / 10, true, "gps_observed"),
                          {unassigned, missed, no_points, noise, ground});
  for (Mode mode : {Mode::kLocal, Mode::kMap}) {
    const auto frame = frame_boundary::BuildV2(empty, mode);
    ASSERT_TRUE(frame.valid);
    EXPECT_EQ(int(frame.array.status), int(ArrayV2::OBSERVED_EMPTY));
    EXPECT_EQ(frame.array.reason, "observed_empty");
    EXPECT_TRUE(frame.array.objects.empty());
  }
}

TEST(LidarObjectV2Builder, EmptySceneIsObservedEmptyOnlyWithValidInputs) {
  UseCommonGeometry();
  const std::int64_t t = 14 * kSec;
  EXPECT_TRUE(frame_boundary::BuildV2(Scan(t, MapPose(t, true, "gps_observed"), {}), Mode::kMap).valid);
  EXPECT_TRUE(frame_boundary::BuildV2(Scan(t + 1, MapPose(t + 1, false, "gps_stale"), {}), Mode::kLocal).valid);
  // Unleveled scan (no StoreLeveling), map pose for another stamp, invalid map pose, mode none.
  const auto unleveled = frame_boundary::BuildV2(Scan(t + 2, MapPose(t + 2, true, "gps_observed"), {}, 1, false), Mode::kLocal);
  EXPECT_FALSE(unleveled.valid); EXPECT_EQ(unleveled.reason, "leveling_transform_invalid");
  const auto mismatch = frame_boundary::BuildV2(Scan(t + 3, MapPose(t + 2, true, "gps_observed"), {}), Mode::kMap);
  EXPECT_FALSE(mismatch.valid); EXPECT_EQ(mismatch.reason, "map_pose_time_mismatch");
  const auto denied = frame_boundary::BuildV2(Scan(t + 4, MapPose(t + 4, false, "gps_invalid"), {}), Mode::kMap);
  EXPECT_FALSE(denied.valid); EXPECT_EQ(denied.reason, "map_pose_invalid:gps_invalid");
  auto bad_time = Scan(t + 5, MapPose(t + 5, true, "gps_observed"), {});
  bad_time.frame_context.time.processing_ns = bad_time.frame_context.time.receipt_ns - 1;
  const auto time = frame_boundary::BuildV2(bad_time, Mode::kLocal);
  EXPECT_FALSE(time.valid); EXPECT_EQ(time.reason, "time_invalid:processing_before_receipt");
  const auto none = frame_boundary::BuildV2(Scan(t + 6, MapPose(t + 6, true, "gps_observed"), {Detection(1, 9., 0.)}), Mode::kNone);
  EXPECT_FALSE(none.valid); EXPECT_TRUE(none.array.objects.empty());
}

TEST(LidarObjectV2Builder, WarmupObjectIsCarriedAsUnknownWithoutVelocity) {
  UseCommonGeometry();
  auto ready = Detection(1, 10., 0.);
  auto warm = Detection(2, 14., 0.); warm.motion_valid = false; warm.s32_MotionState = 0;
  auto stale = Detection(3, 18., 0.); stale.motion_valid = false;
  stale.s32_MotionState = static_cast<int>(MotionState::Dynamic);   // non-finite sample branch
  const auto lidar = Scan(15 * kSec, MapPose(15 * kSec, true, "gps_observed"), {ready, warm, stale});
  EXPECT_FALSE(frame_boundary::LegacyMotionReady(lidar));   // legacy frame withheld as a whole
  const auto frame = frame_boundary::BuildV2(lidar, Mode::kMap);
  ASSERT_TRUE(frame.valid); ASSERT_EQ(frame.array.objects.size(), 3U);
  EXPECT_TRUE(frame.array.objects[0].motion_valid);
  for (unsigned i : {1U, 2U}) {
    EXPECT_FALSE(frame.array.objects[i].motion_valid);
    EXPECT_EQ(int(frame.array.objects[i].motion_state), int(ObjectV2::MOTION_UNKNOWN));
    EXPECT_FALSE(frame.array.objects[i].is_dynamic);
    EXPECT_FALSE(frame.array.objects[i].absolute_velocity_valid);
    EXPECT_FALSE(frame.array.objects[i].relative_velocity_valid);
  }
}

TEST(LidarObjectV2Builder, GateInputsUseOneContextOfThisScan) {
  UseCommonGeometry();
  const auto a = Scan(16 * kSec, MapPose(16 * kSec, true, "gps_observed"), {Detection(1, 10., 0.)}, 41);
  // A later scan prepared but rejected by LidarProcess: Current() moves on, a.frame_context does not.
  frame_boundary::PrepareScan(Time(16 * kSec + kSec / 10), Local(), st_VehicleSpec,
                              MapPose(16 * kSec + kSec / 10, false, "gps_stale"), 42);
  const auto processed = frame_boundary::GateInputs(true, "local_ready", true, a, 41);
  EXPECT_TRUE(processed.common_geometry);
  EXPECT_TRUE(processed.map_pose_valid); EXPECT_EQ(processed.map_reason, "gps_observed");
  EXPECT_TRUE(processed.map_pose_matches_scan);
  EXPECT_TRUE(processed.context_current); EXPECT_TRUE(processed.leveling_valid);
  EXPECT_TRUE(processed.time_error.empty());
  EXPECT_EQ(output_gate::Decide(processed).v2, Mode::kMap);
  // Same frame_context but a newer generation: the copy is stale -> nothing.
  const auto stale = frame_boundary::GateInputs(true, "local_ready", true, a, 42);
  EXPECT_FALSE(stale.context_current);
  EXPECT_EQ(output_gate::Decide(stale).reason, "context_stale");
  // Unprocessed scan: map/time from Current() of THIS scan, never generation/leveling.
  const auto rejected = frame_boundary::GateInputs(true, "local_ready", false, a, 42);
  EXPECT_FALSE(rejected.map_pose_valid); EXPECT_EQ(rejected.map_reason, "gps_stale");
  EXPECT_FALSE(rejected.context_current); EXPECT_FALSE(rejected.leveling_valid);
  EXPECT_EQ(output_gate::Decide(rejected).v2, Mode::kNone);
  EXPECT_EQ(output_gate::Decide(rejected).reason, "lidar_input_invalid");
  // Pose not ready: PrepareScan did not run for this scan, so nothing is read at all.
  const auto no_pose = frame_boundary::GateInputs(false, "imu_input_invalid", false, a, 43);
  EXPECT_FALSE(no_pose.map_pose_valid); EXPECT_TRUE(no_pose.map_reason.empty());
  EXPECT_TRUE(no_pose.time_error.empty());
  EXPECT_EQ(output_gate::Decide(no_pose).reason, "local_input_invalid:imu_input_invalid");
  // Production: common geometry OFF reads nothing and keeps the legacy decision.
  frame_boundary::ConfigureForTest(false, false, "");
  const auto off = frame_boundary::GateInputs(true, "", true, a, 41);
  EXPECT_FALSE(off.common_geometry); EXPECT_FALSE(off.map_pose_valid); EXPECT_TRUE(off.map_reason.empty());
  EXPECT_EQ(output_gate::Decide(off).reason, "legacy_mode");
  UseCommonGeometry();
}

TEST(LidarObjectV2Builder, GateInputsFromRealContextsFollowTheTruthTable) {
  UseCommonGeometry();
  struct Row { const char* reason; bool valid; bool store; Mode mode; };
  const Row rows[] = {{"gps_observed", true, true, Mode::kMap},
                      {"gps_bounded_prediction", true, true, Mode::kMap},
                      {"gps_missing", false, true, Mode::kLocal},
                      {"gps_invalid", false, true, Mode::kLocal},
                      {"gps_stale", false, true, Mode::kLocal},
                      {"gps_recovering", false, true, Mode::kLocal},
                      {"map_speed_invalid", false, true, Mode::kNone},
                      {"local_motion_invalid", false, true, Mode::kNone},
                      {"gps_observed", true, false, Mode::kNone},    // leveling never stored
                      {"gps_stale", false, false, Mode::kNone}};
  std::int64_t t = 17 * kSec; std::uint64_t generation = 100;
  for (const auto& row : rows) {
    t += kSec / 10; ++generation;
    const auto lidar = Scan(t, MapPose(t, row.valid, row.reason), {Detection(1, 10., 0.)}, generation, row.store);
    const auto gate = frame_boundary::GateInputs(true, "local_ready", true, lidar, generation);
    const auto decision = output_gate::Decide(gate);
    EXPECT_EQ(decision.v2, row.mode) << row.reason << " store=" << row.store;
    if (decision.v2 != Mode::kNone) {
      EXPECT_TRUE(frame_boundary::BuildV2(lidar, decision.v2).valid) << row.reason;
    }
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
