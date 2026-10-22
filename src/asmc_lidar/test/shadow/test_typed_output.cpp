// TypedOutput: Main FrameBoundaryPublisher Publish()/Invalid() semantics.
#include <asmc_lidar/projection.hpp>
#include <asmc_lidar/shadow/runtime.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <set>
#include <vector>

namespace {

using asmc::lidar::Context;
using asmc::lidar::Pose;
using asmc::lidar::Vec;
using asmc::lidar::pipeline::ScanResult;
using asmc::lidar::pipeline::TrackedObject;
using asmc::lidar::shadow::TypedOutput;
using Array = asmc_msgs::LidarObjectArray;

constexpr std::int64_t kStamp = 1700000000LL * 1000000000LL + 123456789LL;

Context MakeContext(std::int64_t stamp, bool map_valid) {
  Context c;
  c.time = {stamp, 5000000000LL, 5001000000LL, "ros", "fixture_ns", "steady"};
  c.frame = asmc::lidar::Frame::LeveledSensor;
  c.base_from_sensor = Pose::Identity();
  c.base_from_sensor.translation() = Vec(0.8, 0.0, 1.45);
  c.base_from_sensor.linear() = asmc::lidar::rpy(0.01, -0.02, 0.03);
  c.calibration_source = "fixture";
  c.applied_leveling = asmc::lidar::leveling_rotation(0.04, -0.03, c.base_from_sensor.rotation());
  c.leveling_source = "fixture";
  c.map_pose.valid = map_valid;
  c.map_pose.query_stamp_ns = stamp;
  c.map_pose.clock = "ros";
  c.map_pose.reason = map_valid ? "gps_observed" : "gps_stale";
  if (map_valid) {
    c.map_pose.map_from_base.linear() = asmc::lidar::rpy(0.002, -0.001, 1.1);
    c.map_pose.map_from_base.translation() = Vec(-120.5, 340.25, 12.0);
  }
  return c;
}

ScanResult MakeResult(std::int64_t stamp, bool map_valid) {
  ScanResult r;
  r.processed = true;
  r.stamp_ns = stamp;
  r.context = MakeContext(stamp, map_valid);
  return r;
}

TrackedObject MakeTrack(int id, float cx, float cy, float yaw) {
  TrackedObject o;
  o.s32_Id = id; o.u16_Idx = static_cast<std::uint16_t>(id + 3); o.s32_Age = 4; o.s32_MissCount = 0;
  o.str_ObjectClass = "vehicle"; o.str_YoloClass = "car";
  o.f32_OBB_CenterX = cx; o.f32_OBB_CenterY = cy; o.f32_OBB_CenterZ = -0.6f;
  o.f32_OBB_Width = 4.2f; o.f32_OBB_Height = 1.9f; o.f32_OBB_ZMin = -1.3f; o.f32_OBB_ZMax = 0.1f;
  o.f32_OBB_YawRad = yaw;
  const float c = std::cos(yaw), s = std::sin(yaw), hx = 2.1f, hy = 0.95f;
  const float corner[4][2] = {{-hx, -hy}, {hx, -hy}, {hx, hy}, {-hx, hy}};
  for (int i = 0; i < 4; ++i) {
    const float x = cx + c * corner[i][0] - s * corner[i][1];
    const float y = cy + s * corner[i][0] + c * corner[i][1];
    o.OBB_Corners_Body[i] = {x, y, o.f32_OBB_ZMin};
    o.OBB_Corners_Body[i + 4] = {x, y, o.f32_OBB_ZMax};
  }
  for (int i = 0; i < 7; ++i) {
    pcl::PointXYZI p; p.x = cx + 0.1f * i; p.y = cy; p.z = -0.5f; p.intensity = 3.f;
    o.pcl_ClusterPoints.push_back(p);
  }
  return o;
}

// Independent of asmc::lidar::base_from_input: base <- leveled sensor.
Pose BaseFromLevel(const Context& c) {
  Pose p = Pose::Identity();
  p.linear() = c.base_from_sensor.linear() * c.applied_leveling.transpose();
  p.translation() = c.base_from_sensor.translation();
  return p;
}

Vec ToVec(const geometry_msgs::Point& p) { return Vec(p.x, p.y, p.z); }

void ExpectGeometry(const asmc_msgs::LidarObject& msg, const TrackedObject& o, const Pose& t) {
  EXPECT_NEAR((ToVec(msg.center) - t * Vec(o.f32_OBB_CenterX, o.f32_OBB_CenterY, o.f32_OBB_CenterZ)).norm(), 0., 1e-9);
  for (int i = 0; i < 8; ++i) {
    const auto& p = o.OBB_Corners_Body[i];
    EXPECT_NEAR((ToVec(msg.obb_corners[i]) - t * Vec(p.x, p.y, p.z)).norm(), 0., 1e-9) << "corner " << i;
  }
  const Vec axis = t.linear() * Vec(std::cos(o.f32_OBB_YawRad), std::sin(o.f32_OBB_YawRad), 0.);
  EXPECT_NEAR(msg.yaw, std::atan2(axis.y(), axis.x()), 1e-9);
  // The published yaw follows the transformed bottom edge 0->1 (no tilt here
  // beyond the small leveling/map angles, so only the axis is compared above).
}

std::set<int> Ids(const Array& a) {
  std::set<int> ids;
  for (const auto& o : a.objects) ids.insert(o.track_id);
  return ids;
}

TEST(TypedOutput, ObservedLocalAndMapMatchIndependentTransform) {
  TypedOutput output;
  const auto result = MakeResult(kStamp, true);
  const std::vector<TrackedObject> objects = {MakeTrack(3, 12.f, 3.f, 0.2f), MakeTrack(7, -5.f, -4.f, -1.3f)};
  const auto arrays = output.Observed(result, objects);
  ASSERT_TRUE(arrays.accepted);
  EXPECT_TRUE(arrays.observed);
  const Pose local_t = BaseFromLevel(result.context);
  const Pose map_t = result.context.map_pose.map_from_base * local_t;
  EXPECT_NEAR((arrays.base_from_level.matrix() - local_t.matrix()).norm(), 0., 1e-12);

  EXPECT_EQ(arrays.local.header.frame_id, "base_link");
  EXPECT_EQ(arrays.map.header.frame_id, "map");
  EXPECT_EQ(static_cast<std::int64_t>(arrays.local.header.stamp.toNSec()), kStamp);
  EXPECT_EQ(static_cast<std::int64_t>(arrays.map.header.stamp.toNSec()), kStamp);
  EXPECT_EQ(arrays.local.stamp_provenance, "fixture_ns");  // from context.time
  EXPECT_EQ(arrays.map.stamp_provenance, "fixture_ns");
  EXPECT_EQ(arrays.local.status, Array::OBSERVED);
  EXPECT_EQ(arrays.local.reason, "observed");
  EXPECT_EQ(arrays.map.status, Array::OBSERVED);
  EXPECT_EQ(arrays.map.reason, "gps_observed");
  ASSERT_EQ(arrays.local.objects.size(), 2u);
  ASSERT_EQ(arrays.map.objects.size(), 2u);
  for (std::size_t i = 0; i < objects.size(); ++i) {
    ExpectGeometry(arrays.local.objects[i], objects[i], local_t);
    ExpectGeometry(arrays.map.objects[i], objects[i], map_t);
  }
  EXPECT_EQ(output.processed(), 1u);
  EXPECT_EQ(output.invalid(), 0u);
  EXPECT_EQ(output.input_reason(), "valid");
  EXPECT_EQ(output.map_reason(), "gps_observed");
}

TEST(TypedOutput, MessageFieldsFollowMain) {
  TypedOutput output;
  auto o = MakeTrack(11, 8.f, 1.f, 0.4f);
  o.s32_Age = -3;
  o.str_ObjectClass = "pedestrian"; o.str_YoloClass = "person";
  const auto arrays = output.Observed(MakeResult(kStamp, true), {o});
  ASSERT_EQ(arrays.local.objects.size(), 1u);
  const auto& m = arrays.local.objects[0];
  EXPECT_EQ(m.track_id, 11);
  EXPECT_EQ(m.detection_index, 14u);
  EXPECT_EQ(m.observation_state, asmc_msgs::LidarObject::OBSERVED);
  EXPECT_EQ(m.age, 0u);  // max(0, age)
  EXPECT_EQ(m.miss_count, 0u);
  EXPECT_EQ(m.classification, "pedestrian");
  EXPECT_EQ(m.camera_class, "person");
  EXPECT_EQ(m.length, static_cast<double>(4.2f));  // f32_OBB_Width
  EXPECT_EQ(m.width, static_cast<double>(1.9f));   // f32_OBB_Height
  EXPECT_EQ(m.height, static_cast<double>(0.1f - (-1.3f)));  // float ZMax - ZMin
  EXPECT_EQ(m.source_point_count, 7u);
  EXPECT_FALSE(m.relative_velocity_valid);
  EXPECT_FALSE(m.absolute_velocity_valid);
  EXPECT_FALSE(m.dynamic_valid);
  EXPECT_FALSE(m.yaw_rate_valid);
  EXPECT_FALSE(m.is_dynamic);
}

TEST(TypedOutput, FiltersMissedNegativeIdEmptyAndNoise) {
  TypedOutput output;
  std::vector<TrackedObject> objects;
  objects.push_back(MakeTrack(5, 10.f, 0.f, 0.f));
  auto detection = MakeTrack(-1, 11.f, 1.f, 0.f); objects.push_back(detection);
  auto missed = MakeTrack(6, 12.f, 2.f, 0.f); missed.s32_MissCount = 2; objects.push_back(missed);
  auto empty = MakeTrack(7, 13.f, 3.f, 0.f); empty.pcl_ClusterPoints.clear(); objects.push_back(empty);
  auto noise = MakeTrack(8, 14.f, 4.f, 0.f); noise.str_ObjectClass = "noise"; objects.push_back(noise);
  auto ground = MakeTrack(9, 15.f, 5.f, 0.f); ground.str_ObjectClass = "지면노이즈"; objects.push_back(ground);
  auto unknown = MakeTrack(10, 16.f, 6.f, 0.f); unknown.str_ObjectClass = "unknown"; objects.push_back(unknown);
  // An excluded object never invalidates the frame, even with bad dimensions.
  auto bad_missed = MakeTrack(12, 17.f, 7.f, 0.f); bad_missed.s32_MissCount = 1;
  bad_missed.f32_OBB_Width = NAN; objects.push_back(bad_missed);
  const auto arrays = output.Observed(MakeResult(kStamp, true), objects);
  EXPECT_EQ(arrays.local.status, Array::OBSERVED);
  EXPECT_EQ(Ids(arrays.local), (std::set<int>{5, 10}));
  EXPECT_EQ(Ids(arrays.map), (std::set<int>{5, 10}));
}

TEST(TypedOutput, ObservedEmptyWhenNothingIsExportable) {
  TypedOutput output;
  auto missed = MakeTrack(6, 12.f, 2.f, 0.f); missed.s32_MissCount = 1;
  const auto arrays = output.Observed(MakeResult(kStamp, true), {missed});
  ASSERT_TRUE(arrays.accepted);
  EXPECT_EQ(arrays.local.status, Array::OBSERVED_EMPTY);
  EXPECT_EQ(arrays.local.reason, "observed_empty");
  EXPECT_EQ(arrays.map.status, Array::OBSERVED_EMPTY);
  EXPECT_EQ(arrays.map.reason, "observed_empty");
  EXPECT_TRUE(arrays.local.objects.empty());
  EXPECT_TRUE(arrays.map.objects.empty());
  EXPECT_EQ(output.processed(), 1u);
}

TEST(TypedOutput, MapPoseUnavailableKeepsLocalObjects) {
  TypedOutput output;
  const auto arrays = output.Observed(MakeResult(kStamp, false), {MakeTrack(1, 9.f, 0.f, 0.f)});
  EXPECT_EQ(arrays.local.status, Array::OBSERVED);
  EXPECT_EQ(arrays.local.objects.size(), 1u);
  EXPECT_EQ(arrays.map.status, Array::POSE_UNAVAILABLE);
  EXPECT_EQ(arrays.map.reason, "gps_stale");
  EXPECT_TRUE(arrays.map.objects.empty());
  EXPECT_EQ(output.map_reason(), "gps_stale");
  // Empty frame with an unavailable pose: still POSE_UNAVAILABLE.
  const auto empty = output.Observed(MakeResult(kStamp + 1, false), {});
  EXPECT_EQ(empty.local.status, Array::OBSERVED_EMPTY);
  EXPECT_EQ(empty.map.status, Array::POSE_UNAVAILABLE);
  EXPECT_EQ(empty.map.reason, "gps_stale");
}

TEST(TypedOutput, PreservedQuirkMapTimeMismatchStaysObservedEmpty) {
  // Main: a valid map pose whose query stamp differs from the scan gives no map
  // geometry (transform's map check) but the map array keeps OBSERVED_EMPTY.
  TypedOutput output;
  auto result = MakeResult(kStamp, true);
  result.context.map_pose.query_stamp_ns = kStamp - 1;
  const auto arrays = output.Observed(result, {MakeTrack(1, 9.f, 0.f, 0.f)});
  EXPECT_EQ(arrays.local.status, Array::OBSERVED);
  EXPECT_EQ(arrays.map.status, Array::OBSERVED_EMPTY);
  EXPECT_EQ(arrays.map.reason, "observed_empty");
  EXPECT_TRUE(arrays.map.objects.empty());
}

TEST(TypedOutput, InvalidDimensionMakesBothArraysInputInvalid) {
  for (int variant = 0; variant < 3; ++variant) {
    TypedOutput output;
    auto bad = MakeTrack(2, 10.f, 0.f, 0.f);
    if (variant == 0) bad.f32_OBB_Width = NAN;
    if (variant == 1) bad.f32_OBB_Height = -0.5f;
    if (variant == 2) bad.f32_OBB_ZMax = bad.f32_OBB_ZMin - 0.2f;  // negative height
    const auto arrays = output.Observed(MakeResult(kStamp, true), {MakeTrack(1, 5.f, 0.f, 0.f), bad});
    ASSERT_TRUE(arrays.accepted) << variant;
    EXPECT_TRUE(arrays.observed);
    EXPECT_EQ(arrays.local.status, Array::INPUT_INVALID) << variant;
    EXPECT_EQ(arrays.map.status, Array::INPUT_INVALID) << variant;
    EXPECT_EQ(arrays.local.reason, "invalid_object_dimensions");
    EXPECT_EQ(arrays.map.reason, "invalid_object_dimensions");
    EXPECT_TRUE(arrays.local.objects.empty());
    EXPECT_TRUE(arrays.map.objects.empty());
    EXPECT_EQ(output.input_reason(), "invalid_object_dimensions");
    // Preserved quirk: Main counts the frame as processed and as invalid.
    EXPECT_EQ(output.processed(), 1u);
    EXPECT_EQ(output.invalid(), 1u);
  }
}

TEST(TypedOutput, NonFiniteGeometryMakesBothArraysInputInvalid) {
  TypedOutput output;
  auto bad = MakeTrack(2, 10.f, 0.f, 0.f);
  bad.OBB_Corners_Body[5].z = NAN;
  const auto arrays = output.Observed(MakeResult(kStamp, true), {bad});
  EXPECT_EQ(arrays.local.status, Array::INPUT_INVALID);
  EXPECT_EQ(arrays.local.reason, "nonfinite_or_invalid_geometry");
  EXPECT_EQ(arrays.map.status, Array::INPUT_INVALID);
  EXPECT_TRUE(arrays.local.objects.empty());
}

TEST(TypedOutput, InvalidSemantics) {
  TypedOutput output;
  const auto arrays = output.Invalid(kStamp, "IMU is stale for LiDAR scan");
  ASSERT_TRUE(arrays.accepted);
  EXPECT_FALSE(arrays.observed);
  EXPECT_EQ(arrays.local.header.frame_id, "base_link");
  EXPECT_EQ(arrays.map.header.frame_id, "map");
  EXPECT_EQ(static_cast<std::int64_t>(arrays.local.header.stamp.toNSec()), kStamp);
  EXPECT_EQ(static_cast<std::int64_t>(arrays.map.header.stamp.toNSec()), kStamp);
  EXPECT_EQ(arrays.local.status, Array::INPUT_INVALID);
  EXPECT_EQ(arrays.map.status, Array::INPUT_INVALID);
  EXPECT_EQ(arrays.local.reason, "IMU is stale for LiDAR scan");
  EXPECT_EQ(arrays.map.reason, "IMU is stale for LiDAR scan");
  EXPECT_EQ(arrays.local.stamp_provenance, "bridge_receive_ns");  // Main Empty() value
  EXPECT_TRUE(arrays.local.objects.empty());
  EXPECT_EQ(output.invalid(), 1u);
  EXPECT_EQ(output.processed(), 0u);
  EXPECT_EQ(output.input_reason(), "IMU is stale for LiDAR scan");
  EXPECT_EQ(output.map_reason(), "input_invalid");

  TypedOutput custom("logical_ns");
  EXPECT_EQ(custom.Invalid(kStamp, "x").map.stamp_provenance, "logical_ns");
}

TEST(TypedOutput, TimeOrLevelingErrorBecomesInvalid) {
  TypedOutput output;
  auto result = MakeResult(kStamp, true);
  result.context.time.clock.clear();
  auto arrays = output.Observed(result, {MakeTrack(1, 5.f, 0.f, 0.f)});
  ASSERT_TRUE(arrays.accepted);
  EXPECT_FALSE(arrays.observed);
  EXPECT_EQ(arrays.local.status, Array::INPUT_INVALID);
  EXPECT_EQ(arrays.local.reason, "clock_unknown");
  EXPECT_EQ(arrays.map.reason, "clock_unknown");
  EXPECT_EQ(arrays.local.stamp_provenance, "bridge_receive_ns");

  result = MakeResult(kStamp + 1, true);
  result.context.leveling_source.clear();
  arrays = output.Observed(result, {MakeTrack(1, 5.f, 0.f, 0.f)});
  ASSERT_TRUE(arrays.accepted);
  EXPECT_EQ(arrays.local.status, Array::INPUT_INVALID);
  EXPECT_EQ(arrays.local.reason, "leveling_transform_invalid");
  EXPECT_EQ(output.invalid(), 2u);
  EXPECT_EQ(output.processed(), 0u);
}

TEST(TypedOutput, StampGuardRejectsZeroDuplicateAndRegressing) {
  TypedOutput output;
  EXPECT_FALSE(output.Invalid(0, "x").accepted);
  EXPECT_EQ(output.stamp_errors(), 1u);
  // zero stamp through Observed: validate_time error -> Invalid(0) -> rejected
  EXPECT_FALSE(output.Observed(MakeResult(0, true), {}).accepted);
  EXPECT_EQ(output.stamp_errors(), 2u);
  EXPECT_TRUE(output.Observed(MakeResult(kStamp, true), {}).accepted);
  const auto duplicate = output.Observed(MakeResult(kStamp, true), {MakeTrack(1, 5.f, 0.f, 0.f)});
  EXPECT_FALSE(duplicate.accepted);
  EXPECT_TRUE(duplicate.local.objects.empty());
  EXPECT_FALSE(output.Invalid(kStamp, "x").accepted);
  EXPECT_FALSE(output.Invalid(kStamp - 1, "x").accepted);
  EXPECT_FALSE(output.Observed(MakeResult(kStamp - 5, true), {}).accepted);
  EXPECT_EQ(output.stamp_errors(), 6u);
  EXPECT_TRUE(output.Invalid(kStamp + 1, "x").accepted);
  EXPECT_TRUE(output.Observed(MakeResult(kStamp + 2, true), {}).accepted);
  EXPECT_EQ(output.processed(), 2u);
  EXPECT_EQ(output.invalid(), 1u);
}

}  // namespace

int main(int argc, char** argv) { testing::InitGoogleTest(&argc, argv); return RUN_ALL_TESTS(); }
