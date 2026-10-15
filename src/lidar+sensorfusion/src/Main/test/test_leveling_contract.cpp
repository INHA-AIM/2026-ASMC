// B0 leveling / yaw contract tests (roll/pitch exactly once, StoreLeveling once per scan,
// GPS-independent leveling, local/map consistency, no OBB yaw double rotation, Kalman frame).
// ROS-free: no ros::init, no master, no NodeHandle, no ros::Time::now().
// Only test seam used: frame_boundary::ConfigureForTest(enabled,use_roll,calibration_id).
// Tests marked "RED on TARGET" fail on fcfe8c58 + seam and pass after the leveling/yaw fix;
// tests marked "GREEN-guard" already pass on TARGET and pin behaviour that must not regress.
#include <gtest/gtest.h>

#include <Geometry/FrameBoundary.hpp>
#include <Lidar/ImuRansac/ImuRansac.hpp>
#include <Lidar/Kalman/kalman_utils.hpp>
#include <Lidar/Tracking/tracking_utils.hpp>
#include <Parser/SensorPoseHistory.hpp>
#include <PlannerInterface/PlannerObjectAdapter.hpp>
#include <asmc_lidar/projection.hpp>

#include <cmath>
#include <deque>
#include <limits>
#include <string>
#include <vector>

extern VehicleSpec st_VehicleSpec;

namespace {
using asmc::lidar::Rot;
using asmc::lidar::Vec;

constexpr std::int64_t kSec = 1000000000LL;

void SetVehicle(float roll_deg = 0.f, float pitch_deg = 0.f, float yaw_deg = 0.f) {
  st_VehicleSpec = VehicleSpec{};
  st_VehicleSpec.st_LiDARPos = {0.80f, 0.0f, 1.45f};
  st_VehicleSpec.st_LiDARRotationDeg = {roll_deg, pitch_deg, yaw_deg};
  st_VehicleSpec.st_GPSPos = {0.35f, 0.0f, 1.25f};
}
GPS_IMU Local(double roll, double pitch, double yaw, double speed = 0.0) {
  GPS_IMU out{};
  out.st_IMU.b_ImuReady = true;
  out.st_IMU.f32_Roll_rad = float(roll); out.st_IMU.f32_Pitch_rad = float(pitch);
  out.st_IMU.f32_Yaw_rad = float(yaw);
  out.st_IMU.f32_Roll_deg = float(roll * 180. / M_PI);
  out.st_IMU.f32_Pitch_deg = float(pitch * 180. / M_PI);
  out.st_IMU.f32_Yaw_deg = float(yaw * 180. / M_PI);
  out.st_CAN.f32_CurrentVel_MPS = float(speed); out.st_CAN.f32_CurrentVel_KPH = float(speed * 3.6);
  return out;
}
asmc::lidar::Observation Time(std::int64_t ns) { return {ns, ns, ns, "ros", "bridge_receive_ns", ""}; }
asmc::lidar::PoseSelection MapPose(std::int64_t ns, const GPS_IMU& local, bool valid,
                                   const std::string& reason = "gps_observed") {
  asmc::lidar::PoseSelection map;
  map.valid = valid; map.reason = valid ? "gps_observed" : reason;
  map.query_stamp_ns = ns; map.clock = "ros";
  map.map_from_base.linear() = frame_boundary::MapRotation(local);
  map.map_from_base.translation() = Vec(100.0, 200.0, 30.0);
  return map;
}
void Prepare(std::int64_t ns, const GPS_IMU& local, const asmc::lidar::PoseSelection& map,
             std::uint64_t generation) {
  frame_boundary::PrepareScan(Time(ns), local, st_VehicleSpec, map, generation);
}
pcl::PointCloud<pcl::PointXYZI>::Ptr Cloud(const std::vector<Vec>& points) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  for (const auto& v : points) {
    pcl::PointXYZI p; p.x = float(v.x()); p.y = float(v.y()); p.z = float(v.z()); p.intensity = 1.f;
    cloud->push_back(p);
  }
  return cloud;
}
Vec P(const pcl::PointXYZI& p) { return Vec(p.x, p.y, p.z); }
const std::vector<Vec> kRaw = {{20., 3., -1.45}, {12., -2.5, .7}, {6., 1., -.3}, {35., -8., .2}};
Rot Ry(double p) { return asmc::lidar::rpy(0, p, 0); }
asmc::lidar::Object PointObject(const pcl::PointCloud<pcl::PointXYZI>& cloud, std::size_t center) {
  asmc::lidar::Object o; o.id = 7; o.center = P(cloud.points[center]); o.long_axis = Vec::UnitX();
  for (unsigned i = 0; i < 8; ++i) o.corners[i] = P(cloud.points[i % cloud.size()]);
  return o;
}

// Rectangle (length along yaw) in the CURRENT leveled frame, perimeter points at z=-.5..0.5.
LiDARObjectInfo Rectangle(const Vec& c, double yaw, double length = 4.0, double width = 1.8) {
  LiDARObjectInfo out{};
  out.s32_Id = -1; out.str_ObjectClass = "vehicle"; out.str_YoloClass = "car";
  out.f32_CoreX = float(c.x()); out.f32_CoreY = float(c.y()); out.f32_CoreZ = float(c.z());
  out.f32_OBB_CenterX = out.f32_CoreX; out.f32_OBB_CenterY = out.f32_CoreY; out.f32_OBB_CenterZ = out.f32_CoreZ;
  out.f32_OBB_Width = float(length); out.f32_OBB_Height = float(width);
  out.f32_OBB_ZMin = float(c.z() - .5); out.f32_OBB_ZMax = float(c.z() + .5);
  out.f32_OBB_YawRad = float(yaw);
  out.f32_CoreE = NAN; out.f32_CoreN = NAN;
  const double cs = std::cos(yaw), sn = std::sin(yaw);
  const double sign[4][2] = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
  for (int i = 0; i < 4; ++i) {
    const double u = sign[i][0] * .5 * length, v = sign[i][1] * .5 * width;
    const float x = float(c.x() + u * cs - v * sn), y = float(c.y() + u * sn + v * cs);
    out.OBB_Corners_Body[i] = {x, y, out.f32_OBB_ZMin};
    out.OBB_Corners_Body[i + 4] = {x, y, out.f32_OBB_ZMax};
  }
  out.f32_MinX = out.f32_MinY = 1e9f; out.f32_MaxX = out.f32_MaxY = -1e9f;
  for (int k = 0; k <= 40; ++k) {
    const double t = k / 40.0;
    for (const double z : {-.5, 0., .5}) {
      const double q[4][2] = {{(t - .5) * length, -.5 * width}, {(t - .5) * length, .5 * width},
                              {-.5 * length, (t - .5) * width}, {.5 * length, (t - .5) * width}};
      for (const auto& e : q) {
        pcl::PointXYZI p;
        p.x = float(c.x() + e[0] * cs - e[1] * sn); p.y = float(c.y() + e[0] * sn + e[1] * cs);
        p.z = float(c.z() + z); p.intensity = 1.f;
        out.pcl_ClusterPoints.push_back(p);
        out.f32_MinX = std::min(out.f32_MinX, p.x); out.f32_MaxX = std::max(out.f32_MaxX, p.x);
        out.f32_MinY = std::min(out.f32_MinY, p.y); out.f32_MaxY = std::max(out.f32_MaxY, p.y);
      }
    }
  }
  out.f32_MinZ = out.f32_OBB_ZMin; out.f32_MaxZ = out.f32_OBB_ZMax;
  return out;
}
double AxisError(double a, double b) { return std::fabs(std::remainder(a - b, M_PI)); }
double CornerYaw(const LiDARObjectInfo& o) {
  return std::atan2(o.OBB_Corners_Body[1].y - o.OBB_Corners_Body[0].y,
                    o.OBB_Corners_Body[1].x - o.OBB_Corners_Body[0].x);
}
// Same mapping as FrameBoundaryPublisher::Publish (FrameBoundaryPublisher.cpp:102-108).
asmc::lidar::Object ToBoundary(const LiDARObjectInfo& o) {
  asmc::lidar::Object b; b.id = o.s32_Id; b.classification = o.str_ObjectClass;
  b.center = Vec(o.f32_OBB_CenterX, o.f32_OBB_CenterY, o.f32_OBB_CenterZ);
  b.long_axis = Vec(std::cos(o.f32_OBB_YawRad), std::sin(o.f32_OBB_YawRad), 0);
  for (unsigned i = 0; i < 8; ++i)
    b.corners[i] = Vec(o.OBB_Corners_Body[i].x, o.OBB_Corners_Body[i].y, o.OBB_Corners_Body[i].z);
  return b;
}
double GeometryCornerYaw(const asmc::lidar::Geometry& g) {
  const Vec e = g.corners[1] - g.corners[0];
  return std::atan2(e.y(), e.x());
}
// World-fixed point seen in the leveled frame of a scan (M = mount, L = stored leveling).
Vec WorldToLeveled(const Vec& world) {
  const auto& c = frame_boundary::Current();
  const Vec base = c.map_pose.map_from_base.inverse() * world;
  return c.applied_leveling * (c.base_from_sensor.inverse() * base);
}
void TrackingConfig() {
  st_Tracking.s32_NewTrackId = 100; st_Tracking.b_UseMapAssociation = true;
  st_Tracking.f32_COST_W_DIST = 1; st_Tracking.f32_COST_W_IOU = 1;
  st_Tracking.f32_COST_THRESHOLD = 3; st_Tracking.f32_COST_INF = 9999;
  st_Tracking.f32_ContinuityBonus = .2; st_Tracking.f32_MergedTrackRetireDistance = .5;
  st_Tracking.f32_DuplicateIouThreshold = .35; st_Tracking.f32_DuplicateCenterDistance = .6;
}
bool FiniteMapPoint(const pcl::PointXYZI& p, const GPS_IMU& local) {
  const auto m = TransformLidarPointToMap({p.x, p.y, p.z}, local, st_VehicleSpec);
  return std::isfinite(m.x) && std::isfinite(m.y) && std::isfinite(m.z);
}

class LevelingContract : public ::testing::Test {
 protected:
  void SetUp() override {
    SetVehicle(); TrackingConfig();
    frame_boundary::ConfigureForTest(true, true, "unit_test_calibration");
  }
  void TearDown() override { frame_boundary::ConfigureForTest(false, false, ""); SetVehicle(); }
};
}  // namespace

// ---------------------------------------------------------------- StoreLeveling exactly once per scan
// RED on TARGET: FrameBoundary.cpp:159 pre-sets leveling_source (PrepareScan alone passes
// base_from_input with identity leveling) and FrameBoundary.cpp:208 accepts further stores.
TEST_F(LevelingContract, StoreLevelingExactlyOncePerScan) {
  for (const bool roll : {false, true}) {
    frame_boundary::ConfigureForTest(true, roll, "unit_test_calibration");
    const GPS_IMU local = Local(.06, -.04, .3);
    Prepare(10 * kSec, local, MapPose(10 * kSec, local, true), 1);
    auto raw = Cloud(kRaw);
    EXPECT_FALSE(asmc::lidar::base_from_input(frame_boundary::Current()).has_value())
        << "PrepareScan alone must not prove a leveled input (roll=" << roll << ")";
    // Consumers of an unleveled scan (e.g. LidarProcess.cpp:163-174 early return) fail closed.
    const auto unleveled = asmc::lidar::transform(PointObject(*raw, 0), frame_boundary::Current());
    EXPECT_FALSE(unleveled.valid) << "roll=" << roll;
    EXPECT_FALSE(FiniteMapPoint(raw->points[0], local)) << "map adapter on an unleveled scan (roll=" << roll << ")";
    pcl::PointCloud<pcl::PointXYZI>::Ptr leveled(new pcl::PointCloud<pcl::PointXYZI>);
    CorrectingPoints(raw, leveled, local);  // the one and only StoreLeveling of this scan
    const auto base = asmc::lidar::base_from_input(frame_boundary::Current());
    ASSERT_TRUE(base.has_value());
    const auto mount = frame_boundary::SensorMount(st_VehicleSpec);
    for (std::size_t i = 0; i < kRaw.size(); ++i)
      EXPECT_LT((*base * P(leveled->points[i]) - mount * kRaw[i]).norm(), 1e-4);
    EXPECT_TRUE(FiniteMapPoint(leveled->points[0], local));
    frame_boundary::StoreLeveling(frame_boundary::Current().applied_leveling);  // 2nd store, same scan
    EXPECT_FALSE(asmc::lidar::base_from_input(frame_boundary::Current()).has_value())
        << "a second leveling store in one scan must fail closed (roll=" << roll << ")";
    EXPECT_FALSE(FiniteMapPoint(leveled->points[0], local)) << "roll=" << roll;
    Prepare(10 * kSec + kSec / 10, local, MapPose(10 * kSec + kSec / 10, local, true), 2);
    CorrectingPoints(raw, leveled, local);
    EXPECT_TRUE(asmc::lidar::base_from_input(frame_boundary::Current()).has_value())
        << "the next scan starts a new single-store window";
    // An invalid first store is not repaired by a later valid store in the same scan.
    Prepare(10 * kSec + kSec / 5, local, MapPose(10 * kSec + kSec / 5, local, true), 3);
    frame_boundary::StoreLeveling(Rot::Constant(std::numeric_limits<double>::quiet_NaN()));
    EXPECT_FALSE(asmc::lidar::base_from_input(frame_boundary::Current()).has_value());
    CorrectingPoints(raw, leveled, local);
    EXPECT_FALSE(asmc::lidar::base_from_input(frame_boundary::Current()).has_value())
        << "invalid store followed by another store in one scan (roll=" << roll << ")";
  }
}

// API contract of the single-store guard (StoreLeveling result + LevelingStoreCount). RED on
// TARGET as a compile failure: StoreLeveling returns void and LevelingStoreCount does not exist.
TEST_F(LevelingContract, StoreLevelingReportsResultAndCountPerScan) {
  for (const bool roll : {false, true}) {
    frame_boundary::ConfigureForTest(true, roll, "unit_test_calibration");
    const GPS_IMU local = Local(.03, .02, .1);
    const Rot level = asmc::lidar::leveling_rotation(.03, .02, Rot::Identity(), roll, true);
    Prepare(16 * kSec, local, MapPose(16 * kSec, local, true), 1);
    EXPECT_EQ(frame_boundary::LevelingStoreCount(), 0u);
    EXPECT_TRUE(frame_boundary::Current().leveling_source.empty());
    EXPECT_TRUE(frame_boundary::StoreLeveling(level));
    EXPECT_EQ(frame_boundary::LevelingStoreCount(), 1u);
    EXPECT_EQ(frame_boundary::Current().leveling_source, "Main::CorrectingPoints");
    EXPECT_FALSE(frame_boundary::StoreLeveling(Ry(.3))) << "second store in one scan";
    EXPECT_EQ(frame_boundary::LevelingStoreCount(), 2u);
    EXPECT_TRUE(frame_boundary::Current().leveling_source.empty());
    EXPECT_LT((frame_boundary::Current().applied_leveling - level).norm(), 1e-12)
        << "a rejected store never replaces the applied rotation";
    Prepare(16 * kSec + kSec / 10, local, MapPose(16 * kSec + kSec / 10, local, true), 2);
    EXPECT_EQ(frame_boundary::LevelingStoreCount(), 0u) << "PrepareScan opens a new window";
    EXPECT_FALSE(frame_boundary::StoreLeveling(Rot::Constant(std::numeric_limits<double>::quiet_NaN())));
    EXPECT_EQ(frame_boundary::LevelingStoreCount(), 1u);
    EXPECT_FALSE(asmc::lidar::base_from_input(frame_boundary::Current()).has_value());
    // Runtime path: one CorrectingPoints per scan == exactly one store.
    Prepare(16 * kSec + kSec / 5, local, MapPose(16 * kSec + kSec / 5, local, true), 3);
    auto raw = Cloud(kRaw);
    pcl::PointCloud<pcl::PointXYZI>::Ptr out(new pcl::PointCloud<pcl::PointXYZI>);
    CorrectingPoints(raw, out, local);
    EXPECT_EQ(frame_boundary::LevelingStoreCount(), 1u) << "roll=" << roll;
    EXPECT_TRUE(asmc::lidar::base_from_input(frame_boundary::Current()).has_value());
  }
  frame_boundary::ConfigureForTest(false, false, "");
  auto raw = Cloud(kRaw);
  pcl::PointCloud<pcl::PointXYZI>::Ptr out(new pcl::PointCloud<pcl::PointXYZI>);
  CorrectingPoints(raw, out, Local(.03, .02, .1));
  EXPECT_EQ(frame_boundary::LevelingStoreCount(), 0u) << "legacy CorrectingPoints never stores";
}

// GREEN-guard: the rotation applied to points is exactly the stored one; legacy stores nothing.
TEST_F(LevelingContract, CorrectingPointsAppliesExactlyTheStoredRotation) {
  for (const bool roll : {false, true})
    for (const double r : {-.08, 0., .06})
      for (const double p : {-.05, 0., .07}) {
        frame_boundary::ConfigureForTest(true, roll, "unit_test_calibration");
        SetVehicle(2.f, -1.5f, 3.f);  // non-zero mount
        const GPS_IMU local = Local(r, p, .4);
        Prepare(11 * kSec, local, MapPose(11 * kSec, local, true), 1);
        auto raw = Cloud(kRaw);
        pcl::PointCloud<pcl::PointXYZI>::Ptr out(new pcl::PointCloud<pcl::PointXYZI>);
        CorrectingPoints(raw, out, local);
        const Rot expected = asmc::lidar::leveling_rotation(r, p,
            frame_boundary::SensorMount(st_VehicleSpec).rotation(), roll, true);
        EXPECT_LT((frame_boundary::Current().applied_leveling - expected).norm(), 1e-6);
        EXPECT_EQ(frame_boundary::Current().leveling_source, "Main::CorrectingPoints");
        for (std::size_t i = 0; i < kRaw.size(); ++i)
          EXPECT_LT((P(out->points[i]) - expected * kRaw[i]).norm(), 1e-4);
      }
  frame_boundary::ConfigureForTest(false, false, "");
  SetVehicle();
  const GPS_IMU local = Local(.05, .04, .2);
  auto raw = Cloud(kRaw);
  pcl::PointCloud<pcl::PointXYZI>::Ptr out(new pcl::PointCloud<pcl::PointXYZI>);
  CorrectingPoints(raw, out, local);
  for (std::size_t i = 0; i < kRaw.size(); ++i)
    EXPECT_LT((P(out->points[i]) - Ry(.04) * kRaw[i]).norm(), 1e-4) << "legacy = Ry(+pitch) only";
  EXPECT_TRUE(frame_boundary::Current().leveling_source.empty()) << "legacy never stores leveling";
}

// ---------------------------------------------------------------- roll/pitch exactly once
// GREEN-guard on TARGET: local = M*raw (IMU roll/pitch net 0), map = R_rpy*(M*raw)+t (once each).
TEST_F(LevelingContract, RollPitchExactlyOnceInLocalAndMap) {
  for (const bool roll : {false, true}) {
    frame_boundary::ConfigureForTest(true, roll, "unit_test_calibration");
    SetVehicle(2.f, -1.5f, 3.f);
    const GPS_IMU local = Local(.07, -.05, .9);
    Prepare(12 * kSec, local, MapPose(12 * kSec, local, true), 1);
    auto raw = Cloud(kRaw);
    pcl::PointCloud<pcl::PointXYZI>::Ptr out(new pcl::PointCloud<pcl::PointXYZI>);
    CorrectingPoints(raw, out, local);
    const auto& ctx = frame_boundary::Current();
    const auto mount = frame_boundary::SensorMount(st_VehicleSpec);
    asmc::lidar::Object o; o.id = 1; o.center = P(out->points[0]);
    o.long_axis = ctx.applied_leveling * Vec::UnitX();
    for (unsigned i = 0; i < 8; ++i) o.corners[i] = P(out->points[i % kRaw.size()]);
    const auto res = asmc::lidar::transform(o, ctx);
    ASSERT_TRUE(res.valid) << res.reason;
    ASSERT_TRUE(res.map.has_value());
    const Vec truth_local = mount * kRaw[0], truth_map = ctx.map_pose.map_from_base * truth_local;
    EXPECT_LT((res.base.center - truth_local).norm(), 1e-4);
    EXPECT_LT((res.map->center - truth_map).norm(), 1e-4);
    EXPECT_LT((res.base.long_axis - mount.rotation() * Vec::UnitX()).norm(), 1e-6);
    const auto poa = TransformLidarPointToMap({out->points[0].x, out->points[0].y, out->points[0].z},
                                              local, st_VehicleSpec);
    EXPECT_LT((Vec(poa.x, poa.y, poa.z) - truth_map).norm(), 1e-4) << "legacy-format map adapter";
    // Negative control: an input leveled twice is caught by the same check.
    const Vec twice = ctx.applied_leveling * P(out->points[0]);
    EXPECT_GT((*asmc::lidar::base_from_input(ctx) * twice - truth_local).norm(), 1e-2);
  }
}

// GREEN-guard: the common map helpers (Body2ENU/MinMax2ENU) apply pitch once (Global.cpp:684-688,778-793)
// for both lidar_level_roll settings.
TEST_F(LevelingContract, CommonMapHelpersApplyPitchOnce) {
  for (const bool roll : {false, true}) {
    frame_boundary::ConfigureForTest(true, roll, "unit_test_calibration");
    const GPS_IMU local = Local(.05, .06, .4);
    Prepare(13 * kSec, local, MapPose(13 * kSec, local, true), 1);
    auto raw = Cloud(kRaw);
    pcl::PointCloud<pcl::PointXYZI>::Ptr out(new pcl::PointCloud<pcl::PointXYZI>);
    CorrectingPoints(raw, out, local);
    const auto& ctx = frame_boundary::Current();
    const Vec truth = ctx.map_pose.map_from_base * (frame_boundary::SensorMount(st_VehicleSpec) * kRaw[0]);
    LiDARObjectInfo object{};
    object.f32_CoreX = out->points[0].x; object.f32_CoreY = out->points[0].y; object.f32_CoreZ = out->points[0].z;
    object.f32_MinX = object.f32_MaxX = object.f32_CoreX;
    object.f32_MinY = object.f32_MaxY = object.f32_CoreY;
    object.f32_MinZ = object.f32_MaxZ = object.f32_CoreZ;
    Body2ENU(st_VehicleSpec, local, object);
    MinMax2ENU(st_VehicleSpec, local, object);
    EXPECT_NEAR(object.f32_CoreE, truth.x(), 2e-3) << "roll=" << roll;
    EXPECT_NEAR(object.f32_CoreN, truth.y(), 2e-3) << "roll=" << roll;
    EXPECT_NEAR(object.f32_MinE_X, truth.x(), 2e-3); EXPECT_NEAR(object.f32_MaxE_Y, truth.y(), 2e-3);
    EXPECT_NEAR(object.f32_MinE_Z, truth.z(), 2e-3);
  }
}

// ---------------------------------------------------------------- legacy characterization
// GREEN on TARGET as a CHARACTERIZATION of a KNOWN legacy defect (not a fix, B1):
//   map adapter (PlannerObjectAdapter.cpp:93-107) undoes Ry(p) -> correct once;
//   Body2ENU legacy (Global.cpp:690-720) does not -> pitch twice.
// If the legacy branch is ever fixed, flip the second block to EXPECT_NEAR(truth).
TEST_F(LevelingContract, LegacyBody2EnuAppliesPitchTwice_KnownDefect) {
  frame_boundary::ConfigureForTest(false, false, "");
  GPS_IMU local = Local(0., 3. * M_PI / 180., .4);
  local.st_GPS.f64_E = 100.; local.st_GPS.f64_N = 200.; local.st_GPS.f64_U = 30.;
  auto raw = Cloud({kRaw[0]});
  pcl::PointCloud<pcl::PointXYZI>::Ptr leveled(new pcl::PointCloud<pcl::PointXYZI>);
  CorrectingPoints(raw, leveled, local);
  const Rot R = asmc::lidar::rpy(0., 3. * M_PI / 180., .4);
  const Vec lever(.8 - .35, 0., 1.45 - 1.25), gps(100., 200., 30.);
  const Vec truth = gps + R * (kRaw[0] + lever);
  const auto poa = TransformLidarPointToMap({leveled->points[0].x, leveled->points[0].y,
                                             leveled->points[0].z}, local, st_VehicleSpec);
  EXPECT_LT((Vec(poa.x, poa.y, poa.z) - truth).norm(), 2e-3) << "legacy output adapter is once";
  LiDARObjectInfo object{};
  object.f32_CoreX = leveled->points[0].x; object.f32_CoreY = leveled->points[0].y;
  object.f32_CoreZ = leveled->points[0].z;
  Body2ENU(st_VehicleSpec, local, object);
  const Vec doubled = gps + R * (P(leveled->points[0]) + lever);  // = R*Ry(p)*raw: pitch twice
  EXPECT_NEAR(object.f32_CoreE, doubled.x(), 5e-3);
  EXPECT_NEAR(object.f32_CoreN, doubled.y(), 5e-3);
  EXPECT_GT(std::hypot(object.f32_CoreE - truth.x(), object.f32_CoreN - truth.y()), .1)
      << "documented legacy pitch double (~0.16 m at 20 m, 3 deg)";
}

// ---------------------------------------------------------------- GPS independence of leveling
// GREEN-guard: same scan IMU under healthy/invalid/stale/recovering/missing/recovered map states,
// for both lidar_level_roll settings and a non-zero mount.
TEST_F(LevelingContract, GpsStateDoesNotChangeLevelingOrLocalOutput) {
  for (const bool roll : {false, true}) {
    frame_boundary::ConfigureForTest(true, roll, "unit_test_calibration");
    SetVehicle(2.f, -1.5f, 3.f);
    const GPS_IMU local = Local(.04, -.03, 1.1);
    const std::vector<std::pair<bool, std::string>> states = {
        {true, "gps_observed"}, {false, "gps_invalid"}, {false, "gps_stale"},
        {false, "gps_recovering"}, {false, "gps_missing"}, {true, "gps_recovered"}};
    std::vector<std::vector<float>> clouds; std::vector<asmc::lidar::Result> results;
    std::vector<Rot> levels;
    std::uint64_t generation = 0;
    for (const auto& s : states) {
      const std::int64_t ns = 14 * kSec + std::int64_t(generation) * kSec / 10;
      Prepare(ns, local, MapPose(ns, local, s.first, s.second), ++generation);
      auto raw = Cloud(kRaw);
      pcl::PointCloud<pcl::PointXYZI>::Ptr out(new pcl::PointCloud<pcl::PointXYZI>);
      CorrectingPoints(raw, out, local);
      levels.push_back(frame_boundary::Current().applied_leveling);
      std::vector<float> flat;
      for (const auto& p : out->points) { flat.push_back(p.x); flat.push_back(p.y); flat.push_back(p.z); }
      clouds.push_back(flat);
      results.push_back(asmc::lidar::transform(PointObject(*out, 1), frame_boundary::Current()));
      EXPECT_EQ(results.back().map.has_value(), s.first) << s.second;
      EXPECT_EQ(FiniteMapPoint(out->points[1], local), s.first)
          << "map adapter fails closed without GPS: " << s.second;
    }
    for (std::size_t i = 1; i < states.size(); ++i) {
      EXPECT_EQ((levels[i] - levels[0]).norm(), 0.) << "leveling changed with " << states[i].second;
      EXPECT_EQ(clouds[i], clouds[0]) << "leveled cloud changed with " << states[i].second;
      ASSERT_TRUE(results[i].valid) << results[i].reason;
      EXPECT_EQ((results[i].base.center - results[0].base.center).norm(), 0.) << states[i].second;
      for (unsigned k = 0; k < 8; ++k)
        EXPECT_EQ((results[i].base.corners[k] - results[0].base.corners[k]).norm(), 0.);
    }
  }
}

// GREEN-guard: GPS denial and the common 3-consecutive-good recovery at the SensorPoseHistory
// boundary, using only the full-ns API (no ros::Time::now()). Local IMU/CAN readiness is independent.
TEST_F(LevelingContract, GpsDenialStaleAndThreeSampleRecovery) {
  SensorPoseHistory history;
  const std::int64_t t0 = 50 * kSec;
  const GPS_IMU motion = Local(.02, -.01, .3, 2.0);
  auto feed_local = [&](std::int64_t until) {
    for (std::int64_t t = t0; t <= until; t += kSec / 40) {
      history.AddImu(std::uint64_t(t / 1000000), motion.st_IMU, nullptr, std::uint64_t(t), std::uint64_t(t + 1000));
      history.AddCan(std::uint64_t(t / 1000000), motion.st_CAN, nullptr, std::uint64_t(t), std::uint64_t(t + 1000));
    }
  };
  GPS_DATA gps{}; gps.f64_E = 100; gps.f64_N = 200; gps.f64_U = 30; gps.b_isJamming = false;
  auto good = [&](std::int64_t t) { history.AddGps(std::uint64_t(t / 1000000), gps, std::uint64_t(t), std::uint64_t(t + 1000000)); };
  auto at = [&](std::int64_t scan) {
    const asmc::lidar::Observation o{scan, scan + 1000000, scan + 2000000, "ros", "bridge_receive_ns", "steady"};
    return history.MapAt(o, motion);
  };
  feed_local(t0 + 3 * kSec);
  EXPECT_EQ(at(t0 + kSec / 20).reason, "gps_missing");
  good(t0); good(t0 + kSec / 5); good(t0 + 2 * kSec / 5);
  EXPECT_TRUE(at(t0 + 9 * kSec / 20).valid);
  EXPECT_EQ(at(t0 + 701 * kSec / 1000).reason, "gps_stale");
  history.AddGpsInvalidAt(std::uint64_t(t0 + 750 * kSec / 1000), std::uint64_t(t0 + 751 * kSec / 1000));
  EXPECT_EQ(at(t0 + 760 * kSec / 1000).reason, "gps_invalid");
  good(t0 + 800 * kSec / 1000);
  EXPECT_EQ(at(t0 + 810 * kSec / 1000).reason, "gps_recovering");
  good(t0 + kSec);
  EXPECT_EQ(at(t0 + 1010 * kSec / 1000).reason, "gps_recovering");
  good(t0 + 1200 * kSec / 1000);
  EXPECT_TRUE(at(t0 + 1210 * kSec / 1000).valid) << "third consecutive good sample recovers";
  GPS_IMU resolved{}; LocalInputSelection sel; std::string reason;
  for (const std::int64_t scan : {t0 + 760 * kSec / 1000, t0 + 810 * kSec / 1000, t0 + 1210 * kSec / 1000})
    EXPECT_TRUE(history.WaitForLocalLookupNs(std::uint64_t(scan), 0., &resolved, &sel, &reason))
        << "local IMU+CAN readiness must not depend on GPS: " << reason;
}

// GREEN-guard: the TemporalThread order (main.cpp:489-515) driven through SensorPoseHistory across
// healthy -> stale -> invalid -> recovering -> recovered. The leveling, the leveled cloud and the local
// result are bitwise identical in every phase; map exists only when the pose history says valid and
// then equals map_from_base * local. The GPS NaN written into the scan pose (main.cpp:513) never
// reaches the leveling.
TEST_F(LevelingContract, PoseHistoryDenialAndRecoveryKeepLevelingAndLocalBitwise) {
  for (const bool roll : {false, true}) {
    frame_boundary::ConfigureForTest(true, roll, "unit_test_calibration");
    SetVehicle(1.f, -2.f, .5f);
    SensorPoseHistory history;
    const std::int64_t t0 = 60 * kSec;
    const GPS_IMU motion = Local(.05, -.04, .3, 2.0);
    for (std::int64_t t = t0; t <= t0 + 3 * kSec; t += kSec / 40) {
      history.AddImu(std::uint64_t(t / 1000000), motion.st_IMU, nullptr, std::uint64_t(t), std::uint64_t(t + 1000));
      history.AddCan(std::uint64_t(t / 1000000), motion.st_CAN, nullptr, std::uint64_t(t), std::uint64_t(t + 1000));
    }
    GPS_DATA gps{}; gps.f64_E = 100; gps.f64_N = 200; gps.f64_U = 30; gps.b_isJamming = false;
    auto good = [&](std::int64_t t) { history.AddGps(std::uint64_t(t / 1000000), gps, std::uint64_t(t), std::uint64_t(t + 1000000)); };
    std::vector<std::vector<float>> clouds; std::vector<asmc::lidar::Result> results;
    std::vector<Rot> levels; std::vector<std::string> phases;
    std::uint64_t generation = 0;
    auto scan = [&](std::int64_t ns, const std::string& expected) {
      GPS_IMU pose{}; LocalInputSelection sel; std::string reason;
      ASSERT_TRUE(history.WaitForLocalLookupNs(std::uint64_t(ns), 0., &pose, &sel, &reason)) << reason;
      const asmc::lidar::Observation o{ns, ns + 1000000, ns + 2000000, "ros", "bridge_receive_ns", "steady"};
      const auto map = history.MapAt(o, pose);
      EXPECT_EQ(map.valid ? std::string("valid") : map.reason, expected) << "roll=" << roll;
      frame_boundary::PrepareScan(o, pose, st_VehicleSpec, map, ++generation);
      if (map.valid) {
        const Vec antenna = map.map_from_base.translation() + map.map_from_base.rotation() *
            Vec(st_VehicleSpec.st_GPSPos.f32_X, st_VehicleSpec.st_GPSPos.f32_Y, st_VehicleSpec.st_GPSPos.f32_Z);
        pose.st_GPS.f64_E = antenna.x(); pose.st_GPS.f64_N = antenna.y(); pose.st_GPS.f64_U = antenna.z();
      } else {
        pose.st_GPS.f64_E = pose.st_GPS.f64_N = pose.st_GPS.f64_U = NAN;
      }
      auto raw = Cloud(kRaw);
      pcl::PointCloud<pcl::PointXYZI>::Ptr out(new pcl::PointCloud<pcl::PointXYZI>);
      CorrectingPoints(raw, out, pose);
      levels.push_back(frame_boundary::Current().applied_leveling);
      std::vector<float> flat;
      for (const auto& p : out->points) { flat.push_back(p.x); flat.push_back(p.y); flat.push_back(p.z); }
      clouds.push_back(flat);
      results.push_back(asmc::lidar::transform(PointObject(*out, 2), frame_boundary::Current()));
      phases.push_back(expected);
      ASSERT_TRUE(results.back().valid) << results.back().reason;
      EXPECT_EQ(results.back().map.has_value(), map.valid) << expected;
      if (results.back().map) {
        EXPECT_LT((results.back().map->center - map.map_from_base * results.back().base.center).norm(), 1e-9);
      }
    };
    good(t0); good(t0 + kSec / 5); good(t0 + 2 * kSec / 5);
    scan(t0 + 9 * kSec / 20, "valid");
    scan(t0 + 701 * kSec / 1000, "gps_stale");
    history.AddGpsInvalidAt(std::uint64_t(t0 + 750 * kSec / 1000), std::uint64_t(t0 + 751 * kSec / 1000));
    scan(t0 + 760 * kSec / 1000, "gps_invalid");
    good(t0 + 800 * kSec / 1000);
    scan(t0 + 810 * kSec / 1000, "gps_recovering");
    good(t0 + kSec);
    scan(t0 + 1010 * kSec / 1000, "gps_recovering");
    good(t0 + 1200 * kSec / 1000);
    scan(t0 + 1210 * kSec / 1000, "valid");
    ASSERT_EQ(clouds.size(), 6u);
    const Rot expected = asmc::lidar::leveling_rotation(motion.st_IMU.f32_Roll_rad, motion.st_IMU.f32_Pitch_rad,
        frame_boundary::SensorMount(st_VehicleSpec).rotation(), roll, true);
    for (std::size_t i = 0; i < clouds.size(); ++i) {
      EXPECT_LT((levels[i] - expected).norm(), 1e-12) << phases[i];
      EXPECT_EQ(clouds[i], clouds[0]) << "leveled cloud changed in phase " << phases[i];
      EXPECT_EQ((results[i].base.center - results[0].base.center).norm(), 0.) << phases[i];
      for (unsigned k = 0; k < 8; ++k)
        EXPECT_EQ((results[i].base.corners[k] - results[0].base.corners[k]).norm(), 0.) << phases[i];
    }
  }
}

// ---------------------------------------------------------------- same object local vs map
// GREEN-guard: local and map come from ONE transform of ONE context (FBP uses lidar.frame_context,
// PlannerObjectAdapter uses Current(); LidarProcess.cpp:272 makes them the same scan).
TEST_F(LevelingContract, SameObjectLocalAndMapFromOneScanContext) {
  for (const bool roll : {false, true}) {
    frame_boundary::ConfigureForTest(true, roll, "unit_test_calibration");
    const GPS_IMU local = Local(.06, .05, -.7);
    Prepare(15 * kSec, local, MapPose(15 * kSec, local, true), 1);
    auto raw = Cloud(kRaw);
    pcl::PointCloud<pcl::PointXYZI>::Ptr out(new pcl::PointCloud<pcl::PointXYZI>);
    CorrectingPoints(raw, out, local);
    const asmc::lidar::Context snapshot = frame_boundary::Current();  // == st_LiDAR.frame_context
    const auto box = Rectangle(P(out->points[2]), .3);
    const auto a = asmc::lidar::transform(ToBoundary(box), snapshot);
    const auto b = asmc::lidar::transform(ToBoundary(box), frame_boundary::Current());
    ASSERT_TRUE(a.valid && a.map && b.map);
    const auto& T = snapshot.map_pose.map_from_base;
    EXPECT_LT((a.map->center - T * a.base.center).norm(), 1e-9);
    for (unsigned k = 0; k < 8; ++k) {
      EXPECT_LT((a.map->corners[k] - T * a.base.corners[k]).norm(), 1e-9);
      EXPECT_EQ((a.base.corners[k] - b.base.corners[k]).norm(), 0.);
      const auto poa = TransformLidarPointToMap({box.OBB_Corners_Body[k].x, box.OBB_Corners_Body[k].y,
                                                 box.OBB_Corners_Body[k].z}, local, st_VehicleSpec);
      EXPECT_LT((Vec(poa.x, poa.y, poa.z) - a.map->corners[k]).norm(), 1e-9);
    }
    EXPECT_LT(AxisError(*a.base.yaw, GeometryCornerYaw(a.base)), 1e-6) << "roll=" << roll;
    EXPECT_LT(AxisError(*a.map->yaw, GeometryCornerYaw(*a.map)), 1e-6) << "roll=" << roll;
  }
}

// ---------------------------------------------------------------- no OBB yaw double rotation
// RED on TARGET: Tracking.cpp:760 rotates the previous yaw once (correct), stabilizeMatchedOBB
// writes a current-frame yaw (Tracking.cpp:761,216), then kalman_filter_update (Tracking.cpp:764)
// -> compensate_ego_yaw (Kalman.cpp:158) -> AlignTrackingFrame (Kalman.cpp:54-55) rotates it again
// (FrameBoundary.cpp:203-204). Expected TARGET failure: published - truth = -(ego yaw delta) = -0.1 rad.
TEST_F(LevelingContract, NoObbYawDoubleRotationAcrossTwoScans) {
  struct Case { bool roll; double r1, p1, r2, p2; };
  for (const Case k : {Case{true, .05, -.04, -.03, .06}, Case{false, 0., -.04, 0., .06}}) {
    frame_boundary::ConfigureForTest(true, k.roll, "unit_test_calibration");
    st_Tracking.s32_NewTrackId = 100;
    const Vec world_center(112., 202., 30.);  // map frame; ego static at map (100,200,30)
    const double world_yaw = .7, yaw1 = .2, yaw2 = .3;
    const GPS_IMU first = Local(k.r1, k.p1, yaw1);
    Prepare(20 * kSec, first, MapPose(20 * kSec, first, true), 1);
    frame_boundary::StoreLeveling(asmc::lidar::leveling_rotation(k.r1, k.p1, Rot::Identity(), k.roll, true));
    const Vec c1 = WorldToLeveled(world_center);
    auto previous = Rectangle(c1, world_yaw - yaw1);
    previous.s32_Id = 42; previous.s32_Age = 10; previous.b_KfInitialized = true; previous.s32_KfInitCount = 2;
    previous.Eigen_KfState_CA << float(c1.x()), float(c1.y()), 0, 0, 0, 0;
    previous.f32_PrevYaw = float(yaw1);
    frame_boundary::RecordTrackingFrame(previous);   // LidarProcess.cpp:299-300

    const GPS_IMU second = Local(k.r2, k.p2, yaw2);
    Prepare(20 * kSec + kSec / 10, second, MapPose(20 * kSec + kSec / 10, second, true), 2);
    frame_boundary::StoreLeveling(asmc::lidar::leveling_rotation(k.r2, k.p2, Rot::Identity(), k.roll, true));
    const Vec c2 = WorldToLeveled(world_center);
    const double truth = world_yaw - yaw2;
    std::deque<std::vector<LiDARObjectInfo>> past{{previous}};
    std::vector<LiDARObjectInfo> detections{Rectangle(c2, truth)};
    CompareCentroids(past, detections, .1, second);
    ASSERT_EQ(detections.size(), 1u);
    ASSERT_EQ(detections.front().s32_Id, 42);
    const auto& tracked = detections.front();
    EXPECT_LT(AxisError(tracked.f32_OBB_YawRad, truth), 1e-3)
        << "roll=" << k.roll << " published " << tracked.f32_OBB_YawRad << " truth " << truth
        << " (TARGET expectation: truth - 0.1)";
    EXPECT_LT(AxisError(tracked.f32_OBB_YawRad, CornerYaw(tracked)), 1e-3) << "yaw vs own corners";
    // What /lidar/local_object and /lidar/map_object (and the v1 /perception/lidar/objects_local|objects_map) would
    // carry: FrameBoundaryPublisher.cpp BuildV2 and Publisher::Publish apply asmc::lidar::transform per object, as here.
    const auto res = asmc::lidar::transform(ToBoundary(tracked), frame_boundary::Current());
    ASSERT_TRUE(res.valid && res.map);
    EXPECT_LT(AxisError(*res.base.yaw, GeometryCornerYaw(res.base)), 1e-3) << "local yaw vs corners";
    EXPECT_LT(AxisError(*res.map->yaw, GeometryCornerYaw(*res.map)), 1e-3) << "map yaw vs corners";
  }
}

// ---------------------------------------------------------------- Kalman frame contract
// RED on TARGET: update path rotates the CURRENT detection's yaw (Kalman.cpp:158 -> :54-55).
TEST_F(LevelingContract, KalmanUpdateKeepsCurrentDetectionObbYaw) {
  const GPS_IMU first = Local(.04, .02, .2);
  Prepare(31 * kSec, first, MapPose(31 * kSec, first, false, "gps_invalid"), 1);
  frame_boundary::StoreLeveling(asmc::lidar::leveling_rotation(.04, .02, Rot::Identity(), true, true));
  LiDARObjectInfo current = Rectangle(Vec(10., 1., 0.), .33);
  current.b_KfInitialized = true; current.s32_KfInitCount = 2;
  current.Eigen_KfState_CA << 10.f, 1.f, 0, 0, 0, 0;
  current.f32_PrevYaw = .2f;
  frame_boundary::RecordTrackingFrame(current);  // inherited previous-scan tracking basis
  const GPS_IMU second = Local(.04, .02, .3);
  Prepare(31 * kSec + kSec / 10, second, MapPose(31 * kSec + kSec / 10, second, false, "gps_invalid"), 2);
  frame_boundary::StoreLeveling(asmc::lidar::leveling_rotation(.04, .02, Rot::Identity(), true, true));
  const float before = current.f32_OBB_YawRad;
  kalman_filter_update(current, 10.f, 1.f, 10.f, 1.f, .1, second.st_IMU.f32_Yaw_deg);
  EXPECT_FLOAT_EQ(current.f32_OBB_YawRad, before) << "TARGET expectation: before - 0.1";
  EXPECT_NEAR(current.f32_PrevYaw, .3, 1e-5);
}

// GREEN-guard: miss-track predict re-expresses a previous-scan OBB yaw exactly once.
TEST_F(LevelingContract, MissTrackPredictReexpressesPreviousObbYawOnce) {
  const GPS_IMU first = Local(-.03, .05, .2);
  Prepare(32 * kSec, first, MapPose(32 * kSec, first, true), 1);
  frame_boundary::StoreLeveling(asmc::lidar::leveling_rotation(-.03, .05, Rot::Identity(), true, true));
  LiDARObjectInfo missing = Rectangle(Vec(10., 1., 0.), .33);
  missing.b_KfInitialized = true; missing.Eigen_KfState_CA << 10.f, 1.f, 0, 0, 0, 0;
  missing.f32_PrevYaw = .2f;
  frame_boundary::RecordTrackingFrame(missing);
  const GPS_IMU second = Local(.02, -.04, .3);
  Prepare(32 * kSec + kSec / 10, second, MapPose(32 * kSec + kSec / 10, second, true), 2);
  frame_boundary::StoreLeveling(asmc::lidar::leveling_rotation(.02, -.04, Rot::Identity(), true, true));
  kalman_filter_predict(missing, .1, second.st_IMU.f32_Yaw_deg);
  EXPECT_NEAR(missing.f32_OBB_YawRad, .33 - .1, 1e-5);
}

// GREEN-guard: KF re-expression of a world-fixed point under roll/pitch + non-zero mount
// (ROS-free port of test_frame_boundary_ros.cpp:121-154), plus yaw of the aligned copy once.
TEST_F(LevelingContract, KalmanStateReexpressionExactUnderRollPitchAndMount) {
  for (const bool roll : {false, true}) {
    frame_boundary::ConfigureForTest(true, roll, "unit_test_calibration");
    SetVehicle(4.f, -3.f, 8.f);
    const Vec world(112., 202., 31.);
    GPS_IMU first = Local(.1, -.08, .2);
    Prepare(25 * kSec, first, MapPose(25 * kSec, first, true), 1);
    frame_boundary::StoreLeveling(asmc::lidar::leveling_rotation(.1, -.08,
        frame_boundary::SensorMount(st_VehicleSpec).rotation(), roll, true));
    const Vec p1 = WorldToLeveled(world);
    LiDARObjectInfo previous = Rectangle(p1, .5);
    previous.s32_Id = 42; previous.b_KfInitialized = true; previous.s32_KfInitCount = 2;
    previous.Eigen_KfState_CA << float(p1.x()), float(p1.y()), 0, 0, 0, 0;
    previous.f32_PrevYaw = .2f;
    frame_boundary::RecordTrackingFrame(previous);
    GPS_IMU second = Local(-.12, .06, .28);
    Prepare(25 * kSec + kSec / 10, second, MapPose(25 * kSec + kSec / 10, second, true), 2);
    frame_boundary::StoreLeveling(asmc::lidar::leveling_rotation(-.12, .06,
        frame_boundary::SensorMount(st_VehicleSpec).rotation(), roll, true));
    const Vec p2 = WorldToLeveled(world);
    auto aligned = previous;
    frame_boundary::AlignTrackingFrame(aligned, .28);
    EXPECT_NEAR(aligned.Eigen_KfState_CA(0), p2.x(), 1e-4) << "roll=" << roll;
    EXPECT_NEAR(aligned.Eigen_KfState_CA(1), p2.y(), 1e-4);
    EXPECT_NEAR(aligned.tracking_height_input, p2.z(), 1e-4);
  }
}

// GREEN-guard (legacy unchanged): yaw-only state rotation, OBB yaw untouched, roll/pitch never used.
TEST_F(LevelingContract, LegacyKalmanPathUnchanged) {
  frame_boundary::ConfigureForTest(false, false, "");
  std::vector<Eigen::Matrix<float, 6, 1>> states;
  for (const auto rp : {std::make_pair(0., 0.), std::make_pair(.1, -.08)}) {
    LiDARObjectInfo legacy = Rectangle(Vec(10., 1., 0.), .33);
    legacy.b_KfInitialized = true; legacy.s32_KfInitCount = 2;
    legacy.Eigen_KfState_CA << 10.f, 1.f, 1.f, 0, 0, 0;
    legacy.f32_PrevYaw = .2f;
    const GPS_IMU local = Local(rp.first, rp.second, .3);
    kalman_filter_update(legacy, 10.f, 1.f, 10.f, 1.f, .1, local.st_IMU.f32_Yaw_deg);
    EXPECT_FLOAT_EQ(legacy.f32_OBB_YawRad, .33f);
    EXPECT_NEAR(legacy.f32_PrevYaw, .3, 1e-5);
    states.push_back(legacy.Eigen_KfState_CA);
  }
  EXPECT_LT((states[0] - states[1]).norm(), 1e-6);
}

// RED on TARGET: over 30 scans at +0.02 rad/scan the double rotation accumulates
// (aspect >= 1.2, EMA alpha 0.25: steady error ~ -0.02/0.25 = -0.08 rad, Tracking.cpp:43-44,245-249).
TEST_F(LevelingContract, PublishedYawErrorDoesNotAccumulate) {
  const Vec world_center(112., 202., 30.);
  const double world_yaw = .7;
  std::deque<std::vector<LiDARObjectInfo>> history;
  double worst = 0.;
  for (unsigned frame = 0; frame < 30; ++frame) {
    const double yaw = .02 * frame;
    const GPS_IMU local = Local(.03 * std::sin(.1 * frame), .02 * std::cos(.1 * frame), yaw);
    const std::int64_t ns = 40 * kSec + std::int64_t(frame) * kSec / 10;
    Prepare(ns, local, MapPose(ns, local, true), frame + 1);
    frame_boundary::StoreLeveling(asmc::lidar::leveling_rotation(local.st_IMU.f32_Roll_rad,
        local.st_IMU.f32_Pitch_rad, Rot::Identity(), true, true));
    const double truth = world_yaw - double(local.st_IMU.f32_Yaw_rad);
    std::vector<LiDARObjectInfo> objects{Rectangle(WorldToLeveled(world_center), truth)};
    if (history.empty()) {
      objects[0].s32_Id = 42; objects[0].s32_Age = 1; objects[0].b_KfInitialized = false;
      kalman_filter_update(objects[0], objects[0].f32_CoreX, objects[0].f32_CoreY,
                           objects[0].f32_CoreX, objects[0].f32_CoreY, .1, local.st_IMU.f32_Yaw_deg);
    } else {
      CompareCentroids(history, objects, .1, local);
    }
    ASSERT_FALSE(objects.empty());
    ASSERT_EQ(objects.front().s32_Id, 42);
    frame_boundary::RecordTrackingFrame(objects.front());
    worst = std::max(worst, AxisError(objects.front().f32_OBB_YawRad, truth));
    history.clear(); history.push_back(objects);
  }
  EXPECT_LT(worst, 2e-3) << "worst published yaw error over 30 scans [rad]";
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
