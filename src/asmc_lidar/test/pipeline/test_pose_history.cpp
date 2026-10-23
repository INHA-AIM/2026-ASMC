// PoseHistory: common-path port of Main SensorPoseHistory (8f93c22e).
// Cases from Main test/test_sensor_pose_history.cpp are ported to the ns/common
// API; legacy Lookup/WaitForLookup cases are replaced (see comments).
#include <asmc_lidar/geometry/pose_history.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <thread>

namespace {

using namespace asmc::lidar::pipeline;
namespace al = asmc::lidar;

constexpr std::uint64_t kMs = 1000000ULL;
constexpr std::uint64_t kT0 = 1700000000ULL * 1000000000ULL;  // realistic ROS ns stamp

ImuData Imu(double yaw) {
  ImuData imu{};
  imu.f32_Yaw_rad = yaw;
  imu.b_ImuReady = true;
  return imu;
}

CanData Can(double speed_mps) {
  CanData can{};
  can.f32_CurrentVel_MPS = speed_mps;
  can.f32_CurrentVel_KPH = speed_mps * 3.6;
  return can;
}

GpsData Gps(double east, double north, double up = 0.) {
  GpsData gps{};
  gps.f64_E = east;
  gps.f64_N = north;
  gps.f64_U = up;
  gps.f64_Latitude = 37.24;
  gps.f64_Longitude = 126.77;
  return gps;
}

// Callback-shaped helpers: stamp_ms from the source ns, receipt 2 ms later.
bool AddImuAt(PoseHistory& h, std::uint64_t t, double yaw, std::string* reason = nullptr) {
  return h.AddImu(t / kMs, Imu(yaw), reason, t, t + 2 * kMs);
}
bool AddCanAt(PoseHistory& h, std::uint64_t t, double v, std::string* reason = nullptr) {
  return h.AddCan(t / kMs, Can(v), reason, t, t + 2 * kMs);
}
void AddGpsAt(PoseHistory& h, std::uint64_t t, const GpsData& gps) { h.AddGps(t / kMs, gps, t, t + 2 * kMs); }

// Main FrameBoundary::ScanTime: ros source stamp, steady receipt.
al::Observation ScanTime(std::uint64_t stamp_ns, std::uint64_t receipt_ns) {
  al::Observation t;
  t.stamp_ns = static_cast<al::Ns>(stamp_ns);
  t.receipt_ns = static_cast<al::Ns>(receipt_ns);
  t.processing_ns = static_cast<al::Ns>(receipt_ns);
  t.clock = "ros";
  t.stamp_provenance = "bridge_receive_ns";
  t.receipt_clock = "steady";
  return t;
}

LocalState Local(double yaw, double speed) {
  LocalState local{};
  local.st_IMU = Imu(yaw);
  local.st_CAN = Can(speed);
  return local;
}

double YawOf(const al::Rot& r) { return std::atan2(r(1, 0), r(0, 0)); }

const al::Vec kLever(0.35f, 0.f, 1.25f);

PoseHistoryConfig EnuConfig() {
  PoseHistoryConfig config;
  config.imu_yaw_is_true_enu = true;
  return config;
}

}  // namespace

// ---- Ported from Main test_sensor_pose_history.cpp (common/ns API) ----

// Main UsesPastImuEvenWhenFutureSampleIsBuffered, via WaitForLocalLookupNs.
TEST(PoseHistory, UsesPastImuEvenWhenFutureSampleIsBuffered) {
  PoseHistory h;
  ASSERT_TRUE(AddImuAt(h, kT0 + 1190 * kMs, 0.1));
  ASSERT_TRUE(AddImuAt(h, kT0 + 1210 * kMs, 0.11));
  ASSERT_TRUE(AddCanAt(h, kT0 + 1190 * kMs, 0.0));
  LocalState local{};
  LocalInputSelection sel;
  std::string reason;
  ASSERT_TRUE(h.WaitForLocalLookupNs(kT0 + 1200 * kMs, 0., &local, &sel, &reason));
  EXPECT_EQ("local_ready", reason);
  EXPECT_NEAR(0.1, local.st_IMU.f32_Yaw_rad, 1e-6);
  EXPECT_EQ(kT0 + 1190 * kMs, sel.imu_stamp_ns);
  EXPECT_EQ(kT0 + 1192 * kMs, sel.imu_receipt_ns);
  EXPECT_EQ(kT0 + 1190 * kMs, sel.can_stamp_ns);
}

// Main RejectsAlternatingImuYawSpike.
TEST(PoseHistory, RejectsAlternatingImuYawSpike) {
  PoseHistory h;
  std::string reason;
  EXPECT_TRUE(AddImuAt(h, kT0 + 1000 * kMs, -1.53, &reason));
  EXPECT_FALSE(AddImuAt(h, kT0 + 1010 * kMs, -2.55, &reason));
  EXPECT_EQ("implausible IMU yaw jump", reason);
  EXPECT_TRUE(AddImuAt(h, kT0 + 1020 * kMs, -1.54, &reason));
}

// Main RejectsAlternatingVehicleSpeedSpike.
TEST(PoseHistory, RejectsAlternatingVehicleSpeedSpike) {
  PoseHistory h;
  std::string reason;
  EXPECT_TRUE(AddCanAt(h, kT0 + 1000 * kMs, 10.0, &reason));
  EXPECT_FALSE(AddCanAt(h, kT0 + 1001 * kMs, 0.0, &reason));
  EXPECT_EQ("implausible vehicle-speed jump", reason);
  EXPECT_TRUE(AddCanAt(h, kT0 + 1020 * kMs, 10.1, &reason));
}

// Main WaitsForCallbackOrderJitterAtSameSourceTime (threaded, ns API).
TEST(PoseHistory, WaitReturnsOnceSatisfyingSampleArrives) {
  PoseHistory h;
  const auto scan = kT0 + 1250 * kMs;
  std::thread delayed([&h, scan]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    AddImuAt(h, scan, 0.0);
    AddCanAt(h, scan, 1.0);
  });
  LocalState local{};
  LocalInputSelection sel;
  std::string reason;
  const auto start = std::chrono::steady_clock::now();
  EXPECT_TRUE(h.WaitForLocalLookupNs(scan, 1.0, &local, &sel, &reason));
  const double waited = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  EXPECT_EQ("local_ready", reason);
  EXPECT_LT(waited, 0.9);  // woke on the sample, not on the deadline
  EXPECT_EQ(scan, sel.imu_stamp_ns);
  delayed.join();
}

// Main WaitTimeoutStillRejectsMissingPose (ns API).
TEST(PoseHistory, WaitTimeoutStillRejectsMissingPose) {
  PoseHistory h;
  LocalState local{};
  std::string reason;
  const auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(h.WaitForLocalLookupNs(kT0 + 1200 * kMs, 0.02, &local, nullptr, &reason));
  const double waited = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  EXPECT_EQ("no IMU at or before LiDAR scan", reason);
  EXPECT_GE(waited, 0.019);
}

// Main TrueEnuHeadingAndExtrapolationUseUtmGrid, rewritten for MapAt: the map
// rotation uses the grid yaw (K-City, ~-0.02356 rad), the local lookup keeps the
// raw ENU yaw, and an out-of-zone GPS sample is dropped (no barrier).
TEST(PoseHistory, TrueEnuHeadingAndPredictionUseUtmGrid) {
  PoseHistory h(EnuConfig());
  ASSERT_TRUE(AddImuAt(h, kT0 + 880 * kMs, M_PI / 2.));  // IMU before the first GPS
  ASSERT_TRUE(AddCanAt(h, kT0 + 880 * kMs, 10.));
  // Attitude ages 20/60/110 ms (an exact 120 ms age is stale, see below).
  for (const int ms : {900, 940, 990}) AddGpsAt(h, kT0 + ms * kMs, Gps(10., 20.));
  ASSERT_TRUE(AddImuAt(h, kT0 + 1000 * kMs, M_PI / 2.));
  ASSERT_TRUE(AddCanAt(h, kT0 + 1000 * kMs, 10.));
  const auto scan = kT0 + 1100 * kMs;
  LocalState local{};
  ASSERT_TRUE(h.WaitForLocalLookupNs(scan, 0., &local, nullptr, nullptr));
  EXPECT_NEAR(M_PI / 2., local.st_IMU.f32_Yaw_rad, 1e-6);  // raw ENU yaw locally

  const auto pose = h.MapAt(ScanTime(scan, scan + 5 * kMs), local);
  ASSERT_TRUE(pose.valid) << pose.reason;
  EXPECT_EQ("gps_bounded_prediction", pose.reason);
  const double expected_yaw = M_PI / 2. - 0.02356;  // value of the Main test
  EXPECT_NEAR(expected_yaw, YawOf(pose.map_from_base.linear()), 0.0001);
  const al::Rot r = pose.map_from_base.linear();
  const al::Vec expected = al::Vec(10., 20., 0.) + r * al::Vec(10., 0., 0.) * 0.11 - r * kLever;
  EXPECT_NEAR(expected.x(), pose.map_from_base.translation().x(), 1e-6);
  EXPECT_NEAR(expected.y(), pose.map_from_base.translation().y(), 1e-6);
  EXPECT_NEAR(expected.z(), pose.map_from_base.translation().z(), 1e-6);

  auto invalid = Gps(1e6, 20.);
  invalid.f64_Latitude = invalid.f64_Longitude = 0.;
  AddGpsAt(h, kT0 + 1050 * kMs, invalid);
  const auto again = h.MapAt(ScanTime(scan, scan + 5 * kMs), local);
  ASSERT_TRUE(again.valid) << again.reason;
  EXPECT_LT(again.map_from_base.translation().x(), 11.);
}

// Main ExtrapolatesGpsToLidarTimestamp / RejectsStaleGpsInsteadOfReusingOldPose /
// RejectsImpossibleGpsHistorySpeed exercise the legacy GPS-gated Lookup, which
// is not ported. Their common-path counterparts are the MapAt tests below
// (CAN-speed prediction, gps_stale, map_speed_invalid).

// ---- Local lookup: staleness, barriers, causality ----

// Main compares double(age_ns)*1e-9 > 0.12: 120 ms exactly rounds to
// 0.12000000000000001 and is stale (preserved quirk); 119.999999 ms is fresh.
TEST(PoseHistory, ImuStalenessBoundary120ms) {
  PoseHistory h;
  ASSERT_TRUE(AddImuAt(h, kT0, 0.));
  ASSERT_TRUE(AddCanAt(h, kT0 + 100 * kMs, 0.));
  LocalState local{};
  std::string reason;
  EXPECT_TRUE(h.WaitForLocalLookupNs(kT0 + 120 * kMs - 1, 0., &local, nullptr, &reason)) << reason;
  EXPECT_FALSE(h.WaitForLocalLookupNs(kT0 + 120 * kMs, 0., &local, nullptr, &reason));
  EXPECT_EQ("IMU is stale for LiDAR scan", reason);
  EXPECT_EQ(double(120 * kMs) * 1e-9 > 0.12, true);
  EXPECT_FALSE(h.WaitForLocalLookupNs(kT0 + 120 * kMs + 1, 0., &local, nullptr, &reason));
  EXPECT_EQ("IMU is stale for LiDAR scan", reason);
}

TEST(PoseHistory, CanStalenessBoundary120ms) {
  PoseHistory h;
  ASSERT_TRUE(AddCanAt(h, kT0, 0.));
  ASSERT_TRUE(AddImuAt(h, kT0 + 100 * kMs, 0.));
  LocalState local{};
  std::string reason;
  EXPECT_TRUE(h.WaitForLocalLookupNs(kT0 + 120 * kMs - 1, 0., &local, nullptr, &reason)) << reason;
  EXPECT_FALSE(h.WaitForLocalLookupNs(kT0 + 120 * kMs, 0., &local, nullptr, &reason));
  EXPECT_EQ("vehicle speed is stale for LiDAR scan", reason);
}

TEST(PoseHistory, MissingInputsReportMainReasons) {
  PoseHistory h;
  LocalState local{};
  std::string reason;
  EXPECT_FALSE(h.WaitForLocalLookupNs(0, 0., &local, nullptr, &reason));
  EXPECT_EQ("invalid local timestamp", reason);
  EXPECT_FALSE(h.WaitForLocalLookupNs(kT0, 0., nullptr, nullptr, &reason));
  EXPECT_EQ("invalid local timestamp", reason);
  ASSERT_TRUE(AddImuAt(h, kT0, 0.));
  EXPECT_FALSE(h.WaitForLocalLookupNs(kT0, 0., &local, nullptr, &reason));
  EXPECT_EQ("no vehicle speed at or before LiDAR scan", reason);
}

TEST(PoseHistory, NoFutureSampleIsUsed) {
  PoseHistory h;
  ASSERT_TRUE(AddImuAt(h, kT0 + 10 * kMs, 0.3));
  ASSERT_TRUE(AddCanAt(h, kT0 + 10 * kMs, 1.));
  LocalState local{};
  local.st_GPS.f64_E = 42.;
  std::string reason;
  EXPECT_FALSE(h.WaitForLocalLookupNs(kT0 + 10 * kMs - 1, 0., &local, nullptr, &reason));
  EXPECT_EQ("no IMU at or before LiDAR scan", reason);
  ASSERT_TRUE(h.WaitForLocalLookupNs(kT0 + 10 * kMs, 0., &local, nullptr, &reason));  // equal stamp is used
  EXPECT_NEAR(0.3, local.st_IMU.f32_Yaw_rad, 1e-6);
  EXPECT_EQ(42., local.st_GPS.f64_E);  // only st_IMU/st_CAN are written
}

TEST(PoseHistory, NewerInvalidImuBlocksFallback) {
  PoseHistory h;
  ASSERT_TRUE(AddImuAt(h, kT0, 0.));
  ASSERT_TRUE(AddCanAt(h, kT0, 0.));
  ImuData bad = Imu(0.);
  bad.b_ImuReady = false;
  std::string reason;
  EXPECT_FALSE(h.AddImu((kT0 + 10 * kMs) / kMs, bad, &reason, kT0 + 10 * kMs, kT0 + 12 * kMs));
  EXPECT_EQ("invalid IMU sample", reason);
  LocalState local{};
  EXPECT_TRUE(h.WaitForLocalLookupNs(kT0 + 5 * kMs, 0., &local, nullptr, &reason)) << reason;  // barrier is newer
  EXPECT_FALSE(h.WaitForLocalLookupNs(kT0 + 10 * kMs, 0., &local, nullptr, &reason));
  EXPECT_EQ("imu_input_invalid", reason);
  EXPECT_FALSE(h.WaitForLocalLookupNs(kT0 + 20 * kMs, 0., &local, nullptr, &reason));
  EXPECT_EQ("imu_input_invalid", reason);
  ASSERT_TRUE(AddImuAt(h, kT0 + 30 * kMs, 0.));
  LocalInputSelection sel;
  EXPECT_TRUE(h.WaitForLocalLookupNs(kT0 + 40 * kMs, 0., &local, &sel, &reason)) << reason;
  EXPECT_EQ(kT0 + 30 * kMs, sel.imu_stamp_ns);
}

TEST(PoseHistory, RejectedYawJumpIsABarrier) {
  PoseHistory h;
  ASSERT_TRUE(AddImuAt(h, kT0, 0.));
  ASSERT_TRUE(AddCanAt(h, kT0, 0.));
  EXPECT_FALSE(AddImuAt(h, kT0 + 10 * kMs, 1.0));
  LocalState local{};
  std::string reason;
  EXPECT_FALSE(h.WaitForLocalLookupNs(kT0 + 20 * kMs, 0., &local, nullptr, &reason));
  EXPECT_EQ("imu_input_invalid", reason);
}

// Zero header stamp: CanFromInput gives NaN, stamp_ms falls back to ROS now and
// the barrier is raised at now_ms * 1e6 (source ns is 0), like Main.
TEST(PoseHistory, NewerInvalidCanBlocksFallbackIncludingZeroStamp) {
  PoseHistory h;
  ASSERT_TRUE(AddImuAt(h, kT0, 0.));
  ASSERT_TRUE(AddCanAt(h, kT0, 5.));
  CanInput zero{};
  zero.vx = 5.;
  const auto now = kT0 + 15 * kMs + 123;
  std::string reason;
  EXPECT_FALSE(h.AddCan(SourceStampMs(zero.stamp_ns, now), CanFromInput(zero), &reason, zero.stamp_ns, now));
  EXPECT_EQ("invalid vehicle-speed sample", reason);
  LocalState local{};
  EXPECT_TRUE(h.WaitForLocalLookupNs(kT0 + 15 * kMs - 1, 0., &local, nullptr, &reason)) << reason;
  EXPECT_FALSE(h.WaitForLocalLookupNs(kT0 + 15 * kMs, 0., &local, nullptr, &reason));
  EXPECT_EQ("speed_input_invalid", reason);
  EXPECT_FALSE(AddCanAt(h, kT0 + 20 * kMs, -1., &reason));
  EXPECT_EQ("invalid vehicle-speed sample", reason);
}

TEST(PoseHistory, WaitIgnoresSamplesThatCannotSatisfyAndTimesOut) {
  PoseHistory h;
  ASSERT_TRUE(AddCanAt(h, kT0, 0.));
  const auto scan = kT0 + 100 * kMs;
  std::thread future([&h, scan]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    AddImuAt(h, scan + 1, 0.);  // newer than the scan: wakes the waiter, never used
  });
  LocalState local{};
  std::string reason;
  const auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(h.WaitForLocalLookupNs(scan, 0.05, &local, nullptr, &reason));
  const double waited = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  EXPECT_EQ("no IMU at or before LiDAR scan", reason);
  EXPECT_GE(waited, 0.049);
  future.join();
}

TEST(PoseHistory, ZeroTimeoutDoesNotWait) {
  PoseHistory h;
  LocalState local{};
  std::string reason;
  const auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(h.WaitForLocalLookupNs(kT0, 0., &local, nullptr, &reason));
  EXPECT_LT(std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count(), 0.01);
}

TEST(PoseHistory, StampRegressionsAndReanchor) {
  PoseHistory h;
  std::string reason;
  ASSERT_TRUE(AddImuAt(h, kT0 + 1000 * kMs, 0.));
  EXPECT_FALSE(AddImuAt(h, kT0 + 999 * kMs, 0., &reason));
  EXPECT_EQ("IMU timestamp moved backwards", reason);
  // 2 s without samples: the yaw-jump allowance is skipped.
  EXPECT_TRUE(AddImuAt(h, kT0 + 3000 * kMs, 3.0, &reason)) << reason;
  ASSERT_TRUE(AddCanAt(h, kT0 + 1000 * kMs, 0.));
  EXPECT_FALSE(AddCanAt(h, kT0 + 999 * kMs, 0., &reason));
  EXPECT_EQ("vehicle-speed timestamp moved backwards", reason);
  EXPECT_TRUE(AddCanAt(h, kT0 + 3000 * kMs, 30., &reason)) << reason;
  // Allowance grows with time since the last accepted sample: 5 deg + 120 deg/s.
  EXPECT_FALSE(AddImuAt(h, kT0 + 3100 * kMs, 3.0 - 0.0872664626 - 2.0943951024 * 0.1 - 0.01, &reason));
  EXPECT_TRUE(AddImuAt(h, kT0 + 3200 * kMs, 3.0 - 0.0872664626 - 2.0943951024 * 0.2 + 0.01, &reason)) << reason;
}

TEST(PoseHistory, SubMillisecondSamplesAreOrderedBySourceStamp) {
  PoseHistory h;
  const auto t = kT0 + 1000 * kMs;
  ASSERT_TRUE(h.AddImu(t / kMs, Imu(0.02), nullptr, t + 600000, t + kMs));
  ASSERT_TRUE(h.AddImu(t / kMs, Imu(0.01), nullptr, t + 200000, t + kMs));  // same ms, earlier source
  ASSERT_TRUE(AddCanAt(h, t, 0.));
  LocalState local{};
  LocalInputSelection sel;
  ASSERT_TRUE(h.WaitForLocalLookupNs(t + 400000, 0., &local, &sel, nullptr));
  EXPECT_EQ(t + 200000, sel.imu_stamp_ns);
  EXPECT_NEAR(0.01, local.st_IMU.f32_Yaw_rad, 1e-7);
  // An equal source stamp replaces the stored sample.
  ASSERT_TRUE(h.AddImu(t / kMs, Imu(0.015), nullptr, t + 200000, t + kMs));
  ASSERT_TRUE(h.WaitForLocalLookupNs(t + 400000, 0., &local, &sel, nullptr));
  EXPECT_NEAR(0.015, local.st_IMU.f32_Yaw_rad, 1e-7);
}

TEST(PoseHistory, PrunesHistoryOlderThanFiveSecondsKeepingTwo) {
  PoseHistory h;
  ASSERT_TRUE(AddImuAt(h, kT0 + 1000 * kMs, 0.));
  ASSERT_TRUE(AddImuAt(h, kT0 + 1100 * kMs, 0.));
  ASSERT_TRUE(AddCanAt(h, kT0 + 1000 * kMs, 0.));
  LocalState local{};
  std::string reason;
  ASSERT_TRUE(h.WaitForLocalLookupNs(kT0 + 1050 * kMs, 0., &local, nullptr, &reason));
  ASSERT_TRUE(AddImuAt(h, kT0 + 6200 * kMs, 0.));  // 1000 + 5000 < 6200 -> front pruned
  EXPECT_FALSE(h.WaitForLocalLookupNs(kT0 + 1050 * kMs, 0., &local, nullptr, &reason));
  EXPECT_EQ("no IMU at or before LiDAR scan", reason);
}

TEST(PoseHistory, ConfigureClearsHistoryAndBarriers) {
  PoseHistory h;
  ASSERT_TRUE(AddImuAt(h, kT0, 0.));
  ASSERT_TRUE(AddCanAt(h, kT0, 0.));
  h.Configure(PoseHistoryConfig{});
  LocalState local{};
  std::string reason;
  EXPECT_FALSE(h.WaitForLocalLookupNs(kT0, 0., &local, nullptr, &reason));
  EXPECT_EQ("no IMU at or before LiDAR scan", reason);
}

// ---- Bounded map pose ----

TEST(PoseHistory, MapPoseRecoveryNeedsThreeConsecutiveNormalSamples) {
  PoseHistory h;
  ASSERT_TRUE(AddImuAt(h, kT0, 0.));
  const auto local = Local(0., 0.);
  AddGpsAt(h, kT0 + 10 * kMs, Gps(1., 2., 3.));
  EXPECT_EQ("gps_recovering", h.MapAt(ScanTime(kT0 + 10 * kMs, kT0 + 20 * kMs), local).reason);
  AddGpsAt(h, kT0 + 60 * kMs, Gps(1., 2., 3.));
  EXPECT_EQ("gps_recovering", h.MapAt(ScanTime(kT0 + 60 * kMs, kT0 + 70 * kMs), local).reason);
  AddGpsAt(h, kT0 + 110 * kMs, Gps(1., 2., 3.));
  const auto pose = h.MapAt(ScanTime(kT0 + 110 * kMs, kT0 + 120 * kMs), local);
  ASSERT_TRUE(pose.valid) << pose.reason;
  EXPECT_EQ("gps_observed", pose.reason);
  // Lever arm removed with identity attitude (float lever like st_VehicleSpec).
  EXPECT_DOUBLE_EQ(1. - double(0.35f), pose.map_from_base.translation().x());
  EXPECT_DOUBLE_EQ(2., pose.map_from_base.translation().y());
  EXPECT_DOUBLE_EQ(3. - double(1.25f), pose.map_from_base.translation().z());
}

// Feed three good GPS samples at 0/50/100 ms (IMU yaw yaw_gps before them).
void ThreeGood(PoseHistory& h, double yaw_gps = 0.) {
  ASSERT_TRUE(AddImuAt(h, kT0, yaw_gps));
  for (int i = 0; i < 3; ++i) AddGpsAt(h, kT0 + 50 * i * kMs, Gps(100., 200., 5.));
}

TEST(PoseHistory, PredictionUsesCanSpeedOnlyUpTo300ms) {
  PoseHistory h;
  ThreeGood(h);
  const auto local = Local(0., 10.);
  const auto last = kT0 + 100 * kMs;
  const auto at300 = h.MapAt(ScanTime(last + 300 * kMs, last + 301 * kMs), local);
  ASSERT_TRUE(at300.valid) << at300.reason;
  EXPECT_EQ("gps_bounded_prediction", at300.reason);
  EXPECT_NEAR(100. + 10. * 0.3 - double(0.35f), at300.map_from_base.translation().x(), 1e-9);
  EXPECT_NEAR(200., at300.map_from_base.translation().y(), 1e-9);
  // No reuse of the last GPS beyond 300 ms (integer ns comparison).
  EXPECT_EQ("gps_stale", h.MapAt(ScanTime(last + 300 * kMs + 1, last + 301 * kMs), local).reason);
}

TEST(PoseHistory, BoundedWindowFollowsGpsMaxAge) {
  PoseHistoryConfig config;
  config.gps_max_age_s = 0.1;
  PoseHistory h(config);
  ThreeGood(h);
  const auto last = kT0 + 100 * kMs;
  const auto local = Local(0., 1.);
  EXPECT_TRUE(h.MapAt(ScanTime(last + 100 * kMs, last + 101 * kMs), local).valid);
  EXPECT_EQ("gps_stale", h.MapAt(ScanTime(last + 100 * kMs + 1, last + 101 * kMs), local).reason);
}

// The lever arm is removed with the attitude of the GPS observation, and the
// scan attitude rotates the CAN prediction.
TEST(PoseHistory, LeverArmRemovedWithGpsTimeAttitude) {
  PoseHistory h;
  ThreeGood(h, M_PI / 2.);
  const auto last = kT0 + 100 * kMs;
  const auto pose = h.MapAt(ScanTime(last + 100 * kMs, last + 101 * kMs), Local(0., 10.));
  ASSERT_TRUE(pose.valid) << pose.reason;
  EXPECT_NEAR(0., YawOf(pose.map_from_base.linear()), 1e-12);  // scan attitude
  const al::Rot r_gps = al::rpy(0., 0., float(M_PI / 2.));
  const al::Vec expected = al::Vec(100., 200., 5.) + al::Vec(10. * 0.1, 0., 0.) - r_gps * kLever;
  EXPECT_NEAR(expected.x(), pose.map_from_base.translation().x(), 1e-9);
  EXPECT_NEAR(expected.y(), pose.map_from_base.translation().y(), 1e-9);  // 200 - 0.35
  EXPECT_NEAR(expected.z(), pose.map_from_base.translation().z(), 1e-9);
}

TEST(PoseHistory, LeverArmComesFromConfig) {
  PoseHistoryConfig config;
  config.gps_in_base = Point3D{1.f, 0.5f, 2.f};
  PoseHistory h(config);
  ThreeGood(h);
  const auto pose = h.MapAt(ScanTime(kT0 + 100 * kMs, kT0 + 110 * kMs), Local(0., 0.));
  ASSERT_TRUE(pose.valid) << pose.reason;
  EXPECT_DOUBLE_EQ(99., pose.map_from_base.translation().x());
  EXPECT_DOUBLE_EQ(199.5, pose.map_from_base.translation().y());
  EXPECT_DOUBLE_EQ(3., pose.map_from_base.translation().z());
}

TEST(PoseHistory, GpsWithoutFreshAttitudeIsInvalid) {
  {  // no IMU at or before the GPS stamp
    PoseHistory h;
    for (int i = 0; i < 3; ++i) AddGpsAt(h, kT0 + 50 * i * kMs, Gps(1., 2.));
    EXPECT_EQ("gps_invalid", h.MapAt(ScanTime(kT0 + 100 * kMs, kT0 + 110 * kMs), Local(0., 0.)).reason);
  }
  {  // IMU older than imu_max_age_s
    PoseHistory h;
    ASSERT_TRUE(AddImuAt(h, kT0, 0.));
    for (int i = 0; i < 3; ++i) AddGpsAt(h, kT0 + (100 + 50 * i) * kMs, Gps(1., 2.));
    EXPECT_EQ("gps_invalid", h.MapAt(ScanTime(kT0 + 200 * kMs, kT0 + 210 * kMs), Local(0., 0.)).reason);
  }
  {  // invalid IMU barrier between the attitude and the GPS stamp
    PoseHistory h;
    ASSERT_TRUE(AddImuAt(h, kT0, 0.));
    ImuData bad = Imu(0.);
    bad.b_ImuReady = false;
    h.AddImu((kT0 + 10 * kMs) / kMs, bad, nullptr, kT0 + 10 * kMs, kT0 + 12 * kMs);
    for (int i = 0; i < 3; ++i) AddGpsAt(h, kT0 + (20 + 10 * i) * kMs, Gps(1., 2.));
    EXPECT_EQ("gps_invalid", h.MapAt(ScanTime(kT0 + 40 * kMs, kT0 + 50 * kMs), Local(0., 0.)).reason);
  }
}

TEST(PoseHistory, InvalidGpsBarriersAndRecovery) {
  PoseHistory h;
  ThreeGood(h);
  const auto local = Local(0., 0.);
  h.AddGpsInvalidAt(kT0 + 150 * kMs, kT0 + 152 * kMs);
  EXPECT_EQ("gps_invalid", h.MapAt(ScanTime(kT0 + 160 * kMs, kT0 + 170 * kMs), local).reason);
  EXPECT_TRUE(h.MapAt(ScanTime(kT0 + 140 * kMs, kT0 + 145 * kMs), local).valid);  // before arrival
  ASSERT_TRUE(AddImuAt(h, kT0 + 190 * kMs, 0.));
  AddGpsAt(h, kT0 + 200 * kMs, Gps(100., 200., 5.));
  AddGpsAt(h, kT0 + 250 * kMs, Gps(100., 200., 5.));
  EXPECT_EQ("gps_recovering", h.MapAt(ScanTime(kT0 + 250 * kMs, kT0 + 260 * kMs), local).reason);
  AddGpsAt(h, kT0 + 280 * kMs, Gps(100., 200., 5.));
  EXPECT_TRUE(h.MapAt(ScanTime(kT0 + 280 * kMs, kT0 + 290 * kMs), local).valid);
  // Unknown-stamp barrier (source ns 0): later scans are invalid until 3 new
  // consecutive samples arrive after it.
  h.AddGpsInvalidAt(0, kT0 + 295 * kMs);
  EXPECT_EQ("gps_invalid", h.MapAt(ScanTime(kT0 + 300 * kMs, kT0 + 310 * kMs), local).reason);
}

TEST(PoseHistory, GpsSilenceOver300msIsStaleAndRestartsRecovery) {
  PoseHistory h;
  ThreeGood(h);
  const auto local = Local(0., 0.);
  EXPECT_EQ("gps_stale", h.MapAt(ScanTime(kT0 + 401 * kMs, kT0 + 402 * kMs), local).reason);
  ASSERT_TRUE(AddImuAt(h, kT0 + 450 * kMs, 0.));
  AddGpsAt(h, kT0 + 500 * kMs, Gps(100., 200., 5.));  // gap 400 ms > 300 ms resets the count
  EXPECT_EQ("gps_recovering", h.MapAt(ScanTime(kT0 + 500 * kMs, kT0 + 510 * kMs), local).reason);
}

TEST(PoseHistory, LateGpsArrivalCannotEnableEarlierScan) {
  PoseHistory h;
  ThreeGood(h);
  // receipt of the third GPS is kT0 + 102 ms; a scan received before that sees two.
  EXPECT_EQ("gps_recovering", h.MapAt(ScanTime(kT0 + 100 * kMs, kT0 + 101 * kMs), Local(0., 0.)).reason);
}

TEST(PoseHistory, MapSpeedInvalid) {
  PoseHistory h;
  ThreeGood(h);
  const auto t = ScanTime(kT0 + 100 * kMs, kT0 + 110 * kMs);
  EXPECT_EQ("map_speed_invalid", h.MapAt(t, Local(0., 35.5)).reason);
  EXPECT_EQ("map_speed_invalid", h.MapAt(t, Local(0., std::numeric_limits<double>::quiet_NaN())).reason);
  EXPECT_TRUE(h.MapAt(t, Local(0., 35.)).valid);
}

TEST(PoseHistory, TrueEnuMapNeedsGpsForGridYaw) {
  PoseHistory h(EnuConfig());
  ASSERT_TRUE(AddImuAt(h, kT0, 0.));
  EXPECT_EQ("gps_missing", h.MapAt(ScanTime(kT0 + 10 * kMs, kT0 + 11 * kMs), Local(0., 0.)).reason);
  auto jammed = Gps(1., 2.);
  jammed.b_isJamming = true;  // dropped under true ENU, no barrier
  AddGpsAt(h, kT0 + 5 * kMs, jammed);
  EXPECT_EQ("gps_missing", h.MapAt(ScanTime(kT0 + 10 * kMs, kT0 + 11 * kMs), Local(0., 0.)).reason);
}

// GridImu at K-City: grid yaw = true yaw - 0.02356 rad (Main test value).
TEST(PoseHistory, GridImuMeridianConvergenceAtKCity) {
  auto imu = Imu(M_PI / 2.);
  imu.f32_Roll_rad = 0.01f;
  imu.f32_Pitch_rad = -0.02f;
  const auto grid = GridImu(imu, Gps(0., 0.), true);
  const double offset = double(grid.f32_Yaw_rad) - double(imu.f32_Yaw_rad);
  EXPECT_LT(offset, 0.);
  EXPECT_NEAR(-0.02356, offset, 0.0001);
  EXPECT_FLOAT_EQ(float(grid.f32_Yaw_rad * 180. / M_PI), grid.f32_Yaw_deg);
  EXPECT_EQ(imu.f32_Roll_rad, grid.f32_Roll_rad);
  EXPECT_EQ(imu.f32_Pitch_rad, grid.f32_Pitch_rad);
  const auto same = GridImu(imu, Gps(0., 0.), false);
  EXPECT_EQ(imu.f32_Yaw_rad, same.f32_Yaw_rad);
  // remainder() wraps to (-pi, pi].
  EXPECT_NEAR(M_PI - 0.02356 + 0.5 - 2. * M_PI, GridImu(Imu(M_PI + 0.5 - 1e-7), Gps(0., 0.), true).f32_Yaw_rad,
              2e-4);
}

TEST(PoseHistory, SourceStampMsUsesRosNowOnlyForZeroHeader) {
  EXPECT_EQ(1700000000123ULL, SourceStampMs(kT0 + 123999999ULL, 5));
  EXPECT_EQ(42ULL, SourceStampMs(0, 42999999ULL));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
