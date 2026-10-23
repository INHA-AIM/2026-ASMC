// GpsSanityFilter / GpsIngest / ImuFromInput / CanFromInput: common-path port of
// Main GpsSanityFilter.cpp and the Parser.cpp callbacks (8f93c22e).
#include <asmc_lidar/geometry/pose_history.hpp>

#include <GeographicLib/UTMUPS.hpp>
#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace {

using namespace asmc::lidar::pipeline;
namespace al = asmc::lidar;

constexpr std::uint64_t kMs = 1000000ULL;
constexpr std::uint64_t kT0 = 1700000000ULL * 1000000000ULL;
constexpr double kOriginE = 302595.0, kOriginN = 4124145.0;  // HD-map origin (UTM 52N)
const double kNaN = std::numeric_limits<double>::quiet_NaN();

GpsIngestConfig IngestConfig(double altitude_origin = 0.) {
  GpsIngestConfig config;
  config.map_origin = al::Vec(kOriginE, kOriginN, altitude_origin);
  config.map_altitude_origin_m = altitude_origin;
  return config;
}

// MORAI GPSMessage for local map (x, y): lat/lon from UTM 52N, offsets = origin.
GpsInput Msg(double x, double y, std::uint64_t stamp_ns, double altitude = 10.) {
  GpsInput in;
  GeographicLib::UTMUPS::Reverse(52, true, kOriginE + x, kOriginN + y, in.latitude, in.longitude);
  in.altitude = altitude;
  in.east_offset = kOriginE;
  in.north_offset = kOriginN;
  in.status = 1;
  in.stamp_ns = stamp_ns;
  return in;
}

al::Observation ScanTime(std::uint64_t stamp_ns, std::uint64_t receipt_ns) {
  al::Observation t;
  t.stamp_ns = static_cast<al::Ns>(stamp_ns);
  t.receipt_ns = t.processing_ns = static_cast<al::Ns>(receipt_ns);
  t.clock = "ros";
  t.stamp_provenance = "bridge_receive_ns";
  t.receipt_clock = "steady";
  return t;
}

LocalState Local() {
  LocalState local{};
  local.st_IMU.b_ImuReady = true;
  return local;
}

PoseHistoryConfig LaunchPose() {
  PoseHistoryConfig config;
  config.imu_yaw_is_true_enu = true;  // main.launch
  return config;
}

// IMU before every GPS so the bounded samples carry a fresh attitude.
void FeedImu(PoseHistory& h, std::uint64_t t) {
  ImuInput imu;
  imu.stamp_ns = t;
  ASSERT_TRUE(h.AddImu(SourceStampMs(t, 0), ImuFromInput(imu), nullptr, t, t + kMs));
}

void Feed(GpsIngest& ingest, PoseHistory& h, const GpsInput& msg) {
  GpsIngest::Apply(ingest.Evaluate(msg, msg.stamp_ns + kMs), msg.stamp_ns + 2 * kMs, h);
}

}  // namespace

// ---- Ported from Main test_gps_sanity_filter.cpp ----

TEST(GpsSanityFilter, RejectsZeroCoordinateProjectionOutsideMap) {
  GpsSanityFilter filter;
  EXPECT_FALSE(filter.Evaluate(-136000.0, -4124000.0, 10.0).accepted);
  EXPECT_TRUE(filter.Evaluate(-120.0, -410.0, 10.1).accepted);
}

TEST(GpsSanityFilter, RejectsAlternatingHundredMeterJump) {
  GpsSanityFilter filter;
  EXPECT_TRUE(filter.Evaluate(-60.0, 100.0, 10.0).accepted);
  const auto bad = filter.Evaluate(120.0, 100.0, 10.1);
  EXPECT_FALSE(bad.accepted);
  EXPECT_EQ("implausible position jump", bad.reason);
  EXPECT_TRUE(filter.Evaluate(-58.0, 100.0, 10.2).accepted);
}

TEST(GpsSanityFilter, AllowsFastButPlausibleMotion) {
  GpsSanityFilter filter;
  EXPECT_TRUE(filter.Evaluate(-100.0, -400.0, 10.0).accepted);
  EXPECT_TRUE(filter.Evaluate(-88.0, -400.0, 10.2).accepted);
}

TEST(GpsSanityFilter, ReanchorsAfterSustainedGapForRespawn) {
  GpsSanityFilter filter;
  EXPECT_TRUE(filter.Evaluate(-100.0, -400.0, 10.0).accepted);
  EXPECT_FALSE(filter.Evaluate(100.0, 300.0, 10.2).accepted);
  EXPECT_TRUE(filter.Evaluate(100.0, 300.0, 12.1).accepted);
}

// ---- Additional sanity-gate rules ----

TEST(GpsSanityFilter, ReasonsJumpAndStampRules) {
  GpsSanityFilter filter;
  EXPECT_EQ("non-finite coordinate or stamp", filter.Evaluate(kNaN, 0., 10.).reason);
  EXPECT_EQ("non-finite coordinate or stamp", filter.Evaluate(0., 0., 0.).reason);
  EXPECT_EQ("outside configured local-map bounds", filter.Evaluate(200., 0., 10.).reason);
  EXPECT_EQ("initial valid sample", filter.Evaluate(0., 0., 10.).reason);
  const auto cont = filter.Evaluate(1.2, 1.6, 9.9995);  // -0.5 ms is tolerated, elapsed clamped to 0
  EXPECT_TRUE(cont.accepted);
  EXPECT_EQ("continuous sample", cont.reason);
  EXPECT_NEAR(2., cont.jump_m, 1e-12);
  EXPECT_DOUBLE_EQ(3., cont.allowed_jump_m);
  const auto back = filter.Evaluate(1.2, 1.6, 9.9995 - 0.0011);
  EXPECT_FALSE(back.accepted);
  EXPECT_EQ("source timestamp moved backwards", back.reason);
  // Rejected samples do not move the anchor: the gap is measured from 9.9995.
  EXPECT_FALSE(filter.Evaluate(-300., -700., 11.).accepted);
  const auto re = filter.Evaluate(-300., -700., 12.);
  EXPECT_TRUE(re.accepted);
  EXPECT_EQ("reanchored after input gap", re.reason);
  filter.Reset();
  EXPECT_EQ("initial valid sample", filter.Evaluate(100., 100., 1.).reason);
}

// ---- IMU / CAN conversion (Parser.cpp callbacks, common branch) ----

TEST(ImuFromInput, EulerAnglesAndReadyRule) {
  ImuInput in;
  in.stamp_ns = kT0;
  in.qz = std::sin(0.25);
  in.qw = std::cos(0.25);
  in.ax = 1.5;
  in.wz = 0.25;
  const auto imu = ImuFromInput(in);
  EXPECT_TRUE(imu.b_ImuReady);
  EXPECT_NEAR(0.5, imu.f32_Yaw_rad, 1e-6);
  EXPECT_FLOAT_EQ(0.f, imu.f32_Roll_rad);
  EXPECT_FLOAT_EQ(0.f, imu.f32_Pitch_rad);
  EXPECT_FLOAT_EQ(1.5f, imu.f32_AccelX);
  EXPECT_FLOAT_EQ(0.25f, imu.f32_AngularVelZ);
  // Main: float products (IMU_DATA fields) promoted by the double 2.0 factor.
  const float qz = float(in.qz), qw = float(in.qw);
  const float yaw = std::atan2(2.0 * (qw * qz + 0.f * 0.f), 1.0 - 2.0 * (0.f * 0.f + qz * qz));
  EXPECT_EQ(yaw, imu.f32_Yaw_rad);
  EXPECT_EQ(float(yaw * 180.0f / M_PI), imu.f32_Yaw_deg);  // Main float Rad2Deg

  in.stamp_ns = 0;  // zero header stamp
  EXPECT_FALSE(ImuFromInput(in).b_ImuReady);
  ImuInput loose;
  loose.stamp_ns = kT0;
  loose.qw = std::sqrt(1.0011);
  EXPECT_FALSE(ImuFromInput(loose).b_ImuReady);
  loose.qw = std::sqrt(1.0009);
  EXPECT_TRUE(ImuFromInput(loose).b_ImuReady);
  loose.qw = kNaN;
  EXPECT_FALSE(ImuFromInput(loose).b_ImuReady);
}

// Main does not clamp asin: a near-unit quaternion can give a NaN pitch, which
// AddImu then rejects (and raises the IMU barrier).
TEST(ImuFromInput, UnclampedPitchIsRejectedByHistory) {
  ImuInput in;
  in.stamp_ns = kT0;
  in.qy = in.qw = 0.7072;
  const auto imu = ImuFromInput(in);
  EXPECT_TRUE(imu.b_ImuReady);
  EXPECT_TRUE(std::isnan(imu.f32_Pitch_rad));
  PoseHistory h;
  std::string reason;
  EXPECT_FALSE(h.AddImu(SourceStampMs(in.stamp_ns, 0), imu, &reason, in.stamp_ns, kT0 + kMs));
  EXPECT_EQ("invalid IMU sample", reason);
}

TEST(CanFromInput, SpeedNormSteerAndZeroStamp) {
  CanInput in;
  in.stamp_ns = kT0;
  in.vx = 3.;
  in.vy = 4.;
  in.wheel_angle_deg = 10.f;
  const auto can = CanFromInput(in);
  EXPECT_FLOAT_EQ(5.f, can.f32_CurrentVel_MPS);
  EXPECT_FLOAT_EQ(18.f, can.f32_CurrentVel_KPH);
  EXPECT_EQ(10.f, can.f32_Steer_deg);
  EXPECT_EQ(float(10.f * M_PI / 180.0f), can.f32_Steer_rad);  // Main float Deg2Rad
  in.vx = -3.;
  in.vy = -4.;  // reverse still reads positive (|v| norm, preserved)
  EXPECT_FLOAT_EQ(5.f, CanFromInput(in).f32_CurrentVel_MPS);
  in.stamp_ns = 0;
  const auto zero = CanFromInput(in);
  EXPECT_TRUE(std::isnan(zero.f32_CurrentVel_MPS));
  EXPECT_TRUE(std::isnan(zero.f32_CurrentVel_KPH));
}

// ---- GPS ingestion (GPSParser_Callback, common branch) ----

TEST(GpsIngest, ProjectsToMapWithOffsetsAndAltitudeOrigin) {
  GpsIngest ingest(IngestConfig(2.0));
  const auto msg = Msg(-100., 50., kT0 + 123456789ULL, 10.5);
  const auto r = ingest.Evaluate(msg, 0);
  ASSERT_TRUE(r.accepted) << r.reason;
  EXPECT_TRUE(r.reason.empty());
  EXPECT_NEAR(-100., r.data.f64_E, 1e-6);
  EXPECT_NEAR(50., r.data.f64_N, 1e-6);
  EXPECT_DOUBLE_EQ(8.5, r.data.f64_U);
  EXPECT_DOUBLE_EQ(msg.latitude, r.data.f64_Latitude);
  EXPECT_FALSE(r.data.b_isJamming);
  EXPECT_EQ(msg.stamp_ns, r.source_ns);
  EXPECT_EQ((kT0 + 123456789ULL) / kMs, r.stamp_ms);
  EXPECT_EQ("initial valid sample", r.sanity.reason);
  // E/N follow UTMUPS::Forward in the standard zone exactly.
  int zone;
  bool north;
  double e, n;
  GeographicLib::UTMUPS::Forward(msg.latitude, msg.longitude, zone, north, e, n);
  EXPECT_EQ(52, zone);
  EXPECT_EQ(e - kOriginE, r.data.f64_E);
  EXPECT_EQ(n - kOriginN, r.data.f64_N);
}

TEST(GpsIngest, RejectsInvalidMessagesBeforeProjection) {
  GpsIngest ingest(IngestConfig());
  const auto good = Msg(0., 0., kT0);
  auto status0 = good;
  status0.status = 0;
  auto r = ingest.Evaluate(status0, 0);
  EXPECT_FALSE(r.accepted);
  EXPECT_EQ("invalid status: status 0", r.reason);
  EXPECT_TRUE(r.data.b_isJamming);
  EXPECT_EQ(kT0, r.source_ns);

  auto zero = good;
  zero.stamp_ns = 0;
  r = ingest.Evaluate(zero, kT0);
  EXPECT_FALSE(r.accepted);
  EXPECT_EQ("invalid status: zero stamp", r.reason);
  EXPECT_EQ(0u, r.source_ns);  // original stamp: unknown-stamp barrier downstream

  for (int field = 0; field < 5; ++field) {
    auto bad = good;
    double* values[] = {&bad.latitude, &bad.longitude, &bad.altitude, &bad.east_offset, &bad.north_offset};
    *values[field] = field % 2 ? std::numeric_limits<double>::infinity() : kNaN;
    r = ingest.Evaluate(bad, 0);
    EXPECT_FALSE(r.accepted) << field;
    EXPECT_EQ("invalid status: non-finite field", r.reason) << field;
  }

  auto offset = good;
  offset.east_offset += 2e-6;
  EXPECT_EQ("invalid status: map offsets mismatch", ingest.Evaluate(offset, 0).reason);
  offset = good;
  offset.north_offset -= 2e-6;
  EXPECT_FALSE(ingest.Evaluate(offset, 0).accepted);
  offset = good;
  offset.east_offset += 0.5e-6;  // within 1e-6
  EXPECT_TRUE(ingest.Evaluate(offset, 0).accepted);

  auto origin = good;
  origin.stamp_ns = kT0 + 100 * kMs;
  origin.latitude = origin.longitude = 0.;
  r = ingest.Evaluate(origin, 0);
  EXPECT_FALSE(r.accepted);
  EXPECT_EQ("invalid status: zero latitude/longitude", r.reason);
  EXPECT_TRUE(ingest.MapOffsetsValid(kOriginE, kOriginN));
  EXPECT_FALSE(ingest.MapOffsetsValid(kNaN, kOriginN));
}

TEST(GpsIngest, SanityGateUsesMapCoordinatesAndSourceSeconds) {
  GpsIngest ingest(IngestConfig());
  auto r = ingest.Evaluate(Msg(500., 0., kT0), 0);  // outside the K-City bounds
  EXPECT_FALSE(r.accepted);
  EXPECT_EQ("outside configured local-map bounds", r.reason);
  EXPECT_EQ(kT0, r.source_ns);
  ASSERT_TRUE(ingest.Evaluate(Msg(0., 0., kT0), 0).accepted);
  r = ingest.Evaluate(Msg(100., 0., kT0 + 100 * kMs), 0);
  EXPECT_FALSE(r.accepted);
  EXPECT_EQ("implausible position jump", r.reason);
  EXPECT_NEAR(100., r.sanity.jump_m, 1e-6);
  // Seconds come from ros::Time::toSec() (double(sec) + 1e-9 * double(nsec)),
  // so at epoch magnitude the 0.1 s gap carries ~1e-7 s of rounding, like Main.
  const auto to_sec = [](std::uint64_t ns) {
    return double(std::uint32_t(ns / 1000000000ULL)) + 1e-9 * double(std::uint32_t(ns % 1000000000ULL));
  };
  EXPECT_DOUBLE_EQ(3. + 80. * (to_sec(kT0 + 100 * kMs) - to_sec(kT0)), r.sanity.allowed_jump_m);
  EXPECT_NEAR(11., r.sanity.allowed_jump_m, 1e-4);
  r = ingest.Evaluate(Msg(0., 0., kT0 - 2 * kMs), 0);
  EXPECT_EQ("source timestamp moved backwards", r.reason);
  EXPECT_TRUE(ingest.Evaluate(Msg(5., 0., kT0 + 200 * kMs), 0).accepted);
}

// ---- GPS ingestion into the bounded map pose ----

TEST(GpsIngest, ValidStreamGivesMapPoseAndRejectionsGiveGpsInvalid) {
  struct Case {
    const char* name;
    void (*spoil)(GpsInput*);
  };
  const Case cases[] = {
      {"status0", [](GpsInput* m) { m->status = 0; }},
      {"zero_lat_lon", [](GpsInput* m) { m->latitude = m->longitude = 0.; }},
      {"nan_latitude", [](GpsInput* m) { m->latitude = std::numeric_limits<double>::quiet_NaN(); }},
      {"offset_mismatch", [](GpsInput* m) { m->east_offset += 1.; }},
      {"zero_stamp", [](GpsInput* m) { m->stamp_ns = 0; }},
      {"out_of_bounds", [](GpsInput* m) {
         GeographicLib::UTMUPS::Reverse(52, true, kOriginE + 1000., kOriginN, m->latitude, m->longitude);
       }},
  };
  for (const auto& c : cases) {
    PoseHistory h(LaunchPose());
    GpsIngest ingest(IngestConfig());
    FeedImu(h, kT0);
    for (int i = 0; i < 3; ++i) Feed(ingest, h, Msg(0., 0., kT0 + 50 * i * kMs));
    const auto ok = h.MapAt(ScanTime(kT0 + 110 * kMs, kT0 + 110 * kMs), Local());
    ASSERT_TRUE(ok.valid) << c.name << " " << ok.reason;
    EXPECT_NEAR(-0.35, ok.map_from_base.translation().x(), 0.02) << c.name;  // lever, grid-yaw rotated
    EXPECT_NEAR(10. - 1.25, ok.map_from_base.translation().z(), 1e-6) << c.name;

    auto bad = Msg(0., 0., kT0 + 150 * kMs);
    c.spoil(&bad);
    const auto r = ingest.Evaluate(bad, kT0 + 151 * kMs);
    EXPECT_FALSE(r.accepted) << c.name;
    GpsIngest::Apply(r, kT0 + 152 * kMs, h);
    EXPECT_EQ("gps_invalid", h.MapAt(ScanTime(kT0 + 160 * kMs, kT0 + 160 * kMs), Local()).reason) << c.name;
  }
}

TEST(GpsIngest, RecoveryNeedsThreeConsecutiveNormalSamples) {
  PoseHistory h(LaunchPose());
  GpsIngest ingest(IngestConfig());
  FeedImu(h, kT0);
  auto status0 = Msg(0., 0., kT0 + 10 * kMs);
  status0.status = 0;
  Feed(ingest, h, status0);
  Feed(ingest, h, Msg(0., 0., kT0 + 50 * kMs));
  EXPECT_EQ("gps_recovering", h.MapAt(ScanTime(kT0 + 60 * kMs, kT0 + 60 * kMs), Local()).reason);
  Feed(ingest, h, Msg(0., 0., kT0 + 80 * kMs));
  EXPECT_EQ("gps_recovering", h.MapAt(ScanTime(kT0 + 90 * kMs, kT0 + 90 * kMs), Local()).reason);
  Feed(ingest, h, Msg(0., 0., kT0 + 110 * kMs));
  const auto pose = h.MapAt(ScanTime(kT0 + 110 * kMs, kT0 + 120 * kMs), Local());
  EXPECT_TRUE(pose.valid) << pose.reason;
  EXPECT_EQ("gps_observed", pose.reason);
}

TEST(GpsIngest, SilenceOver300msIsStale) {
  PoseHistory h(LaunchPose());
  GpsIngest ingest(IngestConfig());
  FeedImu(h, kT0);
  for (int i = 0; i < 3; ++i) Feed(ingest, h, Msg(0., 0., kT0 + 50 * i * kMs));
  FeedImu(h, kT0 + 350 * kMs);
  LocalState local = Local();
  local.st_CAN.f32_CurrentVel_MPS = 10.f;
  EXPECT_TRUE(h.MapAt(ScanTime(kT0 + 400 * kMs, kT0 + 400 * kMs), local).valid);
  EXPECT_EQ("gps_stale", h.MapAt(ScanTime(kT0 + 400 * kMs + 1, kT0 + 401 * kMs), local).reason);
}

// Deliberate deviation from Main: an out-of-range latitude/longitude does not let
// GeographicErr escape (Main's GPS callback would terminate the process); the
// sample takes the invalid path and becomes a barrier like any unusable GPS.
TEST(GpsIngest, OutOfRangeLatitudeIsInvalidWithoutThrowing) {
  GpsIngest ingest(IngestConfig());
  auto bad = Msg(0., 0., kT0);
  bad.latitude = 95.0;
  GpsIngestResult r;
  EXPECT_NO_THROW(r = ingest.Evaluate(bad, 0));
  EXPECT_FALSE(r.accepted);
  EXPECT_EQ("invalid status: latitude/longitude out of range", r.reason);
  EXPECT_EQ(kT0, r.source_ns);
  bad = Msg(0., 0., kT0 + 100000000ULL);
  bad.longitude = 1000.0;
  EXPECT_NO_THROW(r = ingest.Evaluate(bad, 0));
  EXPECT_FALSE(r.accepted);
  // A following normal sample is still accepted (the filter state was not touched).
  EXPECT_TRUE(ingest.Evaluate(Msg(0., 0., kT0 + 200000000ULL), 0).accepted);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
