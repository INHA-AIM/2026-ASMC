// B0 GPS/IMU/CAN validity, GPS denial and recovery through the common-geometry path.
// ROS-free: no master, no NodeHandle, no socket. Seams: frame_boundary::ConfigureForTest and
// ros::Time::init() (wall clock only; the Parser callbacks read ros::Time::now()).
//
// Chain under test, in main.cpp TemporalThread order:
//   samples -> SensorPoseHistory full-ns API (source stamp + steady receipt, as Parser.cpp calls it
//   with common geometry) -> WaitForLocalLookupNs (IMU/CAN <= 0.12 s, invalid barriers) -> MapAt
//   (CAN speed <= 35 m/s, BoundedPoseHistory: arrival cut, barriers, GPS <= 0.30 s, 3 consecutive
//   good samples) -> PrepareScan + one StoreLeveling (CorrectingPoints) -> frame_boundary::GateInputs
//   -> output_gate::Decide: 'M' /lidar/map_object, 'L' /lidar/local_object, 'N' nothing on V2.
// The tick layout is the integration feeder's (b0_feeder.py): IMU at S-40 ms and S-5 ms, GPS at
// S-30 ms, CAN at S-5 ms, LiDAR scan at S, 10 Hz; every sample arrives 1 ms after its nominal time.
#include <gtest/gtest.h>

#include <Geometry/FrameBoundary.hpp>
#include <Geometry/FrameBoundaryPublisher.hpp>
#include <Geometry/OutputGate.hpp>
#include <Parser/GpsSanityFilter.hpp>
#include <Parser/Parser.hpp>
#include <Parser/SensorPoseHistory.hpp>
#include <SensorFusion/FusionTrace.hpp>
#include <asmc_lidar/projection.hpp>

#include <boost/make_shared.hpp>
#include <morai_msgs/EgoVehicleStatus.h>
#include <morai_msgs/GPSMessage.h>
#include <sensor_msgs/Imu.h>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

extern VehicleSpec st_VehicleSpec;

namespace {
using asmc::lidar::Ns;
using output_gate::GpsState;
using output_gate::Mode;

constexpr Ns kMs = 1000000LL;
constexpr Ns kSec = 1000000000LL;
constexpr Ns kMaxGpsAge = 300 * kMs;                 // main.launch lidar_pose_gps_max_age_s
constexpr double kRoll = 0.02, kPitch = -0.01, kYaw = 0.3;

void UseCommonGeometry() {
  frame_boundary::ConfigureForTest(true, true, "unit_test_calibration");
  st_VehicleSpec = VehicleSpec{};
  st_VehicleSpec.st_LiDARPos = {0.80f, 0.0f, 1.45f};
  st_VehicleSpec.st_LiDARRotationDeg = {0.f, 0.f, 0.f};
  st_VehicleSpec.st_GPSPos = {0.35f, 0.0f, 1.25f};
}

enum class Gps { kGood, kStatus0, kLatLon0, kStamp0, kNan, kBadOffset, kJump, kSilent };
enum class Imu { kGood, kQuatInvalid, kSilent };
enum class Can { kGood, kSilent, kStamp0 };
const char* Name(Gps g) {
  switch (g) {
    case Gps::kGood: return "good"; case Gps::kStatus0: return "status0"; case Gps::kLatLon0: return "latlon0";
    case Gps::kStamp0: return "stamp0"; case Gps::kNan: return "nan"; case Gps::kBadOffset: return "bad_offset";
    case Gps::kJump: return "sanity_jump"; case Gps::kSilent: default: return "silent";
  }
}

// Faults of one 100 ms tick. Lags shift the source stamp only (the sample still arrives on time);
// a negative GPS lag is a GPS clock running ahead.
struct Tick {
  Gps gps = Gps::kGood;
  Imu imu = Imu::kGood;
  Can can = Can::kGood;
  Ns gps_lag = 0, imu_lag = 0, can_lag = 0;
  double can_speed_mps = 2.0;
  Ns lidar_offset = 0;        // LiDAR source clock offset against IMU/CAN/GPS (clock mismatch)
};

struct ScanOut {
  Ns stamp = 0;
  bool pose_ready = false;
  std::string pose_reason;
  asmc::lidar::PoseSelection map;
  output_gate::Inputs in;
  output_gate::Decision d;
  char mode() const { return d.v2 == Mode::kMap ? 'M' : d.v2 == Mode::kLocal ? 'L' : 'N'; }
  Ns gps_age() const { return map.sample_stamps.empty() ? -1 : stamp - map.sample_stamps.back(); }
};

// One SensorPoseHistory + GpsSanityFilter fed exactly like Parser.cpp's common-geometry callbacks.
// The GPS ingest mirrors GPSParser_Callback (Parser.cpp): malformed message -> AddGpsInvalidAt(original
// stamp, receipt); sanity rejection -> AddGpsInvalidAt; otherwise AddGps(stamp_ms, data, stamp_ns, receipt).
// GpsDenialRecovery.ParserCallbacksTurnEveryMalformedInputIntoABarrier drives the real callbacks.
class Rig {
 public:
  explicit Rig(Ns t0 = 1800000000LL * kSec) : t0_(t0) { UseCommonGeometry(); }

  ScanOut Step(const Tick& f) {
    const Ns s = Nominal(k_++);
    ImuSample(s - 40 * kMs, f);
    GpsSample(s - 30 * kMs, f);
    ImuSample(s - 5 * kMs, f);
    CanSample(s - 5 * kMs, f);
    return Scan(s + f.lidar_offset);
  }
  std::vector<ScanOut> Run(int scans, const Tick& f) {
    std::vector<ScanOut> out;
    for (int i = 0; i < scans; ++i) out.push_back(Step(f));
    return out;
  }
  // Scan as main.cpp processes it (LidarProcess accepted the cloud whenever the pose was ready).
  ScanOut Scan(Ns stamp) {
    return ScanWithTime({stamp, stamp + kMs, stamp + 2 * kMs, "ros", "bridge_receive_ns", "steady"});
  }
  ScanOut ScanWithTime(const asmc::lidar::Observation& time) {
    ScanOut out;
    out.stamp = time.stamp_ns;
    GPS_IMU pose{};
    LocalInputSelection selection;
    out.pose_ready = history_.WaitForLocalLookupNs(std::uint64_t(time.stamp_ns), 0.0, &pose, &selection,
                                                   &out.pose_reason);
    LiDAR lidar;
    ++generation_;
    if (out.pose_ready) {
      out.map = history_.MapAt(time, pose);
      frame_boundary::PrepareScan(time, pose, st_VehicleSpec, out.map, generation_);
      frame_boundary::StoreLeveling(asmc::lidar::leveling_rotation(
          pose.st_IMU.f32_Roll_rad, pose.st_IMU.f32_Pitch_rad,
          frame_boundary::SensorMount(st_VehicleSpec).rotation(), frame_boundary::UseRoll(), true));
      lidar.frame_context = frame_boundary::Current();   // LidarProcess.cpp copy
    }
    out.in = frame_boundary::GateInputs(out.pose_ready, out.pose_reason, out.pose_ready, lidar, generation_);
    out.d = output_gate::Decide(out.in);
    return out;
  }
  SensorPoseHistory& history() { return history_; }
  Ns Nominal(int k) const { return t0_ + Ns(k) * 100 * kMs; }
  int next_tick() const { return k_; }
  void AddGoodGpsAt(Ns stamp, Ns arrival) {
    GPS_DATA g{}; g.f64_E = 100.0; g.f64_N = 200.0; g.f64_U = 30.0; g.b_isJamming = false;
    history_.AddGps(std::uint64_t(stamp / kMs), g, std::uint64_t(stamp), std::uint64_t(arrival));
  }

 private:
  void ImuSample(Ns nominal, const Tick& f) {
    if (f.imu == Imu::kSilent) return;
    const Ns stamp = nominal - f.imu_lag;
    IMU_DATA imu{};
    imu.f32_Roll_rad = float(kRoll); imu.f32_Pitch_rad = float(kPitch); imu.f32_Yaw_rad = float(kYaw);
    imu.b_ImuReady = f.imu != Imu::kQuatInvalid;   // IMUParser_Callback: |q| != 1 -> not ready
    history_.AddImu(std::uint64_t(stamp / kMs), imu, nullptr, std::uint64_t(stamp), std::uint64_t(nominal + kMs));
  }
  void CanSample(Ns nominal, const Tick& f) {
    if (f.can == Can::kSilent) return;
    CAN_DATA can{};
    can.f32_CurrentVel_MPS = float(f.can_speed_mps); can.f32_CurrentVel_KPH = float(f.can_speed_mps * 3.6);
    if (f.can == Can::kStamp0) {
      // CANParser_Callback: zero header -> speed NaN, stamp_ms = receipt time, source_ns = 0.
      can.f32_CurrentVel_MPS = can.f32_CurrentVel_KPH = std::numeric_limits<float>::quiet_NaN();
      history_.AddCan(std::uint64_t((nominal + kMs) / kMs), can, nullptr, 0, std::uint64_t(nominal + kMs));
      return;
    }
    const Ns stamp = nominal - f.can_lag;
    history_.AddCan(std::uint64_t(stamp / kMs), can, nullptr, std::uint64_t(stamp), std::uint64_t(nominal + kMs));
  }
  void GpsSample(Ns nominal, const Tick& f) {
    if (f.gps == Gps::kSilent) return;
    const Ns stamp = nominal - f.gps_lag;
    const Ns arrival = nominal + kMs;
    switch (f.gps) {
      case Gps::kStatus0: case Gps::kLatLon0: case Gps::kNan: case Gps::kBadOffset:
        history_.AddGpsInvalidAt(std::uint64_t(stamp), std::uint64_t(arrival)); return;
      case Gps::kStamp0:
        history_.AddGpsInvalidAt(0, std::uint64_t(arrival)); return;
      default: break;
    }
    const double t = double(nominal - t0_) * 1e-9;
    GPS_DATA g{};
    g.f64_E = 100.0 + 2.0 * t * std::cos(kYaw) + (f.gps == Gps::kJump ? 60.0 : 0.0);
    g.f64_N = 200.0 + 2.0 * t * std::sin(kYaw);
    g.f64_U = 30.0; g.b_isJamming = false;
    const auto sanity = sanity_.Evaluate(g.f64_E, g.f64_N, double(stamp) * 1e-9);
    if (!sanity.accepted) {
      history_.AddGpsInvalidAt(std::uint64_t(stamp), std::uint64_t(arrival));
      return;
    }
    history_.AddGps(std::uint64_t(stamp / kMs), g, std::uint64_t(stamp), std::uint64_t(arrival));
  }

  Ns t0_;
  int k_ = 0;
  std::uint64_t generation_ = 0;
  SensorPoseHistory history_;
  GpsSanityFilter sanity_;
};

std::string Modes(const std::vector<ScanOut>& scans) {
  std::string out;
  for (const auto& s : scans) out += s.mode();
  return out;
}
std::string Repeat(char c, int n) { return std::string(std::size_t(n), c); }

// Row invariants of every scan (checked by every test through Expect()).
void CheckInvariants(const std::vector<ScanOut>& scans, const std::string& where) {
  for (std::size_t i = 0; i < scans.size(); ++i) {
    const auto& s = scans[i];
    SCOPED_TRACE(where + " scan " + std::to_string(i) + " reason=" + s.d.reason);
    if (s.mode() == 'M') {
      // A map frame needs a valid map pose of THIS scan, built from a GPS sample at or before the
      // scan and at most 0.30 s old: GPS is never reused beyond its maximum age.
      ASSERT_TRUE(s.pose_ready);
      ASSERT_TRUE(s.map.valid);
      ASSERT_EQ(s.map.query_stamp_ns, s.stamp);
      ASSERT_FALSE(s.map.sample_stamps.empty());
      ASSERT_GE(s.gps_age(), 0);
      ASSERT_LE(s.gps_age(), kMaxGpsAge);
      ASSERT_EQ(s.d.gps, GpsState::kHealthy);
    } else if (s.mode() == 'L') {
      // local_v2 only for a GPS denial with valid local inputs.
      ASSERT_TRUE(s.pose_ready);
      ASSERT_FALSE(s.map.valid);
      ASSERT_TRUE(s.d.gps == GpsState::kDenied || s.d.gps == GpsState::kRecovering) << s.map.reason;
    }
    if (!s.pose_ready) { ASSERT_EQ(s.mode(), 'N'); }
    if (s.mode() != 'M') { ASSERT_FALSE(s.d.legacy_permitted); }
  }
}
void Expect(const std::vector<ScanOut>& scans, const std::string& modes, const std::string& where) {
  CheckInvariants(scans, where);
  EXPECT_EQ(Modes(scans), modes) << where;
}
int CountMode(const std::vector<ScanOut>& scans, char mode) {
  int n = 0;
  for (const auto& s : scans) n += s.mode() == mode;
  return n;
}

}  // namespace

// Startup: no map frame before three consecutive good GPS samples; local_v2 meanwhile.
TEST(GpsDenialRecovery, StartupNeedsThreeConsecutiveGoodSamples) {
  Rig rig;
  const auto start = rig.Run(20, Tick{});
  Expect(start, "LLM" + Repeat('M', 17), "startup");
  EXPECT_EQ(start[0].d.reason, "local_only:gps_recovering");
  EXPECT_EQ(start[1].d.reason, "local_only:gps_recovering");
  EXPECT_EQ(start[2].d.reason, "map:gps_bounded_prediction");
  EXPECT_EQ(start[2].gps_age(), 30 * kMs);
}

// GPS status==0, lat/lon (0,0), header stamp 0, NaN fix, wrong map offsets and a sanity-rejected jump
// all become invalid barriers: the first scan after onset is local_v2 only (never map), and map
// output returns only on the third consecutive good sample.
TEST(GpsDenialRecovery, EveryInvalidGpsClassDisablesMapOnTheFirstScanAndNeedsThreeGoodSamples) {
  for (const Gps fault : {Gps::kStatus0, Gps::kLatLon0, Gps::kStamp0, Gps::kNan, Gps::kBadOffset, Gps::kJump}) {
    SCOPED_TRACE(Name(fault));
    Rig rig;
    Expect(rig.Run(20, Tick{}), "LLM" + Repeat('M', 17), "healthy");
    Tick bad; bad.gps = fault;
    // A held 60 m offset is accepted by the sanity filter after ~0.73 s (3 m + 80 m/s * dt), so
    // the jump is a single sample (fault catalogue S07); the other classes last 1 s.
    const int scans = fault == Gps::kJump ? 1 : 10;
    const auto denied = rig.Run(scans, bad);
    Expect(denied, Repeat('L', scans), "denied");
    for (const auto& s : denied) {
      EXPECT_EQ(s.d.gps, GpsState::kDenied);
      EXPECT_EQ(s.d.reason, "local_only:gps_invalid");
    }
    const auto back = rig.Run(15, Tick{});
    Expect(back, "LLM" + Repeat('M', 12), "recovery");
    EXPECT_EQ(back[0].d.reason, "local_only:gps_recovering");
    EXPECT_EQ(back[1].d.reason, "local_only:gps_recovering");
  }
}

// GPS topic silence: bounded prediction only while the last good sample is <= 0.30 s old, then
// local_v2. A silence longer than 0.30 s restarts the consecutive count.
TEST(GpsDenialRecovery, SilenceIsBoundedToMaxAgeThenLocalOnly) {
  Rig rig;
  rig.Run(20, Tick{});
  Tick silent; silent.gps = Gps::kSilent;
  const auto gap = rig.Run(10, silent);
  Expect(gap, "MM" + Repeat('L', 8), "silence");
  EXPECT_EQ(gap[0].gps_age(), 130 * kMs);
  EXPECT_EQ(gap[1].gps_age(), 230 * kMs);
  EXPECT_EQ(gap[2].d.reason, "local_only:gps_stale");
  Expect(rig.Run(15, Tick{}), "LLM" + Repeat('M', 12), "after silence");
}

// Stale GPS stamps (sample arrives, stamp older than 0.30 s at the scan) never give map output;
// the 0.30 s bound is hard (age 290 ms map, 310 ms local).
TEST(GpsDenialRecovery, StaleGpsBeyondMaxAgeIsLocalOnly) {
  {
    Rig rig;
    rig.Run(20, Tick{});
    Tick stale; stale.gps_lag = 500 * kMs;
    const auto late = rig.Run(12, stale);
    Expect(late, "MM" + Repeat('L', 10), "stale 0.5 s");
    for (std::size_t i = 2; i < late.size(); ++i) EXPECT_EQ(late[i].d.reason, "local_only:gps_stale") << i;
    Expect(rig.Run(15, Tick{}), "LLM" + Repeat('M', 12), "after stale");
  }
  for (const auto& row : {std::make_pair(Ns(260), 'M'), std::make_pair(Ns(280), 'L')}) {
    Rig rig;
    Tick lag; lag.gps_lag = row.first * kMs;    // age at the scan = 30 ms + lag
    const auto scans = rig.Run(12, lag);
    CheckInvariants(scans, "boundary");
    EXPECT_EQ(scans.back().mode(), row.second) << "gps age " << (30 + row.first) << " ms";
    EXPECT_EQ(scans.back().gps_age(), (30 + row.first) * kMs);
  }
}

// Recovery with 1, 2 or 3 good GPS samples after a denial (fault catalogue S17/S18/S19): only the
// third consecutive good sample gives the first map_v2 scan.
TEST(GpsDenialRecovery, RecoveryNeedsExactlyThreeConsecutiveGoodSamples) {
  const char* expected[] = {"LLLLLL", "LLLLLL", "LLMLLL"};
  for (int good = 1; good <= 3; ++good) {
    SCOPED_TRACE("good samples " + std::to_string(good));
    Rig rig;
    rig.Run(20, Tick{});
    Tick denied; denied.gps = Gps::kStatus0;
    Expect(rig.Run(5, denied), Repeat('L', 5), "denied");
    std::vector<ScanOut> partial;
    for (int i = 0; i < 6; ++i) {
      Tick t; t.gps = i < good ? Gps::kGood : Gps::kStatus0;
      partial.push_back(rig.Step(t));
    }
    Expect(partial, expected[good - 1], "partial recovery");
    Expect(rig.Run(15, Tick{}), "LLM" + Repeat('M', 12), "full recovery");
  }
}

// An invalid sample during recovery restarts the count (S20); so does a gap above 0.30 s between
// good samples (bounded_pose.cpp consecutive reset).
TEST(GpsDenialRecovery, InvalidSampleOrGapDuringRecoveryRestartsTheCount) {
  {
    Rig rig;
    rig.Run(20, Tick{});
    Tick denied; denied.gps = Gps::kStatus0;
    rig.Run(5, denied);
    Expect(rig.Run(2, Tick{}), "LL", "two good");
    Expect(rig.Run(1, denied), "L", "invalid during recovery");
    Expect(rig.Run(15, Tick{}), "LLM" + Repeat('M', 12), "restarted count");
  }
  {
    Rig rig;
    rig.Run(20, Tick{});
    Tick denied; denied.gps = Gps::kStatus0;
    rig.Run(5, denied);
    std::vector<ScanOut> sparse;   // a good sample every 0.4 s: never 3 consecutive within 0.30 s
    for (int i = 0; i < 20; ++i) {
      Tick t; t.gps = (i % 4 == 0) ? Gps::kGood : Gps::kSilent;
      sparse.push_back(rig.Step(t));
    }
    Expect(sparse, Repeat('L', 20), "sparse good samples");
    Expect(rig.Run(15, Tick{}), "LLM" + Repeat('M', 12), "dense good samples");
  }
}

// A GPS sample that arrives after the scan was received cannot make that scan's map output
// available (arrival cut); the same sample counts for a scan received after it.
TEST(GpsDenialRecovery, LateGpsArrivalCannotReviveAnEarlierScan) {
  Rig rig;
  rig.Run(20, Tick{});
  Tick denied; denied.gps = Gps::kStatus0;
  rig.Run(5, denied);
  Expect(rig.Run(2, Tick{}), "LL", "two good");
  const Ns s = rig.Nominal(rig.next_tick());
  rig.AddGoodGpsAt(s - 30 * kMs, s + 50 * kMs);            // third good sample, received 50 ms after S
  GPS_IMU pose{}; LocalInputSelection selection; std::string reason;
  // Fresh IMU/CAN for S so that only the map pose is in question.
  IMU_DATA imu{}; imu.b_ImuReady = true; imu.f32_Yaw_rad = float(kYaw);
  CAN_DATA can{}; can.f32_CurrentVel_MPS = 2.f;
  rig.history().AddImu(std::uint64_t((s - 5 * kMs) / kMs), imu, nullptr, std::uint64_t(s - 5 * kMs), std::uint64_t(s - 4 * kMs));
  rig.history().AddCan(std::uint64_t((s - 5 * kMs) / kMs), can, nullptr, std::uint64_t(s - 5 * kMs), std::uint64_t(s - 4 * kMs));
  ASSERT_TRUE(rig.history().WaitForLocalLookupNs(std::uint64_t(s), 0.0, &pose, &selection, &reason)) << reason;
  const auto before = rig.history().MapAt({s, s + kMs, s + 2 * kMs, "ros", "bridge_receive_ns", "steady"}, pose);
  EXPECT_FALSE(before.valid);
  EXPECT_EQ(before.reason, "gps_recovering");
  const auto after = rig.history().MapAt({s, s + 60 * kMs, s + 61 * kMs, "ros", "bridge_receive_ns", "steady"}, pose);
  EXPECT_TRUE(after.valid) << after.reason;
  const auto scan = rig.Scan(s);                             // main.cpp: receipt S+1 ms -> still local
  EXPECT_EQ(scan.mode(), 'L');
  EXPECT_EQ(scan.d.reason, "local_only:gps_recovering");
}

// IMU quaternion invalid (b_ImuReady=false), IMU stale (> 0.12 s) and IMU silence: no V2 output at
// all (not local_v2). GPS samples taken without a fresh valid IMU fail the IMU gate, so map output
// afterwards needs the 3-sample recovery.
TEST(GpsDenialRecovery, ImuInvalidStaleOrSilentIsNoneNeverLocal) {
  {
    Rig rig;
    rig.Run(20, Tick{});
    Tick quat; quat.imu = Imu::kQuatInvalid;
    const auto bad = rig.Run(5, quat);
    Expect(bad, Repeat('N', 5), "quaternion invalid");
    EXPECT_EQ(bad[0].pose_reason, "imu_input_invalid");
    EXPECT_EQ(bad[0].d.reason, "local_input_invalid:imu_input_invalid");
    for (const auto& s : bad) EXPECT_FALSE(s.pose_ready);
    Expect(rig.Run(15, Tick{}), "LLM" + Repeat('M', 12), "after quaternion invalid");
  }
  {
    Rig rig;
    rig.Run(20, Tick{});
    Tick lag; lag.imu_lag = 200 * kMs;
    const auto stale = rig.Run(10, lag);
    Expect(stale, "M" + Repeat('N', 9), "IMU stamps 0.2 s late");
    EXPECT_EQ(stale[1].pose_reason, "IMU is stale for LiDAR scan");
    Expect(rig.Run(15, Tick{}), "LLM" + Repeat('M', 12), "after IMU stale");
  }
  {
    Rig rig;
    rig.Run(20, Tick{});
    Tick silent; silent.imu = Imu::kSilent;
    Expect(rig.Run(10, silent), "M" + Repeat('N', 9), "IMU silence");
    Expect(rig.Run(15, Tick{}), "LLM" + Repeat('M', 12), "after IMU silence");
  }
  for (const auto& row : {std::make_pair(Ns(119), true), std::make_pair(Ns(121), false)}) {
    Rig rig;
    rig.Run(5, Tick{});
    const Ns s = rig.Nominal(rig.next_tick()) + 2 * kSec;    // only the new IMU/CAN below are recent
    IMU_DATA imu{}; imu.b_ImuReady = true; imu.f32_Yaw_rad = float(kYaw);
    CAN_DATA can{}; can.f32_CurrentVel_MPS = 2.f;
    rig.history().AddImu(std::uint64_t((s - row.first * kMs) / kMs), imu, nullptr, std::uint64_t(s - row.first * kMs), std::uint64_t(s - row.first * kMs + kMs));
    rig.history().AddCan(std::uint64_t((s - 5 * kMs) / kMs), can, nullptr, std::uint64_t(s - 5 * kMs), std::uint64_t(s - 4 * kMs));
    const auto scan = rig.Scan(s);
    EXPECT_EQ(scan.pose_ready, row.second) << "IMU age " << row.first << " ms: " << scan.pose_reason;
    if (!row.second) { EXPECT_EQ(scan.mode(), 'N'); }
  }
}

// CAN stale (> 0.12 s), silent or stamp 0 (NaN speed barrier): no V2 output at all. CAN does not
// gate GPS samples, so map output returns on the first scan with fresh CAN.
TEST(GpsDenialRecovery, CanStaleSilentOrInvalidIsNoneNeverLocal) {
  {
    Rig rig;
    rig.Run(20, Tick{});
    Tick lag; lag.can_lag = 200 * kMs;
    const auto stale = rig.Run(10, lag);
    Expect(stale, "M" + Repeat('N', 9), "CAN stamps 0.2 s late");
    EXPECT_EQ(stale[1].pose_reason, "vehicle speed is stale for LiDAR scan");
    EXPECT_EQ(stale[1].d.reason, "local_input_invalid:vehicle speed is stale for LiDAR scan");
    Expect(rig.Run(15, Tick{}), Repeat('M', 15), "after CAN stale");
  }
  {
    Rig rig;
    rig.Run(20, Tick{});
    Tick silent; silent.can = Can::kSilent;
    Expect(rig.Run(10, silent), "M" + Repeat('N', 9), "CAN silence");
    Expect(rig.Run(15, Tick{}), Repeat('M', 15), "after CAN silence");
  }
  {
    Rig rig;
    rig.Run(20, Tick{});
    Tick zero; zero.can = Can::kStamp0;
    const auto bad = rig.Run(10, zero);
    Expect(bad, Repeat('N', 10), "CAN stamp 0");
    EXPECT_EQ(bad[0].pose_reason, "speed_input_invalid");
    Expect(rig.Run(15, Tick{}), Repeat('M', 15), "after CAN stamp 0");
  }
  for (const auto& row : {std::make_pair(Ns(119), true), std::make_pair(Ns(121), false)}) {
    Rig rig;
    rig.Run(5, Tick{});
    const Ns s = rig.Nominal(rig.next_tick()) + 2 * kSec;
    IMU_DATA imu{}; imu.b_ImuReady = true; imu.f32_Yaw_rad = float(kYaw);
    CAN_DATA can{}; can.f32_CurrentVel_MPS = 2.f;
    rig.history().AddImu(std::uint64_t((s - 5 * kMs) / kMs), imu, nullptr, std::uint64_t(s - 5 * kMs), std::uint64_t(s - 4 * kMs));
    rig.history().AddCan(std::uint64_t((s - row.first * kMs) / kMs), can, nullptr, std::uint64_t(s - row.first * kMs), std::uint64_t(s - row.first * kMs + kMs));
    const auto scan = rig.Scan(s);
    EXPECT_EQ(scan.pose_ready, row.second) << "CAN age " << row.first << " ms: " << scan.pose_reason;
  }
}

// CAN speed above lidar_pose_maximum_speed_mps (35 m/s) invalidates the map pose for a non-GPS
// reason: nothing on V2 (not local_v2, D7), and map returns as soon as the speed is plausible again
// (no GPS recovery needed because GPS stayed valid).
TEST(GpsDenialRecovery, CanSpeedAboveLimitIsNoneNotLocal) {
  Rig rig;
  rig.Run(20, Tick{});
  std::vector<ScanOut> ramp;
  double speed = 2.0;
  for (int i = 0; i < 30; ++i) {          // +2.4 m/s per 0.1 s stays inside the CAN jump gate (1 + 15*dt)
    speed = std::min(36.0, speed + 2.4);
    Tick t; t.can_speed_mps = speed;
    ramp.push_back(rig.Step(t));
  }
  CheckInvariants(ramp, "ramp");
  for (const auto& s : ramp) EXPECT_TRUE(s.pose_ready);   // local input stays valid
  // 2.0 + 2.4*n > 35 from n = 14 (35.6 m/s).
  EXPECT_EQ(Modes(ramp), Repeat('M', 13) + Repeat('N', 17));
  EXPECT_EQ(ramp.back().map.reason, "map_speed_invalid");
  EXPECT_EQ(ramp.back().d.gps, GpsState::kNonGpsInvalid);
  EXPECT_EQ(ramp.back().d.reason, "map_pose_invalid:map_speed_invalid");
  std::vector<ScanOut> down;
  for (int i = 0; i < 5; ++i) {
    speed -= 2.4;
    Tick t; t.can_speed_mps = speed;
    down.push_back(rig.Step(t));
  }
  Expect(down, Repeat('M', 5), "speed back below the limit");
}

// Clock mismatch: a LiDAR clock in another epoch or ahead of the inputs, and IMU stamps 1 s behind,
// give no V2 output; a scan whose receipt time is invalid is none too. A GPS clock running 10 s
// ahead never yields a map pose from a sample stamped after the scan (safety); the resulting outage
// after the GPS clock returns (sanity filter anchored in the future, B-11) is recorded, not asserted.
TEST(GpsDenialRecovery, ClockMismatchNeverProducesMapOrLocalFromInconsistentTime) {
  const Ns offsets[] = {Ns(100) * kSec - Ns(1800000000) * kSec, Ns(10) * kSec};
  for (const Ns offset : offsets) {
    Rig rig;
    rig.Run(20, Tick{});
    Tick other; other.lidar_offset = offset;
    const auto scans = rig.Run(10, other);
    Expect(scans, Repeat('N', 10), "LiDAR clock offset " + std::to_string(offset));
    EXPECT_TRUE(scans[0].pose_reason == "no IMU at or before LiDAR scan" ||
                scans[0].pose_reason == "IMU is stale for LiDAR scan") << scans[0].pose_reason;
  }
  {
    Rig rig;
    rig.Run(20, Tick{});
    Tick behind; behind.imu_lag = kSec;
    Expect(rig.Run(10, behind), "M" + Repeat('N', 9), "IMU 1 s behind");
  }
  {
    Rig rig;
    rig.Run(20, Tick{});
    const Ns s = rig.Nominal(rig.next_tick());
    rig.Run(1, Tick{});
    const auto before = rig.ScanWithTime({s + 100 * kMs, s + 99 * kMs, s + 102 * kMs, "ros", "bridge_receive_ns", ""});
    EXPECT_EQ(before.mode(), 'N');
    EXPECT_EQ(before.d.reason, "time_invalid:receipt_before_stamp");
    const auto clock = rig.ScanWithTime({s + 100 * kMs, s + 101 * kMs, s + 102 * kMs, "ros", "bridge_receive_ns", "system"});
    EXPECT_EQ(clock.mode(), 'N');
    EXPECT_EQ(clock.d.reason, "time_invalid:receipt_clock_unsupported");
  }
  {
    Rig rig;
    rig.Run(20, Tick{});
    Tick future; future.gps_lag = -10 * kSec;
    const auto ahead = rig.Run(5, future);
    Expect(ahead, "MMLLL", "GPS clock 10 s ahead");
    const auto back = rig.Run(25, Tick{});
    CheckInvariants(back, "GPS clock back");
    for (const auto& s : back) {
      if (s.mode() == 'M') { EXPECT_LE(s.map.sample_stamps.back(), s.stamp); }
    }
    ::testing::Test::RecordProperty("future_gps_outage_local_scans", CountMode(back, 'L'));
  }
}

// "GPS is never reused indefinitely": after the GPS invalid onset no map frame at all, and after
// a silence only the bounded-prediction frames within 0.30 s; long denials (10 s, beyond the 5 s
// history window) never revive the last good sample.
TEST(GpsDenialRecovery, GpsIsNeverReusedBeyondMaxAgeAfterOnset) {
  {
    Rig rig;
    rig.Run(20, Tick{});
    Tick invalid; invalid.gps = Gps::kStatus0;
    const auto denied = rig.Run(100, invalid);
    CheckInvariants(denied, "invalid onset");
    EXPECT_EQ(CountMode(denied, 'M'), 0);
    EXPECT_EQ(CountMode(denied, 'L'), 100);
  }
  {
    Rig rig;
    rig.Run(20, Tick{});
    Tick silent; silent.gps = Gps::kSilent;
    const auto quiet = rig.Run(100, silent);
    CheckInvariants(quiet, "silence");
    EXPECT_EQ(CountMode(quiet, 'M'), 2);
    int beyond = 0;
    for (const auto& s : quiet) beyond += s.mode() == 'M' && s.gps_age() > kMaxGpsAge;
    EXPECT_EQ(beyond, 0);
    Tick one; // a single good sample after 10 s: recovering, not a revival of the old track
    Expect(rig.Run(1, one), "L", "single sample after 10 s silence");
  }
  {
    Rig rig;
    rig.Run(20, Tick{});
    Tick stale; stale.gps_lag = 500 * kMs;
    const auto late = rig.Run(100, stale);
    CheckInvariants(late, "stale stamps");
    EXPECT_EQ(CountMode(late, 'M'), 2);
  }
}

// Real Parser.cpp callbacks (common geometry): every malformed GPS message becomes an invalid barrier
// at its own source stamp (0 for a zero header), so the map pose of the next scan is invalid and
// the scan is local_v2 only; a zero quaternion or a NaN vehicle speed invalidates the local pose
// (no V2 at all) until a newer valid sample arrives. ConfigureForTest leaves the map origin at 0,
// so a well-formed fix fails the sanity bounds here and is a barrier too; acceptance of good fixes
// is covered by the rig above and by the netns integration matrix (real map origin).
TEST(GpsDenialRecovery, ParserCallbacksTurnEveryMalformedInputIntoABarrier) {
  UseCommonGeometry();
  const Ns base = 1900000000LL * kSec;
  const double lat = 37.2400, lon = 126.7700;
  auto quat = [](double r, double p, double y) {
    geometry_msgs::Quaternion q;
    const double cr = std::cos(r / 2), sr = std::sin(r / 2), cp = std::cos(p / 2), sp = std::sin(p / 2);
    const double cy = std::cos(y / 2), sy = std::sin(y / 2);
    q.x = sr * cp * cy - cr * sp * sy; q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy; q.w = cr * cp * cy + sr * sp * sy;
    return q;
  };
  auto imu = [&](Ns stamp, bool valid) {
    auto m = boost::make_shared<sensor_msgs::Imu>();
    m->header.stamp.fromNSec(std::uint64_t(stamp));
    // Heading 0: an all-zero quaternion also decodes to yaw 0, so only the |q| == 1 check (not the IMU
    // yaw-jump gate) can reject it.
    if (valid) m->orientation = quat(kRoll, kPitch, 0.0);   // else all-zero quaternion (|q| = 0)
    IMUParser_Callback(m);
  };
  auto can = [&](Ns stamp, double speed) {
    auto m = boost::make_shared<morai_msgs::EgoVehicleStatus>();
    m->header.stamp.fromNSec(std::uint64_t(stamp));
    m->velocity.x = speed;
    CANParser_Callback(m);
  };
  struct GpsMsg { const char* name; int status; double lat, lon, alt, east, north; bool zero_stamp; };
  auto gps = [&](Ns stamp, const GpsMsg& g) {
    auto m = boost::make_shared<morai_msgs::GPSMessage>();
    if (!g.zero_stamp) m->header.stamp.fromNSec(std::uint64_t(stamp));
    m->status = g.status; m->latitude = g.lat; m->longitude = g.lon; m->altitude = g.alt;
    m->eastOffset = g.east; m->northOffset = g.north;
    GPSParser_Callback(m);
  };
  auto scan = [&](Ns s) {
    ScanOut out; out.stamp = s;
    GPS_IMU pose{}; LocalInputSelection selection;
    out.pose_ready = LookupLocalStateForLidarNs(std::uint64_t(s), &pose, &selection, &out.pose_reason);
    LiDAR lidar;
    if (out.pose_ready) {
      const auto now = Ns(fusion_trace::SteadyNowNs());
      const asmc::lidar::Observation time{s, now, now, "ros", "bridge_receive_ns", "steady"};
      out.map = LookupMapPoseForLidar(time, pose);
      frame_boundary::PrepareScan(time, pose, st_VehicleSpec, out.map, 1);
      frame_boundary::StoreLeveling(asmc::lidar::leveling_rotation(kRoll, kPitch,
          frame_boundary::SensorMount(st_VehicleSpec).rotation(), true, true));
      lidar.frame_context = frame_boundary::Current();
    }
    out.in = frame_boundary::GateInputs(out.pose_ready, out.pose_reason, out.pose_ready, lidar, 1);
    out.d = output_gate::Decide(out.in);
    return out;
  };
  int k = 0;
  auto next = [&]() { return base + Ns(k++) * 100 * kMs; };
  auto fresh_local = [&](Ns s) { imu(s - 40 * kMs, true); imu(s - 5 * kMs, true); can(s - 5 * kMs, 2.0); };

  Ns s = next();
  fresh_local(s);
  auto first = scan(s);
  ASSERT_TRUE(first.pose_ready) << first.pose_reason;
  EXPECT_EQ(first.map.reason, "gps_missing");
  EXPECT_EQ(first.mode(), 'L');
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const GpsMsg malformed[] = {
      {"status0", 0, lat, lon, 30.0, 0.0, 0.0, false},
      {"latlon0", 1, 0.0, 0.0, 30.0, 0.0, 0.0, false},
      {"stamp0", 1, lat, lon, 30.0, 0.0, 0.0, true},
      {"nan_latitude", 1, nan, lon, 30.0, 0.0, 0.0, false},
      {"nan_altitude", 1, lat, lon, nan, 0.0, 0.0, false},
      {"wrong_east_offset", 1, lat, lon, 30.0, 1.0, 0.0, false},
      {"outside_sanity_bounds", 1, lat, lon, 30.0, 0.0, 0.0, false},
  };
  for (const auto& g : malformed) {
    SCOPED_TRACE(g.name);
    s = next();
    fresh_local(s);
    gps(s - 30 * kMs, g);
    const auto out = scan(s);
    ASSERT_TRUE(out.pose_ready) << out.pose_reason;      // local input does not depend on GPS
    EXPECT_FALSE(out.map.valid);
    EXPECT_EQ(out.map.reason, "gps_invalid");
    ASSERT_EQ(out.map.sample_stamps.size(), 1U);
    EXPECT_EQ(out.map.sample_stamps[0], g.zero_stamp ? 0 : s - 30 * kMs);   // this message's barrier
    EXPECT_EQ(out.mode(), 'L');
    EXPECT_EQ(out.d.reason, "local_only:gps_invalid");
  }
  // IMU with an all-zero quaternion -> b_ImuReady=false -> invalid barrier -> no V2 at all.
  s = next();
  imu(s - 40 * kMs, false); imu(s - 5 * kMs, false); can(s - 5 * kMs, 2.0);
  auto out = scan(s);
  EXPECT_FALSE(out.pose_ready);
  EXPECT_EQ(out.pose_reason, "imu_input_invalid");
  EXPECT_EQ(out.mode(), 'N');
  s = next();
  fresh_local(s);
  out = scan(s);
  EXPECT_TRUE(out.pose_ready) << out.pose_reason;
  EXPECT_EQ(out.mode(), 'L');
  // Vehicle speed NaN with a valid header -> invalid barrier -> no V2 at all.
  s = next();
  imu(s - 40 * kMs, true); imu(s - 5 * kMs, true); can(s - 5 * kMs, nan);
  out = scan(s);
  EXPECT_FALSE(out.pose_ready);
  EXPECT_EQ(out.pose_reason, "speed_input_invalid");
  EXPECT_EQ(out.mode(), 'N');
  s = next();
  fresh_local(s);
  out = scan(s);
  EXPECT_TRUE(out.pose_ready) << out.pose_reason;
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::Time::init();
  return RUN_ALL_TESTS();
}
