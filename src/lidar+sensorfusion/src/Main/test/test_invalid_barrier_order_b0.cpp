// B0 / F1 regression (integration scenario S11_imu_quat_invalid): an invalid IMU or vehicle-speed sample
// is a barrier for every lookup whose window [selected valid sample, query source stamp] contains it,
// INDEPENDENT of when later samples arrive. SensorPoseHistory used to keep ONE max-stamp barrier: the next
// invalid sample (stamped after the scan) that was processed while WaitForLocalLookupNs waited
// (lidar_pose_wait_timeout_s = 0.05) moved the barrier past the scan, and the final lookup selected the
// 105 ms old valid IMU again (SensorPoseHistory.hpp: future samples are not used, so online and bag replay
// do not depend on callback timing; docs/contracts/perception/lidar-object-v2.md section 4).
// ROS-free: no master, no NodeHandle, no socket. Seams: frame_boundary::ConfigureForTest, ros::Time::init().
// Arrival order is controlled by call order, so every case is deterministic. No case depends on another case, on
// --gtest_repeat iterations or on --gtest_shuffle order: every case uses its own local SensorPoseHistory, except the
// Parser-path cases, which share the process-global history of Parser.cpp and therefore each take a fresh base time
// (NextParserBase) and first check that nothing left in that history is usable at it (NothingUsableAt).
#include <gtest/gtest.h>

#include <Geometry/FrameBoundary.hpp>
#include <Parser/Parser.hpp>
#include <Parser/SensorPoseHistory.hpp>

#include <boost/make_shared.hpp>
#include <morai_msgs/EgoVehicleStatus.h>
#include <sensor_msgs/Imu.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <thread>
#include <vector>

extern VehicleSpec st_VehicleSpec;

namespace {
using Ns = std::int64_t;
constexpr Ns kMs = 1000000LL;
constexpr Ns kSec = 1000000000LL;
constexpr double kYaw = 0.3;

void UseGeometry(bool common) {
  frame_boundary::ConfigureForTest(common, true, "unit_test_calibration");
  st_VehicleSpec = VehicleSpec{};
  st_VehicleSpec.st_LiDARPos = {0.80f, 0.0f, 1.45f};
  st_VehicleSpec.st_LiDARRotationDeg = {0.f, 0.f, 0.f};
  st_VehicleSpec.st_GPSPos = {0.35f, 0.0f, 1.25f};
}

IMU_DATA Imu(bool ready) {
  IMU_DATA d{};
  d.f32_Roll_rad = 0.02f; d.f32_Pitch_rad = -0.01f; d.f32_Yaw_rad = float(kYaw);
  d.b_ImuReady = ready;                       // IMUParser_Callback: |q| != 1 -> not ready
  return d;
}
CAN_DATA Can(double mps) {
  CAN_DATA d{};
  d.f32_CurrentVel_MPS = float(mps); d.f32_CurrentVel_KPH = float(mps * 3.6);
  return d;
}

// Full-ns API exactly as Parser.cpp calls it with common geometry; every sample arrives 1 ms after its stamp.
void AddImu(SensorPoseHistory& h, Ns stamp, bool ready) {
  h.AddImu(std::uint64_t(stamp / kMs), Imu(ready), nullptr, std::uint64_t(stamp), std::uint64_t(stamp + kMs));
}
void AddCan(SensorPoseHistory& h, Ns stamp, double mps) {
  h.AddCan(std::uint64_t(stamp / kMs), Can(mps), nullptr, std::uint64_t(stamp), std::uint64_t(stamp + kMs));
}
// CANParser_Callback with a zero header: speed NaN, stamp_ms = receipt time, source_ns = 0.
void AddCanStamp0(SensorPoseHistory& h, Ns receipt) {
  h.AddCan(std::uint64_t(receipt / kMs), Can(std::numeric_limits<double>::quiet_NaN()), nullptr, 0,
           std::uint64_t(receipt));
}
void AddGps(SensorPoseHistory& h, Ns stamp, Ns arrival) {
  GPS_DATA g{}; g.f64_E = 100.0; g.f64_N = 200.0; g.f64_U = 30.0; g.b_isJamming = false;
  h.AddGps(std::uint64_t(stamp / kMs), g, std::uint64_t(stamp), std::uint64_t(arrival));
}

struct Local {
  bool ready = false;
  std::string reason;
  Ns sel_imu = 0, sel_can = 0;
};
Local Lookup(const SensorPoseHistory& h, Ns s, double timeout_s = 0.0) {
  Local out; GPS_IMU pose{}; LocalInputSelection sel;
  out.ready = h.WaitForLocalLookupNs(std::uint64_t(s), timeout_s, &pose, &sel, &out.reason);
  if (out.ready) { out.sel_imu = Ns(sel.imu_stamp_ns) - s; out.sel_can = Ns(sel.can_stamp_ns) - s; }
  return out;
}

// Parser-path cases drive the real callbacks, which write the process-global history of Parser.cpp (internal
// linkage, no reset seam). Each such case takes a fresh base from a process-wide monotonic counter, 1000 s after the
// previous one: far beyond history_duration_s (5 s) and stream_reanchor_after_s (2 s), so nothing an earlier case,
// --gtest_repeat iteration or --gtest_shuffle order left behind can be selected, lie as a barrier inside a lookup
// window, or reject a new sample as "timestamp moved backwards". Every base stays years ahead of the wall clock that
// dates zero headers. (Fixed bases were order-dependent: the 2.1e9 s case run first made the 2.0e9 s case fail.)
Ns NextParserBase() {
  static Ns next = 2000000000LL * kSec;
  const Ns base = next;
  next += 1000 * kSec;
  return base;
}
// Precondition of every Parser-path case: the global history holds nothing usable at its base (fresh process: no
// IMU at all; after other cases or iterations: only samples at least 997 s old).
::testing::AssertionResult NothingUsableAt(Ns base) {
  Local out; GPS_IMU pose{}; LocalInputSelection sel;
  out.ready = LookupLocalStateForLidarNs(std::uint64_t(base), &pose, &sel, &out.reason);
  if (out.ready) return ::testing::AssertionFailure() << "a sample left in the global history is usable at the base";
  if (out.reason != "no IMU at or before LiDAR scan" && out.reason != "IMU is stale for LiDAR scan")
    return ::testing::AssertionFailure() << "unexpected global-history state at the base: " << out.reason;
  return ::testing::AssertionSuccess();
}

// The integration feeder's tick (b0_feeder.py / fault_catalogue_g4.json): IMU S-40 and S-5, GPS S-30,
// CAN S-5, LiDAR scan at S, 10 Hz.
constexpr Ns kT0 = 1800000000LL * kSec;
Ns Tick(int k) { return kT0 + Ns(k) * 100 * kMs; }
void GoodTicks(SensorPoseHistory& h, int n, bool gps) {
  for (int k = 0; k < n; ++k) {
    const Ns s = Tick(k);
    AddImu(h, s - 40 * kMs, true);
    if (gps) AddGps(h, s - 30 * kMs, s - 29 * kMs);
    AddImu(h, s - 5 * kMs, true);
    AddCan(h, s - 5 * kMs, 2.0);
    const auto scan = Lookup(h, s);
    ASSERT_TRUE(scan.ready) << "good tick " << k << ": " << scan.reason;
    ASSERT_EQ(scan.sel_imu, -5 * kMs);
  }
}
}  // namespace

// RED on the single max-stamp barrier: the first scan after the IMU-invalid onset must be NONE for
// every arrival order of the onset samples and of the NEXT tick's invalid samples (S+60, S+95) that a
// delayed or waiting lookup can already see. The answer must equal the in-order answer.
TEST(InvalidBarrierOrderB0, ImuInvalidOnsetIsNoneForEveryArrivalOrderOfLaterInvalidSamples) {
  UseGeometry(true);
  const std::vector<std::vector<Ns>> onset_orders = {{-40, -5}, {-5, -40}};
  const std::vector<std::vector<Ns>> late_sets = {{}, {60}, {95}, {60, 95}, {95, 60}};
  for (const auto& onset : onset_orders) {
    for (const auto& late : late_sets) {
      SCOPED_TRACE(::testing::Message() << "onset order " << onset[0] << "," << onset[1]
                                        << " late invalid samples before the lookup: " << late.size());
      SensorPoseHistory h;
      GoodTicks(h, 20, false);
      const Ns s = Tick(20);
      for (const Ns off : onset) AddImu(h, s + off * kMs, false);
      AddCan(h, s - 5 * kMs, 2.0);
      for (const Ns off : late) AddImu(h, s + off * kMs, false);
      const auto scan = Lookup(h, s);
      EXPECT_FALSE(scan.ready) << "scan S reused the valid IMU at S" << scan.sel_imu / kMs << " ms";
      EXPECT_EQ(scan.reason, "imu_input_invalid");
    }
  }
}

// RED: main.cpp TemporalThread order. The lookup is already waiting (first try failed with
// imu_input_invalid) when the next invalid IMU sample (S+60, AddImu invalid path does not notify) and a GPS
// sample (AddGps notifies) arrive. Whatever the interleaving, the scan must stay NONE.
TEST(InvalidBarrierOrderB0, ImuInvalidOnsetStaysNoneWhileTheNextInvalidSampleArrivesDuringTheWait) {
  UseGeometry(true);
  SensorPoseHistory h;
  GoodTicks(h, 20, true);
  const Ns s = Tick(20);
  AddImu(h, s - 40 * kMs, false);
  AddGps(h, s - 30 * kMs, s - 29 * kMs);
  AddImu(h, s - 5 * kMs, false);
  AddCan(h, s - 5 * kMs, 2.0);
  Local waited;
  std::thread worker([&] { waited = Lookup(h, s, 0.30); });
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  AddImu(h, s + 60 * kMs, false);
  AddGps(h, s + 70 * kMs, s + 71 * kMs);      // wakes WaitForLocalLookupNs
  worker.join();
  EXPECT_FALSE(waited.ready) << "selected IMU S" << waited.sel_imu / kMs << " ms";
  EXPECT_EQ(waited.reason, "imu_input_invalid");
  const auto again = Lookup(h, s);           // and the decision is not a function of the wait
  EXPECT_FALSE(again.ready);
  EXPECT_EQ(again.reason, "imu_input_invalid");
}

// RED: the real Parser callbacks (all-zero quaternion like the S11 feeder) and the real 50 ms pose wait
// (LookupLocalStateForLidarNs, Parser.cpp). The next tick's first invalid sample is processed before the
// scan's lookup ends.
TEST(InvalidBarrierOrderB0, ParserPathQuatInvalidOnsetIsNoneWhenTheNextInvalidSampleArrivesFirst) {
  UseGeometry(true);
  const Ns base = NextParserBase();        // Parser globals: fresh base, independent of every other case
  ASSERT_TRUE(NothingUsableAt(base));
  auto imu = [](Ns stamp, bool valid) {
    auto m = boost::make_shared<sensor_msgs::Imu>();
    m->header.stamp.fromNSec(std::uint64_t(stamp));
    if (valid) m->orientation.w = 1.0;       // heading 0; else all-zero quaternion (|q| = 0), yaw also 0
    IMUParser_Callback(m);
  };
  auto can = [](Ns stamp) {
    auto m = boost::make_shared<morai_msgs::EgoVehicleStatus>();
    m->header.stamp.fromNSec(std::uint64_t(stamp));
    m->velocity.x = 2.0;
    CANParser_Callback(m);
  };
  auto scan = [](Ns s) {
    Local out; GPS_IMU pose{}; LocalInputSelection sel;
    out.ready = LookupLocalStateForLidarNs(std::uint64_t(s), &pose, &sel, &out.reason);
    if (out.ready) out.sel_imu = Ns(sel.imu_stamp_ns) - s;
    return out;
  };
  int k = 0;
  for (; k < 20; ++k) {
    const Ns s = base + Ns(k) * 100 * kMs;
    imu(s - 40 * kMs, true); imu(s - 5 * kMs, true); can(s - 5 * kMs);
    ASSERT_TRUE(scan(s).ready);
  }
  const Ns s = base + Ns(k) * 100 * kMs;
  imu(s - 40 * kMs, false); imu(s - 5 * kMs, false); can(s - 5 * kMs);
  imu(s + 60 * kMs, false);                  // next tick's sample, processed before the lookup's deadline
  const auto onset = scan(s);
  EXPECT_FALSE(onset.ready) << "selected IMU S" << onset.sel_imu / kMs << " ms";
  EXPECT_EQ(onset.reason, "imu_input_invalid");
  // The rest of the invalid phase is NONE by freshness, and recovery needs only fresh valid samples.
  const Ns s1 = s + 100 * kMs;
  imu(s1 - 5 * kMs, false); can(s1 - 5 * kMs);
  const auto next = scan(s1);
  EXPECT_FALSE(next.ready);
  EXPECT_EQ(next.reason, "IMU is stale for LiDAR scan");
  const Ns s2 = s1 + 100 * kMs;
  imu(s2 - 40 * kMs, true); imu(s2 - 5 * kMs, true); can(s2 - 5 * kMs);
  const auto back = scan(s2);
  EXPECT_TRUE(back.ready) << back.reason;
  EXPECT_EQ(back.sel_imu, -5 * kMs);
}

// RED: the same masking for the vehicle-speed barrier (NaN speed with a valid stamp, and the zero-stamp
// message whose barrier is its receipt time) when the next tick's invalid sample arrives first.
TEST(InvalidBarrierOrderB0, CanInvalidOnsetIsNoneWhenTheNextInvalidSampleArrivesFirst) {
  UseGeometry(true);
  {
    SensorPoseHistory h;
    GoodTicks(h, 20, false);
    const Ns s = Tick(20);
    AddImu(h, s - 40 * kMs, true); AddImu(h, s - 5 * kMs, true);
    AddCan(h, s - 5 * kMs, std::numeric_limits<double>::quiet_NaN());
    AddCan(h, s + 95 * kMs, std::numeric_limits<double>::quiet_NaN());
    const auto scan = Lookup(h, s);
    EXPECT_FALSE(scan.ready) << "selected CAN S" << scan.sel_can / kMs << " ms";
    EXPECT_EQ(scan.reason, "speed_input_invalid");
  }
  {
    SensorPoseHistory h;
    GoodTicks(h, 20, false);
    const Ns s = Tick(20);
    AddImu(h, s - 40 * kMs, true); AddImu(h, s - 5 * kMs, true);
    AddCanStamp0(h, s - 4 * kMs);
    AddCanStamp0(h, s + 96 * kMs);
    const auto scan = Lookup(h, s);
    EXPECT_FALSE(scan.ready) << "selected CAN S" << scan.sel_can / kMs << " ms";
    EXPECT_EQ(scan.reason, "speed_input_invalid");
  }
}

// RED: the GPS attitude gate (AddGps) used the same max-stamp barrier. A GPS sample processed after an
// invalid IMU stamped later than the GPS must still fail the gate (the window [attitude, GPS] contains the
// invalid S-40 sample), so it is not counted as a valid map sample.
TEST(InvalidBarrierOrderB0, GpsAttitudeGateHoldsWhenALaterInvalidImuIsProcessedFirst) {
  UseGeometry(true);
  SensorPoseHistory h;
  GoodTicks(h, 20, true);
  const Ns s = Tick(20);
  AddImu(h, s - 40 * kMs, false);
  AddImu(h, s - 5 * kMs, false);
  AddGps(h, s - 30 * kMs, s - 2 * kMs);       // GPS callback runs after both invalid IMU callbacks
  GPS_IMU local{}; local.st_IMU = Imu(true); local.st_CAN = Can(2.0);
  const asmc::lidar::Observation t{s, s + kMs, s + 2 * kMs, "ros", "bridge_receive_ns", "steady"};
  const auto map = h.MapAt(t, local);
  EXPECT_FALSE(map.valid) << map.reason;
  EXPECT_EQ(map.reason, "gps_invalid");
}

// Guards (GREEN before and after the fix): the barrier must not over-reach.
TEST(InvalidBarrierOrderB0, LaterInvalidSampleNeverInvalidatesAnEarlierScan) {
  UseGeometry(true);
  SensorPoseHistory h;
  GoodTicks(h, 20, false);
  const Ns s = Tick(20);
  AddImu(h, s - 40 * kMs, true); AddImu(h, s - 5 * kMs, true); AddCan(h, s - 5 * kMs, 2.0);
  AddImu(h, s + 60 * kMs, false); AddCan(h, s + 95 * kMs, std::numeric_limits<double>::quiet_NaN());
  const auto scan = Lookup(h, s);
  EXPECT_TRUE(scan.ready) << scan.reason;
  EXPECT_EQ(scan.sel_imu, -5 * kMs);
  EXPECT_EQ(scan.sel_can, -5 * kMs);
}

TEST(InvalidBarrierOrderB0, ValidSampleAfterTheBarrierEndsItAndANewerBarrierStartsAgain) {
  UseGeometry(true);
  {
    SensorPoseHistory h;
    GoodTicks(h, 20, false);
    const Ns s = Tick(20);
    AddImu(h, s - 40 * kMs, false); AddImu(h, s - 5 * kMs, true); AddCan(h, s - 5 * kMs, 2.0);
    const auto scan = Lookup(h, s);
    EXPECT_TRUE(scan.ready) << scan.reason;
    EXPECT_EQ(scan.sel_imu, -5 * kMs);
  }
  {
    SensorPoseHistory h;
    GoodTicks(h, 20, false);
    const Ns s = Tick(20);
    AddImu(h, s - 40 * kMs, false); AddImu(h, s - 20 * kMs, true); AddImu(h, s - 5 * kMs, false);
    AddCan(h, s - 5 * kMs, 2.0);
    const auto scan = Lookup(h, s);
    EXPECT_FALSE(scan.ready);
    EXPECT_EQ(scan.reason, "imu_input_invalid");
  }
}

// A long invalid stream (60 s at 40 Hz, above the retained-stamp cap) followed by valid samples: the scans
// recover, and a new onset is again NONE although many older invalid stamps were dropped.
TEST(InvalidBarrierOrderB0, LongInvalidStreamRecoversAndANewOnsetIsStillNone) {
  UseGeometry(true);
  SensorPoseHistory h;
  GoodTicks(h, 5, false);
  int k = 5;
  for (; k < 605; ++k) {
    const Ns s = Tick(k);
    AddImu(h, s - 40 * kMs, false); AddImu(h, s - 30 * kMs, false);
    AddImu(h, s - 15 * kMs, false); AddImu(h, s - 5 * kMs, false);
    AddCan(h, s - 5 * kMs, 2.0);
    EXPECT_FALSE(Lookup(h, s).ready);
  }
  for (int r = 0; r < 3; ++r, ++k) {
    const Ns s = Tick(k);
    AddImu(h, s - 40 * kMs, true); AddImu(h, s - 5 * kMs, true); AddCan(h, s - 5 * kMs, 2.0);
    const auto scan = Lookup(h, s);
    EXPECT_TRUE(scan.ready) << "recovery tick " << r << ": " << scan.reason;
  }
  const Ns s = Tick(k);
  AddImu(h, s - 40 * kMs, false); AddImu(h, s - 5 * kMs, false); AddCan(h, s - 5 * kMs, 2.0);
  AddImu(h, s + 60 * kMs, false);
  const auto onset = Lookup(h, s);
  EXPECT_FALSE(onset.ready);
  EXPECT_EQ(onset.reason, "imu_input_invalid");
}

// Production legacy path (enable_common_geometry=false): barriers are neither recorded nor consulted, as
// before. Legacy Parser calls pass source_ns = 0 and use the ms lookups.
TEST(InvalidBarrierOrderB0, LegacyPathNeverConsultsBarriers) {
  UseGeometry(false);
  SensorPoseHistory h;
  for (int k = 0; k < 20; ++k) {
    const Ns s = Tick(k);
    h.AddImu(std::uint64_t((s - 40 * kMs) / kMs), Imu(true), nullptr, 0, 0);
    h.AddImu(std::uint64_t((s - 5 * kMs) / kMs), Imu(true), nullptr, 0, 0);
    h.AddCan(std::uint64_t((s - 5 * kMs) / kMs), Can(2.0), nullptr, 0, 0);
  }
  const Ns s = Tick(20);
  h.AddImu(std::uint64_t((s - 40 * kMs) / kMs), Imu(false), nullptr, 0, 0);
  h.AddImu(std::uint64_t((s - 5 * kMs) / kMs), Imu(false), nullptr, 0, 0);
  h.AddCan(std::uint64_t((s - 5 * kMs) / kMs), Can(std::numeric_limits<double>::quiet_NaN()), nullptr, 0, 0);
  GPS_IMU pose{}; std::string reason;
  EXPECT_TRUE(h.WaitForLocalLookup(std::uint64_t(s / kMs), 0.0, &pose, &reason)) << reason;
  EXPECT_EQ(reason, "local_ready");
  h.AddImu(std::uint64_t((s + 60 * kMs) / kMs), Imu(false), nullptr, 0, 0);
  EXPECT_TRUE(h.WaitForLocalLookup(std::uint64_t(s / kMs), 0.0, &pose, &reason)) << reason;
  frame_boundary::ConfigureForTest(true, true, "unit_test_calibration");
}

// F1b (integration scenario S16_can_stamp0): a zero-header vehicle-speed sample has no source time; Parser dates
// it with its callback receipt ROS time. When the callback runs a few ms after the scan stamp (Main's spin
// cadence, 0.4-11 ms observed) the receipt-dated barrier lies after the scan and the 105 ms old valid CAN was
// reused. The sample arrived after that valid sample and before the lookup, so it must not be reused.
TEST(InvalidBarrierOrderB0, ZeroHeaderCanReceivedJustAfterTheScanStampStillBlocksTheOldSample) {
  UseGeometry(true);
  for (const Ns receipt : {Ns(-4), Ns(1), Ns(7), Ns(11)}) {
    SCOPED_TRACE(::testing::Message() << "zero-header CAN receipt S" << receipt << " ms");
    SensorPoseHistory h;
    GoodTicks(h, 20, false);
    const Ns s = Tick(20);
    AddImu(h, s - 40 * kMs, true); AddImu(h, s - 5 * kMs, true);
    AddCanStamp0(h, s + receipt * kMs);
    const auto scan = Lookup(h, s);
    EXPECT_FALSE(scan.ready) << "selected CAN S" << scan.sel_can / kMs << " ms";
    EXPECT_EQ(scan.reason, "speed_input_invalid");
  }
  // A valid sample after the zero-header one ends the barrier.
  SensorPoseHistory h;
  GoodTicks(h, 20, false);
  const Ns s = Tick(20);
  AddImu(h, s - 40 * kMs, true); AddImu(h, s - 5 * kMs, true);
  AddCanStamp0(h, s - 30 * kMs);
  AddCan(h, s - 5 * kMs, 2.0);
  const auto scan = Lookup(h, s);
  EXPECT_TRUE(scan.ready) << scan.reason;
  EXPECT_EQ(scan.sel_can, -5 * kMs);
}

// F1b through the real IMU callback: a zero header is dated with ros::Time::now() (wall clock), which has no
// relation to the source clock of the scan (here a source clock years ahead, cf. S09 sim epoch).
TEST(InvalidBarrierOrderB0, ParserPathZeroHeaderImuBlocksTheOldSampleWhateverTheReceiptClock) {
  UseGeometry(true);
  const Ns base = NextParserBase();        // Parser globals: fresh base, independent of every other case
  ASSERT_TRUE(NothingUsableAt(base));
  auto imu = [](Ns stamp, bool valid) {
    auto m = boost::make_shared<sensor_msgs::Imu>();
    if (stamp) m->header.stamp.fromNSec(std::uint64_t(stamp));
    m->orientation.w = valid ? 1.0 : 0.0;
    IMUParser_Callback(m);
  };
  auto can = [](Ns stamp) {
    auto m = boost::make_shared<morai_msgs::EgoVehicleStatus>();
    m->header.stamp.fromNSec(std::uint64_t(stamp));
    m->velocity.x = 2.0;
    CANParser_Callback(m);
  };
  auto scan = [](Ns s) {
    Local out; GPS_IMU pose{}; LocalInputSelection sel;
    out.ready = LookupLocalStateForLidarNs(std::uint64_t(s), &pose, &sel, &out.reason);
    return out;
  };
  for (int k = 0; k < 20; ++k) {
    const Ns s = base + Ns(k) * 100 * kMs;
    imu(s - 40 * kMs, true); imu(s - 5 * kMs, true); can(s - 5 * kMs);
    ASSERT_TRUE(scan(s).ready);
  }
  const Ns s = base + 20 * 100 * kMs;
  imu(s - 40 * kMs, true);
  imu(0, true);                              // zero header (b_ImuReady=false), unit quaternion
  can(s - 5 * kMs);
  const auto out = scan(s);
  EXPECT_FALSE(out.ready);
  EXPECT_EQ(out.reason, "imu_input_invalid");
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::Time::init();
  return RUN_ALL_TESTS();
}
