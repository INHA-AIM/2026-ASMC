// B0 output gate truth table (Geometry/OutputGate.hpp). Pure gtest: no ROS master, no sockets,
// no files. Every assertion encodes one row of the common-geometry truth table or the
// production (common geometry OFF) equivalence with the pre-B0 main.cpp rule.
#include <gtest/gtest.h>

#include <Geometry/OutputGate.hpp>
#include <asmc_lidar/bounded_pose.hpp>

#include <string>
#include <vector>

using output_gate::Decide;
using output_gate::Decision;
using output_gate::GpsState;
using output_gate::Inputs;
using output_gate::LegacyPlan;
using output_gate::Mode;
using output_gate::PlanLegacy;

namespace {

Inputs Healthy(const std::string& map_reason = "gps_observed") {
  Inputs in;
  in.common_geometry = true;
  in.pose_ready = true;
  in.pose_reason = "local_ready";
  in.lidar_processed = true;
  in.context_current = true;
  in.leveling_valid = true;
  in.map_pose_valid = true;
  in.map_pose_matches_scan = true;
  in.map_reason = map_reason;
  return in;
}
Inputs WithMapReason(const std::string& reason) {
  Inputs in = Healthy(reason);
  in.map_pose_valid = false;
  in.map_pose_matches_scan = false;
  return in;
}

// Pre-B0 main.cpp legacy rule (map_ready/legacy_ready), the reference for production equivalence.
bool PreB0LegacyPublish(const Inputs& in, bool legacy_motion_ready) {
  if (!in.common_geometry) return in.lidar_processed;
  const bool map_ready = in.map_pose_valid && in.time_error.empty();
  return in.lidar_processed && map_ready && legacy_motion_ready;
}

// Every map-pose reason the producers can emit (bounded_pose.cpp, SensorPoseHistory.cpp MapAt,
// frame_time_core.cpp validate_time, PoseSelection default) plus unknown/empty strings.
const std::vector<std::string> kGpsDenied{"gps_missing", "gps_invalid", "gps_stale"};
const std::vector<std::string> kGpsRecovering{"gps_recovering"};
const std::vector<std::string> kGpsHealthy{"gps_observed", "gps_bounded_prediction"};
const std::vector<std::string> kNonGps{
    "map_speed_invalid", "local_motion_invalid", "pose_missing", "zero_or_negative_stamp",
    "clock_unknown", "stamp_provenance_missing", "receipt_clock_unsupported", "receipt_before_stamp",
    "receipt_missing", "processing_before_receipt", "pose_invalid_or_time_mismatch", "", "new_reason"};
const std::vector<std::string> kLocalLookupFailures{
    "IMU is stale for LiDAR scan", "imu_input_invalid", "speed_input_invalid",
    "vehicle speed is stale for LiDAR scan", "no IMU at or before LiDAR scan",
    "no vehicle speed at or before LiDAR scan", "invalid local timestamp", ""};

std::vector<std::string> AllMapReasons() {
  std::vector<std::string> all = kGpsDenied;
  all.insert(all.end(), kGpsRecovering.begin(), kGpsRecovering.end());
  all.insert(all.end(), kGpsHealthy.begin(), kGpsHealthy.end());
  all.insert(all.end(), kNonGps.begin(), kNonGps.end());
  return all;
}

}  // namespace

// Production (common geometry OFF): no V2, no SuppressMap, no SetLegacyReason, and the pre-B0
// rule (publish iff lidar_processed) for motion-ready frames. KNOWN_GAP (B1): in this mode
// LegacyMotionReady is not applied, so warm-up frames still reach /last_lidar and UDP as before;
// that publication is deliberately NOT asserted as correct anywhere in these tests.
TEST(OutputGateB0, ProductionModeHasNoV2AndKeepsPreB0RuleForMotionReadyFrames) {
  for (bool pose : {false, true}) for (bool processed : {false, true}) for (bool motion : {false, true}) {
    if (processed && !pose) continue;  // main.cpp: lidar_processed = pose_ready && LidarProcess()
    for (const auto& reason : AllMapReasons()) {
      Inputs in; in.common_geometry = false; in.pose_ready = pose; in.lidar_processed = processed;
      in.map_reason = reason;
      const Decision d = Decide(in);
      EXPECT_EQ(d.v2, Mode::kNone);                // no V2 at all in production
      EXPECT_EQ(d.gps, GpsState::kNotEvaluated);
      const auto plan = PlanLegacy(d, in, motion);
      EXPECT_FALSE(plan.suppress_markers);         // SuppressMap only when Enabled()
      EXPECT_FALSE(plan.report_reason);            // SetLegacyReason only when Enabled()
      if (motion) {
        EXPECT_EQ(plan.publish, processed);
        EXPECT_EQ(plan.publish, PreB0LegacyPublish(in, motion));
      }
    }
  }
}

TEST(OutputGateB0, EveryGpsFaultClassMapsToItsOutput) {
  struct Row { std::string reason; Mode mode; GpsState gps; };
  std::vector<Row> rows;
  for (const auto& r : kGpsDenied) rows.push_back({r, Mode::kLocal, GpsState::kDenied});
  for (const auto& r : kGpsRecovering) rows.push_back({r, Mode::kLocal, GpsState::kRecovering});
  for (const auto& r : kNonGps) rows.push_back({r, Mode::kNone, GpsState::kNonGpsInvalid});
  for (const auto& row : rows) {
    const Decision d = Decide(WithMapReason(row.reason));
    EXPECT_EQ(d.v2, row.mode) << row.reason;
    EXPECT_EQ(d.gps, row.gps) << row.reason;
    EXPECT_EQ(output_gate::ClassifyGps(false, row.reason), row.gps) << row.reason;
    const std::string expected = row.mode == Mode::kLocal
        ? "local_only:" + row.reason
        : "map_pose_invalid:" + (row.reason.empty() ? std::string("unknown") : row.reason);
    EXPECT_EQ(d.reason, expected);
    const auto plan = PlanLegacy(d, WithMapReason(row.reason), true);
    EXPECT_FALSE(plan.publish) << row.reason;           // never legacy without a valid map pose
    EXPECT_TRUE(plan.suppress_markers) << row.reason;   // map markers DELETEALL
    EXPECT_EQ(plan.reason, "pose_unavailable") << row.reason;
  }
  for (const auto& r : kGpsHealthy) {
    EXPECT_EQ(output_gate::ClassifyGps(true, r), GpsState::kHealthy);
    const Decision d = Decide(Healthy(r));
    EXPECT_EQ(d.v2, Mode::kBoth) << r;
    EXPECT_EQ(d.reason, "local_and_map:" + r);
  }
  // A valid map pose is healthy whatever its reason text says.
  EXPECT_EQ(output_gate::ClassifyGps(true, "gps_stale"), GpsState::kHealthy);
}

TEST(OutputGateB0, LocalPoseInvalidPublishesNothing) {
  for (const auto& reason : kLocalLookupFailures) {
    Inputs in = Healthy(); in.pose_ready = false; in.pose_reason = reason; in.lidar_processed = false;
    const Decision d = Decide(in);
    EXPECT_EQ(d.v2, Mode::kNone) << reason;
    EXPECT_EQ(d.gps, GpsState::kNotEvaluated);
    EXPECT_FALSE(d.legacy_permitted);
    EXPECT_TRUE(d.suppress_legacy_markers);
    EXPECT_EQ(d.reason, "local_input_invalid:" + (reason.empty() ? std::string("unknown") : reason));
    const auto plan = PlanLegacy(d, in, true);
    EXPECT_FALSE(plan.publish); EXPECT_TRUE(plan.suppress_markers); EXPECT_EQ(plan.reason, "input_invalid");
  }
}

TEST(OutputGateB0, LidarTimeLevelingContextOrMapTransformInvalidPublishesNothing) {
  struct Case { const char* name; void (*mutate)(Inputs&); const char* reason; };
  const Case cases[] = {
      {"lidar", [](Inputs& in) { in.lidar_processed = false; }, "lidar_input_invalid"},
      {"context", [](Inputs& in) { in.context_current = false; }, "context_stale"},
      {"time", [](Inputs& in) { in.time_error = "receipt_before_stamp"; }, "time_invalid:receipt_before_stamp"},
      {"level", [](Inputs& in) { in.leveling_valid = false; }, "leveling_transform_invalid"},
      {"map_time", [](Inputs& in) { in.map_pose_matches_scan = false; }, "map_pose_time_mismatch"},
  };
  std::vector<std::string> reasons = kGpsHealthy;
  reasons.insert(reasons.end(), kGpsDenied.begin(), kGpsDenied.end());
  reasons.insert(reasons.end(), kGpsRecovering.begin(), kGpsRecovering.end());
  for (const auto& c : cases) for (const auto& gps : reasons) {
    Inputs in = Healthy(gps);
    in.map_pose_valid = in.map_pose_matches_scan = gps == "gps_observed" || gps == "gps_bounded_prediction";
    if (std::string(c.name) == "map_time" && !in.map_pose_valid) continue;
    c.mutate(in);
    const Decision d = Decide(in);
    EXPECT_EQ(d.v2, Mode::kNone) << c.name << "/" << gps;   // not even local_v2 during GPS denial
    EXPECT_EQ(d.reason, c.reason) << c.name;
    const auto plan = PlanLegacy(d, in, true);
    EXPECT_FALSE(plan.publish); EXPECT_TRUE(plan.suppress_markers);
  }
}

TEST(OutputGateB0, GpsDeniedOrRecoveringSelectsLocalOnly) {
  std::vector<std::string> reasons = kGpsDenied;
  reasons.insert(reasons.end(), kGpsRecovering.begin(), kGpsRecovering.end());
  for (const auto& reason : reasons) for (bool motion : {false, true}) {
    const Inputs in = WithMapReason(reason);
    const Decision d = Decide(in);
    EXPECT_EQ(d.v2, Mode::kLocal) << reason;
    EXPECT_FALSE(d.legacy_permitted);
    const auto plan = PlanLegacy(d, in, motion);
    EXPECT_FALSE(plan.publish);                 // no /last_lidar, /lidar/tracked_objects_map, motion_debug, UDP
    EXPECT_TRUE(plan.suppress_markers);         // map markers DELETEALL
    EXPECT_TRUE(plan.report_reason);
  }
}

TEST(OutputGateB0, HealthySelectsLocalAndMapAndKeepsFrameWideLegacyMotionGate) {
  for (const auto& reason : kGpsHealthy) {
    const Inputs in = Healthy(reason);
    const Decision d = Decide(in);
    EXPECT_EQ(d.v2, Mode::kBoth);
    EXPECT_TRUE(d.legacy_permitted);
    EXPECT_FALSE(d.suppress_legacy_markers);
    EXPECT_EQ(d.gps, GpsState::kHealthy);
    const auto ready = PlanLegacy(d, in, true);
    EXPECT_TRUE(ready.publish); EXPECT_FALSE(ready.suppress_markers); EXPECT_EQ(ready.reason, "available");
    const auto warmup = PlanLegacy(d, in, false);   // any warm-up object -> whole legacy frame withheld
    EXPECT_FALSE(warmup.publish); EXPECT_TRUE(warmup.suppress_markers);
    EXPECT_EQ(warmup.reason, "motion_state_unrepresentable");
  }
}

TEST(OutputGateB0, DowngradeAfterBuilderOrStampRejectionPublishesNothing) {
  for (const char* reason : {"invalid_object_dimensions", "nonfinite_or_invalid_geometry",
                             "v2_stamp_rejected", "context_stamp_mismatch"}) {
    const Inputs in = Healthy();
    const Decision d = output_gate::Downgrade(Decide(in), reason);
    EXPECT_EQ(d.v2, Mode::kNone);
    EXPECT_EQ(d.gps, GpsState::kHealthy);
    EXPECT_EQ(d.reason, reason);
    const auto plan = PlanLegacy(d, in, true);
    EXPECT_FALSE(plan.publish); EXPECT_TRUE(plan.suppress_markers); EXPECT_EQ(plan.reason, "frame_invalid");
  }
  // A local frame downgraded by the builder is silent too (no local_v2, no legacy).
  const Inputs denied = WithMapReason("gps_stale");
  const Decision d = output_gate::Downgrade(Decide(denied), "invalid_object_dimensions");
  EXPECT_EQ(d.v2, Mode::kNone); EXPECT_EQ(d.gps, GpsState::kDenied);
  EXPECT_FALSE(PlanLegacy(d, denied, true).publish);
}

// Map output is deactivated on the first denied scan: the legacy map markers get DELETEALL on
// that very scan and on every scan until map output and LegacyMotionReady are both back.
TEST(OutputGateB0, DeleteAllOnEveryScanWhileMapOutputIsDeactivated) {
  struct Scan { Inputs in; bool motion; Mode v2; bool publish; bool suppress; };
  const std::vector<Scan> scans{
      {Healthy("gps_observed"), true, Mode::kBoth, true, false},
      {WithMapReason("gps_invalid"), true, Mode::kLocal, false, true},          // deactivation
      {WithMapReason("gps_stale"), true, Mode::kLocal, false, true},
      {WithMapReason("gps_recovering"), true, Mode::kLocal, false, true},
      {Healthy("gps_bounded_prediction"), false, Mode::kBoth, false, true},    // tracks still warming up
      {Healthy("gps_bounded_prediction"), true, Mode::kBoth, true, false},     // legacy back
      {WithMapReason("map_speed_invalid"), true, Mode::kNone, false, true},    // CAN invalid: nothing
  };
  for (std::size_t i = 0; i < scans.size(); ++i) {
    const Decision d = Decide(scans[i].in);
    const LegacyPlan plan = PlanLegacy(d, scans[i].in, scans[i].motion);
    EXPECT_EQ(d.v2, scans[i].v2) << i;
    EXPECT_EQ(plan.publish, scans[i].publish) << i;
    EXPECT_EQ(plan.suppress_markers, scans[i].suppress) << i;
    EXPECT_NE(plan.publish, plan.suppress_markers) << i;
  }
}

// Exhaustive property check over every boolean combination and every reason class.
TEST(OutputGateB0, ExhaustiveInvariants) {
  const auto reasons = AllMapReasons();
  std::size_t checked = 0;
  for (int bits = 0; bits < (1 << 8); ++bits) for (const auto& reason : reasons) {
    Inputs in;
    in.common_geometry = bits & 1; in.pose_ready = bits & 2; in.lidar_processed = (bits & 4) && in.pose_ready;
    in.context_current = bits & 8; in.leveling_valid = bits & 16; in.map_pose_valid = bits & 32;
    in.map_pose_matches_scan = bits & 64; in.time_error = (bits & 128) ? "clock_unknown" : "";
    in.map_reason = reason; in.pose_reason = in.pose_ready ? "local_ready" : "imu_input_invalid";
    for (bool motion : {false, true}) {
      const Decision d = Decide(in);
      const auto plan = PlanLegacy(d, in, motion);
      ASSERT_FALSE(d.reason.empty());
      if (!in.common_geometry) {
        ASSERT_EQ(d.v2, Mode::kNone);
        if (motion) { ASSERT_EQ(plan.publish, PreB0LegacyPublish(in, motion)); }   // production unchanged
        ASSERT_FALSE(plan.suppress_markers);
        ASSERT_FALSE(plan.report_reason);
      } else {
        const bool inputs_valid = in.pose_ready && in.lidar_processed && in.context_current &&
            in.time_error.empty() && in.leveling_valid;
        const GpsState gps = output_gate::ClassifyGps(in.map_pose_valid, in.map_reason);
        if (plan.publish) { ASSERT_EQ(d.v2, Mode::kBoth); }                 // legacy only with map_v2
        if (plan.publish) { ASSERT_TRUE(motion); }                          // LegacyMotionReady kept
        if (plan.publish) { ASSERT_TRUE(PreB0LegacyPublish(in, motion)); }  // B0 only tightens
        if (d.v2 != Mode::kBoth) { ASSERT_FALSE(plan.publish); }
        ASSERT_EQ(plan.suppress_markers, !plan.publish);                    // DELETEALL iff withheld
        ASSERT_TRUE(plan.report_reason);
        if (!inputs_valid) { ASSERT_EQ(d.v2, Mode::kNone); }                // row 3: nothing on V2
        if (d.v2 == Mode::kBoth) {
          ASSERT_TRUE(inputs_valid && in.map_pose_valid && in.map_pose_matches_scan);
        }
        if (d.v2 == Mode::kLocal) {
          ASSERT_TRUE(inputs_valid);
          ASSERT_TRUE(gps == GpsState::kDenied || gps == GpsState::kRecovering);
        }
        if (inputs_valid && (gps == GpsState::kDenied || gps == GpsState::kRecovering)) {
          ASSERT_EQ(d.v2, Mode::kLocal);
        }
        if (inputs_valid && gps == GpsState::kHealthy && in.map_pose_matches_scan) {
          ASSERT_EQ(d.v2, Mode::kBoth);
          ASSERT_EQ(plan.publish, motion);
        }
        if (gps == GpsState::kNonGpsInvalid) { ASSERT_EQ(d.v2, Mode::kNone); }
      }
      ++checked;
    }
  }
  EXPECT_EQ(checked, 256U * reasons.size() * 2U);
}

// Recovery latch lives in BoundedPoseHistory (3 consecutive good samples). Feed its reasons
// through the gate: denial -> local only; the first scan with 3 good samples -> local and map.
TEST(OutputGateB0, BoundedPoseRecoveryDrivesLocalThenMap) {
  using asmc::lidar::Ns;
  asmc::lidar::BoundedPoseHistory history(300000000LL, 3);
  asmc::lidar::PoseSample s;
  s.clock = "ros"; s.valid = true; s.reference_is_gps = true; s.gps_in_base = asmc::lidar::Vec(.35, 0, 1.25);
  s.map_from_reference.translation() = asmc::lidar::Vec(100, 200, 5);
  auto add = [&](Ns t, bool valid, Ns arrival = -1) {
    s.stamp_ns = t; s.arrival_ns = arrival < 0 ? t : arrival; s.valid = valid;
    ASSERT_TRUE(history.append(s).empty());
  };
  auto mode_at = [&](Ns t) {
    const asmc::lidar::Observation time{t, t, t, "ros", "bridge_receive_ns", ""};
    const auto map = history.at(time, asmc::lidar::Rot::Identity(), asmc::lidar::Vec::Zero());
    Inputs in = Healthy(map.reason);
    in.map_pose_valid = map.valid;
    in.map_pose_matches_scan = map.valid && map.query_stamp_ns == time.stamp_ns && map.clock == time.clock;
    return Decide(in);
  };
  EXPECT_EQ(mode_at(100000000).reason, "local_only:gps_missing");           // startup, no GPS
  add(200000000, true);
  EXPECT_EQ(mode_at(250000000).reason, "local_only:gps_recovering");        // 1 good
  add(400000000, true);
  EXPECT_EQ(mode_at(450000000).reason, "local_only:gps_recovering");        // 2 good: still latched
  add(600000000, true);
  EXPECT_EQ(mode_at(650000000).v2, Mode::kBoth);                            // 3rd good: first map scan
  EXPECT_EQ(mode_at(901000000).reason, "local_only:gps_stale");             // silence > 0.30 s
  add(800000000, false, 905000000);                                         // invalid sample
  EXPECT_EQ(mode_at(910000000).reason, "local_only:gps_invalid");
  add(1000000000, true); add(1200000000, true);
  EXPECT_EQ(mode_at(1250000000).reason, "local_only:gps_recovering");
  add(1400000000, true);
  EXPECT_EQ(mode_at(1400000000).v2, Mode::kBoth);                           // recovers exactly at 3rd good
  // A gap above 0.30 s between good samples restarts the count.
  add(1800000000, true);
  EXPECT_EQ(mode_at(1800000000).reason, "local_only:gps_recovering");
  s.stamp_ns = 0; s.arrival_ns = 1850000000; s.valid = false; s.arrival_clock = "";
  // A zero-stamp barrier is accepted only with a steady arrival clock (Parser common path).
  EXPECT_EQ(history.append(s), "pose_time_invalid");
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
