#include <asmc_lidar/tracking/track.hpp>
// Ports of Main test_fusion_time_matcher.cpp, test_fusion_class_policy.cpp,
// test_fusion_replay.cpp and test_fusion_state_propagation.cpp (8f93c22e) onto
// TrackedObject and the explicit mission_profile flag, plus the box filter
// assertions of Main test_camera_objects_adapter.py.

#include <asmc_lidar/fusion/fusion_policy.hpp>

#include <gtest/gtest.h>

#include <deque>
#include <limits>
#include <map>
#include <string>
#include <tuple>
#include <vector>

using namespace asmc::lidar::pipeline;

namespace {
constexpr std::int64_t kMs = 1000000;
constexpr bool kLegacy = false;  // Main fusion_class::MissionProfile() static default
constexpr bool kMission = true;  // mission16_v1 (runtime default)
}  // namespace

// ---------------------------------------------------------------------------
// test_fusion_time_matcher.cpp

TEST(FusionTimeMatcher, WaitsForFutureFrameBeforeChoosingNearest) {
  const auto decision = fusion_sync::DecideMatch(100 * kMs, {72 * kMs, 91 * kMs}, 80 * kMs, 50 * kMs, 200 * kMs);
  EXPECT_EQ(fusion_sync::MatchAction::Wait, decision.action);
}

TEST(FusionTimeMatcher, ChoosesCloserPastAfterFutureArrives) {
  const auto decision =
      fusion_sync::DecideMatch(100 * kMs, {72 * kMs, 91 * kMs, 132 * kMs}, 90 * kMs, 50 * kMs, 200 * kMs);
  EXPECT_EQ(fusion_sync::MatchAction::Match, decision.action);
  EXPECT_EQ(1u, decision.yolo_index);
  EXPECT_EQ(9 * kMs, decision.offset_ns);
}

TEST(FusionTimeMatcher, ChoosesCloserFutureFrame) {
  const auto decision = fusion_sync::DecideMatch(100 * kMs, {61 * kMs, 118 * kMs}, 90 * kMs, 50 * kMs, 200 * kMs);
  EXPECT_EQ(fusion_sync::MatchAction::Match, decision.action);
  EXPECT_EQ(1u, decision.yolo_index);
  EXPECT_EQ(18 * kMs, decision.offset_ns);
}

TEST(FusionTimeMatcher, DropsOnlyAfterDecisionIsSettled) {
  auto decision = fusion_sync::DecideMatch(100 * kMs, {30 * kMs}, 199 * kMs, 50 * kMs, 200 * kMs);
  EXPECT_EQ(fusion_sync::MatchAction::Wait, decision.action);
  decision = fusion_sync::DecideMatch(100 * kMs, {30 * kMs}, 200 * kMs, 50 * kMs, 200 * kMs);
  EXPECT_EQ(fusion_sync::MatchAction::Drop, decision.action);
}

TEST(FusionTimeMatcher, NeverRelaxesMaximumOffsetAtTimeout) {
  const auto decision = fusion_sync::DecideMatch(100 * kMs, {40 * kMs, 160 * kMs}, 250 * kMs, 50 * kMs, 200 * kMs);
  EXPECT_EQ(fusion_sync::MatchAction::Drop, decision.action);
}

TEST(FusionTimeMatcher, ReuseDoesNotDeleteAndUniqueDoes) {
  struct Frame {
    bool consumed = false;
  };
  std::vector<Frame> reuse(1), unique(1);
  EXPECT_FALSE(fusion_sync::ConsumeMatchedFrame(reuse, 0, true));
  EXPECT_EQ(1u, reuse.size());
  EXPECT_TRUE(fusion_sync::ConsumeMatchedFrame(reuse, 0, true));
  EXPECT_FALSE(fusion_sync::ConsumeMatchedFrame(unique, 0, false));
  EXPECT_TRUE(unique.empty());
}

TEST(FusionTimeMatcher, InvalidatedBeforeEmptyCannotResurrect) {
  // A fresh normal empty observation is a barrier to an earlier detection.
  // The entry index remains stable while the invalidated stamp is skipped.
  const auto decision = fusion_sync::DecideMatch(105 * kMs, {0, 100 * kMs, 150 * kMs}, 10 * kMs, 50 * kMs, 200 * kMs);
  EXPECT_EQ(fusion_sync::MatchAction::Match, decision.action);
  EXPECT_EQ(1u, decision.yolo_index);
  EXPECT_EQ(5 * kMs, decision.offset_ns);
  const auto invalid_only = fusion_sync::DecideMatch(1, {0}, 200 * kMs, 50 * kMs, 200 * kMs);
  EXPECT_EQ(fusion_sync::MatchAction::Drop, invalid_only.action);
}

namespace {
struct StampFrame {
  std::int64_t stamp_ns;
  std::vector<int> boxes;
};
}  // namespace

TEST(FusionTimeMatcher, LatestEmptyAlsoBlocksReuseForWaitingOlderLidar) {
  const std::vector<StampFrame> frames{{90 * kMs, {1}}, {120 * kMs, {}}, {150 * kMs, {2}}};
  const auto barrier = fusion_sync::LatestEmptyStamp(frames);
  EXPECT_EQ(static_cast<std::uint64_t>(120 * kMs), barrier);
  std::vector<std::int64_t> stamps;
  for (const auto& frame : frames) stamps.push_back(fusion_sync::EligibleObservationStamp(frame, barrier));
  const auto result = fusion_sync::DecideMatch(100 * kMs, stamps, 10 * kMs, 50 * kMs, 200 * kMs);
  EXPECT_EQ(fusion_sync::MatchAction::Match, result.action);
  EXPECT_EQ(1u, result.yolo_index);  // Empty, rather than resurrected 90ms box.
  EXPECT_EQ(0u, fusion_sync::LatestEmptyStamp(std::vector<StampFrame>{{150 * kMs, {1}}}));
}

TEST(FusionTimeMatcher, LatestEmptyRetainsEarlierNormalEmptyObservations) {
  const std::vector<StampFrame> frames{{90 * kMs, {1}}, {120 * kMs, {}}, {180 * kMs, {}}};
  const auto barrier = fusion_sync::LatestEmptyStamp(frames);
  std::vector<std::int64_t> stamps;
  for (const auto& frame : frames) stamps.push_back(fusion_sync::EligibleObservationStamp(frame, barrier));
  EXPECT_EQ(0, stamps[0]);
  EXPECT_EQ(120 * kMs, stamps[1]);
  const auto result = fusion_sync::DecideMatch(130 * kMs, stamps, 10 * kMs, 50 * kMs, 200 * kMs);
  EXPECT_EQ(fusion_sync::MatchAction::Match, result.action);
  EXPECT_EQ(1u, result.yolo_index);
  EXPECT_EQ(10 * kMs, result.offset_ns);
  // Legacy unique matching (zero barrier) retains the earlier detection.
  EXPECT_EQ(90 * kMs, fusion_sync::EligibleObservationStamp(frames[0], 0));
}

// ---------------------------------------------------------------------------
// test_fusion_class_policy.cpp (legacy profile unless stated: Main's static default)

TEST(FusionClassPolicy, NormalizesSevenLabelsWithLegacyVehicleCategory) {
  EXPECT_EQ("person", fusion_class::Normalize("Person", kLegacy));
  EXPECT_EQ("car", fusion_class::Normalize("car", kLegacy));
  EXPECT_EQ("car", fusion_class::Normalize("TRUCK", kLegacy));
  EXPECT_EQ("car", fusion_class::Normalize("bus", kLegacy));
  EXPECT_EQ("bicycle", fusion_class::Normalize("bicycle", kLegacy));
}

TEST(FusionClassPolicy, InitialClassNeedsFiveConsistentObservations) {
  TrackedObject object;
  for (int i = 0; i < 4; ++i) {
    fusion_class::Observe(object, "truck", "truck", 5, 3, kLegacy);
    EXPECT_EQ("unknown", object.camera.str_ObjectClass);
  }
  fusion_class::Observe(object, "truck", "truck", 5, 3, kLegacy);
  EXPECT_EQ("car", object.camera.str_ObjectClass);
  EXPECT_EQ("truck", object.camera.str_YoloClass);
}

TEST(FusionClassPolicy, ConfirmedClassChangesAfterThreeOppositeObservations) {
  TrackedObject object;
  object.camera.str_ObjectClass = "car";
  fusion_class::Observe(object, "person", "person", 5, 3, kLegacy);
  fusion_class::Observe(object, "person", "person", 5, 3, kLegacy);
  EXPECT_EQ("car", object.camera.str_ObjectClass);
  fusion_class::Observe(object, "person", "person", 5, 3, kLegacy);
  EXPECT_EQ("person", object.camera.str_ObjectClass);
}

TEST(FusionClassPolicy, ThreeMissesKeepClassAndFourthResetsIt) {
  TrackedObject object;
  object.camera.str_ObjectClass = "person";
  for (int i = 0; i < 3; ++i) {
    fusion_class::Miss(object, 3);
    EXPECT_EQ("person", object.camera.str_ObjectClass);
  }
  fusion_class::Miss(object, 3);
  EXPECT_EQ("unknown", object.camera.str_ObjectClass);
}

TEST(FusionClassPolicy, UnsupportedObservationDoesNotEraseConfirmedClass) {
  TrackedObject object;
  object.camera.str_ObjectClass = "car";
  fusion_class::Observe(object, "train", "train", 5, 3, kLegacy);
  EXPECT_EQ("car", object.camera.str_ObjectClass);
}

TEST(FusionClassPolicy, SevenLabelsAndRepeatedSourceCannotConfirm) {
  for (const std::string label : {"person", "dog", "bicycle", "motorcycle", "car", "truck", "bus"}) {
    TrackedObject object;
    EXPECT_NE("unknown", fusion_class::Normalize(label, kLegacy));
    ASSERT_TRUE(fusion_class::BeginObservation(object, 100, 1));
    fusion_class::Observe(object, label, label, 2, 3, kLegacy);
    EXPECT_FALSE(fusion_class::BeginObservation(object, 100, 1));
    EXPECT_FALSE(fusion_class::BeginObservation(object, 99, 1));
    EXPECT_EQ(1, object.camera.continuity_counter);
    ASSERT_TRUE(fusion_class::BeginObservation(object, 200, 1));
    fusion_class::Observe(object, label, label, 2, 3, kLegacy);
    EXPECT_EQ(label, object.camera.str_YoloClass);
    EXPECT_TRUE(fusion_class::IsConfirmed(object.camera.str_ObjectClass, kLegacy));
    ASSERT_TRUE(fusion_class::BeginObservation(object, 201, 2));
    EXPECT_EQ(0, object.camera.class_miss_counter);
  }
}

TEST(FusionClassPolicy, EpochInvalidationRemovesOldClassWithoutResettingGeometryOrTrack) {
  TrackedObject object;
  object.tracking.s32_Id = 12;
  object.detection.f32_CoreX = 7.5f;
  object.detection.pcl_ClusterPoints.push_back(pcl::PointXYZI());
  object.camera.str_ObjectClass = "car";
  object.camera.str_YoloClass = "truck";
  object.camera.continuity_counter = 4;
  object.camera.class_miss_counter = 2;
  object.camera.camera_observation_ns = 300;
  fusion_class::InvalidateObservation(object, 2);
  EXPECT_EQ(12, object.tracking.s32_Id);
  EXPECT_EQ(7.5f, object.detection.f32_CoreX);
  EXPECT_EQ(1u, object.detection.pcl_ClusterPoints.size());
  EXPECT_EQ("unknown", object.camera.str_ObjectClass);
  EXPECT_EQ("unknown", object.camera.str_YoloClass);
  EXPECT_EQ(0, object.camera.continuity_counter);
  EXPECT_EQ(0, object.camera.class_miss_counter);
  EXPECT_EQ(0u, object.camera.camera_observation_ns);
  EXPECT_EQ(2u, object.camera.camera_observation_epoch);
  ASSERT_TRUE(fusion_class::BeginObservation(object, 400, 2));
  fusion_class::Observe(object, "truck", "truck", 5, 3, kLegacy);
  EXPECT_EQ("unknown", object.camera.str_ObjectClass);
  object.camera.str_ObjectClass = "noise";
  fusion_class::InvalidateObservation(object, 3);
  EXPECT_EQ("noise", object.camera.str_ObjectClass);
}

TEST(FusionClassPolicy, MissionGroupsPreserveRawAndDoNotSetMotion) {
  EXPECT_EQ("vehicle", fusion_class::Normalize("truck", kMission));
  EXPECT_EQ("static_obstacle", fusion_class::Normalize("motorcycle", kMission));
  EXPECT_EQ("dynamic_obstacle", fusion_class::Normalize("bicycle", kMission));
  EXPECT_EQ("dynamic_obstacle", fusion_class::Normalize("stroller", kMission));
  EXPECT_EQ("unknown", fusion_class::Normalize("building", kMission));
  EXPECT_EQ("vehicle", fusion_class::Normalize("vehicle", kMission));
  TrackedObject object;
  object.tracking.f32_Velocity = 3.f;
  object.tracking.s32_DynamicCount = 2;
  fusion_class::Observe(object, "vehicle", "truck", 1, 1, kMission);
  EXPECT_EQ("vehicle", object.camera.str_ObjectClass);
  EXPECT_EQ("truck", object.camera.str_YoloClass);
  // TrackedObject has no b_IsDynamic; motion bookkeeping is untouched.
  EXPECT_EQ(3.f, object.tracking.f32_Velocity);
  EXPECT_EQ(2, object.tracking.s32_DynamicCount);
  EXPECT_EQ("car", fusion_class::Normalize("truck", kLegacy));
}

TEST(FusionClassPolicy, MissionGroupsMatchAsmcCameraMissionTable) {
  // asmc_camera/detection/mission.py GROUPS.
  const std::map<std::string, std::vector<std::string>> groups{
      {"vehicle", {"car", "bus", "truck"}},
      {"dynamic_obstacle", {"person", "dog", "deer", "bicycle", "stroller"}},
      {"static_obstacle", {"motorcycle", "bollard", "box", "concrete_barrier", "steel_barricade",
                           "traffic_drum", "plastic_barrier", "traffic_cone"}}};
  std::size_t count = 0;
  for (const auto& group : groups)
    for (const auto& name : group.second) {
      EXPECT_EQ(group.first, fusion_class::MissionGroup(name)) << name;
      EXPECT_EQ(group.first, fusion_class::Normalize(name, kMission)) << name;
      EXPECT_TRUE(fusion_class::IsConfirmed(group.first, kMission));
      ++count;
    }
  EXPECT_EQ(16u, count);
  EXPECT_EQ(16u, fusion_adapter::MissionNames().size());
  for (const auto& name : fusion_adapter::MissionNames()) EXPECT_NE("unknown", fusion_class::MissionGroup(name));
  EXPECT_FALSE(fusion_class::IsConfirmed("car", kMission));
  EXPECT_EQ("unknown", fusion_class::RawLabel(""));
  EXPECT_EQ("truck", fusion_class::RawLabel("Truck"));
}

// ---------------------------------------------------------------------------
// test_fusion_replay.cpp

namespace {
using ReplayObservation = std::tuple<int, int, std::string, std::string, std::string, int, int>;

// Same complete observations, different result delivery. Uses the actual
// matcher and class state machine, including unique consumption.
std::vector<ReplayObservation> Replay(int delay_ms) {
  std::deque<int> pending;
  std::vector<std::int64_t> yolo;
  TrackedObject object;
  std::vector<ReplayObservation> output;
  int lidar_index = 0, yolo_index = 0;
  for (int wall = 1000; wall <= 6000; ++wall) {
    if (lidar_index < 50 && wall == 1000 + lidar_index * 80) {
      pending.push_back(wall);
      ++lidar_index;
    }
    if (yolo_index < 82 && wall == 1000 + yolo_index * 50 + delay_ms) {
      yolo.push_back((1000 + yolo_index * 50) * kMs);
      ++yolo_index;
    }
    while (!pending.empty()) {
      const int lidar = pending.front();
      const auto decision = fusion_sync::DecideMatch(lidar * kMs, yolo, (wall - lidar) * kMs, 50 * kMs, 200 * kMs);
      if (decision.action == fusion_sync::MatchAction::Wait) break;
      pending.pop_front();
      if (decision.action != fusion_sync::MatchAction::Match) {
        ADD_FAILURE() << "unexpected unmatched complete fixture at " << lidar;
        continue;
      }
      const int image = static_cast<int>(yolo[decision.yolo_index] / kMs);
      yolo.erase(yolo.begin() + decision.yolo_index);
      if (image >= 3500 && image < 3950) {
        fusion_class::Miss(object, 3);
      } else {
        const std::string label = image < 2500 ? "truck" : "person";
        fusion_class::Observe(object, label, label, 5, 3, kLegacy);
      }
      output.emplace_back(lidar, image, object.camera.str_ObjectClass, object.camera.str_YoloClass, object.camera.last_seen_class,
                          object.camera.continuity_counter, object.camera.class_miss_counter);
    }
  }
  return output;
}
}  // namespace

TEST(FusionReplay, DeliveryLatencyDoesNotChangeCompleteInputClassification) {
  const auto old_path = Replay(55);
  const auto new_path = Replay(12);
  ASSERT_EQ(50u, old_path.size());
  EXPECT_EQ(old_path, new_path);
  EXPECT_EQ("unknown", std::get<2>(old_path[3]));
  EXPECT_EQ("car", std::get<2>(old_path[4]));
  EXPECT_EQ("truck", std::get<3>(old_path[4]));
  EXPECT_EQ("person", std::get<2>(old_path.back()));
}

TEST(FusionReplay, OffsetBoundaryAndTieRemainStrict) {
  auto decision = fusion_sync::DecideMatch(100 * kMs, {50 * kMs, 150 * kMs}, 20 * kMs, 50 * kMs, 200 * kMs);
  EXPECT_EQ(fusion_sync::MatchAction::Match, decision.action);
  EXPECT_EQ(0u, decision.yolo_index);
  decision = fusion_sync::DecideMatch(100 * kMs, {49 * kMs, 151 * kMs}, 200 * kMs, 50 * kMs, 200 * kMs);
  EXPECT_EQ(fusion_sync::MatchAction::Drop, decision.action);
}
// FusionReplay.TraceIsLazyThreadSafeAndFlushesPartialTail: FusionTrace is not ported.

// ---------------------------------------------------------------------------
// test_fusion_state_propagation.cpp

namespace {
using State = std::tuple<std::string, std::string, std::string, int, int, bool>;
State state(const TrackedObject& o) {
  return State{o.camera.str_ObjectClass, o.camera.str_YoloClass, o.camera.last_seen_class, o.camera.continuity_counter, o.camera.class_miss_counter,
               o.camera.is_class_locked};
}
struct PropagationResult {
  std::vector<std::tuple<int, int, State>> fused;
  std::map<int, State> published;
};
// Deterministic fixture for state propagation after recorded association;
// not a replacement for projection/IoU (see test_front_camera_fusion.cpp).
PropagationResult run(int delay, int lidar_stride = 1, bool lose_history = false) {
  std::vector<TrackedObject> history(1);
  history[0].tracking.s32_Id = 7;
  std::deque<std::pair<int, TrackedObject>> pending;
  std::vector<std::int64_t> yolo;
  PropagationResult result;
  for (int wall = 100; wall <= 1200; wall += 100) {
    for (int image = 100; image <= 800; image += 100)
      if (image + delay > wall - 100 && image + delay <= wall) yolo.push_back(image * kMs);
    std::vector<TrackedObject> current(1);
    current[0].tracking.s32_Id = 7;
    if (!lose_history) fusion_state::Copy(history[0], current[0]);
    if (wall <= 800 && (wall / 100 - 1) % lidar_stride == 0) pending.emplace_back(wall, current[0]);
    while (!pending.empty()) {
      const auto decision = fusion_sync::DecideMatch(pending.front().first * kMs, yolo,
                                                     (wall - pending.front().first) * kMs, 50 * kMs, 200 * kMs);
      if (decision.action == fusion_sync::MatchAction::Wait) break;
      if (decision.action != fusion_sync::MatchAction::Match) {
        ADD_FAILURE() << "fixture unexpectedly dropped";
        pending.pop_front();
        continue;
      }
      auto historical = pending.front();
      pending.pop_front();
      const int image = static_cast<int>(yolo[decision.yolo_index] / kMs);
      yolo.erase(yolo.begin() + decision.yolo_index);
      fusion_state::Copy(current[0], historical.second, true);
      fusion_class::Observe(historical.second, "car", "truck", 5, 3, kLegacy);
      fusion_state::Copy(historical.second, current[0], true);
      result.fused.emplace_back(historical.first, image, state(historical.second));
    }
    fusion_state::Persist(current, history);
    if (wall <= 800) result.published.emplace(wall, state(current[0]));
  }
  return result;
}
}  // namespace

TEST(FusionStatePropagation, CopiesOnlyClassificationAndPreservesLockSemantics) {
  TrackedObject source, target;
  source.camera.str_ObjectClass = "car";
  source.camera.str_YoloClass = "truck";
  source.camera.last_seen_class = "person";
  source.camera.continuity_counter = 2;
  source.camera.class_miss_counter = 1;
  source.camera.is_class_locked = true;
  source.camera.camera_observation_ns = 123;
  source.camera.camera_observation_epoch = 9;
  target.tracking.s32_Id = 9;
  target.detection.f32_CoreX = 100;
  target.tracking.s32_MissCount = 4;
  fusion_state::Copy(source, target);
  EXPECT_EQ(state(source), state(target));
  EXPECT_EQ(123u, target.camera.camera_observation_ns);
  EXPECT_EQ(9u, target.camera.camera_observation_epoch);
  EXPECT_EQ(9, target.tracking.s32_Id);
  EXPECT_EQ(100, target.detection.f32_CoreX);
  EXPECT_EQ(4, target.tracking.s32_MissCount);
  fusion_state::Copy(source, target, true);
  EXPECT_FALSE(target.camera.is_class_locked);
}

TEST(FusionStatePropagation, PersistsByIdWithoutCopyingHistoricalGeometry) {
  std::vector<TrackedObject> current(2), history(1);
  current[0].tracking.s32_Id = history[0].tracking.s32_Id = 7;
  current[1].tracking.s32_Id = 8;
  current[0].camera.str_ObjectClass = "person";
  current[0].detection.f32_CoreX = 99;
  history[0].detection.f32_CoreX = 1;
  fusion_state::Persist(current, history);
  EXPECT_EQ("person", history[0].camera.str_ObjectClass);
  EXPECT_EQ(1, history[0].detection.f32_CoreX);
  EXPECT_EQ(1u, history.size());
}

TEST(FusionStatePropagation, SameHistoricalUpdatesCanHaveDifferentPublicationPhase) {
  const auto early = run(12), late = run(120);
  ASSERT_EQ(8u, early.fused.size());
  EXPECT_EQ(early.fused, late.fused);
  EXPECT_NE(early.published, late.published);
  EXPECT_EQ("car", std::get<0>(early.published.at(600)));
  EXPECT_EQ("unknown", std::get<0>(late.published.at(600)));
}

TEST(FusionStatePropagation, SelectionChangesCounterHistory) {
  EXPECT_NE(run(12).fused, run(12, 2).fused);
  EXPECT_EQ("unknown", std::get<0>(run(12, 2).published.at(800)));
}

TEST(FusionStatePropagation, NegativeControlLosingCarryChangesContinuousResult) {
  EXPECT_NE(run(12).fused, run(12, 1, true).fused);
}

TEST(FusionStatePropagation, ReuseAfterHistoryCopyCannotIncrementConfirmationOrMiss) {
  TrackedObject previous, live, historical;
  previous.tracking.s32_Id = live.tracking.s32_Id = historical.tracking.s32_Id = 7;
  ASSERT_TRUE(fusion_class::BeginObservation(previous, 100, 1));
  fusion_class::Observe(previous, "dog", "dog", 5, 3, kLegacy);
  fusion_state::Copy(previous, live);
  fusion_state::Copy(live, historical, true);
  EXPECT_FALSE(fusion_class::BeginObservation(historical, 100, 1));
  EXPECT_EQ(1, historical.camera.continuity_counter);
  fusion_class::Miss(historical, 3);
  fusion_state::Copy(historical, previous);
  EXPECT_FALSE(fusion_class::BeginObservation(previous, 100, 1));
  EXPECT_EQ(1, previous.camera.class_miss_counter);
  ASSERT_TRUE(fusion_class::BeginObservation(previous, 200, 1));
  fusion_class::Miss(previous, 3);
  EXPECT_EQ(2, previous.camera.class_miss_counter);
}

// ---------------------------------------------------------------------------
// test_camera_objects_adapter.py (box filter only; whole-frame drop is the receiver's)

namespace {
asmc::lidar::camera::Box MakeBox(float x1, float y1, float x2, float y2, const std::string& source,
                                 float confidence, bool confidence_valid = true) {
  asmc::lidar::camera::Box box;
  box.xmin = x1;
  box.ymin = y1;
  box.xmax = x2;
  box.ymax = y2;
  box.class_name = "vehicle";
  box.source_class_name = source;
  box.confidence = confidence;
  box.confidence_valid = confidence_valid;
  return box;
}
}  // namespace

TEST(FusionAdapter, PreservesRawLabelOrderAndTruncatesCoordinates) {
  std::vector<asmc::lidar::camera::Box> boxes;
  for (const auto& entry : std::vector<std::pair<std::string, float>>{
           {"truck", 0.9f}, {"person", 0.6f}, {"person", 0.71f}, {"car", 0.8f}})
    boxes.push_back(MakeBox(1.9f, 2.1f, 30.9f, 40.2f, entry.first, entry.second));
  const auto out = fusion_adapter::ConvertBoxes(boxes, fusion_adapter::LegacyNames(), 0.7);
  ASSERT_EQ(3u, out.size());
  EXPECT_EQ("truck", out[0].str_raw_class);
  EXPECT_EQ("person", out[1].str_raw_class);
  EXPECT_EQ("car", out[2].str_raw_class);
  EXPECT_EQ("truck", out[0].str_class);  // not grouped by the adapter
  EXPECT_EQ(1.f, out[0].min_x);
  EXPECT_EQ(2.f, out[0].min_y);
  EXPECT_EQ(30.f, out[0].max_x);
  EXPECT_EQ(40.f, out[0].max_y);
  // Serialized float32 confidence is compared in double: 0.7f < 0.7 is filtered.
  EXPECT_TRUE(fusion_adapter::ConvertBoxes({MakeBox(1, 2, 3, 4, "car", 0.7f)}, fusion_adapter::LegacyNames(), 0.7).empty());
  // -0.0 becomes +0.0 like Python int().
  const auto zero = fusion_adapter::ConvertBoxes({MakeBox(-0.f, 0.5f, 3, 4, "car", 0.9f)}, fusion_adapter::LegacyNames(), 0.);
  ASSERT_EQ(1u, zero.size());
  EXPECT_FALSE(std::signbit(zero[0].min_x));
  EXPECT_EQ(0.f, zero[0].min_y);
}

TEST(FusionAdapter, InvalidConfidenceAndNonfiniteBoxAreFiltered) {
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  const std::vector<asmc::lidar::camera::Box> boxes{MakeBox(0, 0, 1, 1, "car", 0.9f, false),
                                                    MakeBox(0, 0, 1, 1, "car", nan),
                                                    MakeBox(inf, 0, 1, 1, "car", 0.9f)};
  EXPECT_TRUE(fusion_adapter::ConvertBoxes(boxes, fusion_adapter::LegacyNames(), 0.).empty());
}

TEST(FusionAdapter, ConsumerCutoffCanPassProducerResultsOrFilterMore) {
  const std::vector<asmc::lidar::camera::Box> boxes{MakeBox(1, 2, 20, 30, "car", 0.60f)};
  EXPECT_EQ(1u, fusion_adapter::ConvertBoxes(boxes, fusion_adapter::LegacyNames(), 0.).size());
  EXPECT_EQ(0u, fusion_adapter::ConvertBoxes(boxes, fusion_adapter::LegacyNames(), .65).size());
  for (double bad : {-1., 1.01, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()})
    EXPECT_FALSE(fusion_adapter::ValidThreshold(bad));
  EXPECT_TRUE(fusion_adapter::ValidThreshold(0.) && fusion_adapter::ValidThreshold(1.));
}

TEST(FusionAdapter, SevenRawLabelsExplicitListAndMission16) {
  std::vector<asmc::lidar::camera::Box> boxes;
  for (const auto& label : fusion_adapter::LegacyNames()) boxes.push_back(MakeBox(1, 2, 20, 30, label, .9f));
  auto out = fusion_adapter::ConvertBoxes(boxes, fusion_adapter::LegacyNames(), 0.);
  ASSERT_EQ(7u, out.size());
  for (std::size_t i = 0; i < out.size(); ++i) EXPECT_EQ(boxes[i].source_class_name, out[i].str_raw_class);
  out = fusion_adapter::ConvertBoxes(boxes, {"bus", "truck", "person", " Car "}, 0.);
  ASSERT_EQ(4u, out.size());
  EXPECT_EQ("person", out[0].str_raw_class);
  EXPECT_EQ("car", out[1].str_raw_class);
  EXPECT_EQ("bus", out[2].str_raw_class);
  EXPECT_EQ("truck", out[3].str_raw_class);
  // mission16_v1: asmc_camera mission CLASSES, case-insensitive source names.
  const std::vector<asmc::lidar::camera::Box> mission{MakeBox(1, 2, 20, 30, "Traffic_Cone", .9f),
                                                      MakeBox(1, 2, 20, 30, "stroller", .9f),
                                                      MakeBox(1, 2, 20, 30, "train", .9f)};
  out = fusion_adapter::ConvertBoxes(mission, fusion_adapter::MissionNames(), 0.);
  ASSERT_EQ(2u, out.size());
  EXPECT_EQ("Traffic_Cone", out[0].str_raw_class);
  EXPECT_EQ("static_obstacle", fusion_class::Normalize(out[0].str_class, kMission));
  EXPECT_EQ("traffic_cone", fusion_class::RawLabel(out[0].str_raw_class));
  EXPECT_TRUE(fusion_adapter::ConvertBoxes(mission, fusion_adapter::LegacyNames(), 0.).empty());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
