#include <SensorFusion/FusionClassState.hpp>
#include <SensorFusion/FusionClassPolicy.hpp>
#include <SensorFusion/FusionTimeMatcher.hpp>
#include <gtest/gtest.h>
#include <deque>
#include <map>
#include <tuple>

namespace {
constexpr std::int64_t kMs = 1000000;
using State = std::tuple<std::string, std::string, std::string, int, int, bool>;
State state(const LiDARObjectInfo& o) {
    return {o.str_ObjectClass, o.str_YoloClass, o.last_seen_class,
            o.continuity_counter, o.class_miss_counter, o.is_class_locked};
}
struct Result {
    std::vector<std::tuple<int, int, State>> fused;
    std::map<int, State> published;
};
// Deterministic fixture for state propagation after recorded association;
// not a replacement for 3D projection/IoU or the end-to-end runtime gate.
Result run(int delay, int lidar_stride = 1, bool lose_history = false) {
    std::vector<LiDARObjectInfo> history(1);
    history[0].s32_Id = 7;
    std::deque<std::pair<int, LiDARObjectInfo>> pending;
    std::vector<std::int64_t> yolo;
    Result result;
    for (int wall = 100; wall <= 1200; wall += 100) {
        for (int image = 100; image <= 800; image += 100)
            if (image + delay > wall - 100 && image + delay <= wall)
                yolo.push_back(image * kMs);
        std::vector<LiDARObjectInfo> current(1);
        current[0].s32_Id = 7;
        if (!lose_history) fusion_state::Copy(history[0], current[0]);
        if (wall <= 800 && (wall / 100 - 1) % lidar_stride == 0)
            pending.emplace_back(wall, current[0]);
        while (!pending.empty()) {
            const auto decision = fusion_sync::DecideMatch(pending.front().first * kMs,
                yolo, (wall - pending.front().first) * kMs, 50 * kMs, 200 * kMs);
            if (decision.action == fusion_sync::MatchAction::Wait) break;
            if (decision.action != fusion_sync::MatchAction::Match) {
                ADD_FAILURE() << "fixture unexpectedly dropped"; pending.pop_front(); continue;
            }
            auto historical = pending.front(); pending.pop_front();
            const int image = yolo[decision.yolo_index] / kMs;
            yolo.erase(yolo.begin() + decision.yolo_index);
            fusion_state::Copy(current[0], historical.second, true);
            fusion_class::Observe(historical.second, "car", "truck", 5, 3);
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
    LiDARObjectInfo source, target;
    source.str_ObjectClass = "car"; source.str_YoloClass = "truck";
    source.last_seen_class = "person"; source.continuity_counter = 2;
    source.class_miss_counter = 1; source.is_class_locked = true;
    source.camera_observation_ns=123; source.camera_observation_epoch=9;
    target.s32_Id = 9; target.f32_CoreX = 100; target.s32_MissCount = 4;
    fusion_state::Copy(source, target);
    EXPECT_EQ(state(source), state(target));
    EXPECT_EQ(123u,target.camera_observation_ns); EXPECT_EQ(9u,target.camera_observation_epoch);
    EXPECT_EQ(9, target.s32_Id); EXPECT_EQ(100, target.f32_CoreX);
    EXPECT_EQ(4, target.s32_MissCount);
    fusion_state::Copy(source, target, true);
    EXPECT_FALSE(target.is_class_locked);
}

TEST(FusionStatePropagation, PersistsByIdWithoutCopyingHistoricalGeometry) {
    std::vector<LiDARObjectInfo> current(2), history(1);
    current[0].s32_Id = history[0].s32_Id = 7;
    current[1].s32_Id = 8;
    current[0].str_ObjectClass = "person"; current[0].f32_CoreX = 99;
    history[0].f32_CoreX = 1;
    fusion_state::Persist(current, history);
    EXPECT_EQ("person", history[0].str_ObjectClass);
    EXPECT_EQ(1, history[0].f32_CoreX); EXPECT_EQ(1u, history.size());
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

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

TEST(FusionStatePropagation, ReuseAfterHistoryCopyCannotIncrementConfirmationOrMiss) {
    LiDARObjectInfo previous, live, historical;
    previous.s32_Id=live.s32_Id=historical.s32_Id=7;
    ASSERT_TRUE(fusion_class::BeginObservation(previous,100,1));
    fusion_class::Observe(previous,"dog","dog",5,3);
    fusion_state::Copy(previous,live);
    fusion_state::Copy(live,historical,true);
    EXPECT_FALSE(fusion_class::BeginObservation(historical,100,1));
    EXPECT_EQ(1,historical.continuity_counter);
    fusion_class::Miss(historical,3);
    fusion_state::Copy(historical,previous);
    EXPECT_FALSE(fusion_class::BeginObservation(previous,100,1));
    EXPECT_EQ(1,previous.class_miss_counter);
    ASSERT_TRUE(fusion_class::BeginObservation(previous,200,1));
    fusion_class::Miss(previous,3);
    EXPECT_EQ(2,previous.class_miss_counter);
}
