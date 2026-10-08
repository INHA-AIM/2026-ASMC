#include <SensorFusion/FusionTimeMatcher.hpp>

#include <gtest/gtest.h>

namespace {
constexpr std::int64_t kMs = 1000000;
}

TEST(FusionTimeMatcher, WaitsForFutureFrameBeforeChoosingNearest) {
    const auto decision = fusion_sync::DecideMatch(
        100 * kMs, {72 * kMs, 91 * kMs}, 80 * kMs, 50 * kMs, 200 * kMs);
    EXPECT_EQ(fusion_sync::MatchAction::Wait, decision.action);
}

TEST(FusionTimeMatcher, ChoosesCloserPastAfterFutureArrives) {
    const auto decision = fusion_sync::DecideMatch(
        100 * kMs, {72 * kMs, 91 * kMs, 132 * kMs},
        90 * kMs, 50 * kMs, 200 * kMs);
    EXPECT_EQ(fusion_sync::MatchAction::Match, decision.action);
    EXPECT_EQ(1u, decision.yolo_index);
    EXPECT_EQ(9 * kMs, decision.offset_ns);
}

TEST(FusionTimeMatcher, ChoosesCloserFutureFrame) {
    const auto decision = fusion_sync::DecideMatch(
        100 * kMs, {61 * kMs, 118 * kMs},
        90 * kMs, 50 * kMs, 200 * kMs);
    EXPECT_EQ(fusion_sync::MatchAction::Match, decision.action);
    EXPECT_EQ(1u, decision.yolo_index);
    EXPECT_EQ(18 * kMs, decision.offset_ns);
}

TEST(FusionTimeMatcher, DropsOnlyAfterDecisionIsSettled) {
    auto decision = fusion_sync::DecideMatch(
        100 * kMs, {30 * kMs}, 199 * kMs, 50 * kMs, 200 * kMs);
    EXPECT_EQ(fusion_sync::MatchAction::Wait, decision.action);

    decision = fusion_sync::DecideMatch(
        100 * kMs, {30 * kMs}, 200 * kMs, 50 * kMs, 200 * kMs);
    EXPECT_EQ(fusion_sync::MatchAction::Drop, decision.action);
}

TEST(FusionTimeMatcher, NeverRelaxesMaximumOffsetAtTimeout) {
    const auto decision = fusion_sync::DecideMatch(
        100 * kMs, {40 * kMs, 160 * kMs},
        250 * kMs, 50 * kMs, 200 * kMs);
    EXPECT_EQ(fusion_sync::MatchAction::Drop, decision.action);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

TEST(FusionTimeMatcher, ReuseDoesNotDeleteAndUniqueDoes) {
    struct Frame { bool consumed = false; };
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
    const auto decision = fusion_sync::DecideMatch(
        105 * kMs, {0, 100 * kMs, 150 * kMs}, 10 * kMs, 50 * kMs, 200 * kMs);
    EXPECT_EQ(fusion_sync::MatchAction::Match, decision.action);
    EXPECT_EQ(1u, decision.yolo_index);
    EXPECT_EQ(5 * kMs, decision.offset_ns);
    const auto invalid_only = fusion_sync::DecideMatch(1, {0}, 200*kMs, 50*kMs, 200*kMs);
    EXPECT_EQ(fusion_sync::MatchAction::Drop, invalid_only.action);
}

TEST(FusionTimeMatcher, LatestEmptyAlsoBlocksReuseForWaitingOlderLidar) {
    struct Stamp { std::uint64_t ns; std::uint64_t toNSec() const { return ns; } };
    struct Frame { Stamp stamp; std::vector<int> boxes; };
    const std::vector<Frame> frames{{{90*kMs},{1}},{{120*kMs},{}},{{150*kMs},{2}}};
    const auto barrier = fusion_sync::LatestEmptyStamp(frames);
    EXPECT_EQ(120*kMs, barrier);
    std::vector<std::int64_t> stamps;
    for (const auto& frame : frames)
        stamps.push_back(frame.stamp.ns < barrier ? 0 : frame.stamp.ns);
    const auto result = fusion_sync::DecideMatch(100*kMs, stamps, 10*kMs, 50*kMs, 200*kMs);
    EXPECT_EQ(fusion_sync::MatchAction::Match, result.action);
    EXPECT_EQ(1u, result.yolo_index); // Empty, rather than resurrected 90ms box.
    EXPECT_EQ(0u, fusion_sync::LatestEmptyStamp(std::vector<Frame>{{{150*kMs},{1}}}));
}
