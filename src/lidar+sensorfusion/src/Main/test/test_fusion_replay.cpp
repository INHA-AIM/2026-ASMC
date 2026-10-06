#include <SensorFusion/FusionClassPolicy.hpp>
#include <SensorFusion/FusionTimeMatcher.hpp>
#include <SensorFusion/FusionTrace.hpp>
#include <gtest/gtest.h>

#include <cstdio>
#include <deque>
#include <fstream>
#include <iterator>
#include <thread>
#include <tuple>
#include <unistd.h>
#include <vector>

namespace {
constexpr std::int64_t kMs = 1000000;
using Observation = std::tuple<int, int, std::string, std::string, std::string, int, int>;

// Same complete observations, different result delivery. Use the actual
// matcher and class state machine, including unique YOLO consumption.
std::vector<Observation> Replay(int delay_ms) {
    std::deque<int> pending;
    std::vector<std::int64_t> yolo;
    LiDARObjectInfo object;
    std::vector<Observation> output;
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
            const auto decision = fusion_sync::DecideMatch(
                lidar * kMs, yolo, (wall - lidar) * kMs, 50 * kMs, 200 * kMs);
            if (decision.action == fusion_sync::MatchAction::Wait) break;
            pending.pop_front();
            if (decision.action != fusion_sync::MatchAction::Match) {
                ADD_FAILURE() << "unexpected unmatched complete fixture at " << lidar;
                continue;
            }
            const int image = static_cast<int>(yolo[decision.yolo_index] / kMs);
            yolo.erase(yolo.begin() + decision.yolo_index);
            if (image >= 3500 && image < 3950) fusion_class::Miss(object, 3);
            else {
                const std::string label = image < 2500 ? "truck" : "person";
                fusion_class::Observe(object, label, label, 5, 3);
            }
            output.emplace_back(lidar, image, object.str_ObjectClass,
                object.str_YoloClass, object.last_seen_class,
                object.continuity_counter, object.class_miss_counter);
        }
    }
    return output;
}
}

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
    auto decision = fusion_sync::DecideMatch(100 * kMs,
        {50 * kMs, 150 * kMs}, 20 * kMs, 50 * kMs, 200 * kMs);
    EXPECT_EQ(fusion_sync::MatchAction::Match, decision.action);
    EXPECT_EQ(0u, decision.yolo_index);
    decision = fusion_sync::DecideMatch(100 * kMs,
        {49 * kMs, 151 * kMs}, 200 * kMs, 50 * kMs, 200 * kMs);
    EXPECT_EQ(fusion_sync::MatchAction::Drop, decision.action);
}

TEST(FusionReplay, TraceIsLazyThreadSafeAndFlushesPartialTail) {
    fusion_trace::Recorder disabled;
    EXPECT_FALSE(disabled.Enabled());
    const std::string path = "/tmp/fusion-trace-test-" + std::to_string(getpid()) + ".jsonl";
    fusion_trace::Recorder recorder;
    recorder.Configure(path);
    auto write = [&]() {
        for (int index = 0; index < 50; ++index) recorder.Append("{\"event\":\"test\"}");
    };
    std::thread a(write), b(write);
    a.join(); b.join();
    recorder.Flush();
    std::ifstream input(path);
    std::string line;
    int count = 0;
    while (std::getline(input, line)) {
        EXPECT_EQ("{\"event\":\"test\"}", line);
        ++count;
    }
    EXPECT_EQ(100, count);
    EXPECT_EQ("\"a\\\"\\\\\\u000a\"", fusion_trace::Quote("a\"\\\n"));
    std::remove(path.c_str());
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
