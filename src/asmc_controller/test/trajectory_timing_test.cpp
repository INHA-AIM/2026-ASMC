#include <gtest/gtest.h>

#include <limits>

#include "MPC/Reference/trajectory_timing.hpp"

namespace {

// Commit E: ComputeReferenceStartOffset은 ros::Time 없이 순수 double(초)만
// 다룬다 - trajectory/vehicle_state 모두 source timestamp(header.stamp)
// 기준이며, SteadyTime 기반 freshness watchdog과는 완전히 분리된 계산이다.

TEST(ComputeReferenceStartOffsetTest, EqualTimestampsProduceZeroOffset) {
    EXPECT_DOUBLE_EQ(0.0, ComputeReferenceStartOffset(10.0, 10.0));
}

TEST(ComputeReferenceStartOffsetTest, VehicleAfterTrajectoryProducesPositiveOffset) {
    EXPECT_NEAR(0.1, ComputeReferenceStartOffset(10.0, 10.1), 1e-9);
}

TEST(ComputeReferenceStartOffsetTest, VehicleBeforeTrajectoryClampsToZero) {
    // vehicle state가 trajectory보다 과거 timestamp를 갖는 경우는 정상
    // 흐름에서 없어야 하지만, 있더라도 음수 offset으로 reference 시작을
    // 과거로 되돌리지 않는다.
    EXPECT_DOUBLE_EQ(0.0, ComputeReferenceStartOffset(10.0, 9.9));
}

TEST(ComputeReferenceStartOffsetTest, FractionalOffsetIsExact) {
    EXPECT_NEAR(0.05, ComputeReferenceStartOffset(100.0, 100.05), 1e-9);
}

TEST(ComputeReferenceStartOffsetTest, NonFiniteInputsProduceSafeZero) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    EXPECT_DOUBLE_EQ(0.0, ComputeReferenceStartOffset(nan, 10.0));
    EXPECT_DOUBLE_EQ(0.0, ComputeReferenceStartOffset(10.0, nan));
    EXPECT_DOUBLE_EQ(0.0, ComputeReferenceStartOffset(inf, 10.0));
    EXPECT_DOUBLE_EQ(0.0, ComputeReferenceStartOffset(10.0, inf));
}

}  // namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
