#include <gtest/gtest.h>

#include <initializer_list>
#include <vector>

#include "MPC/Reference/path_resampler.hpp"

namespace {

PlannedPath MakePath(std::initializer_list<double> speeds,
                     std::initializer_list<double> diagnostic_accelerations = {}) {
    PlannedPath path;
    path.dt = 0.1;
    const std::vector<double> acceleration_values(diagnostic_accelerations);
    int i = 0;
    for (double speed : speeds) {
        path.x.push_back(static_cast<double>(i));
        path.y.push_back(2.0 * i);
        path.yaw.push_back(0.1 * i);
        path.v.push_back(speed);
        path.a.push_back(acceleration_values.empty()
                             ? 0.0
                             : acceleration_values.at(i));
        path.kappa.push_back(0.01 * i);
        ++i;
    }
    return path;
}

TEST(ReferencePathStorageTest, AccelerationParticipatesInPushSizeAndClear) {
    ReferencePath reference;
    reference.x_ref.push_back(1.0);
    reference.y_ref.push_back(2.0);
    reference.yaw_ref.push_back(0.1);
    reference.v_ref.push_back(3.0);
    reference.a_ref.push_back(-2.0);
    reference.k_ref.push_back(0.01);

    EXPECT_EQ(1u, reference.size());
    ASSERT_EQ(1u, reference.a_ref.size());
    EXPECT_DOUBLE_EQ(-2.0, reference.a_ref.front());

    reference.clear();
    EXPECT_TRUE(reference.empty());
    EXPECT_TRUE(reference.a_ref.empty());
}

TEST(PathResamplerTest, DerivesExactAccelerationOnFinalMpcGrid) {
    const ReferencePath reference = ToReferencePath(
        MakePath({12.0, 11.8, 11.6}), 0.05, 5);

    ASSERT_EQ(5u, reference.size());
    ASSERT_EQ(reference.size(), reference.a_ref.size());
    const std::vector<double> expected_speed =
        {12.0, 11.9, 11.8, 11.7, 11.6};
    for (std::size_t i = 0; i < reference.size(); ++i) {
        EXPECT_NEAR(expected_speed[i], reference.v_ref[i], 1e-12);
        EXPECT_NEAR(-2.0, reference.a_ref[i], 1e-12);
    }
}

TEST(PathResamplerTest, CartesianAccelerationDoesNotAffectMpcReference) {
    const ReferencePath reference = ToReferencePath(
        MakePath({12.0, 11.8, 11.6}, {15.0, -12.0, 8.0}),
        0.05, 5);

    ASSERT_EQ(5u, reference.size());
    for (double acceleration : reference.a_ref) {
        EXPECT_NEAR(-2.0, acceleration, 1e-12);
    }
}

TEST(PathResamplerTest, ConstantSpeedProducesZeroAcceleration) {
    const ReferencePath reference = ToReferencePath(
        MakePath({8.0, 8.0, 8.0, 8.0}), 0.05, 7);

    ASSERT_EQ(7u, reference.size());
    for (double acceleration : reference.a_ref) {
        EXPECT_NEAR(0.0, acceleration, 1e-12);
    }
}

TEST(PathResamplerTest, IncreasingSpeedProducesPositiveAcceleration) {
    const ReferencePath reference = ToReferencePath(
        MakePath({5.0, 5.1, 5.2}), 0.05, 5);

    ASSERT_EQ(5u, reference.size());
    for (double acceleration : reference.a_ref) {
        EXPECT_NEAR(1.0, acceleration, 1e-12);
    }
}

TEST(PathResamplerTest, DerivativeUsesOffsetAppliedFinalVelocityArray) {
    const ReferencePath reference = ToReferencePath(
        MakePath({10.0, 9.8, 9.4, 8.8}), 0.05, 5, 0.05);

    const std::vector<double> expected_speed =
        {9.9, 9.8, 9.6, 9.4, 9.1};
    const std::vector<double> expected_acceleration =
        {-2.0, -3.0, -4.0, -5.0, -6.0};
    ASSERT_EQ(expected_speed.size(), reference.size());
    for (std::size_t i = 0; i < reference.size(); ++i) {
        EXPECT_NEAR(expected_speed[i], reference.v_ref[i], 1e-12);
        EXPECT_NEAR(expected_acceleration[i], reference.a_ref[i], 1e-11);
    }
}

TEST(PathResamplerTest, NonuniformProfileUsesCentralInteriorDifferences) {
    PlannedPath path = MakePath({10.0, 9.9, 9.7, 9.4, 9.2});
    path.dt = 0.05;
    const ReferencePath reference = ToReferencePath(path, 0.05, 5);

    const std::vector<double> expected_acceleration =
        {-2.0, -3.0, -5.0, -5.0, -4.0};
    ASSERT_EQ(expected_acceleration.size(), reference.size());
    for (std::size_t i = 0; i < reference.size(); ++i) {
        EXPECT_NEAR(expected_acceleration[i], reference.a_ref[i], 1e-11);
    }
}

TEST(PathResamplerTest, SinglePointReferenceHasZeroAcceleration) {
    const ReferencePath reference = ToReferencePath(
        MakePath({4.0}), 0.05, 1);

    ASSERT_EQ(1u, reference.size());
    ASSERT_EQ(1u, reference.a_ref.size());
    EXPECT_DOUBLE_EQ(0.0, reference.a_ref.front());
}

TEST(PathResamplerTest, AllSpatialFieldsStillShareInterpolationTimestamp) {
    const ReferencePath reference = ToReferencePath(
        MakePath({5.0, 4.9, 4.8}), 0.05, 4);

    // All fields use the same source indices and interpolation fraction.
    ASSERT_EQ(4u, reference.size());
    EXPECT_DOUBLE_EQ(0.5, reference.x_ref[1]);
    EXPECT_DOUBLE_EQ(1.0, reference.y_ref[1]);
    EXPECT_DOUBLE_EQ(4.95, reference.v_ref[1]);
    EXPECT_DOUBLE_EQ(0.005, reference.k_ref[1]);
}

TEST(PathResamplerTest, MissingDiagnosticAccelerationDoesNotAffectReference) {
    PlannedPath path = MakePath({5.0, 4.9, 4.8});
    path.a.clear();
    const ReferencePath reference = ToReferencePath(path, 0.1, 3);
    ASSERT_EQ(3u, reference.size());
    EXPECT_NEAR(-1.0, reference.a_ref[0], 1e-12);
    EXPECT_NEAR(-1.0, reference.a_ref[1], 1e-12);
    EXPECT_NEAR(-1.0, reference.a_ref[2], 1e-12);
}

}  // namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
