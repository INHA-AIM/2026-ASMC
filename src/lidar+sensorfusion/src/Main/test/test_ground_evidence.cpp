#include <Lidar/ImuRansac/GroundEvidence.hpp>
#include <gtest/gtest.h>
#include <limits>

namespace {
struct Point { double x, y, z; };
GroundPlaneSnapshot FlatPlane() {
    GroundPlaneSnapshot plane;
    plane.valid = true;
    plane.coefficients = {{0.0, 0.0, 1.0, 1.5}};
    return plane;
}
}

TEST(GroundEvidence, OneScanRingHasClearanceDespiteZeroThickness) {
    const std::vector<Point> points{{50, 0, -.8}, {50, .2, -.8}, {50, .4, -.8}};
    const auto result = MeasureObjectGroundEvidence(points, FlatPlane());
    ASSERT_TRUE(result.valid);
    EXPECT_NEAR(.7, result.median_clearance_m, 1e-9);
    EXPECT_NEAR(.4, result.rejection_band_m, 1e-9);
}

TEST(GroundEvidence, MedianRejectsSingleElevatedOutlier) {
    const std::vector<Point> points{{30, 0, -1.48}, {30, .2, -1.47}, {30, .4, 0}};
    const auto result = MeasureObjectGroundEvidence(points, FlatPlane());
    ASSERT_TRUE(result.valid);
    EXPECT_NEAR(.03, result.median_clearance_m, 1e-9);
}

TEST(GroundEvidence, SignedClearanceRejectsBelowPlaneReturns) {
    const std::vector<Point> points{{50, 0, -2.1}, {50, .2, -2.1}, {50, .4, -2.1}};
    EXPECT_NEAR(-.6, MeasureObjectGroundEvidence(points, FlatPlane()).median_clearance_m, 1e-9);
}

TEST(GroundEvidence, SlopedAndTranslatedPlaneUsesLocalRoad) {
    auto plane = FlatPlane();
    plane.coefficients = {{-.1, 0, 1, 1.5}};
    const std::vector<Point> points{{30, 0, 2.2}, {31, 0, 2.3}, {32, 0, 2.4}};
    const auto result = MeasureObjectGroundEvidence(points, plane);
    ASSERT_TRUE(result.valid);
    EXPECT_NEAR(.7 / std::sqrt(1.01), result.median_clearance_m, 1e-9);
}

TEST(GroundEvidence, InvalidPlaneAndTooFewPointsProduceNoEvidence) {
    auto plane = FlatPlane();
    const std::vector<Point> points{{50, 0, -.8}, {50, .2, -.8}};
    EXPECT_FALSE(MeasureObjectGroundEvidence(points, plane).valid);
    plane.valid = false;
    EXPECT_FALSE(MeasureObjectGroundEvidence(points, plane).valid);
    plane = FlatPlane();
    plane.coefficients[3] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(MeasureObjectGroundEvidence(points, plane).valid);
}

TEST(GroundEvidence, NonFiniteReturnsAreNotCounted) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const std::vector<Point> points{{50, 0, -.8}, {50, .2, -.8}, {50, .4, nan}};
    const auto result = MeasureObjectGroundEvidence(points, FlatPlane());
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(2, result.sample_count);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
