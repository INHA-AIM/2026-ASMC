#include <PlannerInterface/DrivableFootprint.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>

namespace {

std::array<cv::Point2d, 4> Box(
    double x0, double y0, double x1, double y1) {
    return {{{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}}};
}

}  // namespace

TEST(DrivableFootprint, FullyInsideAndOutside) {
    cv::Mat road = cv::Mat::zeros(100, 100, CV_8UC1);
    road(cv::Rect(20, 20, 60, 60)).setTo(255);

    EXPECT_NEAR(
        DrivableFootprintRatio(road, 1.0, 0.0, 0.0,
                               Box(30.0, 30.0, 40.0, 40.0), 8),
        1.0, 1e-9);
    EXPECT_NEAR(
        DrivableFootprintRatio(road, 1.0, 0.0, 0.0,
                               Box(0.0, 0.0, 10.0, 10.0), 8),
        0.0, 1e-9);
}

TEST(DrivableFootprint, MeasuresAreaInsteadOfSparseSamples) {
    cv::Mat road = cv::Mat::zeros(100, 100, CV_8UC1);
    road(cv::Rect(0, 0, 50, 100)).setTo(255);

    const double half = DrivableFootprintRatio(
        road, 1.0, 0.0, 0.0, Box(39.5, 40.0, 59.5, 60.0), 8);
    EXPECT_NEAR(half, 0.5, 0.02);
}

TEST(DrivableFootprint, ResolvesFifteenPercentBoundary) {
    cv::Mat road = cv::Mat::zeros(100, 100, CV_8UC1);
    road(cv::Rect(0, 0, 15, 100)).setTo(255);

    const double ratio = DrivableFootprintRatio(
        road, 1.0, 0.0, 0.0, Box(-0.5, 20.0, 99.5, 40.0));
    EXPECT_NEAR(ratio, 0.15, 0.01);
}

TEST(DrivableFootprint, IntersectionRectangleContainsOnlyRoadPortion) {
    cv::Mat road = cv::Mat::zeros(100, 100, CV_8UC1);
    road(cv::Rect(20, 0, 80, 100)).setTo(255);
    std::array<cv::Point2d, 4> clipped;
    ASSERT_TRUE(DrivableFootprintIntersectionRect(
        road, 1.0, 0.0, 0.0, Box(18.0, 40.0, 22.0, 42.0),
        &clipped, 8));
    double min_x = clipped[0].x;
    double max_x = clipped[0].x;
    double min_y = clipped[0].y;
    double max_y = clipped[0].y;
    for (const auto& point : clipped) {
        min_x = std::min(min_x, point.x);
        max_x = std::max(max_x, point.x);
        min_y = std::min(min_y, point.y);
        max_y = std::max(max_y, point.y);
    }
    EXPECT_GE(min_x, 19.4);
    EXPECT_LE(max_x, 22.2);
    EXPECT_LT(max_x - min_x, 3.0);
    EXPECT_NEAR(max_y - min_y, 2.0, 0.3);
}

TEST(DrivableFootprint, TreatsOutsideMapAsNotDrivable) {
    cv::Mat road(100, 100, CV_8UC1, cv::Scalar(255));
    const double half = DrivableFootprintRatio(
        road, 1.0, 0.0, 0.0, Box(-10.5, 40.0, 9.5, 60.0), 8);
    EXPECT_NEAR(half, 0.5, 0.02);
}

TEST(DrivableFootprint, CenterClearanceUsesContinuousCoordinates) {
    cv::Mat road = cv::Mat::zeros(20, 20, CV_8UC1);
    road(cv::Rect(5, 5, 10, 10)).setTo(255);
    const cv::Mat clearance = DrivableClearanceMeters(road, 5.0);
    EXPECT_EQ(DrivablePointClearanceMeters(
                  clearance, 5.0, 0.0, 0.0, {0.8, 2.0}), 0.0);
    EXPECT_GE(DrivablePointClearanceMeters(
                  clearance, 5.0, 0.0, 0.0, {1.05, 2.0}), 0.05);
    EXPECT_GT(DrivablePointClearanceMeters(
                  clearance, 5.0, 0.0, 0.0, {2.0, 2.0}), 0.5);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
