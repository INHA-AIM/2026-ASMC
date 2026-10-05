#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "cost/cost_function.hpp"

namespace {

ReferencePath SinglePointReference(double x, double y, double yaw,
                                   double speed = 0.0,
                                   double curvature = 0.0) {
    ReferencePath ref;
    ref.x_ref.push_back(x);
    ref.y_ref.push_back(y);
    ref.yaw_ref.push_back(yaw);
    ref.v_ref.push_back(speed);
    ref.a_ref.push_back(0.0);
    ref.k_ref.push_back(curvature);
    return ref;
}

TEST(TrackingErrorGeometryTest, StraightPureLateralOffset) {
    const ReferencePath ref = SinglePointReference(0.0, 0.0, 0.0);
    MPCState state;
    state.y = 1.0;

    const TrackingError error = computeTrackingError(state, ref, 0);
    EXPECT_DOUBLE_EQ(1.0, error.contour);
    EXPECT_DOUBLE_EQ(0.0, error.lag);
}

TEST(TrackingErrorGeometryTest, StraightPureLongitudinalLag) {
    const ReferencePath ref = SinglePointReference(0.0, 0.0, 0.0);
    MPCState state;
    state.x = -1.0;

    const TrackingError error = computeTrackingError(state, ref, 0);
    EXPECT_DOUBLE_EQ(0.0, error.contour);
    EXPECT_DOUBLE_EQ(-1.0, error.lag);
}

TEST(TrackingErrorGeometryTest, NinetyDegreeReferenceRotatesGlobalAxes) {
    const ReferencePath ref = SinglePointReference(2.0, 3.0, M_PI_2);

    MPCState tangent_state;
    tangent_state.x = 2.0;
    tangent_state.y = 4.0;
    const TrackingError tangent =
        computeTrackingError(tangent_state, ref, 0);
    EXPECT_NEAR(0.0, tangent.contour, 1e-12);
    EXPECT_NEAR(1.0, tangent.lag, 1e-12);

    MPCState left_normal_state;
    left_normal_state.x = 1.0;
    left_normal_state.y = 3.0;
    const TrackingError normal =
        computeTrackingError(left_normal_state, ref, 0);
    EXPECT_NEAR(1.0, normal.contour, 1e-12);
    EXPECT_NEAR(0.0, normal.lag, 1e-12);
}

TEST(TrackingErrorGeometryTest, RotationPreservesEuclideanNorm) {
    const std::vector<double> yaws = {0.0, 0.3, -1.2, M_PI_2, -M_PI};
    const std::vector<std::pair<double, double>> offsets = {
        {1.0, 0.0}, {0.0, -2.0}, {1.25, -0.75}, {-3.0, 4.0}};
    for (double yaw : yaws) {
        const ReferencePath ref = SinglePointReference(4.0, -2.0, yaw);
        for (const auto& offset : offsets) {
            MPCState state;
            state.x = 4.0 + offset.first;
            state.y = -2.0 + offset.second;
            const TrackingError error = computeTrackingError(state, ref, 0);
            EXPECT_NEAR(offset.first * offset.first +
                            offset.second * offset.second,
                        error.contour * error.contour +
                            error.lag * error.lag,
                        1e-12);
        }
    }
}

TEST(TrackingErrorWeightTest, LateralCostsMoreThanEqualLongitudinalError) {
    const TrackingError lateral{/*contour=*/1.0, /*lag=*/0.0};
    const TrackingError longitudinal{/*contour=*/0.0, /*lag=*/1.0};
    const double lateral_cost =
        computeContourErrorCost(lateral, 6.0) +
        computeLagErrorCost(lateral, 0.5);
    const double longitudinal_cost =
        computeContourErrorCost(longitudinal, 6.0) +
        computeLagErrorCost(longitudinal, 0.5);
    EXPECT_DOUBLE_EQ(6.0, lateral_cost);
    EXPECT_DOUBLE_EQ(0.5, longitudinal_cost);
    EXPECT_GT(lateral_cost, longitudinal_cost);
}

TEST(TrackingErrorWeightTest, EqualWeightsRecoverEuclideanCost) {
    const ReferencePath ref = SinglePointReference(1.0, -2.0, 0.7);
    MPCState state;
    state.x = 2.25;
    state.y = -2.75;
    const TrackingError error = computeTrackingError(state, ref, 0);
    constexpr double weight = 3.0;
    const double decomposed =
        computeContourErrorCost(error, weight) +
        computeLagErrorCost(error, weight);
    const double dx = state.x - ref.x_ref.front();
    const double dy = state.y - ref.y_ref.front();
    EXPECT_NEAR(weight * (dx * dx + dy * dy), decomposed, 1e-12);
}

TEST(TrackingErrorWeightTest, CurvedFrameSeparatesTangentAndNormalPenalty) {
    constexpr double yaw = 0.8;
    const ReferencePath ref =
        SinglePointReference(3.0, 5.0, yaw, 4.0, 0.04);
    const double tangent_x = std::cos(yaw);
    const double tangent_y = std::sin(yaw);
    const double normal_x = -std::sin(yaw);
    const double normal_y = std::cos(yaw);

    MPCState lagged;
    lagged.x = ref.x_ref.front() - tangent_x;
    lagged.y = ref.y_ref.front() - tangent_y;
    MPCState offset;
    offset.x = ref.x_ref.front() + normal_x;
    offset.y = ref.y_ref.front() + normal_y;

    const TrackingError lag_error = computeTrackingError(lagged, ref, 0);
    const TrackingError contour_error = computeTrackingError(offset, ref, 0);
    const double lag_cost =
        computeContourErrorCost(lag_error, 6.0) +
        computeLagErrorCost(lag_error, 0.5);
    const double contour_cost =
        computeContourErrorCost(contour_error, 6.0) +
        computeLagErrorCost(contour_error, 0.5);

    EXPECT_NEAR(0.0, lag_error.contour, 1e-12);
    EXPECT_NEAR(0.0, contour_error.lag, 1e-12);
    EXPECT_GT(contour_cost, lag_cost);
}

TEST(TrackingErrorWeightTest, TerminalPreservesStageContourLagRatio) {
    MPCParams params;
    params.weight_contour_error = 6.0;
    params.weight_lag_error = 0.5;
    params.weight_terminal = 8.0;
    const ReferencePath ref = SinglePointReference(0.0, 0.0, 0.0);
    const std::vector<MPCControl> no_controls;
    const MPCControl previous;

    MPCState contour_state;
    contour_state.y = 1.0;
    const CostBreakdown contour = computeCostBreakdown(
        {contour_state}, no_controls, ref, previous, params);
    MPCState lag_state;
    lag_state.x = -1.0;
    const CostBreakdown lag = computeCostBreakdown(
        {lag_state}, no_controls, ref, previous, params);

    EXPECT_DOUBLE_EQ(8.0, contour.terminal);
    EXPECT_NEAR(8.0 * 0.5 / 6.0, lag.terminal, 1e-12);
    EXPECT_GT(contour.terminal, lag.terminal);
}

}  // namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
