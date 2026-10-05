#include <PlannerInterface/PlanningRelevanceFilter.hpp>
#include <PlannerInterface/PlannerObjectTypes.hpp>

#include <gtest/gtest.h>

namespace {

PlannerObject ObjectAt(double x, double y, double height = 1.0) {
    PlannerObject object;
    object.id = 7;
    object.classification = "unknown";
    object.yolo_class = "unknown";
    object.obb_center_map.x = x;
    object.obb_center_map.y = y;
    object.length = 2.0;
    object.width = 1.0;
    object.height = height;
    return object;
}

PlanningRelevanceDecision Next(
    PlanningRelevanceFilter* filter, const PlannerObject& object,
    bool upstream_keep = true, double ego_speed_mps = 0.0) {
    filter->BeginFrame();
    return filter->Evaluate(
        object, upstream_keep, 0.0, 0.0, ego_speed_mps, 4.6);
}

}  // namespace

TEST(PlanningRelevanceFilter, UpstreamDropCannotBeReintroduced) {
    PlanningRelevanceFilter filter;
    filter.Configure(0.10, 3, 5);
    const PlannerObject object = ObjectAt(30.0, 0.0, 0.5);
    EXPECT_FALSE(Next(&filter, object).keep);
    EXPECT_FALSE(Next(&filter, object).keep);
    ASSERT_TRUE(Next(&filter, object).planning_confirmed);

    const auto result = Next(&filter, object, false);
    EXPECT_FALSE(result.keep);
    EXPECT_EQ("upstream_filter_drop", result.reason);
    EXPECT_FALSE(result.retained_by_height_grace);
}

TEST(PlanningRelevanceFilter, FiveCentimeterGroundFragmentIsDropped) {
    PlanningRelevanceFilter filter;
    filter.Configure(0.10, 3, 5);
    for (int frame = 0; frame < 5; ++frame) {
        const auto result = Next(&filter, ObjectAt(30.0, 0.0, 0.05));
        EXPECT_FALSE(result.keep);
        EXPECT_FALSE(result.planning_confirmed);
        EXPECT_EQ(0, result.good_height_frames);
        EXPECT_EQ("ground_fragment_drop", result.reason);
    }
}

TEST(PlanningRelevanceFilter, UnknownObstacleRequiresContinuousEvidence) {
    PlanningRelevanceFilter filter;
    filter.Configure(0.10, 3, 5);
    const PlannerObject object = ObjectAt(30.0, 0.0, 0.5);
    const auto first = Next(&filter, object);
    EXPECT_FALSE(first.keep);
    EXPECT_EQ("obstacle_confirmation_pending", first.reason);
    EXPECT_EQ(1, first.good_height_frames);
    const auto second = Next(&filter, object);
    EXPECT_FALSE(second.keep);
    EXPECT_EQ("obstacle_confirmation_pending", second.reason);
    EXPECT_EQ(2, second.good_height_frames);
    const auto confirmed = Next(&filter, object);
    EXPECT_TRUE(confirmed.keep);
    EXPECT_TRUE(confirmed.planning_confirmed);
    EXPECT_EQ("obstacle_confirmed_keep", confirmed.reason);
    EXPECT_EQ(3, confirmed.height_confirmations);
    EXPECT_EQ(3, confirmed.good_height_frames);
    EXPECT_EQ(0, confirmed.bad_height_frames);
    EXPECT_EQ(5, confirmed.planning_release_frames);
}

TEST(PlanningRelevanceFilter, OneFrameHeightCollapseRetainsConfirmedTrack) {
    PlanningRelevanceFilter filter;
    filter.Configure(0.10, 3, 5);
    const PlannerObject good = ObjectAt(30.0, 0.0, 0.5);
    EXPECT_FALSE(Next(&filter, good).keep);
    EXPECT_FALSE(Next(&filter, good).keep);
    ASSERT_TRUE(Next(&filter, good).planning_confirmed);

    const auto collapsed = Next(&filter, ObjectAt(30.0, 0.0, 0.05));
    EXPECT_TRUE(collapsed.keep);
    EXPECT_TRUE(collapsed.planning_confirmed);
    EXPECT_TRUE(collapsed.retained_by_height_grace);
    EXPECT_EQ(0, collapsed.good_height_frames);
    EXPECT_EQ(1, collapsed.bad_height_frames);
    EXPECT_EQ("confirmed_obstacle_height_grace", collapsed.reason);
}

TEST(PlanningRelevanceFilter, FourTransientBadFramesRemainKept) {
    PlanningRelevanceFilter filter;
    filter.Configure(0.10, 3, 5);
    const PlannerObject good = ObjectAt(30.0, 0.0, 0.5);
    Next(&filter, good);
    Next(&filter, good);
    ASSERT_TRUE(Next(&filter, good).planning_confirmed);

    for (int bad_frame = 1; bad_frame < 5; ++bad_frame) {
        const auto result = Next(
            &filter, ObjectAt(30.0, 0.0, 0.05));
        EXPECT_TRUE(result.keep) << "bad frame " << bad_frame;
        EXPECT_TRUE(result.planning_confirmed);
        EXPECT_TRUE(result.retained_by_height_grace);
        EXPECT_EQ(bad_frame, result.bad_height_frames);
        EXPECT_EQ("confirmed_obstacle_height_grace", result.reason);
    }
}

TEST(PlanningRelevanceFilter, FifthBadFrameReleasesConfirmedTrack) {
    PlanningRelevanceFilter filter;
    filter.Configure(0.10, 3, 5);
    const PlannerObject good = ObjectAt(30.0, 0.0, 0.5);
    Next(&filter, good);
    Next(&filter, good);
    ASSERT_TRUE(Next(&filter, good).planning_confirmed);

    PlanningRelevanceDecision result;
    for (int bad_frame = 1; bad_frame <= 5; ++bad_frame) {
        result = Next(&filter, ObjectAt(30.0, 0.0, 0.05));
    }
    EXPECT_FALSE(result.keep);
    EXPECT_FALSE(result.planning_confirmed);
    EXPECT_FALSE(result.retained_by_height_grace);
    EXPECT_EQ(5, result.bad_height_frames);
    EXPECT_EQ("confirmed_obstacle_height_release", result.reason);
}

TEST(PlanningRelevanceFilter, GoodFrameInsideGraceRecoversImmediately) {
    PlanningRelevanceFilter filter;
    filter.Configure(0.10, 3, 5);
    const PlannerObject good = ObjectAt(30.0, 0.0, 0.5);
    Next(&filter, good);
    Next(&filter, good);
    ASSERT_TRUE(Next(&filter, good).planning_confirmed);
    EXPECT_TRUE(Next(&filter, ObjectAt(30.0, 0.0, 0.05)).keep);
    EXPECT_TRUE(Next(&filter, ObjectAt(30.0, 0.0, 0.06)).keep);

    const auto recovered = Next(&filter, good);
    EXPECT_TRUE(recovered.keep);
    EXPECT_TRUE(recovered.planning_confirmed);
    EXPECT_FALSE(recovered.retained_by_height_grace);
    EXPECT_EQ(1, recovered.good_height_frames);
    EXPECT_EQ(0, recovered.bad_height_frames);
    EXPECT_EQ("obstacle_confirmed_keep", recovered.reason);
}

TEST(PlanningRelevanceFilter, ReleasedTrackRequiresFullReacquisition) {
    PlanningRelevanceFilter filter;
    filter.Configure(0.10, 3, 5);
    const PlannerObject good = ObjectAt(30.0, 0.0, 0.5);
    Next(&filter, good);
    Next(&filter, good);
    ASSERT_TRUE(Next(&filter, good).planning_confirmed);
    for (int bad_frame = 0; bad_frame < 5; ++bad_frame) {
        Next(&filter, ObjectAt(30.0, 0.0, 0.05));
    }

    const auto first = Next(&filter, good);
    const auto second = Next(&filter, good);
    const auto third = Next(&filter, good);
    EXPECT_FALSE(first.keep);
    EXPECT_FALSE(second.keep);
    EXPECT_EQ("obstacle_confirmation_pending", first.reason);
    EXPECT_EQ("obstacle_confirmation_pending", second.reason);
    EXPECT_TRUE(third.keep);
    EXPECT_TRUE(third.planning_confirmed);
    EXPECT_EQ("obstacle_confirmed_keep", third.reason);
}

TEST(PlanningRelevanceFilter, SeparateTrackerIdDoesNotInheritConfirmation) {
    PlanningRelevanceFilter filter;
    filter.Configure(0.10, 3, 5);
    PlannerObject first = ObjectAt(30.0, 0.0, 0.5);
    first.id = 100;
    Next(&filter, first);
    Next(&filter, first);
    ASSERT_TRUE(Next(&filter, first).planning_confirmed);

    PlannerObject second = ObjectAt(30.0, 0.0, 0.05);
    second.id = 101;
    const auto result = Next(&filter, second);
    EXPECT_FALSE(result.keep);
    EXPECT_FALSE(result.planning_confirmed);
    EXPECT_EQ("ground_fragment_drop", result.reason);
}

TEST(PlanningRelevanceFilter, PreviousAvoidFailureSequenceDoesNotFlicker) {
    PlanningRelevanceFilter filter;
    filter.Configure(0.10, 3, 5);
    EXPECT_FALSE(Next(&filter, ObjectAt(30.0, 0.0, 0.72)).keep);
    EXPECT_FALSE(Next(&filter, ObjectAt(30.0, 0.0, 0.78)).keep);
    ASSERT_TRUE(Next(&filter, ObjectAt(30.0, 0.0, 0.75)).keep);

    const auto first_collapse =
        Next(&filter, ObjectAt(30.0, 0.0, 0.05));
    const auto second_collapse =
        Next(&filter, ObjectAt(30.0, 0.0, 0.06));
    const auto recovered =
        Next(&filter, ObjectAt(30.0, 0.0, 0.71));
    EXPECT_TRUE(first_collapse.keep);
    EXPECT_TRUE(second_collapse.keep);
    EXPECT_TRUE(recovered.keep);
    EXPECT_TRUE(recovered.planning_confirmed);
    EXPECT_EQ(0, recovered.bad_height_frames);
}

TEST(PlanningRelevanceFilter, MissedFrameResetsUnknownConfirmation) {
    PlanningRelevanceFilter filter;
    filter.Configure(0.10, 3, 5);
    const PlannerObject object = ObjectAt(30.0, 0.0, 0.5);
    EXPECT_FALSE(Next(&filter, object).keep);
    filter.BeginFrame();
    const auto after_gap = Next(&filter, object);
    EXPECT_FALSE(after_gap.keep);
    EXPECT_EQ(1, after_gap.height_confirmations);
}

TEST(PlanningRelevanceFilter, ShortDropoutPreservesExistingHeightGrace) {
    PlanningRelevanceFilter filter;
    filter.Configure(0.10, 3, 5);
    const PlannerObject good = ObjectAt(30.0, 0.0, 0.5);
    Next(&filter, good);
    Next(&filter, good);
    ASSERT_TRUE(Next(&filter, good).planning_confirmed);

    filter.BeginFrame();
    const auto after_gap =
        Next(&filter, ObjectAt(30.0, 0.0, 0.05));
    EXPECT_TRUE(after_gap.keep);
    EXPECT_TRUE(after_gap.planning_confirmed);
    EXPECT_TRUE(after_gap.retained_by_height_grace);
    EXPECT_EQ(1, after_gap.bad_height_frames);
    EXPECT_EQ("confirmed_obstacle_height_grace", after_gap.reason);
}

TEST(PlanningRelevanceFilter, CloseObstacleBypassesWarmup) {
    PlanningRelevanceFilter filter;
    filter.Configure(0.10, 3, 5);
    const auto result = Next(&filter, ObjectAt(6.0, 0.0, 0.5), true, 5.0);
    EXPECT_TRUE(result.keep);
    EXPECT_TRUE(result.emergency_override);
    EXPECT_EQ("close_obstacle_immediate_keep", result.reason);
}

TEST(PlanningRelevanceFilter, ClassifiedVehicleBypassesWarmup) {
    PlanningRelevanceFilter filter;
    filter.Configure(0.10, 3, 5);
    // Even a transiently collapsed LiDAR OBB must not remove an object whose
    // camera/fusion semantics already identify it as a vehicle.
    PlannerObject object = ObjectAt(40.0, 0.0, 0.05);
    object.classification = "car";
    const auto result = Next(&filter, object);
    EXPECT_TRUE(result.keep);
    EXPECT_FALSE(result.policy_applied);
    EXPECT_EQ("classified_vehicle_immediate_keep", result.reason);
}

TEST(PlanningRelevanceFilter, PedestrianIsNeverHeightGated) {
    PlanningRelevanceFilter filter;
    filter.Configure(0.10, 3, 5);
    PlannerObject object = ObjectAt(40.0, 0.0, 0.05);
    object.yolo_class = "person";
    const auto result = Next(&filter, object);
    EXPECT_TRUE(result.keep);
    EXPECT_FALSE(result.policy_applied);
    EXPECT_EQ("pedestrian_immediate_keep", result.reason);
}

TEST(PlanningRelevanceFilter, FlatElevatedReturnsAcquireAtLongRange) {
    PlanningRelevanceFilter filter;
    filter.Configure(0.10, 3, 5);
    auto object = ObjectAt(50.0, 0.0, 0.05);
    struct Point { double x, y, z; };
    GroundPlaneSnapshot plane;
    plane.valid = true;
    plane.coefficients = {{0, 0, 1, 1.5}};
    const std::vector<Point> points{{50, 0, -.85}, {50, .2, -.85}, {50, .4, -.85}};
    object.ground_evidence = MeasureObjectGroundEvidence(points, plane);
    const auto first = Next(&filter, object);
    EXPECT_FALSE(first.keep);
    EXPECT_TRUE(first.elevated_ground_evidence);
    EXPECT_FALSE(Next(&filter, object).keep);
    EXPECT_TRUE(Next(&filter, object).planning_confirmed);
}

TEST(PlanningRelevanceFilter, ElevatedScanRingDoesNotResetAcquisition) {
    PlanningRelevanceFilter filter;
    filter.Configure(0.10, 3, 5);
    auto object = ObjectAt(35.0, 0.0, 0.85);
    EXPECT_FALSE(Next(&filter, object).keep);
    object.height = 0.05;
    object.ground_evidence = {true, 6, 0.70, 0.37};
    EXPECT_FALSE(Next(&filter, object).keep);
    object.height = 0.50;
    EXPECT_TRUE(Next(&filter, object).planning_confirmed);
}

TEST(PlanningRelevanceFilter, RoadAndBelowPlaneFragmentsRemainDropped) {
    PlanningRelevanceFilter filter;
    filter.Configure(0.10, 3, 5);
    auto object = ObjectAt(50.0, 0.0, 0.05);
    for (double clearance : {0.05, 0.10, 0.39, 0.40, -0.60}) {
        object.ground_evidence = {true, 8, clearance, 0.40};
        EXPECT_FALSE(Next(&filter, object).keep);
    }
}

TEST(PlanningRelevanceFilter, InsufficientOrUnavailablePlaneDoesNotRescueFlatBox) {
    PlanningRelevanceFilter filter;
    filter.Configure(0.10, 3, 5);
    auto object = ObjectAt(50.0, 0.0, 0.05);
    object.ground_evidence = {false, 8, 0.70, 0.40};
    EXPECT_FALSE(Next(&filter, object).keep);
    object.ground_evidence = {true, 2, 0.70, 0.40};
    EXPECT_FALSE(Next(&filter, object).keep);
}

TEST(PlanningRelevanceFilter, GroundEvidenceCannotBypassUpstreamExclusion) {
    PlanningRelevanceFilter filter;
    filter.Configure(0.10, 3, 5);
    auto object = ObjectAt(50.0, 0.0, 0.05);
    object.ground_evidence = {true, 8, 0.70, 0.40};
    EXPECT_EQ("upstream_filter_drop", Next(&filter, object, false).reason);
}

TEST(PlanningRelevanceFilter, FlatElevatedReturnsPreserveConfirmedObject) {
    PlanningRelevanceFilter filter;
    filter.Configure(0.10, 3, 5);
    auto object = ObjectAt(50.0, 0.0, 0.65);
    Next(&filter, object);
    Next(&filter, object);
    ASSERT_TRUE(Next(&filter, object).planning_confirmed);
    object.height = 0.05;
    object.ground_evidence = {true, 8, 0.65, 0.40};
    for (int frame = 0; frame < 10; ++frame) {
        const auto result = Next(&filter, object);
        EXPECT_TRUE(result.keep);
        EXPECT_EQ(0, result.bad_height_frames);
    }
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}


TEST(PlanningRelevanceFilter, ConfirmedElevatedObjectSurvivesShortDropout) {
    PlanningRelevanceFilter filter;
    filter.Configure(0.10, 3, 5);
    auto object = ObjectAt(30.0, 0.0, 0.05);
    object.ground_evidence.valid = true;
    object.ground_evidence.sample_count = 22;
    object.ground_evidence.median_clearance_m = 1.2;
    object.ground_evidence.rejection_band_m = 0.34;
    Next(&filter, object);
    Next(&filter, object);
    ASSERT_TRUE(Next(&filter, object).planning_confirmed);
    for (int i = 0; i < 5; ++i) filter.BeginFrame();
    object.obb_center_map.x += 0.1;
    const auto result = Next(&filter, object);
    EXPECT_TRUE(result.keep);
    EXPECT_TRUE(result.planning_confirmed);
    EXPECT_TRUE(result.elevated_ground_evidence);
    EXPECT_EQ(1, result.good_height_frames);
    EXPECT_EQ("obstacle_confirmed_keep", result.reason);
}

TEST(PlanningRelevanceFilter, LongDropoutRequiresNewConfirmation) {
    PlanningRelevanceFilter filter;
    filter.Configure(0.10, 3, 5);
    const auto object = ObjectAt(30.0, 0.0);
    Next(&filter, object);
    Next(&filter, object);
    ASSERT_TRUE(Next(&filter, object).planning_confirmed);
    for (int i = 0; i < 6; ++i) filter.BeginFrame();
    const auto result = Next(&filter, object);
    EXPECT_FALSE(result.keep);
    EXPECT_FALSE(result.planning_confirmed);
    EXPECT_EQ(1, result.good_height_frames);
}

TEST(PlanningRelevanceFilter, ReusedIdAtDifferentLocationRequiresConfirmation) {
    PlanningRelevanceFilter filter;
    filter.Configure(0.10, 3, 5);
    auto object = ObjectAt(30.0, 0.0);
    Next(&filter, object);
    Next(&filter, object);
    ASSERT_TRUE(Next(&filter, object).planning_confirmed);
    filter.BeginFrame();
    object.obb_center_map.x = 40.0;
    const auto result = Next(&filter, object);
    EXPECT_FALSE(result.keep);
    EXPECT_FALSE(result.planning_confirmed);
}

TEST(PlanningRelevanceFilter, NewIdDoesNotInheritNearbyConfirmation) {
    PlanningRelevanceFilter filter;
    filter.Configure(0.10, 3, 5);
    auto object = ObjectAt(30.0, 0.0);
    Next(&filter, object);
    Next(&filter, object);
    ASSERT_TRUE(Next(&filter, object).planning_confirmed);
    filter.BeginFrame();
    ++object.id;
    EXPECT_FALSE(Next(&filter, object).keep);
}

TEST(PlanningRelevanceFilter, DropoutsDoNotResetAccumulatedBadHeightEvidence) {
    PlanningRelevanceFilter filter;
    filter.Configure(0.10, 3, 5);
    auto object = ObjectAt(30.0, 0.0);
    Next(&filter, object);
    Next(&filter, object);
    ASSERT_TRUE(Next(&filter, object).planning_confirmed);
    object.height = 0.05;
    for (int i = 1; i <= 5; ++i) {
        filter.BeginFrame();
        const auto result = Next(&filter, object);
        EXPECT_EQ(i, result.bad_height_frames);
        EXPECT_EQ(i < 5, result.keep);
        EXPECT_EQ(i < 5, result.planning_confirmed);
    }
}

TEST(PlanningRelevanceFilter, UpstreamRejectedReappearanceStaysRejected) {
    PlanningRelevanceFilter filter;
    filter.Configure(0.10, 3, 5);
    const auto object = ObjectAt(30.0, 0.0);
    Next(&filter, object);
    Next(&filter, object);
    ASSERT_TRUE(Next(&filter, object).planning_confirmed);
    filter.BeginFrame();
    EXPECT_FALSE(Next(&filter, object, false).keep);
}
