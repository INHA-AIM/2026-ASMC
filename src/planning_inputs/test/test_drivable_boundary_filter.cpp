#include <planning_inputs/filters/DrivableBoundaryFilter.hpp>
#include "planning_inputs/filters/PlannerObjectTypes.hpp"

#include <gtest/gtest.h>
#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <array>
#include <cstdio>
#include <fstream>
#include <limits>
#include <string>

namespace {

class TempMask {
public:
    TempMask() {
        const std::string base =
            std::string(::testing::TempDir()) + "drivable_filter_" +
            std::to_string(reinterpret_cast<std::uintptr_t>(this));
        image = base + ".png";
        metadata = base + ".yaml";
        cv::Mat mask(100, 100, CV_8UC1, cv::Scalar(0));
        mask(cv::Rect(20, 20, 60, 60)).setTo(255);
        EXPECT_TRUE(cv::imwrite(image, mask));
        std::ofstream output(metadata);
        output << "map_id: test_map\n"
               << "pixels_per_meter: 1.0\n"
               << "world_offset: [0.0, 0.0]\n"
               << "local_origin_in_global: [0.0, 0.0, 0.0]\n";
    }

    ~TempMask() {
        std::remove(image.c_str());
        std::remove(metadata.c_str());
    }

    std::string image;
    std::string metadata;
};

PlannerObject ObjectAt(double x, double y,
                       const std::string& classification = "unknown",
                       const std::string& yolo_class = "unknown") {
    PlannerObject object;
    object.id = 1;
    object.classification = classification;
    object.yolo_class = yolo_class;
    object.length = 2.0;
    object.width = 1.0;
    object.height = 1.0;
    const std::array<std::array<double, 2>, 4> xy{{
        {{x - 1.0, y - 0.5}}, {{x + 1.0, y - 0.5}},
        {{x + 1.0, y + 0.5}}, {{x - 1.0, y + 0.5}},
    }};
    for (int layer = 0; layer < 2; ++layer) {
        for (std::size_t i = 0; i < xy.size(); ++i) {
            auto& corner = object.obb_corners_map[layer * 4 + i];
            corner.x = xy[i][0];
            corner.y = xy[i][1];
            corner.z = layer == 0 ? 0.0 : 1.0;
        }
    }
    return object;
}

DrivableBoundaryFilter ReadyFilter(const TempMask& files) {
    DrivableBoundaryFilter filter;
    EXPECT_TRUE(filter.Configure(
        files.image, files.metadata, "test_map", {{0.0, 0.0, 0.0}},
        0.0, 0.3, 5));
    return filter;
}

}  // namespace

TEST(DrivableBoundaryFilter, UnknownOutsideIsDropped) {
    TempMask files;
    const auto result = ReadyFilter(files).Evaluate(ObjectAt(5.0, 5.0));
    EXPECT_FALSE(result.keep);
    EXPECT_TRUE(result.boundary_policy_applied);
    EXPECT_LT(result.road_ratio, 0.3);
    EXPECT_EQ("outside_drivable_drop", result.reason);
}

TEST(DrivableBoundaryFilter, UnknownInsideIsKept) {
    TempMask files;
    const auto result = ReadyFilter(files).Evaluate(ObjectAt(50.0, 50.0));
    EXPECT_TRUE(result.keep);
    EXPECT_TRUE(result.boundary_policy_applied);
    EXPECT_GE(result.road_ratio, 0.3);
    EXPECT_EQ("drivable_overlap_keep", result.reason);
}

TEST(DrivableBoundaryFilter, StaticVehicleUsesRoadOverlap) {
    TempMask files;
    auto filter = ReadyFilter(files);
    const auto outside = filter.Evaluate(ObjectAt(5.0, 5.0, "car"));
    const auto inside = filter.Evaluate(ObjectAt(50.0, 50.0, "car"));
    EXPECT_FALSE(outside.keep);
    EXPECT_TRUE(outside.boundary_policy_applied);
    EXPECT_TRUE(inside.keep);
    EXPECT_TRUE(inside.boundary_policy_applied);
}

TEST(DrivableBoundaryFilter, PedestrianOutsideIsProtected) {
    TempMask files;
    auto filter = ReadyFilter(files);
    for (const std::string label : {"person", "pedestrian"}) {
        const auto result = filter.Evaluate(
            ObjectAt(5.0, 5.0, "unknown", label));
        EXPECT_TRUE(result.keep) << label;
        EXPECT_FALSE(result.boundary_policy_applied) << label;
        EXPECT_EQ("pedestrian_keep", result.reason);
    }
}

TEST(DrivableBoundaryFilter, YoloVehicleOutsideUsesRoadOverlap) {
    TempMask files;
    auto filter = ReadyFilter(files);
    for (const std::string label : {"car", "vehicle", "truck", "bus"}) {
        const auto result = filter.Evaluate(
            ObjectAt(5.0, 5.0, "unknown", label));
        EXPECT_FALSE(result.keep) << label;
        EXPECT_TRUE(result.boundary_policy_applied) << label;
        EXPECT_EQ("outside_drivable_drop", result.reason);
    }
}

TEST(DrivableBoundaryFilter, MotionOutsideMustEnterRoadToBeKept) {
    TempMask files;
    auto filter = ReadyFilter(files);

    PlannerObject dynamic = ObjectAt(5.0, 5.0);
    dynamic.is_dynamic = true;
    EXPECT_FALSE(filter.Evaluate(dynamic).keep);
    EXPECT_EQ("motion_outside_drivable_drop",
              filter.Evaluate(dynamic).reason);

    PlannerObject pending = ObjectAt(5.0, 50.0);
    pending.motion_state = 2;
    pending.velocity_map_mps.x = 10.0;
    EXPECT_TRUE(filter.Evaluate(pending).keep);
    EXPECT_EQ("motion_predicted_drivable_keep",
              filter.Evaluate(pending).reason);
    EXPECT_LT(filter.Evaluate(pending).road_ratio, 0.3);
    EXPECT_GE(filter.Evaluate(pending).predicted_road_ratio, 0.3);

    PlannerObject confirmed = ObjectAt(50.0, 50.0);
    confirmed.motion_state = 3;
    EXPECT_TRUE(filter.Evaluate(confirmed).keep);
    EXPECT_EQ("motion_on_drivable_keep",
              filter.Evaluate(confirmed).reason);

    PlannerObject low_threat = ObjectAt(5.0, 5.0);
    low_threat.motion_state = 1;
    EXPECT_FALSE(filter.Evaluate(low_threat).keep);
}

TEST(DrivableBoundaryFilter, StaticPlanningGeometryUsesOnlyRoadIntersection) {
    TempMask files;
    auto filter = ReadyFilter(files);
    const PlannerObject raw = ObjectAt(19.8, 50.0);
    const auto decision = filter.Evaluate(raw);
    ASSERT_TRUE(decision.keep);

    PlannerObject planning;
    bool clipped = false;
    ASSERT_TRUE(filter.BuildPlanningObject(
        raw, decision, &planning, &clipped));
    EXPECT_TRUE(clipped);
    EXPECT_LT(planning.length, raw.length);
    EXPECT_GT(planning.length, 0.2);
    EXPECT_NEAR(planning.width, raw.width, 0.25);
    EXPECT_GT(planning.obb_center_map.x, raw.obb_center_map.x);
}

TEST(DrivableBoundaryFilter, MovingObjectKeepsPhysicalObb) {
    TempMask files;
    auto filter = ReadyFilter(files);
    PlannerObject raw = ObjectAt(20.5, 50.0);
    raw.motion_state = 3;
    const auto decision = filter.Evaluate(raw);
    ASSERT_TRUE(decision.keep);

    PlannerObject planning;
    bool clipped = true;
    ASSERT_TRUE(filter.BuildPlanningObject(
        raw, decision, &planning, &clipped));
    EXPECT_FALSE(clipped);
    EXPECT_DOUBLE_EQ(raw.length, planning.length);
    EXPECT_DOUBLE_EQ(raw.width, planning.width);
}

TEST(DrivableBoundaryFilter, ObbOverlapWinsWhenCenterIsOutside) {
    TempMask files;
    const auto result = ReadyFilter(files).Evaluate(ObjectAt(19.5, 50.0));
    EXPECT_TRUE(result.keep);
    EXPECT_TRUE(result.boundary_policy_applied);
    EXPECT_GE(result.road_ratio, 0.3);
}

TEST(DrivableBoundaryFilter, InvalidObbFailsOpen) {
    TempMask files;
    PlannerObject object = ObjectAt(5.0, 5.0);
    object.obb_corners_map[0].x =
        std::numeric_limits<double>::quiet_NaN();
    const auto result = ReadyFilter(files).Evaluate(object);
    EXPECT_TRUE(result.keep);
    EXPECT_FALSE(result.boundary_policy_applied);
    EXPECT_EQ("invalid_geometry_keep", result.reason);
}

TEST(DrivableBoundaryFilter, LoadFailureKeepsEveryObject) {
    DrivableBoundaryFilter filter;
    EXPECT_FALSE(filter.Configure(
        "/missing/mask.png", "/missing/mask.yaml", "test_map",
        {{0.0, 0.0, 0.0}}, 1.0, 0.3, 5));
    const auto result = filter.Evaluate(ObjectAt(5.0, 5.0));
    EXPECT_TRUE(result.keep);
    EXPECT_FALSE(result.filter_available);
    EXPECT_EQ(DrivableMaskStatus::kImageLoadFailed, filter.status());
}

TEST(DrivableBoundaryFilter, CornerOrderDoesNotChangeRatio) {
    TempMask files;
    auto filter = ReadyFilter(files);
    PlannerObject original = ObjectAt(20.5, 50.0);
    PlannerObject reordered = original;
    std::reverse(reordered.obb_corners_map.begin(),
                 reordered.obb_corners_map.end());
    const double a = filter.Evaluate(original).road_ratio;
    const double b = filter.Evaluate(reordered).road_ratio;
    EXPECT_DOUBLE_EQ(a, b);
}

TEST(DrivableBoundaryFilter, MapIdMismatchDisablesDeletion) {
    TempMask files;
    DrivableBoundaryFilter filter;
    EXPECT_FALSE(filter.Configure(
        files.image, files.metadata, "another_map", {{0.0, 0.0, 0.0}},
        0.0, 0.3, 5));
    EXPECT_EQ(DrivableMaskStatus::kMapIdMismatch, filter.status());
    EXPECT_NE(std::string::npos,
              filter.status_message().find("map id mismatch"));
    EXPECT_TRUE(filter.Evaluate(ObjectAt(5.0, 5.0)).keep);
}

TEST(DrivableBoundaryFilter, OriginMismatchDisablesDeletion) {
    TempMask files;
    DrivableBoundaryFilter filter;
    EXPECT_FALSE(filter.Configure(
        files.image, files.metadata, "test_map", {{1.0, 2.0, 0.0}},
        0.0, 0.3, 5));
    EXPECT_EQ(DrivableMaskStatus::kOriginMismatch, filter.status());
    EXPECT_TRUE(filter.Evaluate(ObjectAt(5.0, 5.0)).keep);
}

