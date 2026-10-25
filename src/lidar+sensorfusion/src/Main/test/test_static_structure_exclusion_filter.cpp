#include <PlannerInterface/PlannerObjectAdapter.hpp>
#include <PlannerInterface/StaticStructureExclusionFilter.hpp>

#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <string>

namespace {

std::string WriteConfig() {
    const std::string path =
        std::string(::testing::TempDir()) +
        "static_structure_exclusion_filter_test.yaml";
    std::ofstream stream(path);
    stream
        << "zones:\n"
        << "  - name: shaped\n"
        << "    bounds: [0.0, 10.0, 0.0, 10.0]\n"
        << "  - name: unconditional\n"
        << "    bounds: [20.0, 30.0, 20.0, 30.0]\n"
        << "    require_infrastructure_shape: false\n"
        << "  - name: elevated_pole\n"
        << "    bounds: [-60.9, -60.2, -180.8, -179.0]\n"
        << "    require_elevated_pole_shape: true\n";
    return path;
}

PlannerObject ObjectAt(double x, double y) {
    PlannerObject object;
    object.classification = "unknown";
    object.yolo_class = "unknown";
    object.obb_center_map.x = x;
    object.obb_center_map.y = y;
    object.height = 3.0;
    for (std::size_t i = 0; i < object.obb_corners_map.size(); ++i) {
        object.obb_corners_map[i].x = x;
        object.obb_corners_map[i].y = y;
        object.obb_corners_map[i].z = i < 4 ? 0.0 : 3.0;
    }
    return object;
}

}  // namespace

TEST(StaticStructureExclusionFilter, DropsOnlyStaticUnknownInConfiguredZone) {
    const std::string path = WriteConfig();
    StaticStructureExclusionFilter filter;
    ASSERT_TRUE(filter.Configure(path, 2.5, 2.0));

    PlannerObject object = ObjectAt(5.0, 5.0);
    EXPECT_FALSE(filter.Evaluate(object).keep);

    object.classification = "person";
    EXPECT_TRUE(filter.Evaluate(object).keep);
    object.classification = "unknown";
    object.yolo_class = "car";
    EXPECT_TRUE(filter.Evaluate(object).keep);

    object.yolo_class = "unknown";
    object.motion_state = 2;
    EXPECT_TRUE(filter.Evaluate(object).keep);
    object.motion_state = 3;
    EXPECT_TRUE(filter.Evaluate(object).keep);
    object.motion_state = 0;
    object.is_dynamic = true;
    EXPECT_TRUE(filter.Evaluate(object).keep);

    std::remove(path.c_str());
}

TEST(StaticStructureExclusionFilter, KeepsShapeMismatchAndFailsOpen) {
    const std::string path = WriteConfig();
    StaticStructureExclusionFilter filter;
    ASSERT_TRUE(filter.Configure(path, 2.5, 2.0));

    PlannerObject low = ObjectAt(5.0, 5.0);
    low.height = 0.5;
    for (auto& corner : low.obb_corners_map) corner.z = 0.0;
    const auto shaped = filter.Evaluate(low);
    EXPECT_TRUE(shaped.keep);
    EXPECT_EQ("zone_shape_mismatch_keep", shaped.reason);

    low.obb_center_map.x = 25.0;
    low.obb_center_map.y = 25.0;
    EXPECT_FALSE(filter.Evaluate(low).keep);

    StaticStructureExclusionFilter unavailable;
    EXPECT_FALSE(unavailable.Configure("/does/not/exist.yaml", 2.5, 2.0));
    EXPECT_TRUE(unavailable.Evaluate(low).keep);

    std::remove(path.c_str());
}

TEST(StaticStructureExclusionFilter, C091SignNeedsPoleAndGroundEvidence) {
    const std::string path = WriteConfig();
    StaticStructureExclusionFilter filter;
    ASSERT_TRUE(filter.Configure(path, 2.5, 2.0));

    PlannerObject sign = ObjectAt(-60.56466, -179.97424);
    sign.length = 1.10427;
    sign.width = 0.16487;
    sign.height = 2.41325;
    sign.ground_evidence.valid = true;
    sign.ground_evidence.sample_count = 26;
    sign.ground_evidence.median_clearance_m = 2.43769;
    EXPECT_FALSE(filter.Evaluate(sign).keep);
    EXPECT_EQ("configured_elevated_pole_drop", filter.Evaluate(sign).reason);

    // One partial scan of the C091 speed sign previously fell 5 cm beyond
    // the zone and blocked every signal-stop path candidate.
    sign.obb_center_map.x = -60.25853736;
    sign.obb_center_map.y = -179.2504595;
    sign.length = 0.3700;
    sign.width = 0.19495;
    sign.height = 0.42665;
    sign.ground_evidence.sample_count = 8;
    sign.ground_evidence.median_clearance_m = 2.731;
    sign.motion_state = 2;
    EXPECT_FALSE(filter.Evaluate(sign).keep);
    EXPECT_EQ("configured_elevated_pole_drop", filter.Evaluate(sign).reason);
    sign.ground_evidence.median_clearance_m = 0.2;
    EXPECT_TRUE(filter.Evaluate(sign).keep);
    sign.ground_evidence.median_clearance_m = 2.731;
    sign.classification = "car";
    EXPECT_TRUE(filter.Evaluate(sign).keep);

    sign = ObjectAt(-60.56466, -179.97424);
    sign.length = 1.10427;
    sign.width = 0.16487;
    sign.height = 2.41325;
    sign.ground_evidence.valid = true;
    sign.ground_evidence.sample_count = 26;
    sign.ground_evidence.median_clearance_m = 2.43769;

    sign.ground_evidence.valid = false;
    EXPECT_TRUE(filter.Evaluate(sign).keep);
    sign.ground_evidence.valid = true;
    sign.ground_evidence.sample_count = 4;
    EXPECT_TRUE(filter.Evaluate(sign).keep);
    sign.ground_evidence.sample_count = 26;
    sign.ground_evidence.median_clearance_m = 0.2;
    EXPECT_TRUE(filter.Evaluate(sign).keep);
    sign.ground_evidence.median_clearance_m = 2.43769;
    sign.height = 0.05;
    sign.length = 0.39;
    sign.ground_evidence.sample_count = 5;
    EXPECT_FALSE(filter.Evaluate(sign).keep);
    sign.height = 2.41325;
    sign.length = 1.10427;
    sign.width = 0.8;
    EXPECT_TRUE(filter.Evaluate(sign).keep);
    sign.width = 0.16487;
    sign.motion_state = 2;
    sign.is_dynamic = true;
    EXPECT_FALSE(filter.Evaluate(sign).keep);
    sign.ground_evidence.valid = false;
    EXPECT_TRUE(filter.Evaluate(sign).keep);
    sign.ground_evidence.valid = true;
    sign.motion_state = 1;
    sign.is_dynamic = false;
    sign.classification = "person";
    EXPECT_TRUE(filter.Evaluate(sign).keep);
    sign.classification = "unknown";
    sign.obb_center_map.x = -61.2;
    EXPECT_TRUE(filter.Evaluate(sign).keep);

    std::remove(path.c_str());
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
