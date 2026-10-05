#include "planning_inputs/filters/PlannerObjectTypes.hpp"
#include <planning_inputs/filters/StaticStructureExclusionFilter.hpp>

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
        << "    require_infrastructure_shape: false\n";
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

