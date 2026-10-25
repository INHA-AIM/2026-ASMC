// Regression cases ported from Main test/test_known_static_object_filter.cpp (ab8db094).
#include <asmc_lidar/filtering/delivery_filters.hpp>
#include <asmc_lidar/filtering/delivery_filters.hpp>

#include <gtest/gtest.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

using namespace asmc::lidar::filtering;
namespace {

class TempCatalog {
public:
    TempCatalog() {
        path = ::testing::TempDir() + "known_static_" +
            std::to_string(reinterpret_cast<std::uintptr_t>(this)) + ".yaml";
        std::ofstream output(path);
        output << "schema: asmc_known_static_center_trace_v1\n"
               << "map_id: test_map\n"
               << "local_origin_in_global: [1.0, 2.0, 3.0]\n"
               << "observations:\n"
               << "  - track_id: 42\n"
               << "    center: [10.0, 20.0]\n"
               << "    size: [2.0, 0.5, 1.0]\n"
               << "    yaw_rad: 0.0\n";
    }

    ~TempCatalog() { std::remove(path.c_str()); }
    std::string path;
};

DeliveryObject UnknownAt(double x, double y) {
    DeliveryObject object;
    object.classification = "unknown";
    object.yolo_class = "unknown";
    object.cluster_center_map.x = x;
    object.cluster_center_map.y = y;
    object.length = 2.0;
    object.width = 0.5;
    object.height = 1.0;
    object.obb_yaw_map_rad = 0.0;
    return object;
}

}  // namespace

TEST(KnownStaticObjectFilter, DropsMatchingUnknownOnly) {
    TempCatalog catalog;
    KnownStaticObjectFilter filter;
    ASSERT_TRUE(filter.Configure(
        catalog.path, "test_map", {{1.0, 2.0, 3.0}}, 1.0));
    const auto decision = filter.Evaluate(UnknownAt(10.2, 20.1));
    EXPECT_FALSE(decision.keep);
    EXPECT_EQ(decision.reference_track_id, 42);
    EXPECT_EQ(decision.reason, "known_static_match_drop");
}

TEST(KnownStaticObjectFilter, PreservesKnownClassAtCatalogLocation) {
    TempCatalog catalog;
    KnownStaticObjectFilter filter;
    ASSERT_TRUE(filter.Configure(
        catalog.path, "test_map", {{1.0, 2.0, 3.0}}, 1.0));
    DeliveryObject object = UnknownAt(10.0, 20.0);
    object.classification = "car";
    const auto decision = filter.Evaluate(object);
    EXPECT_TRUE(decision.keep);
    EXPECT_EQ(decision.reason, "confirmed_class_keep");
}

TEST(KnownStaticObjectFilter, CatalogMatchOverridesMotionLabelForUnknown) {
    TempCatalog catalog;
    KnownStaticObjectFilter filter;
    ASSERT_TRUE(filter.Configure(
        catalog.path, "test_map", {{1.0, 2.0, 3.0}}, 1.0));

    DeliveryObject pending = UnknownAt(10.0, 20.0);
    pending.motion_state = 2;
    const auto pending_decision = filter.Evaluate(pending);
    EXPECT_FALSE(pending_decision.keep);
    EXPECT_EQ(pending_decision.reason, "known_static_match_drop");

    DeliveryObject dynamic = UnknownAt(10.0, 20.0);
    dynamic.motion_state = 3;
    dynamic.is_dynamic = true;
    const auto dynamic_decision = filter.Evaluate(dynamic);
    EXPECT_FALSE(dynamic_decision.keep);
    EXPECT_EQ(dynamic_decision.reason, "known_static_match_drop");
}

TEST(KnownStaticObjectFilter, TrackIdIsNotUsedAsRuntimeIdentity) {
    TempCatalog catalog;
    KnownStaticObjectFilter filter;
    ASSERT_TRUE(filter.Configure(
        catalog.path, "test_map", {{1.0, 2.0, 3.0}}, 1.0));
    DeliveryObject object = UnknownAt(10.1, 20.1);
    object.id = 999;
    EXPECT_FALSE(filter.Evaluate(object).keep);
}

TEST(KnownStaticObjectFilter, FootprintAxisSwapStillMatches) {
    TempCatalog catalog;
    KnownStaticObjectFilter filter;
    ASSERT_TRUE(filter.Configure(
        catalog.path, "test_map", {{1.0, 2.0, 3.0}}, 1.0));
    DeliveryObject object = UnknownAt(10.0, 20.0);
    object.length = 0.5;
    object.width = 2.0;
    EXPECT_FALSE(filter.Evaluate(object).keep);
}

TEST(KnownStaticObjectFilter, VariableGeometryAtSameBackgroundLocationDrops) {
    TempCatalog catalog;
    KnownStaticObjectFilter filter;
    ASSERT_TRUE(filter.Configure(
        catalog.path, "test_map", {{1.0, 2.0, 3.0}}, 1.0));
    DeliveryObject object = UnknownAt(10.0, 20.0);
    object.length = 5.0;
    object.width = 3.0;
    object.height = 0.2;
    EXPECT_FALSE(filter.Evaluate(object).keep);
}

TEST(KnownStaticObjectFilter, UnknownAwayFromCatalogIsKept) {
    TempCatalog catalog;
    KnownStaticObjectFilter filter;
    ASSERT_TRUE(filter.Configure(
        catalog.path, "test_map", {{1.0, 2.0, 3.0}}, 1.0));
    EXPECT_TRUE(filter.Evaluate(UnknownAt(13.0, 20.0)).keep);
}

TEST(KnownStaticObjectFilter, AppliesOnlyInsideConfiguredRegion) {
    TempCatalog catalog;
    KnownStaticObjectFilter filter;
    ASSERT_TRUE(filter.Configure(
        catalog.path, "test_map", {{1.0, 2.0, 3.0}}, 1.0,
        true, 9.0, 11.0, 19.0, 21.0));
    EXPECT_FALSE(filter.Evaluate(UnknownAt(10.0, 20.0)).keep);

    const auto outside = filter.Evaluate(UnknownAt(12.0, 20.0));
    EXPECT_TRUE(outside.keep);
    EXPECT_FALSE(outside.policy_applied);
    EXPECT_EQ(outside.reason, "outside_application_region_keep");
}

TEST(KnownStaticObjectFilter, InvalidRegionFailsSafeKeep) {
    TempCatalog catalog;
    KnownStaticObjectFilter filter;
    EXPECT_FALSE(filter.Configure(
        catalog.path, "test_map", {{1.0, 2.0, 3.0}}, 1.0,
        true, 11.0, 9.0, 19.0, 21.0));
    EXPECT_TRUE(filter.Evaluate(UnknownAt(10.0, 20.0)).keep);
}

TEST(KnownStaticObjectFilter, MismatchFailsSafeKeep) {
    TempCatalog catalog;
    KnownStaticObjectFilter filter;
    EXPECT_FALSE(filter.Configure(
        catalog.path, "other_map", {{1.0, 2.0, 3.0}}, 1.0));
    EXPECT_TRUE(filter.Evaluate(UnknownAt(10.0, 20.0)).keep);
}

TEST(KnownStaticObjectFilter, DropsObservedLongGuardrailOnlyInTargetRegion) {
    KnownStaticObjectFilter filter;
    const std::string catalog_path =
        (std::filesystem::path(__FILE__).parent_path().parent_path() /
         "../config/delivery/known_static_objects.yaml").string();
    ASSERT_TRUE(filter.Configure(
        catalog_path, "waize_katri_2025-10-23",
        {{302595.0, 4124145.0, 0.0}}, 2.5,
        true, -35.0, 15.0, 108.0, 180.0));

    // 2026-09-14 live run에서 Frenet을 가로막은 긴 고정 가드레일의
    // cluster centers. 가시점 변화로 clean-lap 중심에서 최대 약 2 m 이동한다.
    EXPECT_FALSE(filter.Evaluate(UnknownAt(-4.0633, 134.2587)).keep);
    EXPECT_FALSE(filter.Evaluate(UnknownAt(12.7123, 120.1836)).keep);
    EXPECT_FALSE(filter.Evaluate(UnknownAt(-10.7617, 121.7582)).keep);
    EXPECT_FALSE(filter.Evaluate(UnknownAt(-22.4421, 165.7734)).keep);

    // 구간 밖 UNKNOWN과 동일 위치의 확정 분류 객체는 제거하지 않는다.
    EXPECT_TRUE(filter.Evaluate(UnknownAt(20.0, 140.0)).keep);
    DeliveryObject vehicle = UnknownAt(-4.0633, 134.2587);
    vehicle.classification = "car";
    EXPECT_TRUE(filter.Evaluate(vehicle).keep);
}

TEST(KnownStaticObjectFilter, DropsFollowingLongGuardrailInSecondRegion) {
    KnownStaticObjectFilter filter;
    const std::string catalog_path =
        (std::filesystem::path(__FILE__).parent_path().parent_path() /
         "../config/delivery/known_static_objects.yaml").string();
    ASSERT_TRUE(filter.ConfigureWithRegions(
        catalog_path, "waize_katri_2025-10-23",
        {{302595.0, 4124145.0, 0.0}}, 2.5,
        {{-35.0, 15.0, 108.0, 180.0},
         {-107.0, 15.0, 175.0, 345.0}}));

    // Live centers observed on the long continuation at ego
    // (-73.68, 198.56). Every point already exists in the clean-lap catalog.
    EXPECT_FALSE(filter.Evaluate(UnknownAt(-74.6479, 196.9965)).keep);
    EXPECT_FALSE(filter.Evaluate(UnknownAt(-75.7520, 197.6753)).keep);
    EXPECT_FALSE(filter.Evaluate(UnknownAt(-73.5250, 196.3199)).keep);
    EXPECT_FALSE(filter.Evaluate(UnknownAt(-76.8242, 198.3932)).keep);
    EXPECT_FALSE(filter.Evaluate(UnknownAt(-82.3990, 202.9534)).keep);
    EXPECT_FALSE(filter.Evaluate(UnknownAt(-66.0840, 192.7463)).keep);
    EXPECT_FALSE(filter.Evaluate(UnknownAt(-87.6841, 207.9600)).keep);
    EXPECT_FALSE(filter.Evaluate(UnknownAt(-90.1620, 211.5099)).keep);
    EXPECT_FALSE(filter.Evaluate(UnknownAt(-52.9841, 188.2842)).keep);
    // 2026-09-18 live run: a 42.9 m merged guardrail cluster moved its
    // cluster centroid 2.394 m from the nearest clean-lap reference.
    EXPECT_FALSE(filter.Evaluate(UnknownAt(-79.3629, 204.7071)).keep);

    // A coordinate between the two verified regions is outside policy scope.
    const auto outside = filter.Evaluate(UnknownAt(20.0, 200.0));
    EXPECT_TRUE(outside.keep);
    EXPECT_EQ(outside.reason, "outside_application_region_keep");

    DeliveryObject vehicle = UnknownAt(-82.3990, 202.9534);
    vehicle.classification = "car";
    EXPECT_TRUE(filter.Evaluate(vehicle).keep);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
