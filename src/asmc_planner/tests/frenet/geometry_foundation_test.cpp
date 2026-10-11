#include "Frenet/Geometry/PolygonUtils.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>

namespace {

constexpr double kTolerance = 1e-9;

int g_failures = 0;

void Expect(bool condition, const std::string& name) {
    if (condition) {
        std::cout << "[PASS] " << name << '\n';
        return;
    }
    std::cerr << "[FAIL] " << name << '\n';
    ++g_failures;
}

bool NearlyEqual(double a, double b, double tolerance = kTolerance) {
    return std::abs(a - b) <= tolerance;
}

geometry::Corners8 MakeBox(double center_x, double center_y,
                           double length, double width,
                           double yaw, double z_min, double z_max) {
    const double half_length = length / 2.0;
    const double half_width = width / 2.0;
    const double cosine = std::cos(yaw);
    const double sine = std::sin(yaw);
    const double local_xy[4][2] = {
        {-half_length, -half_width},
        { half_length, -half_width},
        { half_length,  half_width},
        {-half_length,  half_width},
    };

    geometry::Corners8 corners{};
    for (std::size_t i = 0; i < 4; ++i) {
        const double x = center_x + cosine * local_xy[i][0] - sine * local_xy[i][1];
        const double y = center_y + sine * local_xy[i][0] + cosine * local_xy[i][1];
        corners[i] = {x, y, z_min};
        corners[i + 4] = {x, y, z_max};
    }
    return corners;
}

geometry::Polygon2D HullOf(const geometry::Corners8& corners) {
    return geometry::ComputeConvexHull(
        geometry::ProjectCornersToXY(corners));
}

void TestCornerValidationAndHull() {
    const geometry::Corners8 box = MakeBox(10.0, -3.0, 4.0, 2.0, 0.0, 0.5, 2.5);
    const geometry::CornerValidationResult valid = geometry::ValidateCorners8(box);
    Expect(valid.valid, "valid 8-corner box accepted");
    Expect(valid.unique_xy_count == 4,
           "top and bottom XY duplicates collapse to four points");
    Expect(NearlyEqual(valid.xy_area, 8.0), "XY hull area matches rectangle area");

    geometry::Corners8 non_finite = box;
    non_finite[3].x = std::numeric_limits<double>::quiet_NaN();
    const auto non_finite_result = geometry::ValidateCorners8(non_finite);
    Expect(!non_finite_result.valid &&
               non_finite_result.error == geometry::CornerValidationError::NON_FINITE,
           "NaN corner rejected");

    geometry::Corners8 duplicate_only{};
    duplicate_only.fill({1.0, 2.0, 3.0});
    const auto duplicate_result = geometry::ValidateCorners8(duplicate_only);
    Expect(!duplicate_result.valid &&
               duplicate_result.error ==
                   geometry::CornerValidationError::INSUFFICIENT_UNIQUE_XY,
           "duplicate-only corners rejected");

    geometry::Corners8 collinear{};
    for (std::size_t i = 0; i < collinear.size(); ++i) {
        collinear[i] = {static_cast<double>(i), 0.0, 1.0};
    }
    const auto collinear_result = geometry::ValidateCorners8(collinear);
    Expect(!collinear_result.valid &&
               collinear_result.error ==
                   geometry::CornerValidationError::DEGENERATE_XY_AREA,
           "collinear corners rejected as degenerate");

    geometry::Polygon2D unordered = {
        {1.0, 1.0}, {0.5, 0.5}, {0.0, 0.0}, {1.0, 0.0},
        {0.0, 1.0}, {1.0, 1.0}, {0.0, 0.0},
    };
    const geometry::Polygon2D hull = geometry::ComputeConvexHull(unordered);
    Expect(hull.size() == 4, "convex hull removes duplicates and interior points");
    Expect(geometry::PolygonSignedArea(hull) > 0.0,
           "convex hull is counter-clockwise");
    Expect(NearlyEqual(geometry::PolygonArea(hull), 1.0),
           "unordered square hull has expected area");

    geometry::Polygon2D non_finite_points = unordered;
    non_finite_points.push_back(
        {std::numeric_limits<double>::infinity(), 0.0});
    Expect(geometry::ComputeConvexHull(non_finite_points).empty(),
           "convex hull rejects non-finite input");

    geometry::Polygon2D clockwise = {
        {0.0, 0.0}, {0.0, 1.0}, {1.0, 1.0}, {1.0, 0.0},
    };
    Expect(geometry::PolygonSignedArea(clockwise) < 0.0,
           "clockwise polygon detected");
    geometry::EnsureCounterClockwise(clockwise);
    Expect(geometry::PolygonSignedArea(clockwise) > 0.0,
           "polygon winding normalized to counter-clockwise");
}

void TestSAT() {
    const geometry::Polygon2D base = HullOf(MakeBox(0.0, 0.0, 2.0, 2.0, 0.0, 0.0, 1.0));
    const geometry::Polygon2D separated =
        HullOf(MakeBox(3.1, 0.0, 2.0, 2.0, 0.0, 0.0, 1.0));
    const geometry::Polygon2D overlapping =
        HullOf(MakeBox(1.5, 0.0, 2.0, 2.0, 0.0, 0.0, 1.0));
    const geometry::Polygon2D touching =
        HullOf(MakeBox(2.0, 0.0, 2.0, 2.0, 0.0, 0.0, 1.0));
    const geometry::Polygon2D vertex_touching =
        HullOf(MakeBox(2.0, 2.0, 2.0, 2.0, 0.0, 0.0, 1.0));
    const geometry::Polygon2D rotated =
        HullOf(MakeBox(1.8, 0.0, 2.0, 2.0, 0.7853981633974483, 0.0, 1.0));
    const geometry::Polygon2D contained =
        HullOf(MakeBox(0.0, 0.0, 0.5, 0.5, 0.2, 0.0, 1.0));

    Expect(!geometry::CheckConvexPolygonOverlapSAT(base, separated),
           "SAT separates disjoint polygons");
    Expect(geometry::CheckConvexPolygonOverlapSAT(base, overlapping),
           "SAT detects overlapping polygons");
    Expect(geometry::CheckConvexPolygonOverlapSAT(base, touching),
           "SAT treats edge boundary contact as collision");
    Expect(geometry::CheckConvexPolygonOverlapSAT(base, vertex_touching),
           "SAT treats vertex boundary contact as collision");
    Expect(geometry::CheckConvexPolygonOverlapSAT(base, rotated),
           "SAT detects rotated overlap");
    Expect(geometry::CheckConvexPolygonOverlapSAT(base, contained),
           "SAT detects polygon containment");

    geometry::Polygon2D reversed = base;
    std::reverse(reversed.begin(), reversed.end());
    Expect(geometry::CheckConvexPolygonOverlapSAT(reversed, overlapping),
           "SAT is independent of polygon winding");

    const geometry::Polygon2D invalid = {{0.0, 0.0}, {1.0, 0.0}};
    Expect(geometry::CheckConvexPolygonOverlapSAT(base, invalid),
           "invalid polygon is conservatively treated as collision");
}

void TestZOverlap() {
    const geometry::Corners8 low = MakeBox(0.0, 0.0, 2.0, 1.0, 0.0, 0.0, 1.0);
    const geometry::Corners8 high = MakeBox(0.0, 0.0, 2.0, 1.0, 0.0, 2.0, 3.0);
    const geometry::Corners8 touching = MakeBox(0.0, 0.0, 2.0, 1.0, 0.0, 1.0, 2.0);
    const geometry::ZRange low_range = geometry::ComputeZRange(low);
    const geometry::ZRange high_range = geometry::ComputeZRange(high);
    const geometry::ZRange touching_range = geometry::ComputeZRange(touching);

    Expect(low_range.valid && NearlyEqual(low_range.min, 0.0) &&
               NearlyEqual(low_range.max, 1.0),
           "Z range is derived from all eight corners");
    Expect(!geometry::CheckZOverlap(low_range, high_range),
           "separate Z ranges do not overlap");
    Expect(geometry::CheckZOverlap(low_range, touching_range),
           "touching Z boundaries overlap");
    Expect(geometry::CheckZOverlap(low_range, high_range, 1.0),
           "Z margin can create boundary overlap");
    Expect(geometry::CheckZOverlap(low_range, geometry::ZRange{}),
           "invalid Z range conservatively overlaps by default");
    Expect(!geometry::CheckZOverlap(low_range, geometry::ZRange{}, 0.0, false),
           "invalid Z range can be rejected when explicitly requested");
}

void TestTranslation() {
    const geometry::Corners8 original = MakeBox(4.0, -2.0, 4.0, 2.0, 0.4, -0.5, 1.5);
    const geometry::Corners8 translated =
        geometry::TranslateCorners(original, 3.0, -1.5, 2.0);

    bool all_offsets_match = true;
    bool z_preserved = true;
    for (std::size_t i = 0; i < original.size(); ++i) {
        all_offsets_match = all_offsets_match &&
            NearlyEqual(translated[i].x - original[i].x, 6.0) &&
            NearlyEqual(translated[i].y - original[i].y, -3.0);
        z_preserved = z_preserved && NearlyEqual(translated[i].z, original[i].z);
    }
    Expect(all_offsets_match, "constant velocity applies identical XY offset to every corner");
    Expect(z_preserved, "translation preserves every corner Z value");
    Expect(NearlyEqual(geometry::PolygonArea(HullOf(original)),
                       geometry::PolygonArea(HullOf(translated)), 1e-8),
           "translation preserves projected polygon area");

    const geometry::ZRange original_z = geometry::ComputeZRange(original);
    const geometry::ZRange translated_z = geometry::ComputeZRange(translated);
    Expect(NearlyEqual(original_z.min, translated_z.min) &&
               NearlyEqual(original_z.max, translated_z.max),
           "translation preserves Z range");

    const geometry::Corners8 unchanged =
        geometry::TranslateCorners(original, 9.0, -4.0, 0.0);
    bool zero_time_unchanged = true;
    for (std::size_t i = 0; i < original.size(); ++i) {
        zero_time_unchanged = zero_time_unchanged &&
            NearlyEqual(unchanged[i].x, original[i].x) &&
            NearlyEqual(unchanged[i].y, original[i].y) &&
            NearlyEqual(unchanged[i].z, original[i].z);
    }
    Expect(zero_time_unchanged, "zero prediction time leaves corners unchanged");
}

}  // namespace

int main() {
    TestCornerValidationAndHull();
    TestSAT();
    TestZOverlap();
    TestTranslation();

    if (g_failures == 0) {
        std::cout << "All geometry foundation tests passed\n";
        return 0;
    }
    std::cerr << g_failures << " geometry foundation test(s) failed\n";
    return 1;
}
