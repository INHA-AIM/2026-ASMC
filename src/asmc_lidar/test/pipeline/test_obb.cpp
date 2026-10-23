// OBB port tests: obb_util (Main Tracking.cpp:25-265 at 8f93c22e) and the
// detection builder (Objectinfo_for_km). Expected values are computed
// independently in the test (closed-form rectangles), never by the code under test.
#include <asmc_lidar/detection/obb.hpp>
#include <asmc_lidar/tracking/tracker.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace {
using namespace asmc::lidar::pipeline;
namespace ou = asmc::lidar::pipeline::obb_util;

constexpr double kPosTol = 1e-5;  // m
constexpr double kAngTol = 1e-5;  // rad
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

double AxisError(double a, double b) { return std::abs(std::remainder(a - b, M_PI)); }

pcl::PointXYZI Pt(double x, double y, double z) {
  pcl::PointXYZI p;
  p.x = static_cast<float>(x);
  p.y = static_cast<float>(y);
  p.z = static_cast<float>(z);
  p.intensity = 1.f;
  return p;
}

// Closed-form rectangle (centre cx,cy, heading, length along heading, width),
// corners in computeOBBAtYaw order for the given corner yaw.
struct Rect {
  double cx, cy, heading, length, width, zmin, zmax;
  std::array<std::array<double, 2>, 4> Corners(double corner_yaw) const {
    const double c = std::cos(corner_yaw), s = std::sin(corner_yaw);
    // Half extents along the corner-yaw axes (the rectangle is symmetric).
    const double along = std::abs(std::remainder(corner_yaw - heading, M_PI)) < 1e-3 ? length : width;
    const double across = along == length ? width : length;
    const double su[] = {-1, 1, 1, -1}, sv[] = {-1, -1, 1, 1};
    std::array<std::array<double, 2>, 4> out{};
    for (int i = 0; i < 4; ++i) {
      out[i] = {cx + su[i] * .5 * along * c - sv[i] * .5 * across * s,
                cy + su[i] * .5 * along * s + sv[i] * .5 * across * c};
    }
    return out;
  }
};

// Perimeter samples (top and bottom), `per_edge` points per edge incl. corners.
Cloud RectCloud(const Rect& r, int per_edge) {
  Cloud cloud;
  const auto corners = r.Corners(r.heading);
  for (double z : {r.zmin, r.zmax}) {
    for (int e = 0; e < 4; ++e) {
      const auto& a = corners[e];
      const auto& b = corners[(e + 1) % 4];
      for (int k = 0; k < per_edge; ++k) {
        const double t = static_cast<double>(k) / per_edge;
        cloud.push_back(Pt(a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), z));
      }
    }
  }
  return cloud;
}

void ExpectCcwFromPlusZ(const TrackedObject& o) {
  for (int face = 0; face < 2; ++face) {
    for (int i = 0; i < 4; ++i) {
      const auto& a = o.OBB_Corners_Body[4 * face + i];
      const auto& b = o.OBB_Corners_Body[4 * face + (i + 1) % 4];
      const auto& c = o.OBB_Corners_Body[4 * face + (i + 2) % 4];
      const double cross = (b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x);
      EXPECT_GT(cross, 0.) << "face " << face << " corner " << i << " not counter-clockwise";
    }
  }
}

void ExpectObbRule(const TrackedObject& o) {
  const auto& c0 = o.OBB_Corners_Body[0];
  const auto& c1 = o.OBB_Corners_Body[1];
  EXPECT_LT(AxisError(o.f32_OBB_YawRad, std::atan2(c1.y - c0.y, c1.x - c0.x)), kAngTol) << "yaw vs edge 0->1";
  double mx = 0, my = 0, mz = 0;
  for (const auto& c : o.OBB_Corners_Body) {
    mx += c.x / 8.;
    my += c.y / 8.;
    mz += c.z / 8.;
  }
  EXPECT_NEAR(mx, o.f32_OBB_CenterX, kPosTol);
  EXPECT_NEAR(my, o.f32_OBB_CenterY, kPosTol);
  EXPECT_NEAR(mz, o.f32_OBB_CenterZ, kPosTol);
  for (int i = 0; i < 4; ++i) {
    EXPECT_EQ(o.OBB_Corners_Body[i].z, o.f32_OBB_ZMin) << "bottom corner " << i;
    EXPECT_EQ(o.OBB_Corners_Body[i + 4].z, o.f32_OBB_ZMax) << "top corner " << i;
    EXPECT_EQ(o.OBB_Corners_Body[i + 4].x, o.OBB_Corners_Body[i].x);
    EXPECT_EQ(o.OBB_Corners_Body[i + 4].y, o.OBB_Corners_Body[i].y);
  }
  EXPECT_GE(o.f32_OBB_YawRad, -M_PI / 2);
  EXPECT_LT(o.f32_OBB_YawRad, M_PI / 2);
  ExpectCcwFromPlusZ(o);
}

TrackedObject FromCloud(const Cloud& cloud, double yaw) {
  TrackedObject o;
  o.pcl_ClusterPoints = cloud;
  ou::OBB2D obb;
  double z0 = 0, z1 = 0;
  std::array<ou::P3, 8> corners{};
  ou::computeOBBAtYaw(cloud, yaw, obb, z0, z1, corners);
  ou::writeOBBToObject(o, obb, z0, z1, corners);
  return o;
}

TrackedObject LShape(const Cloud& cloud) {
  TrackedObject o;
  o.pcl_ClusterPoints = cloud;
  ou::OBB2D obb;
  double z0 = 0, z1 = 0;
  std::array<ou::P3, 8> corners{};
  ou::computeOBB3D_LShape(cloud, obb, z0, z1, corners);
  ou::writeOBBToObject(o, obb, z0, z1, corners);
  return o;
}

bool AllFinite(const TrackedObject& o) {
  bool ok = std::isfinite(o.f32_OBB_CenterX) && std::isfinite(o.f32_OBB_CenterY) && std::isfinite(o.f32_OBB_CenterZ) &&
            std::isfinite(o.f32_OBB_Width) && std::isfinite(o.f32_OBB_Height) && std::isfinite(o.f32_OBB_ZMin) &&
            std::isfinite(o.f32_OBB_ZMax) && std::isfinite(o.f32_OBB_YawRad) && std::isfinite(o.f32_CoreX) &&
            std::isfinite(o.f32_CoreY) && std::isfinite(o.f32_CoreZ) && std::isfinite(o.f32_Dist);
  for (const auto& c : o.OBB_Corners_Body) ok = ok && std::isfinite(c.x) && std::isfinite(c.y) && std::isfinite(c.z);
  return ok;
}

TEST(ObbUtil, WrapToPiIsRemainder2Pi) {
  EXPECT_DOUBLE_EQ(ou::wrapToPi(1.5 * M_PI), -0.5 * M_PI);
  EXPECT_DOUBLE_EQ(ou::wrapToPi(-1.5 * M_PI), 0.5 * M_PI);
  EXPECT_DOUBLE_EQ(ou::wrapToPi(0.3), 0.3);
  // std::remainder rounds the quotient half to even: +-pi stay +-pi.
  EXPECT_DOUBLE_EQ(ou::wrapToPi(M_PI), M_PI);
  EXPECT_DOUBLE_EQ(ou::wrapToPi(-M_PI), -M_PI);
}

TEST(ObbUtil, NormalizeAxisYawIsHalfOpenMinusHalfPiToHalfPi) {
  EXPECT_DOUBLE_EQ(ou::normalizeAxisYaw(ou::kHalfPi), -ou::kHalfPi);  // +pi/2 -> -pi/2
  EXPECT_DOUBLE_EQ(ou::normalizeAxisYaw(-ou::kHalfPi), -ou::kHalfPi);
  EXPECT_DOUBLE_EQ(ou::normalizeAxisYaw(0.4), 0.4);
  EXPECT_NEAR(ou::normalizeAxisYaw(1.7), 1.7 - M_PI, 1e-15);
  EXPECT_NEAR(ou::normalizeAxisYaw(-3.0), -3.0 + M_PI, 1e-15);
  EXPECT_NEAR(ou::normalizeAxisYaw(7.0), 7.0 - 2 * M_PI, 1e-15);
}

TEST(ObbUtil, ComputeObbAtYawRecoversRotatedRectangleInCornerOrder) {
  const Rect r{12.0, -3.0, 0.3, 4.5, 1.8, -0.7, 0.8};
  const auto o = FromCloud(RectCloud(r, 6), r.heading);
  EXPECT_NEAR(o.f32_OBB_CenterX, r.cx, kPosTol);
  EXPECT_NEAR(o.f32_OBB_CenterY, r.cy, kPosTol);
  EXPECT_NEAR(o.f32_OBB_CenterZ, .5 * (r.zmin + r.zmax), kPosTol);
  EXPECT_NEAR(o.f32_OBB_Width, r.length, kPosTol);
  EXPECT_NEAR(o.f32_OBB_Height, r.width, kPosTol);
  EXPECT_NEAR(o.f32_OBB_ZMin, r.zmin, kPosTol);
  EXPECT_NEAR(o.f32_OBB_ZMax, r.zmax, kPosTol);
  EXPECT_NEAR(o.f32_OBB_YawRad, r.heading, kAngTol);
  const auto expected = r.Corners(r.heading);
  for (int i = 0; i < 4; ++i) {
    EXPECT_NEAR(o.OBB_Corners_Body[i].x, expected[i][0], kPosTol) << i;
    EXPECT_NEAR(o.OBB_Corners_Body[i].y, expected[i][1], kPosTol) << i;
  }
  ExpectObbRule(o);
}

// Preserved quirk: corners follow the un-normalized computation yaw, the stored
// yaw is normalized. For yaw >= pi/2 corner 0 is (+u,+v) of the stored axis.
TEST(ObbUtil, ComputeObbAtYawCornersUseUnnormalizedYaw) {
  const Rect r{5.0, 2.0, 2.0, 3.0, 1.2, 0.0, 1.0};
  const auto o = FromCloud(RectCloud(r, 5), 2.0);
  EXPECT_NEAR(o.f32_OBB_YawRad, 2.0 - M_PI, kAngTol);
  const auto expected = r.Corners(2.0);  // un-normalized axes
  for (int i = 0; i < 4; ++i) {
    EXPECT_NEAR(o.OBB_Corners_Body[i].x, expected[i][0], kPosTol) << i;
    EXPECT_NEAR(o.OBB_Corners_Body[i].y, expected[i][1], kPosTol) << i;
  }
  const auto normalized = r.Corners(2.0 - M_PI);
  EXPECT_NEAR(o.OBB_Corners_Body[0].x, normalized[2][0], kPosTol) << "index rotated by 2";
  EXPECT_NEAR(o.OBB_Corners_Body[0].y, normalized[2][1], kPosTol);
  // The only invariant: edge 0->1 parallel to +-yaw (and CCW order).
  const auto& c0 = o.OBB_Corners_Body[0];
  const auto& c1 = o.OBB_Corners_Body[1];
  EXPECT_LT(AxisError(o.f32_OBB_YawRad, std::atan2(c1.y - c0.y, c1.x - c0.x)), kAngTol);
  ExpectCcwFromPlusZ(o);
}

TEST(ObbUtil, ComputeObbAtYawSkipsNonFinitePoints) {
  const Rect r{8.0, 1.0, -0.6, 2.0, 1.0, -0.4, 0.6};
  const Cloud clean = RectCloud(r, 4);
  Cloud dirty = clean;
  dirty.push_back(Pt(kNaN, 0, 0));
  dirty.push_back(Pt(100, std::numeric_limits<double>::infinity(), 0));
  dirty.push_back(Pt(-50, 3, kNaN));
  const auto a = FromCloud(clean, r.heading);
  const auto b = FromCloud(dirty, r.heading);
  EXPECT_EQ(a.f32_OBB_CenterX, b.f32_OBB_CenterX);
  EXPECT_EQ(a.f32_OBB_CenterY, b.f32_OBB_CenterY);
  EXPECT_EQ(a.f32_OBB_Width, b.f32_OBB_Width);
  EXPECT_EQ(a.f32_OBB_Height, b.f32_OBB_Height);
  EXPECT_EQ(a.f32_OBB_ZMin, b.f32_OBB_ZMin);
  EXPECT_EQ(a.f32_OBB_ZMax, b.f32_OBB_ZMax);
  for (int i = 0; i < 8; ++i) {
    EXPECT_EQ(a.OBB_Corners_Body[i].x, b.OBB_Corners_Body[i].x);
    EXPECT_EQ(a.OBB_Corners_Body[i].y, b.OBB_Corners_Body[i].y);
  }
  EXPECT_TRUE(AllFinite(b));
}

TEST(ObbUtil, ComputeObbAtYawWithoutFinitePointsReturnsFalseAndZeroBox) {
  for (bool empty : {true, false}) {
    Cloud cloud;
    if (!empty) {
      cloud.push_back(Pt(kNaN, 1, 1));
      cloud.push_back(Pt(1, kNaN, 1));
    }
    ou::OBB2D obb;
    obb.cx = 5;
    obb.w = 3;
    double z0 = 7, z1 = 8;
    std::array<ou::P3, 8> corners;
    corners.fill({9, 9, 9});
    EXPECT_FALSE(ou::computeOBBAtYaw(cloud, 0.3, obb, z0, z1, corners));
    EXPECT_EQ(obb.cx, 0.);
    EXPECT_EQ(obb.cy, 0.);
    EXPECT_EQ(obb.w, 0.);
    EXPECT_EQ(obb.h, 0.);
    EXPECT_EQ(obb.yaw, 0.);
    EXPECT_EQ(z0, 0.);
    EXPECT_EQ(z1, 0.);
    for (const auto& c : corners) {
      EXPECT_EQ(c.x, 0.);
      EXPECT_EQ(c.y, 0.);
      EXPECT_EQ(c.z, 0.);
    }
  }
}

TEST(ObbUtil, ZeroSizeIsInflatedToMinimumSideKeepingCentre) {
  Cloud cloud;
  cloud.push_back(Pt(20.0, -0.5, -1.4));
  const auto o = FromCloud(cloud, 0.0);
  EXPECT_NEAR(o.f32_OBB_Width, ou::kMinimumObbSide, 1e-7);
  EXPECT_NEAR(o.f32_OBB_Height, ou::kMinimumObbSide, 1e-7);
  EXPECT_NEAR(o.f32_OBB_CenterX, 20.0, kPosTol);
  EXPECT_NEAR(o.f32_OBB_CenterY, -0.5, kPosTol);
  EXPECT_NEAR(o.OBB_Corners_Body[0].x, 20.0 - 0.025, kPosTol);
  EXPECT_NEAR(o.OBB_Corners_Body[0].y, -0.5 - 0.025, kPosTol);
  EXPECT_NEAR(o.OBB_Corners_Body[2].x, 20.0 + 0.025, kPosTol);
  EXPECT_NEAR(o.OBB_Corners_Body[2].y, -0.5 + 0.025, kPosTol);
  EXPECT_EQ(o.f32_OBB_ZMin, o.f32_OBB_ZMax);
  EXPECT_TRUE(AllFinite(o));
}

TEST(ObbUtil, LShapeFindsIntegerDegreeHeadingWithLongSideOnYaw) {
  // Headings on the 1 deg grid: the closeness criterion is maximal there.
  for (double deg : {0., 17., 30., 63., 89., 101., 120., 150., -40.}) {
    SCOPED_TRACE(deg);
    const Rect r{10.0, 3.0, deg * M_PI / 180., 4.5, 1.8, -0.5, 1.0};
    const auto o = LShape(RectCloud(r, 12));
    double expected_yaw = r.heading;
    while (expected_yaw >= M_PI / 2) expected_yaw -= M_PI;
    while (expected_yaw < -M_PI / 2) expected_yaw += M_PI;
    EXPECT_NEAR(o.f32_OBB_YawRad, expected_yaw, kAngTol);
    EXPECT_NEAR(o.f32_OBB_Width, r.length, kPosTol);
    EXPECT_NEAR(o.f32_OBB_Height, r.width, kPosTol);
    EXPECT_NEAR(o.f32_OBB_CenterX, r.cx, kPosTol);
    EXPECT_NEAR(o.f32_OBB_CenterY, r.cy, kPosTol);
    EXPECT_GE(o.f32_OBB_Width, o.f32_OBB_Height);
    const auto& c0 = o.OBB_Corners_Body[0];
    const auto& c1 = o.OBB_Corners_Body[1];
    EXPECT_LT(AxisError(o.f32_OBB_YawRad, std::atan2(c1.y - c0.y, c1.x - c0.x)), kAngTol);
    ExpectCcwFromPlusZ(o);
  }
}

// Preserved quirk: with few samples on a short box every point stays within the
// 0.05 m closeness floor one step away from the true heading, the scores tie
// and the strict '>' keeps the smallest theta (29 deg for a 30 deg box here).
TEST(ObbUtil, LShapeScoreTiesKeepSmallestTheta) {
  const Rect r{10, 2, 0.5235987755982988, 3.0, 1.0, -0.5, 0.5};
  const Cloud cloud = RectCloud(r, 5);
  const auto o = LShape(cloud);
  double theta = 0.0;
  for (int k = 0; k < 29; ++k) theta += M_PI / 180.0;  // Main's accumulated grid value
  EXPECT_NEAR(o.f32_OBB_YawRad, theta, kAngTol);
  double umin = 1e9, umax = -1e9;
  for (const auto& p : cloud) {
    const double u = p.x * std::cos(theta) + p.y * std::sin(theta);
    umin = std::min(umin, u);
    umax = std::max(umax, u);
  }
  EXPECT_NEAR(o.f32_OBB_Width, umax - umin, kPosTol);
  EXPECT_GT(o.f32_OBB_Width, r.length);
}

TEST(ObbUtil, LShapeIgnoresNonFinitePoints) {
  const Rect r{6.0, -2.0, 0.5235987755982988, 3.0, 1.2, -0.2, 0.9};
  const Cloud clean = RectCloud(r, 8);
  Cloud dirty = clean;
  dirty.push_back(Pt(kNaN, kNaN, kNaN));
  dirty.push_back(Pt(std::numeric_limits<double>::infinity(), 0, 0));
  const auto a = LShape(clean);
  const auto b = LShape(dirty);
  EXPECT_EQ(a.f32_OBB_YawRad, b.f32_OBB_YawRad);
  EXPECT_EQ(a.f32_OBB_Width, b.f32_OBB_Width);
  EXPECT_EQ(a.f32_OBB_Height, b.f32_OBB_Height);
  EXPECT_EQ(a.f32_OBB_CenterX, b.f32_OBB_CenterX);
  EXPECT_EQ(a.f32_OBB_CenterY, b.f32_OBB_CenterY);
  EXPECT_TRUE(AllFinite(b));
}

TEST(ObbUtil, LShapeBelowTenPointsIsAabbWithLongAxisSwap) {
  // Along x: yaw 0.
  {
    Cloud cloud;
    for (int i = 0; i < 6; ++i) cloud.push_back(Pt(10.0 + 0.2 * i, 1.0, -1.0));
    const auto o = LShape(cloud);
    EXPECT_EQ(o.f32_OBB_YawRad, 0.f);
    EXPECT_NEAR(o.f32_OBB_Width, 1.0, kPosTol);
    EXPECT_NEAR(o.f32_OBB_Height, ou::kMinimumObbSide, 1e-7);
  }
  // Along y: w < h at yaw 0, recomputed at pi/2 and stored as -pi/2 (preserved).
  {
    Cloud cloud;
    for (int i = 0; i < 6; ++i) cloud.push_back(Pt(20.0, -0.5 + 0.2 * i, -1.42));
    const auto o = LShape(cloud);
    EXPECT_NEAR(o.f32_OBB_YawRad, -M_PI / 2, kAngTol);
    EXPECT_NEAR(o.f32_OBB_Width, 1.0, kPosTol);
    EXPECT_NEAR(o.f32_OBB_Height, ou::kMinimumObbSide, 1e-7);
    EXPECT_NEAR(o.f32_OBB_CenterX, 20.0, kPosTol);
    EXPECT_NEAR(o.f32_OBB_CenterY, 0.0, kPosTol);
    // Corners from the un-normalized pi/2 axes: corner 0 = centre - halfW*(0,1) - halfH*(-1,0).
    EXPECT_NEAR(o.OBB_Corners_Body[0].x, 20.0 + 0.025, kPosTol);
    EXPECT_NEAR(o.OBB_Corners_Body[0].y, 0.0 - 0.5, kPosTol);
    const auto& c0 = o.OBB_Corners_Body[0];
    const auto& c1 = o.OBB_Corners_Body[1];
    EXPECT_LT(AxisError(o.f32_OBB_YawRad, std::atan2(c1.y - c0.y, c1.x - c0.x)), kAngTol);
    ExpectCcwFromPlusZ(o);
  }
  // Nine points of a rotated rectangle: no L-shape search, plain AABB at yaw 0.
  {
    const Rect r{4.0, 4.0, 0.4, 2.0, 1.0, 0, 0.5};
    Cloud cloud = RectCloud(r, 1);  // 8 points (4 corners x 2 levels)
    cloud.push_back(Pt(4.0, 4.0, 0.25));
    ASSERT_EQ(cloud.size(), 9u);
    const auto o = LShape(cloud);
    double xmin = 1e9, xmax = -1e9, ymin = 1e9, ymax = -1e9;
    for (const auto& p : cloud) {
      xmin = std::min<double>(xmin, p.x);
      xmax = std::max<double>(xmax, p.x);
      ymin = std::min<double>(ymin, p.y);
      ymax = std::max<double>(ymax, p.y);
    }
    const bool swap = (xmax - xmin) < (ymax - ymin);
    EXPECT_NEAR(o.f32_OBB_YawRad, swap ? -M_PI / 2 : 0.0, kAngTol);
    EXPECT_NEAR(o.f32_OBB_Width, std::max(xmax - xmin, ymax - ymin), kPosTol);
    EXPECT_NEAR(o.f32_OBB_Height, std::min(xmax - xmin, ymax - ymin), kPosTol);
  }
}

TEST(ObbUtil, LShapeEmptyCloudGivesZeroBox) {
  const auto o = LShape(Cloud{});
  EXPECT_EQ(o.f32_OBB_Width, 0.f);
  EXPECT_EQ(o.f32_OBB_Height, 0.f);
  EXPECT_EQ(o.f32_OBB_YawRad, 0.f);
  EXPECT_TRUE(AllFinite(o));
}

TEST(ObbUtil, WriteObbToObjectCastsDoubleToFloat) {
  ou::OBB2D obb;
  obb.cx = 1.0000000001;
  obb.cy = -2.5;
  obb.w = 3.25;
  obb.h = 1.5;
  obb.yaw = 0.7;
  std::array<ou::P3, 8> corners{};
  for (int i = 0; i < 8; ++i) corners[i] = {0.1 * i, -0.2 * i, 0.3 * i};
  TrackedObject o;
  ou::writeOBBToObject(o, obb, -0.3, 1.1, corners);
  EXPECT_EQ(o.f32_OBB_CenterX, static_cast<float>(1.0000000001));
  EXPECT_EQ(o.f32_OBB_CenterZ, static_cast<float>(0.5 * (-0.3 + 1.1)));
  EXPECT_EQ(o.f32_OBB_Width, 3.25f);
  EXPECT_EQ(o.f32_OBB_Height, 1.5f);
  EXPECT_EQ(o.f32_OBB_ZMin, static_cast<float>(-0.3));
  EXPECT_EQ(o.f32_OBB_ZMax, static_cast<float>(1.1));
  EXPECT_EQ(o.f32_OBB_YawRad, static_cast<float>(0.7));
  for (int i = 0; i < 8; ++i) {
    EXPECT_EQ(o.OBB_Corners_Body[i].x, static_cast<float>(0.1 * i));
    EXPECT_EQ(o.OBB_Corners_Body[i].z, static_cast<float>(0.3 * i));
  }
}

// ---- stabilizeMatchedOBB ----
TrackedObject Matched(const Rect& r, double raw_yaw) {
  auto o = FromCloud(RectCloud(r, 6), raw_yaw);
  return o;
}

TEST(StabilizeMatchedObb, SkippedForEmptyClusterOrPreviouslyMissedTrack) {
  const Rect r{10, 0, 0.3, 4.0, 1.5, 0, 1};
  TrackedObject previous;
  previous.f32_OBB_YawRad = 0.2f;
  previous.f32_PrevYaw = 0.f;
  auto current = Matched(r, 0.3);
  const auto before = current;
  previous.s32_MissCount = 1;
  ou::stabilizeMatchedOBB(previous, current, 0.0);
  EXPECT_EQ(current.f32_OBB_YawRad, before.f32_OBB_YawRad);
  EXPECT_EQ(current.OBB_Corners_Body[0].x, before.OBB_Corners_Body[0].x);
  previous.s32_MissCount = 0;
  current.pcl_ClusterPoints.clear();
  ou::stabilizeMatchedOBB(previous, current, 0.0);
  EXPECT_EQ(current.f32_OBB_YawRad, before.f32_OBB_YawRad);
}

TEST(StabilizeMatchedObb, ReliableAspectBlendsQuarterOfAxisErrorAndRefits) {
  const Rect r{10, 0, 0.3, 4.0, 1.5, 0, 1};  // aspect 2.67
  TrackedObject previous;
  previous.f32_OBB_YawRad = 0.2f;
  previous.f32_PrevYaw = 0.f;
  auto current = Matched(r, 0.3);
  ou::stabilizeMatchedOBB(previous, current, 0.0);
  const double raw = static_cast<float>(0.3);
  const double expected = 0.2f + 0.25 * (raw - 0.2f);
  EXPECT_NEAR(current.f32_OBB_YawRad, expected, kAngTol);
  const auto refit = FromCloud(current.pcl_ClusterPoints, expected);
  for (int i = 0; i < 8; ++i) {
    EXPECT_NEAR(current.OBB_Corners_Body[i].x, refit.OBB_Corners_Body[i].x, kPosTol);
    EXPECT_NEAR(current.OBB_Corners_Body[i].y, refit.OBB_Corners_Body[i].y, kPosTol);
  }
  EXPECT_NEAR(current.f32_OBB_Width, refit.f32_OBB_Width, kPosTol);
  ExpectObbRule(current);
}

TEST(StabilizeMatchedObb, NearSquareKeepsThePredictionOnly) {
  const Rect r{10, 0, 0.3, 1.0, 0.9, 0, 1};  // aspect 1.11 < 1.2
  TrackedObject previous;
  previous.f32_OBB_YawRad = 0.2f;
  previous.f32_PrevYaw = 0.f;
  auto current = Matched(r, 0.3);
  ou::stabilizeMatchedOBB(previous, current, 0.0);
  EXPECT_NEAR(current.f32_OBB_YawRad, 0.2f, kAngTol);
  ExpectObbRule(current);
}

TEST(StabilizeMatchedObb, EgoYawDeltaUsesDegreesNowAndRadiansBefore) {
  const Rect r{10, 0, 0.0, 4.0, 1.5, 0, 1};
  TrackedObject previous;
  previous.f32_OBB_YawRad = 0.2f;
  previous.f32_PrevYaw = 0.1f;  // rad
  auto current = Matched(r, 0.0);
  const double ego_deg = 0.35 * 180. / M_PI;
  ou::stabilizeMatchedOBB(previous, current, ego_deg);
  const double predicted = 0.2f - (ego_deg * M_PI / 180. - 0.1f);  // -0.05
  const double expected = predicted + 0.25 * (0.0 - predicted);
  EXPECT_NEAR(current.f32_OBB_YawRad, expected, kAngTol);
}

TEST(StabilizeMatchedObb, AxisErrorIsPiPeriodicAndResultNormalized) {
  const Rect r{10, 0, 1.5, 4.0, 1.5, 0, 1};
  TrackedObject previous;
  previous.f32_PrevYaw = 0.f;
  // pred 1.56, raw -1.50 (same axis as 1.6416): error +0.0816 -> 1.5804 -> -1.5612.
  previous.f32_OBB_YawRad = 1.56f;
  auto current = Matched(r, -1.50);
  ou::stabilizeMatchedOBB(previous, current, 0.0);
  const double pred = 1.56f;
  const double raw = static_cast<float>(-1.50);
  double expected = pred + 0.25 * std::remainder(raw - pred, M_PI);
  ASSERT_GE(expected, M_PI / 2);
  expected -= M_PI;
  EXPECT_NEAR(current.f32_OBB_YawRad, expected, kAngTol);
  EXPECT_LT(current.f32_OBB_YawRad, 0.f);
  ExpectObbRule(current);
}

TEST(StabilizeMatchedObb, ZeroHeightCountsAsReliableAspect) {
  const Rect r{10, 0, 0.3, 4.0, 1.5, 0, 1};
  TrackedObject previous;
  previous.f32_OBB_YawRad = 0.2f;
  auto current = Matched(r, 0.3);
  current.f32_OBB_Height = 0.f;  // aspect = +inf (Main ternary)
  ou::stabilizeMatchedOBB(previous, current, 0.0);
  EXPECT_NEAR(current.f32_OBB_YawRad, 0.2f + 0.25 * (static_cast<float>(0.3) - 0.2f), kAngTol);
}

// ---- detection builder (Objectinfo_for_km) ----
TEST(BuildDetections, SparseLowObstacleKeepsNonDegenerateUnknownObb) {
  // Port of Main test_tracking_association.cpp:173-205.
  CloudPtr non_ground(new Cloud);
  CloudPtr centroids(new Cloud);
  pcl::PointIndices indices;
  for (int index = 0; index < 6; ++index) {
    pcl::PointXYZI point;
    point.x = 20.0F;
    point.y = -0.5F + 0.2F * static_cast<float>(index);
    point.z = -1.42F;
    indices.indices.push_back(index);
    non_ground->push_back(point);
  }
  pcl::PointXYZI centroid;
  centroid.x = 20.0F;
  centroid.y = 0.0F;
  centroid.z = -1.42F;
  centroids->push_back(centroid);
  std::vector<TrackedObject> detections;
  BuildDetections(detections, non_ground, centroids, {indices});
  ASSERT_EQ(detections.size(), 1U);
  EXPECT_EQ(detections.front().str_ObjectClass, "unknown");
  EXPECT_GE(detections.front().f32_OBB_Width, 0.05F);
  EXPECT_GE(detections.front().f32_OBB_Height, 0.05F);
  EXPECT_TRUE(AllFinite(detections.front()));
}

TEST(BuildDetections, FieldsAabbAndCentroidOrder) {
  CloudPtr points(new Cloud);
  CloudPtr centroids(new Cloud);
  const Rect a{10, 2, 0.5235987755982988, 4.5, 1.8, -0.5, 0.5}, b{-4, -6, -0.2, 1.5, 1.0, 0, 1.2};
  std::vector<pcl::PointIndices> clusters(2);
  for (int k = 0; k < 2; ++k) {
    const Cloud c = RectCloud(k == 0 ? a : b, k == 0 ? 12 : 5);
    for (const auto& p : c) {
      clusters[k].indices.push_back(static_cast<int>(points->size()));
      points->push_back(p);
    }
  }
  centroids->push_back(Pt(10.1, 2.1, 0.05));
  centroids->push_back(Pt(-4.2, -5.9, 0.6));
  std::vector<TrackedObject> detections(3);  // cleared by BuildDetections
  BuildDetections(detections, points, centroids, clusters);
  ASSERT_EQ(detections.size(), 2u);
  for (int k = 0; k < 2; ++k) {
    const auto& d = detections[k];
    EXPECT_EQ(d.u16_Idx, k);
    EXPECT_EQ(d.s32_Id, -1);
    EXPECT_EQ(d.s32_Age, 0);
    EXPECT_EQ(d.s32_MissCount, 0);
    EXPECT_EQ(d.s32_KfInitCount, 0);
    EXPECT_FALSE(d.b_KfInitialized);
    EXPECT_TRUE(d.Eigen_KfState_CA.isZero());
    EXPECT_TRUE(d.Eigen_KfP_CA.isIdentity());
    EXPECT_EQ(d.f32_CoreX, centroids->points[k].x);
    EXPECT_EQ(d.f32_CoreY, centroids->points[k].y);
    EXPECT_EQ(d.f32_CoreZ, centroids->points[k].z);
    EXPECT_EQ(d.f32_Dist, std::hypot(d.f32_CoreX, d.f32_CoreY));
    EXPECT_EQ(d.pcl_ClusterPoints.size(), clusters[k].indices.size());
    float xmin = 1e9f, xmax = -1e9f, zmin = 1e9f, zmax = -1e9f;
    for (const auto& p : d.pcl_ClusterPoints) {
      xmin = std::min(xmin, p.x);
      xmax = std::max(xmax, p.x);
      zmin = std::min(zmin, p.z);
      zmax = std::max(zmax, p.z);
    }
    EXPECT_EQ(d.f32_MinX, xmin);
    EXPECT_EQ(d.f32_MaxX, xmax);
    EXPECT_EQ(d.f32_MinZ, zmin);
    EXPECT_EQ(d.f32_MaxZ, zmax);
    EXPECT_GE(d.f32_OBB_Width, d.f32_OBB_Height);
    ExpectObbRule(d);
  }
  EXPECT_NEAR(detections[0].f32_OBB_YawRad, a.heading, kAngTol);
  EXPECT_NEAR(detections[0].f32_OBB_Width, a.length, kPosTol);
}

TEST(BuildDetections, InvalidGeometryNeverProducesNaN) {
  CloudPtr points(new Cloud);
  CloudPtr centroids(new Cloud);
  std::vector<pcl::PointIndices> clusters(4);
  // 0: >= 10 finite points plus NaN ones (skipped by the OBB; a NaN coordinate
  //    never wins a min/max comparison in the AABB, its finite coordinates
  //    still count, so they are chosen inside the finite range here).
  const Rect r{9, 1, 0.3490658503988659, 2.5, 1.0, -0.6, 0.4};
  for (const auto& p : RectCloud(r, 4)) {
    clusters[0].indices.push_back(static_cast<int>(points->size()));
    points->push_back(p);
  }
  for (const auto& p : {Pt(kNaN, kNaN, kNaN), Pt(kNaN, 1.0, 0.0), Pt(9.0, 1.0, kNaN)}) {
    clusters[0].indices.push_back(static_cast<int>(points->size()));
    points->push_back(p);
  }
  // 1: three points (< 10): AABB fallback.
  for (const auto& p : {Pt(30, 5, -1), Pt(30.4, 5.1, -0.9), Pt(30.2, 5.3, -1.1)}) {
    clusters[1].indices.push_back(static_cast<int>(points->size()));
    points->push_back(p);
  }
  // 2: empty cluster (Main: OBB all zero, AABB stays at +-FLT_MAX).
  // 3: a single point: zero size inflated to 0.05.
  clusters[3].indices.push_back(static_cast<int>(points->size()));
  points->push_back(Pt(-7, 2, 0.3));
  for (const auto& c : {Pt(9, 1, -0.1), Pt(30.2, 5.13, -1), Pt(15, 15, 0), Pt(-7, 2, 0.3)}) centroids->push_back(c);

  std::vector<TrackedObject> detections;
  BuildDetections(detections, points, centroids, clusters);
  ASSERT_EQ(detections.size(), 4u);
  for (const auto& d : detections) EXPECT_TRUE(AllFinite(d)) << "cluster " << d.u16_Idx;

  Cloud finite_only;
  for (const auto& p : RectCloud(r, 4)) finite_only.push_back(p);
  const auto reference = LShape(finite_only);
  EXPECT_EQ(detections[0].f32_OBB_YawRad, reference.f32_OBB_YawRad);
  EXPECT_EQ(detections[0].f32_OBB_Width, reference.f32_OBB_Width);
  EXPECT_EQ(detections[0].f32_OBB_CenterX, reference.f32_OBB_CenterX);
  float xmin = 1e9f, ymax = -1e9f, zmin = 1e9f;
  for (const auto& p : finite_only) {
    xmin = std::min(xmin, p.x);
    ymax = std::max(ymax, p.y);
    zmin = std::min(zmin, p.z);
  }
  EXPECT_EQ(detections[0].f32_MinX, xmin);
  EXPECT_EQ(detections[0].f32_MaxY, ymax);
  EXPECT_EQ(detections[0].f32_MinZ, zmin);

  EXPECT_GE(detections[1].f32_OBB_Width, detections[1].f32_OBB_Height);
  EXPECT_TRUE(detections[1].f32_OBB_YawRad == 0.f || std::abs(detections[1].f32_OBB_YawRad + M_PI / 2) < kAngTol);

  EXPECT_EQ(detections[2].f32_OBB_Width, 0.f);
  EXPECT_EQ(detections[2].f32_OBB_Height, 0.f);
  EXPECT_EQ(detections[2].f32_MinX, std::numeric_limits<float>::max());
  EXPECT_EQ(detections[2].f32_MaxX, -std::numeric_limits<float>::max());

  EXPECT_NEAR(detections[3].f32_OBB_Width, ou::kMinimumObbSide, 1e-7);
  EXPECT_NEAR(detections[3].f32_OBB_Height, ou::kMinimumObbSide, 1e-7);
  EXPECT_NEAR(detections[3].f32_OBB_CenterX, -7.0, kPosTol);
}

// Preserved quirk: the detection AABB has no finiteness guard, an infinite
// coordinate is kept (the OBB skips the point).
TEST(BuildDetections, AabbHasNoFinitenessGuard) {
  CloudPtr points(new Cloud);
  CloudPtr centroids(new Cloud);
  pcl::PointIndices cluster;
  for (int i = 0; i < 4; ++i) {
    cluster.indices.push_back(i);
    points->push_back(Pt(5 + 0.1 * i, 1, 0));
  }
  cluster.indices.push_back(4);
  points->push_back(Pt(5, std::numeric_limits<double>::infinity(), 0));
  centroids->push_back(Pt(5.15, 1, 0));
  std::vector<TrackedObject> detections;
  BuildDetections(detections, points, centroids, {cluster});
  ASSERT_EQ(detections.size(), 1u);
  EXPECT_TRUE(std::isinf(detections[0].f32_MaxY));
  EXPECT_TRUE(AllFinite(detections[0]));
  EXPECT_NEAR(detections[0].f32_OBB_Width, 0.3, kPosTol);
}

TEST(BuildDetections, StopsWithoutCentroids) {
  CloudPtr points(new Cloud);
  CloudPtr centroids(new Cloud);
  pcl::PointIndices cluster;
  cluster.indices = {0};
  points->push_back(Pt(1, 1, 1));
  std::vector<TrackedObject> detections;
  BuildDetections(detections, points, centroids, {cluster});
  EXPECT_TRUE(detections.empty());
  // Deviation (defined instead of out-of-range): fewer centroids than clusters stops.
  centroids->push_back(Pt(1, 1, 1));
  BuildDetections(detections, points, centroids, {cluster, cluster});
  EXPECT_EQ(detections.size(), 1u);
}

}  // namespace

int main(int argc, char** argv) { testing::InitGoogleTest(&argc, argv); return RUN_ALL_TESTS(); }
