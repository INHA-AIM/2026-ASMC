// RANSAC ground removal and ground evidence (Main Ransac.cpp / GroundEvidence.hpp at 8f93c22e).
#include <asmc_lidar/preprocessing/ground.hpp>
#include <asmc_lidar/preprocessing/preprocessing.hpp>
#include <asmc_lidar/geometry/projection.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

namespace {

using asmc::lidar::pipeline::Cloud;
using asmc::lidar::pipeline::CloudPtr;
using asmc::lidar::pipeline::GroundConfig;
using asmc::lidar::pipeline::GroundFilter;
using asmc::lidar::pipeline::GroundPlaneSnapshot;
using asmc::lidar::pipeline::LevelPoints;
using asmc::lidar::pipeline::MeasureObjectGroundEvidence;
using asmc::lidar::Rot;
using asmc::lidar::Vec;

constexpr double kPi = 3.14159265358979323846;

pcl::PointXYZI P(float x, float y, float z, float intensity = 0.f) {
  pcl::PointXYZI p;
  p.x = x;
  p.y = y;
  p.z = z;
  p.intensity = intensity;
  return p;
}

CloudPtr NewCloud() { return CloudPtr(new Cloud); }

// Bitwise x, y, z, intensity (padding bytes are not part of the point value).
bool SameBits(const pcl::PointXYZI& a, const pcl::PointXYZI& b) {
  return std::memcmp(&a.x, &b.x, 3 * sizeof(float)) == 0 &&
         std::memcmp(&a.intensity, &b.intensity, sizeof(float)) == 0;
}

// Flat ground z = -1.5 on x in [2, 70], y in [-3, 3] (0.5 m grid).
void AddFlatGround(Cloud& cloud, float z = -1.5f) {
  for (int i = 0; i <= 136; ++i)
    for (int j = 0; j <= 12; ++j) cloud.push_back(P(2.f + 0.5f * i, -3.f + 0.5f * j, z));
}

// ------------------------------------------------- Main test/test_ground_evidence.cpp
struct EvidencePoint {
  double x, y, z;
};
GroundPlaneSnapshot FlatPlane() {
  GroundPlaneSnapshot plane;
  plane.valid = true;
  plane.coefficients = {{0.0, 0.0, 1.0, 1.5}};
  return plane;
}

TEST(GroundEvidence, OneScanRingHasClearanceDespiteZeroThickness) {
  const std::vector<EvidencePoint> points{{50, 0, -.8}, {50, .2, -.8}, {50, .4, -.8}};
  const auto result = MeasureObjectGroundEvidence(points, FlatPlane());
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(.7, result.median_clearance_m, 1e-9);
  EXPECT_NEAR(.4, result.rejection_band_m, 1e-9);
}

TEST(GroundEvidence, MedianRejectsSingleElevatedOutlier) {
  const std::vector<EvidencePoint> points{{30, 0, -1.48}, {30, .2, -1.47}, {30, .4, 0}};
  const auto result = MeasureObjectGroundEvidence(points, FlatPlane());
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(.03, result.median_clearance_m, 1e-9);
}

TEST(GroundEvidence, SignedClearanceRejectsBelowPlaneReturns) {
  const std::vector<EvidencePoint> points{{50, 0, -2.1}, {50, .2, -2.1}, {50, .4, -2.1}};
  EXPECT_NEAR(-.6, MeasureObjectGroundEvidence(points, FlatPlane()).median_clearance_m, 1e-9);
}

TEST(GroundEvidence, SlopedAndTranslatedPlaneUsesLocalRoad) {
  auto plane = FlatPlane();
  plane.coefficients = {{-.1, 0, 1, 1.5}};
  const std::vector<EvidencePoint> points{{30, 0, 2.2}, {31, 0, 2.3}, {32, 0, 2.4}};
  const auto result = MeasureObjectGroundEvidence(points, plane);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(.7 / std::sqrt(1.01), result.median_clearance_m, 1e-9);
}

TEST(GroundEvidence, InvalidPlaneAndTooFewPointsProduceNoEvidence) {
  auto plane = FlatPlane();
  const std::vector<EvidencePoint> points{{50, 0, -.8}, {50, .2, -.8}};
  EXPECT_FALSE(MeasureObjectGroundEvidence(points, plane).valid);
  plane.valid = false;
  EXPECT_FALSE(MeasureObjectGroundEvidence(points, plane).valid);
  plane = FlatPlane();
  plane.coefficients[3] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(MeasureObjectGroundEvidence(points, plane).valid);
}

TEST(GroundEvidence, NonFiniteReturnsAreNotCounted) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const std::vector<EvidencePoint> points{{50, 0, -.8}, {50, .2, -.8}, {50, .4, nan}};
  const auto result = MeasureObjectGroundEvidence(points, FlatPlane());
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(2, result.sample_count);
}

TEST(GroundEvidence, AcceptsPclClusterPoints) {
  Cloud cluster;
  cluster.push_back(P(50, 0, -.8f));
  cluster.push_back(P(50, .2f, -.8f));
  cluster.push_back(P(50, .4f, -.8f));
  const auto result = MeasureObjectGroundEvidence(cluster, FlatPlane());
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(.7, result.median_clearance_m, 1e-6);
}

// ------------------------------------------------- Main test/test_frame_boundary_ros.cpp:557-603
TEST(GroundFilter, RealGroundFilterPreservesIndependentElevatedPointsAcrossAttitude) {
  GroundFilter filter;  // one instance across scans, like Main's singleton
  for (double roll : {0., 5., -5., 10., -10.})
    for (double pitch : {-3., 0., 3.}) {
      const float roll_rad = static_cast<float>(roll * kPi / 180.);
      const float pitch_rad = static_cast<float>(pitch * kPi / 180.);
      const Rot level =
          asmc::lidar::leveling_rotation(roll_rad, pitch_rad, Rot::Identity(), true, true);
      auto raw = NewCloud(), corrected = NewCloud(), remaining = NewCloud();
      // Truth is generated from a known sloped ground before running RANSAC.
      for (unsigned i = 0; i < 256; ++i) {
        const double x = 5. + .3 * (i % 16), y = -2. + .3 * (i / 16);
        const Vec q = level.transpose() * Vec(x, y, -1.45 + .02 * x);
        raw->push_back(P(q.x(), q.y(), q.z(), 0));
      }
      for (unsigned i = 0; i < 24; ++i) {
        const Vec q = level.transpose() * Vec(7. + .1 * (i % 4), 1. + .1 * (i / 4), -.7);
        raw->push_back(P(q.x(), q.y(), q.z(), 1));
      }
      const Rot applied = LevelPoints(raw, corrected, roll_rad, pitch_rad, Rot::Identity(), true);
      EXPECT_LT((applied - level).cwiseAbs().maxCoeff(), 1e-15);
      std::vector<int> kept;
      filter.Filter(corrected, remaining, &kept);
      ASSERT_TRUE(filter.CurrentPlane().valid) << roll << "," << pitch;
      EXPECT_EQ(remaining->size(), 24U) << roll << "," << pitch;
      for (const auto& p : *remaining) EXPECT_EQ(p.intensity, 1);
      ASSERT_EQ(kept.size(), remaining->size());
      for (std::size_t i = 0; i < kept.size(); ++i) EXPECT_EQ(kept[i], static_cast<int>(256 + i));
    }
}

TEST(GroundFilter, GroundFitFailureAfterAttitudeChangePreservesUnprovenPoints) {
  GroundFilter filter;
  auto ground = NewCloud(), leveled = NewCloud(), output = NewCloud();
  for (int x = 0; x < 8; ++x)
    for (int y = 0; y < 8; ++y) ground->push_back(P(5.f + x * .3f, -1.f + y * .3f, -1.5f));
  LevelPoints(ground, leveled, 0.f, 0.f, Rot::Identity(), true);
  filter.Filter(leveled, output, nullptr);
  ASSERT_TRUE(filter.CurrentPlane().valid);
  EXPECT_TRUE(output->empty());

  auto ambiguous = NewCloud();
  for (int i = 0; i < 24; ++i) ambiguous->push_back(P(8.f + i * .02f, .2f, -1.5f));
  // Main's test feeds the unleveled cloud directly.
  filter.Filter(ambiguous, output, nullptr);
  EXPECT_FALSE(filter.CurrentPlane().valid);
  EXPECT_EQ(output->size(), ambiguous->size());
  // Same through the leveling stage after a 10 deg roll change.
  const float roll = static_cast<float>(10 * kPi / 180.);
  LevelPoints(ambiguous, leveled, roll, 0.f, Rot::Identity(), true);
  std::vector<int> kept;
  filter.Filter(leveled, output, &kept);
  EXPECT_FALSE(filter.CurrentPlane().valid);
  ASSERT_EQ(output->size(), ambiguous->size());
  ASSERT_EQ(kept.size(), ambiguous->size());
  for (int i = 0; i < 24; ++i) {
    EXPECT_EQ(kept[i], i);
    EXPECT_EQ(output->points[i].x, leveled->points[i].x);
  }
  EXPECT_EQ(output->width, 24u);
  EXPECT_EQ(output->height, 1u);
  EXPECT_FALSE(output->is_dense);
}

// ------------------------------------------------- new
TEST(GroundFilter, ConfigureOnceClamps) {
  GroundConfig c;
  c.near_threshold_m = 0.0;
  c.far_threshold_m = 0.01;
  c.far_range_m = 0.5;
  c.maximum_tilt_deg = 60.;
  c.seed_min_z_m = -0.65;
  c.seed_max_z_m = -2.0;
  c.maximum_iterations = 5;
  c.minimum_seed_points = 1;
  const GroundFilter filter(c);
  EXPECT_EQ(filter.config().near_threshold_m, 0.02);
  EXPECT_EQ(filter.config().far_threshold_m, 0.02);
  EXPECT_EQ(filter.config().far_range_m, 1.0);
  EXPECT_EQ(filter.config().maximum_tilt_deg, 60.);  // only the radian copy is clamped
  EXPECT_DOUBLE_EQ(filter.maximum_tilt_rad(), 45.0 * kPi / 180.0);
  EXPECT_EQ(filter.config().seed_min_z_m, -2.0);
  EXPECT_EQ(filter.config().seed_max_z_m, -0.65);
  EXPECT_EQ(filter.config().maximum_iterations, 20);
  EXPECT_EQ(filter.config().minimum_seed_points, 3);
  c.maximum_tilt_deg = 0.2;
  EXPECT_DOUBLE_EQ(GroundFilter(c).maximum_tilt_rad(), 1.0 * kPi / 180.0);
  const GroundFilter defaults;
  EXPECT_EQ(defaults.config().near_threshold_m, 0.30);
  EXPECT_EQ(defaults.config().far_threshold_m, 0.40);
  EXPECT_EQ(defaults.config().far_range_m, 50.0);
  EXPECT_DOUBLE_EQ(defaults.maximum_tilt_rad(), 25.0 * kPi / 180.0);
  EXPECT_EQ(defaults.config().maximum_iterations, 80);
  EXPECT_EQ(defaults.config().minimum_seed_points, 30);
}

// 11 x 11 patch around (10, 0, -1.3) tilted about y by tilt_deg.
CloudPtr TiltedPatch(double tilt_deg) {
  auto cloud = NewCloud();
  const double slope = std::tan(tilt_deg * kPi / 180.);
  for (int i = 0; i <= 10; ++i)
    for (int j = 0; j <= 10; ++j) {
      const double x = 9.75 + 0.05 * i, y = -0.25 + 0.05 * j;
      cloud->push_back(P(x, y, -1.3 + slope * (x - 10.)));
    }
  return cloud;
}

TEST(GroundFilter, TiltGateUsesClampedAngle) {
  auto out = NewCloud();
  GroundConfig c;
  c.maximum_tilt_deg = 35.;
  GroundFilter accept(c);
  accept.Filter(TiltedPatch(30.), out, nullptr);
  ASSERT_TRUE(accept.CurrentPlane().valid);
  // PCL refines in float over a 0.5 m patch 10 m away: ~6e-4 rad is its precision.
  EXPECT_NEAR(std::acos(accept.CurrentPlane().coefficients[2]), 30. * kPi / 180., 3e-3);
  EXPECT_TRUE(out->empty());
  c.maximum_tilt_deg = 0.5;  // clamped to 1 deg
  GroundFilter strict(c);
  strict.Filter(TiltedPatch(30.), out, nullptr);
  EXPECT_FALSE(strict.CurrentPlane().valid);
  EXPECT_EQ(out->size(), 121u);
  c.maximum_tilt_deg = 60.;  // clamped to 45 deg: a 50 deg plane is not ground
  GroundFilter clamped(c);
  clamped.Filter(TiltedPatch(50.), out, nullptr);
  EXPECT_FALSE(clamped.CurrentPlane().valid);
  EXPECT_EQ(out->size(), 121u);
}

TEST(GroundFilter, FewerThanThreePointsClearsOutput) {
  GroundFilter filter;
  auto ground = NewCloud(), out = NewCloud();
  AddFlatGround(*ground);
  filter.Filter(ground, out, nullptr);
  ASSERT_TRUE(filter.CurrentPlane().valid);
  auto two = NewCloud();
  two->push_back(P(10, 0, 0.5f));
  two->push_back(P(11, 0, 0.5f));
  out->push_back(P(1, 2, 3));
  std::vector<int> kept{7, 8, 9};
  filter.Filter(two, out, &kept);
  EXPECT_FALSE(filter.CurrentPlane().valid);
  EXPECT_TRUE(out->empty());  // the points are lost, not preserved
  EXPECT_EQ(out->width, 0u);
  EXPECT_EQ(out->height, 0u);
  EXPECT_TRUE(kept.empty());
  // A null input clears the output too.
  out->push_back(P(1, 2, 3));
  filter.Filter(CloudPtr(), out, nullptr);
  EXPECT_TRUE(out->empty());
  // Null output: nothing to write, no crash.
  filter.Filter(ground, CloudPtr(), nullptr);
  EXPECT_FALSE(filter.CurrentPlane().valid);
}

TEST(GroundFilter, PlaneIsNormalizedWithNonNegativeZ) {
  GroundFilter filter;
  auto ground = NewCloud(), out = NewCloud();
  AddFlatGround(*ground);
  filter.Filter(ground, out, nullptr);
  const auto& plane = filter.CurrentPlane();
  ASSERT_TRUE(plane.valid);
  const auto& p = plane.coefficients;
  EXPECT_NEAR(std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]), 1.0, 1e-12);
  EXPECT_GE(p[2], 0.0);
  EXPECT_NEAR(p[2], 1.0, 1e-9);
  EXPECT_NEAR(p[3], 1.5, 1e-6);
  EXPECT_EQ(plane.near_threshold_m, 0.30);
  EXPECT_EQ(plane.far_threshold_m, 0.40);
  EXPECT_EQ(plane.far_range_m, 50.0);
  EXPECT_TRUE(out->empty());
}

TEST(GroundFilter, AdaptiveBandAbsoluteDistance) {
  GroundFilter filter;
  auto cloud = NewCloud(), out = NewCloud();
  AddFlatGround(*cloud);

  // Near (hypot ~ 3 m): band ~0.318. Far (hypot >= 50 m): band 0.40.
  cloud->push_back(P(3.f, 0, -1.5f + 0.34f, 1));   // kept
  cloud->push_back(P(3.f, 0.5f, -1.5f + 0.30f, 2)); // ground
  cloud->push_back(P(60.f, 0, -1.5f + 0.38f, 3));   // ground at range (band 0.40)
  cloud->push_back(P(60.f, 0.5f, -1.5f + 0.42f, 4));  // kept
  cloud->push_back(P(20.f, 0, -1.5f - 0.6f, 5));      // below plane: kept (absolute distance)
  filter.Filter(cloud, out, nullptr);
  ASSERT_TRUE(filter.CurrentPlane().valid);
  ASSERT_EQ(out->size(), 3u);
  EXPECT_EQ(out->points[0].intensity, 1.f);
  EXPECT_EQ(out->points[1].intensity, 4.f);
  EXPECT_EQ(out->points[2].intensity, 5.f);
}

TEST(GroundFilter, SignedHalfspaceRemovesBelowPlaneWithoutChangingElevatedReturns) {
  for (float slope : {0.f, 0.025f, -0.025f}) {
    auto cloud = NewCloud(), legacy = NewCloud(), signed_out = NewCloud();
    AddFlatGround(*cloud);
    for (auto& p : cloud->points) p.z += slope * p.x;
    // Include low returns above the existing upper band and a far sparse return.
    cloud->push_back(P(3, 0, -1.5f + slope * 3 + .36f, 1));
    cloud->push_back(P(60, 0, -1.5f + slope * 60 + .48f, 2));
    cloud->push_back(P(12, 0, -1.5f + slope * 12 - .65f, 3));
    cloud->push_back(P(12, 0, -1.5f + slope * 12 + .1f, 4));
    GroundConfig c;
    GroundFilter before(c);
    c.remove_below_plane = true;
    GroundFilter after(c);
    std::vector<int> indices;
    before.Filter(cloud, legacy, nullptr);
    after.Filter(cloud, signed_out, &indices);
    ASSERT_TRUE(after.CurrentPlane().valid);
    EXPECT_EQ(before.CurrentPlane().coefficients, after.CurrentPlane().coefficients);
    ASSERT_EQ(legacy->size(), 3u);
    ASSERT_EQ(signed_out->size(), 2u);
    ASSERT_EQ(indices.size(), 2u);
    for (std::size_t i = 0; i < signed_out->size(); ++i) {
      EXPECT_TRUE(SameBits(legacy->points[i], signed_out->points[i]));
      EXPECT_TRUE(SameBits(cloud->points[indices[i]], signed_out->points[i]));
    }
  }
}

TEST(GroundFilter, SignedHalfspaceDoesNotInventPlaneAfterFitFailure) {
  GroundConfig c;
  c.remove_below_plane = true;
  GroundFilter filter(c);
  auto cloud = NewCloud(), out = NewCloud();
  AddFlatGround(*cloud);
  filter.Filter(cloud, out, nullptr);
  ASSERT_TRUE(filter.CurrentPlane().valid);
  cloud->clear();
  for (int i = 0; i < 15; ++i) cloud->push_back(P(float(i), 0, -2.5f));
  filter.Filter(cloud, out, nullptr);
  EXPECT_FALSE(filter.CurrentPlane().valid);
  ASSERT_EQ(out->size(), cloud->size());
  for (std::size_t i = 0; i < out->size(); ++i)
    EXPECT_TRUE(SameBits(out->points[i], cloud->points[i]));
}

TEST(GroundFilter, KeptIndicesMatchOutputExactly) {
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  auto cloud = NewCloud();
  AddFlatGround(*cloud);
  for (int i = 0; i < 40; ++i) {
    cloud->push_back(P(12.f + 0.1f * (i % 5), 1.f + 0.1f * (i / 5), -0.4f + 0.01f * i, float(i)));
    if (i % 7 == 0) cloud->push_back(P(nan, 1, 1));
    if (i % 11 == 0) cloud->push_back(P(1, inf, 1));
  }
  cloud->is_dense = false;
  for (bool fit : {true, false}) {
    GroundConfig c;
    if (!fit) c.seed_max_z_m = -1.9;  // no seeds: failure path keeps every finite point
    GroundFilter with(c), without(c);
    auto a = NewCloud(), b = NewCloud();
    std::vector<int> kept;
    with.Filter(cloud, a, &kept);
    without.Filter(cloud, b, nullptr);
    EXPECT_EQ(with.CurrentPlane().valid, fit);
    ASSERT_EQ(a->size(), b->size());
    ASSERT_EQ(kept.size(), a->size());
    EXPECT_EQ(a->width, b->width);
    EXPECT_EQ(a->height, b->height);
    EXPECT_EQ(a->is_dense, b->is_dense);
    for (std::size_t i = 0; i < kept.size(); ++i) {
      if (i) {
        EXPECT_LT(kept[i - 1], kept[i]);
      }
      const auto& src = cloud->points[kept[i]];
      EXPECT_TRUE(SameBits(src, a->points[i])) << i;
      EXPECT_TRUE(SameBits(a->points[i], b->points[i])) << i;
    }
    if (fit) {
      EXPECT_EQ(a->size(), 40u);
    } else {
      std::size_t finite = 0;
      for (const auto& p : cloud->points)
        finite += std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
      EXPECT_EQ(a->size(), finite);
    }
  }
}

TEST(GroundFilter, SeedsOutsideSeedBandFailAndKeepFinitePoints) {
  GroundFilter filter;
  auto cloud = NewCloud(), out = NewCloud();
  AddFlatGround(*cloud, -2.5f);  // below seed_min_z (-2.0)
  cloud->push_back(P(std::numeric_limits<float>::quiet_NaN(), 0, 0));
  cloud->is_dense = false;
  filter.Filter(cloud, out, nullptr);
  EXPECT_FALSE(filter.CurrentPlane().valid);
  EXPECT_EQ(out->size(), cloud->size() - 1);
}

TEST(GroundFilter, FitIsDeterministicAndHistoryFree) {
  auto cloud = NewCloud();
  AddFlatGround(*cloud);
  for (int i = 0; i < 300; ++i) {
    // Noisy, deterministic ground plus clutter.
    const float x = 4.f + 0.17f * i, y = std::sin(0.3f * i) * 2.5f;
    cloud->push_back(P(x, y, -1.5f + 0.05f * std::sin(1.7f * i) + (i % 9 == 0 ? 0.8f : 0.f)));
  }
  GroundFilter a, b;
  auto oa = NewCloud(), ob = NewCloud(), oc = NewCloud();
  auto other = NewCloud();
  AddFlatGround(*other, -1.2f);
  a.Filter(cloud, oa, nullptr);
  b.Filter(other, oc, nullptr);  // different history
  b.Filter(cloud, ob, nullptr);
  ASSERT_TRUE(a.CurrentPlane().valid);
  ASSERT_TRUE(b.CurrentPlane().valid);
  for (int k = 0; k < 4; ++k)
    EXPECT_EQ(a.CurrentPlane().coefficients[k], b.CurrentPlane().coefficients[k]);
  ASSERT_EQ(oa->size(), ob->size());
  for (std::size_t i = 0; i < oa->size(); ++i)
    EXPECT_TRUE(SameBits(oa->points[i], ob->points[i]));
}

// LOCAL-2C audit: fitting and removal are separate decisions. A tight fit must
// neither erase a thin elevated return nor change the absolute-distance rule.
TEST(GroundFilter, TightConsensusRetainsElevatedThinAndBelowPlanePoints) {
  for (double band : {0., .10}) {
    GroundConfig c;
    c.fit_threshold_m = band;
    GroundFilter filter(c);
    auto cloud = NewCloud(), output = NewCloud();
    AddFlatGround(*cloud, -1.8f);
    // A steel bar hit by one ring remains an obstacle despite zero Z span.
    for (int i = 0; i < 24; ++i) cloud->push_back(P(8.f+i*.05f, 0.f, -1.1f, 1));
    // This candidate corrects plane selection, NOT a new below-plane deletion.
    for (int i = 0; i < 8; ++i) cloud->push_back(P(9.f+i*.05f, 1.f, -2.4f, 2));
    filter.Filter(cloud, output, nullptr);
    ASSERT_TRUE(filter.CurrentPlane().valid);
    int elevated = 0, below = 0;
    for (const auto& p : *output) {
      elevated += p.intensity == 1;
      below += p.intensity == 2;
    }
    EXPECT_EQ(elevated, 24);
    EXPECT_EQ(below, 8);
    EXPECT_NEAR(filter.CurrentPlane().coefficients[3], 1.8, .015);
  }
}

}  // namespace

int main(int argc, char** argv) { testing::InitGoogleTest(&argc, argv); return RUN_ALL_TESTS(); }
