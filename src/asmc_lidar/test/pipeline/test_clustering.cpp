// Adaptive Euclidean clustering and centroids (Main Clustering.cpp at 8f93c22e).
#include <asmc_lidar/detection/clustering.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <random>
#include <set>
#include <vector>

namespace {

using asmc::lidar::pipeline::AdaptiveClusterTolerance;
using asmc::lidar::pipeline::Cloud;
using asmc::lidar::pipeline::CloudPtr;
using asmc::lidar::pipeline::Clustering;
using asmc::lidar::pipeline::ClusteringConfig;

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

pcl::PointXYZI P(float x, float y, float z, float intensity = 0.f) {
  pcl::PointXYZI p;
  p.x = x;
  p.y = y;
  p.z = z;
  p.intensity = intensity;
  return p;
}

CloudPtr NewCloud() {
  CloudPtr cloud(new Cloud);
  cloud->is_dense = false;  // ground output metadata
  return cloud;
}

// n points on a line along +y, 0.2 m apart (one component for any tolerance >= 0.55).
void AddLine(Cloud& cloud, float x, float y, int n, float z = 0.f) {
  for (int i = 0; i < n; ++i) cloud.push_back(P(x, y + 0.2f * i, z));
}

struct Result {
  std::vector<pcl::PointIndices> clusters;
  CloudPtr centroids{new Cloud};
};

Result RunClustering(const CloudPtr& input, const ClusteringConfig& config = ClusteringConfig{}) {
  Result r;
  Clustering(input, r.centroids, r.clusters, config);
  return r;
}

using Key = std::array<float, 3>;
std::set<std::vector<Key>> ClusterSets(const Cloud& cloud,
                                       const std::vector<pcl::PointIndices>& clusters) {
  std::set<std::vector<Key>> out;
  for (const auto& c : clusters) {
    std::vector<Key> keys;
    for (int i : c.indices) keys.push_back({cloud.points[i].x, cloud.points[i].y, cloud.points[i].z});
    std::sort(keys.begin(), keys.end());
    out.insert(keys);
  }
  return out;
}

// ------------------------------------------------- Main test/test_adaptive_clustering.cpp
ClusteringConfig AdaptiveConfig() {
  ClusteringConfig config{};
  config.b_Adaptive = true;
  config.f32_Eps = 0.5F;
  config.s32_MinPoints = 5;
  config.f32_AdaptiveNearRange = 8.0F;
  config.f32_AdaptiveFarRange = 50.0F;
  config.f32_AdaptiveNearEps = 0.55F;
  config.f32_AdaptiveFarEps = 0.85F;
  return config;
}

TEST(AdaptiveClustering, ClampsToleranceAtNearAndFarLimits) {
  const auto config = AdaptiveConfig();
  EXPECT_FLOAT_EQ(AdaptiveClusterTolerance(0.0F, config), 0.55F);
  EXPECT_FLOAT_EQ(AdaptiveClusterTolerance(8.0F, config), 0.55F);
  EXPECT_FLOAT_EQ(AdaptiveClusterTolerance(50.0F, config), 0.85F);
  EXPECT_FLOAT_EQ(AdaptiveClusterTolerance(80.0F, config), 0.85F);
}

TEST(AdaptiveClustering, InterpolatesWithoutGlobalLargeRadius) {
  const auto config = AdaptiveConfig();
  EXPECT_NEAR(AdaptiveClusterTolerance(29.0F, config), 0.70F, 1e-5F);
  EXPECT_LT(AdaptiveClusterTolerance(50.0F, config), 1.0F);
}

TEST(AdaptiveClustering, RetainsFixedRadiusWhenDisabled) {
  auto config = AdaptiveConfig();
  config.b_Adaptive = false;
  EXPECT_FLOAT_EQ(AdaptiveClusterTolerance(50.0F, config), 0.5F);
}

// ------------------------------------------------- new
TEST(AdaptiveClustering, LinearInterpolationAndSanitizedConfig) {
  auto c = AdaptiveConfig();
  EXPECT_FLOAT_EQ(AdaptiveClusterTolerance(18.5F, c), 0.55F + 0.25F * 0.3F);
  EXPECT_FLOAT_EQ(AdaptiveClusterTolerance(kNaN, c), 0.55F);   // non-finite -> near range
  EXPECT_FLOAT_EQ(AdaptiveClusterTolerance(-3.F, c), 0.55F);   // clamped to 0
  EXPECT_FLOAT_EQ(AdaptiveClusterTolerance(std::numeric_limits<float>::infinity(), c), 0.55F);
  // NearRange 0 is not positive and silently becomes 8 m (preserved quirk).
  c.f32_AdaptiveNearRange = 0.F;
  EXPECT_FLOAT_EQ(AdaptiveClusterTolerance(8.F, c), 0.55F);
  c = AdaptiveConfig();
  c.f32_AdaptiveFarRange = 8.5F;  // far >= near + 1
  EXPECT_FLOAT_EQ(AdaptiveClusterTolerance(9.F, c), 0.85F);
  EXPECT_FLOAT_EQ(AdaptiveClusterTolerance(8.5F, c), 0.55F + 0.5F * 0.3F);
  c = AdaptiveConfig();
  c.f32_AdaptiveNearEps = -1.F;  // near eps falls back to Eps
  EXPECT_FLOAT_EQ(AdaptiveClusterTolerance(0.F, c), 0.5F);
  c = AdaptiveConfig();
  c.f32_AdaptiveFarEps = 0.3F;  // far eps never below near eps
  EXPECT_FLOAT_EQ(AdaptiveClusterTolerance(80.F, c), 0.55F);
  c = AdaptiveConfig();
  c.f32_Eps = kNaN;
  c.b_Adaptive = false;
  EXPECT_FLOAT_EQ(AdaptiveClusterTolerance(10.F, c), 0.5F);  // kDefaultClusterTolerance
}

TEST(Clustering, ConnectedComponentsIndependentOfPointOrder) {
  std::vector<pcl::PointXYZI> points;
  std::mt19937 rng(11);
  std::uniform_real_distribution<float> jitter(-0.15f, 0.15f);
  const float centers[][2] = {{10, 0}, {10, 2.5f}, {25, -6}, {40, 12}, {6, -3}};
  for (const auto& c : centers)
    for (int i = 0; i < 30; ++i)
      points.push_back(P(c[0] + jitter(rng) + 0.1f * (i % 6), c[1] + jitter(rng) + 0.1f * (i / 6),
                         jitter(rng)));
  for (int i = 0; i < 12; ++i) points.push_back(P(-20.f + 3.f * i, 30.f, 0));  // singletons
  CloudPtr a = NewCloud();
  for (const auto& p : points) a->push_back(p);
  std::shuffle(points.begin(), points.end(), rng);
  CloudPtr b = NewCloud();
  for (const auto& p : points) b->push_back(p);
  const auto ra = RunClustering(a), rb = RunClustering(b);
  EXPECT_EQ(ra.clusters.size(), 5u);
  EXPECT_EQ(ClusterSets(*a, ra.clusters), ClusterSets(*b, rb.clusters));
}

ClusteringConfig FrontRearConfig() {
  auto c = AdaptiveConfig();
  c.f32_AdaptiveNearRange = 10.f;
  c.f32_AdaptiveNearEps = 1.30f; c.f32_AdaptiveFarEps = 1.32f;
  c.f32_AdaptiveRearNearEps = 1.60f; c.f32_AdaptiveRearFarEps = 1.62f;
  c.s32_MinPoints = 1;
  return c;
}

TEST(AdaptiveClustering, FrontRearRangeProfileAndZeroBoundary) {
  const auto c = FrontRearConfig();
  for (float r : {0.f, 10.f}) {
    EXPECT_FLOAT_EQ(AdaptiveClusterTolerance(r, c, 1.f), 1.30f);
    EXPECT_FLOAT_EQ(AdaptiveClusterTolerance(r, c, -1.f), 1.60f);
  }
  EXPECT_NEAR(AdaptiveClusterTolerance(30.f, c, 1.f), 1.31f, 1e-6f);
  EXPECT_NEAR(AdaptiveClusterTolerance(30.f, c, -1.f), 1.61f, 1e-6f);
  for (float r : {50.f, 80.f}) {
    EXPECT_FLOAT_EQ(AdaptiveClusterTolerance(r, c, 1.f), 1.32f);
    EXPECT_FLOAT_EQ(AdaptiveClusterTolerance(r, c, -1.f), 1.62f);
  }
  EXPECT_FLOAT_EQ(AdaptiveClusterTolerance(5.f, c, 0.f), 1.30f);
  EXPECT_FLOAT_EQ(AdaptiveClusterTolerance(5.f, c, -0.f), 1.30f);
  const auto legacy = AdaptiveConfig();
  EXPECT_FLOAT_EQ(AdaptiveClusterTolerance(30.f, legacy, -1.f),
                  AdaptiveClusterTolerance(30.f, legacy, 1.f));
}

TEST(Clustering, MirroredFrontRearActuallyProduceDifferentComponents) {
  const auto c = FrontRearConfig();
  for (float x : {4.f, -4.f}) {
    auto cloud = NewCloud();
    cloud->push_back(P(x, 0, 0)); cloud->push_back(P(x, 1.45f, 0));
    EXPECT_EQ(RunClustering(cloud, c).clusters.size(), x < 0 ? 1U : 2U);
  }
}

TEST(Clustering, AcrossXZeroUsesSmallerToleranceIndependentOfSeedOrder) {
  const auto c = FrontRearConfig();
  for (float half_distance : {0.6f, 0.71f}) {
    for (bool rear_first : {false, true}) {
      auto cloud = NewCloud();
      const float first = rear_first ? -half_distance : half_distance;
      cloud->push_back(P(first, 0, 0)); cloud->push_back(P(-first, 0, 0));
      EXPECT_EQ(RunClustering(cloud, c).clusters.size(), half_distance < 0.65f ? 1U : 2U);
    }
  }
}

TEST(Clustering, EdgeUsesTheSmallerOfBothTolerances) {
  // tol(30) = 0.70714 < d = 0.709 < tol(30.709) = 0.71221: the far point's search
  // radius reaches the near one, but the edge needs d <= min(tol): two clusters,
  // whichever point seeds the search.
  auto c = AdaptiveConfig();
  c.s32_MinPoints = 1;
  for (bool far_first : {false, true}) {
    CloudPtr cloud = NewCloud();
    if (far_first) cloud->push_back(P(30.709f, 0, 0));
    cloud->push_back(P(30.f, 0, 0));
    if (!far_first) cloud->push_back(P(30.709f, 0, 0));
    EXPECT_EQ(RunClustering(cloud, c).clusters.size(), 2u) << far_first;
  }
  CloudPtr joined = NewCloud();
  joined->push_back(P(30.70f, 0, 0));
  joined->push_back(P(30.f, 0, 0));
  EXPECT_EQ(RunClustering(joined, c).clusters.size(), 1u);
}

TEST(Clustering, SizeFilterMinPointsAndOversizeDroppedWhole) {
  CloudPtr cloud = NewCloud();
  AddLine(*cloud, 10, 0, 4);   // < 5: dropped
  AddLine(*cloud, 10, 5, 5);   // == 5: kept
  auto r = RunClustering(cloud);
  ASSERT_EQ(r.clusters.size(), 1u);
  EXPECT_EQ(r.clusters[0].indices.size(), 5u);
  // MinPoints <= 0 is raised to 1: singletons become clusters.
  auto c = AdaptiveConfig();
  c.s32_MinPoints = 0;
  CloudPtr single = NewCloud();
  single->push_back(P(10, 0, 0));
  single->push_back(P(10, 9, 0));
  EXPECT_EQ(RunClustering(single, c).clusters.size(), 2u);

  // 101 x 100 grid (10100 points) is one component above 10000: dropped, not split.
  CloudPtr big = NewCloud();
  for (int i = 0; i < 101; ++i)
    for (int j = 0; j < 100; ++j) big->push_back(P(10.f + 0.1f * i, 0.1f * j, 0));
  AddLine(*big, 10, -20, 6);
  r = RunClustering(big);
  ASSERT_EQ(r.clusters.size(), 1u);
  EXPECT_EQ(r.clusters[0].indices.size(), 6u);
  // Exactly 10000 points is kept.
  CloudPtr limit = NewCloud();
  for (int i = 0; i < 100; ++i)
    for (int j = 0; j < 100; ++j) limit->push_back(P(10.f + 0.1f * i, 0.1f * j, 0));
  r = RunClustering(limit);
  ASSERT_EQ(r.clusters.size(), 1u);
  EXPECT_EQ(r.clusters[0].indices.size(), 10000u);
}

TEST(Clustering, CentroidIsFloatMeanInIndexOrder) {
  CloudPtr cloud = NewCloud();
  const float xs[] = {10.123f, 10.371f, 10.007f, 10.299f, 10.411f, 10.188f, 10.252f};
  for (int i = 0; i < 7; ++i) cloud->push_back(P(xs[i], 0.37f + 0.11f * i, -0.81f + 0.07f * i, 50));
  AddLine(*cloud, 20, 5, 6, 1.3f);
  const auto r = RunClustering(cloud);
  ASSERT_EQ(r.clusters.size(), 2u);
  ASSERT_EQ(r.centroids->size(), 2u);
  for (std::size_t k = 0; k < 2; ++k) {
    float sx = 0, sy = 0, sz = 0;
    double dx = 0, dy = 0, dz = 0;
    const auto& idx = r.clusters[k].indices;
    for (int i : idx) {
      sx += cloud->points[i].x;
      sy += cloud->points[i].y;
      sz += cloud->points[i].z;
      dx += cloud->points[i].x;
      dy += cloud->points[i].y;
      dz += cloud->points[i].z;
    }
    const float n = static_cast<float>(idx.size());
    const auto& c = r.centroids->points[k];
    EXPECT_EQ(c.x, sx / n);
    EXPECT_EQ(c.y, sy / n);
    EXPECT_EQ(c.z, sz / n);
    EXPECT_NEAR(c.x, dx / idx.size(), 1e-5);
    EXPECT_NEAR(c.y, dy / idx.size(), 1e-5);
    EXPECT_NEAR(c.z, dz / idx.size(), 1e-5);
    EXPECT_EQ(c.intensity, 0.f);
  }
  EXPECT_EQ(r.centroids->width, 2u);
  EXPECT_EQ(r.centroids->height, 1u);
  EXPECT_TRUE(r.centroids->is_dense);
}

TEST(Clustering, EmptyInputGetsDummyPoint) {
  CloudPtr cloud(new Cloud);
  Result r;
  r.clusters.resize(3);
  Clustering(cloud, r.centroids, r.clusters, ClusteringConfig{});
  ASSERT_EQ(cloud->points.size(), 1u);
  EXPECT_EQ(cloud->points[0].x, 0.f);
  EXPECT_EQ(cloud->points[0].y, 0.f);
  EXPECT_EQ(cloud->points[0].z, 0.f);
  EXPECT_EQ(cloud->points[0].intensity, 0.f);
  EXPECT_EQ(cloud->width, 0u);  // only points are touched (preserved quirk)
  EXPECT_EQ(cloud->height, 0u);
  EXPECT_TRUE(r.clusters.empty());
  EXPECT_TRUE(r.centroids->empty());
  EXPECT_EQ(r.centroids->width, 0u);
}

TEST(Clustering, NoClusterAppendsDummyToInput) {
  CloudPtr cloud = NewCloud();
  cloud->push_back(P(10, 0, 0, 4));
  cloud->push_back(P(10, 9, 0, 5));
  cloud->push_back(P(kNaN, 1, 1, 6));
  Result r;
  r.clusters.resize(2);
  Clustering(cloud, r.centroids, r.clusters, AdaptiveConfig());
  ASSERT_EQ(cloud->points.size(), 4u);
  EXPECT_EQ(cloud->points[0].intensity, 4.f);
  EXPECT_EQ(cloud->points[3].x, 0.f);
  EXPECT_EQ(cloud->points[3].y, 0.f);
  EXPECT_EQ(cloud->points[3].z, 0.f);
  EXPECT_EQ(cloud->points[3].intensity, 0.f);
  EXPECT_EQ(cloud->width, 3u);  // stale: Main pushes into points only
  EXPECT_TRUE(r.clusters.empty());
  EXPECT_TRUE(r.centroids->empty());
  EXPECT_EQ(r.centroids->width, 0u);  // untouched on this path
  EXPECT_EQ(r.centroids->height, 0u);
}

TEST(Clustering, AdaptiveOrderIsSmallestSeedIndexAndIndicesSorted) {
  // Index 0 belongs to B, 1 to C, 2 to A: cluster order B, C, A.
  CloudPtr cloud = NewCloud();
  std::vector<int> label;
  auto add = [&](int which, int k) {
    const float base[3][2] = {{10, 0}, {10, 5}, {20, 0}};
    cloud->push_back(P(base[which][0], base[which][1] + 0.2f * k, 0, float(which)));
    label.push_back(which);
  };
  add(1, 0); add(2, 0); add(0, 0);
  for (int k = 5; k >= 1; --k) { add(0, k); add(2, k); add(1, k); }
  const auto r = RunClustering(cloud);
  ASSERT_EQ(r.clusters.size(), 3u);
  const int expected[3] = {1, 2, 0};
  for (int k = 0; k < 3; ++k) {
    const auto& idx = r.clusters[k].indices;
    ASSERT_EQ(idx.size(), 6u);
    EXPECT_TRUE(std::is_sorted(idx.begin(), idx.end()));
    for (int i : idx) EXPECT_EQ(label[i], expected[k]);
    EXPECT_FLOAT_EQ(r.centroids->points[k].x, expected[k] == 2 ? 20.f : 10.f);
  }
}

TEST(Clustering, NonFiniteSeedsAreSkipped) {
  CloudPtr cloud = NewCloud();
  AddLine(*cloud, 10, 0, 3);
  cloud->push_back(P(kNaN, kNaN, kNaN));
  AddLine(*cloud, 10, 0.6f, 3);
  const auto r = RunClustering(cloud);
  ASSERT_EQ(r.clusters.size(), 1u);
  EXPECT_EQ(r.clusters[0].indices, (std::vector<int>{0, 1, 2, 4, 5, 6}));
}

TEST(Clustering, PclPathSortsBySizeDescending) {
  auto c = AdaptiveConfig();
  c.b_Adaptive = false;
  CloudPtr cloud = NewCloud();
  AddLine(*cloud, 10, 0, 5);
  AddLine(*cloud, 10, 5, 8);
  AddLine(*cloud, 10, 10, 6);
  const auto r = RunClustering(cloud, c);
  ASSERT_EQ(r.clusters.size(), 3u);
  EXPECT_EQ(r.clusters[0].indices.size(), 8u);
  EXPECT_EQ(r.clusters[1].indices.size(), 6u);
  EXPECT_EQ(r.clusters[2].indices.size(), 5u);
  for (const auto& cl : r.clusters) EXPECT_TRUE(std::is_sorted(cl.indices.begin(), cl.indices.end()));
  EXPECT_NEAR(r.centroids->points[0].y, 5.7, 1e-5);  // centroids follow the PCL order
  // Fixed tolerance = Eps (invalid Eps -> 0.5): 0.45 m joins, 0.55 m does not.
  c.f32_Eps = -1.F;
  c.s32_MinPoints = 1;
  CloudPtr pair = NewCloud();
  pair->push_back(P(30, 0, 0));
  pair->push_back(P(30, 0.45f, 0));
  pair->push_back(P(30, 1.0f, 0));
  EXPECT_EQ(RunClustering(pair, c).clusters.size(), 2u);
}

TEST(Clustering, EnabledFlagIsNotConsulted) {
  auto c = AdaptiveConfig();
  c.b_Enabled = false;
  CloudPtr cloud = NewCloud();
  AddLine(*cloud, 10, 0, 5);
  EXPECT_EQ(RunClustering(cloud, c).clusters.size(), 1u);
}

TEST(Clustering, CentroidsAreAppendedNotCleared) {
  CloudPtr cloud = NewCloud();
  AddLine(*cloud, 10, 0, 5);
  AddLine(*cloud, 10, 5, 5);
  Result r;
  r.centroids->push_back(P(-1, -1, -1, 9));
  Clustering(cloud, r.centroids, r.clusters, AdaptiveConfig());
  ASSERT_EQ(r.centroids->size(), 3u);
  EXPECT_EQ(r.centroids->points[0].intensity, 9.f);
  EXPECT_EQ(r.centroids->width, 3u);
  EXPECT_EQ(r.clusters.size(), 2u);
  EXPECT_EQ(cloud->size(), 10u);  // input not mutated when clusters exist
}

}  // namespace

int main(int argc, char** argv) { testing::InitGoogleTest(&argc, argv); return RUN_ALL_TESTS(); }
