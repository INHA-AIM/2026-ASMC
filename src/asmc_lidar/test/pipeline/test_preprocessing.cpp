// ROI/ego removal, voxel grid and leveling (Main RoiVoxel.cpp / ImuFusion.cpp at 8f93c22e).
#include <asmc_lidar/preprocessing/preprocessing.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <limits>
#include <random>
#include <stdexcept>
#include <vector>

namespace {

using asmc::lidar::pipeline::Cloud;
using asmc::lidar::pipeline::CloudPtr;
using asmc::lidar::pipeline::LevelPoints;
using asmc::lidar::pipeline::RangeConfig;
using asmc::lidar::pipeline::Roi;
using asmc::lidar::pipeline::RoiConfig;
using asmc::lidar::pipeline::VehicleSpec;
using asmc::lidar::pipeline::Voxel;
using asmc::lidar::Rot;

constexpr double kPi = 3.14159265358979323846;
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr float kInf = std::numeric_limits<float>::infinity();

pcl::PointXYZI P(float x, float y, float z, float intensity = 0.f) {
  pcl::PointXYZI p;
  p.x = x;
  p.y = y;
  p.z = z;
  p.intensity = intensity;
  return p;
}

CloudPtr MakeCloud(const std::vector<pcl::PointXYZI>& points, bool dense = false) {
  CloudPtr cloud(new Cloud);
  for (const auto& p : points) cloud->push_back(p);
  cloud->is_dense = dense;
  return cloud;
}

// setting.yaml Vehicle.* (rear-axle base, identity mount at (0.80, 0, 1.45)).
VehicleSpec MainVehicle() {
  VehicleSpec v;
  v.f32_Width = 1.892f;
  v.f32_Length = 4.635f;
  v.f32_WheelBase = 3.f;
  v.f32_FrontOverhang = 0.845f;
  v.f32_RearOverhang = 0.700f;
  v.st_LiDARPos = {0.80f, 0.f, 1.45f};
  v.st_LiDARRotationDeg = {0.f, 0.f, 0.f};
  v.st_GPSPos = {0.35f, 0.f, 1.25f};
  return v;
}

// Ego column moved far outside the ROI so only range/box bounds act.
VehicleSpec NoEgoVehicle() {
  VehicleSpec v = MainVehicle();
  v.st_LiDARPos = {0.f, 100.f, 0.f};
  return v;
}

bool Contains(const Cloud& cloud, const pcl::PointXYZI& p) {
  for (const auto& q : cloud.points) {
    if (q.x == p.x && q.y == p.y && q.z == p.z && q.intensity == p.intensity) return true;
  }
  return false;
}

// ---------------------------------------------------------------- ROI
TEST(Roi, BoxBoundsAreInclusive) {
  const RoiConfig roi;  // -30/50, -20/20, -2/3
  const std::vector<pcl::PointXYZI> inside{P(-30.f, 0, 0), P(50.f, 0, 0), P(10, -20.f, 0),
                                           P(10, 20.f, 0),  P(10, 0, -2.f), P(10, 0, 3.f)};
  const std::vector<pcl::PointXYZI> outside{
      P(std::nextafter(-30.f, -kInf), 0, 0), P(std::nextafter(50.f, kInf), 0, 0),
      P(10, std::nextafter(-20.f, -kInf), 0), P(10, std::nextafter(20.f, kInf), 0),
      P(10, 0, std::nextafter(-2.f, -kInf)),  P(10, 0, std::nextafter(3.f, kInf))};
  std::vector<pcl::PointXYZI> all = inside;
  all.insert(all.end(), outside.begin(), outside.end());
  CloudPtr out(new Cloud);
  Roi(MakeCloud(all), out, RangeConfig{}, roi, NoEgoVehicle());
  ASSERT_EQ(out->size(), inside.size());
  for (const auto& p : inside) EXPECT_TRUE(Contains(*out, p)) << p;
}

TEST(Roi, RangeIsThreeDimensionalAndInclusive) {
  RangeConfig range;
  range.f32_Min = 0.5f;
  range.f32_Max = 40.f;
  const auto in = MakeCloud({
      P(0.5f, 0, 0),                          // r == Min: kept
      P(std::nextafter(0.5f, 0.f), 0, 0),     // r just below Min: dropped
      P(0.3f, 0, 0.45f),                      // xy 0.30 < Min but 3-D 0.54: kept
      P(0.3f, 0.3f, 0),                       // r 0.42: dropped
      P(40.f, 0, 0),                          // r == Max: kept
      P(39.f, 0, 2.9f),                       // 3-D 39.11: kept
      P(39.9f, 0, 2.9f),                      // xy 39.9 < Max but 3-D 40.005: dropped
  });
  CloudPtr out(new Cloud);
  Roi(in, out, range, RoiConfig{}, NoEgoVehicle());
  ASSERT_EQ(out->size(), 4u);
  EXPECT_TRUE(Contains(*out, P(0.5f, 0, 0)));
  EXPECT_TRUE(Contains(*out, P(0.3f, 0, 0.45f)));
  EXPECT_TRUE(Contains(*out, P(40.f, 0, 0)));
  EXPECT_TRUE(Contains(*out, P(39.f, 0, 2.9f)));
}

TEST(Roi, NonFinitePointsAreDropped) {
  const auto in = MakeCloud({P(kNaN, 0, 0), P(10, kNaN, 0), P(10, 0, kNaN), P(kInf, 0, 0),
                             P(10, -kInf, 0), P(10, 0, kInf), P(10, 1, 0)});
  CloudPtr out(new Cloud);
  Roi(in, out, RangeConfig{}, RoiConfig{}, NoEgoVehicle());
  ASSERT_EQ(out->size(), 1u);
  EXPECT_TRUE(Contains(*out, P(10, 1, 0)));
}

TEST(Roi, EgoColumnBoundsAreInclusiveWithoutZLimit) {
  VehicleSpec v = MainVehicle();
  v.st_LiDARPos = {0.f, 0.f, 0.f};  // vehicle frame == sensor frame (exact identity)
  const float x_min = -v.f32_RearOverhang;
  const float x_max = v.f32_WheelBase + v.f32_FrontOverhang;
  const float y_max = 0.5f * v.f32_Width;
  const float y_min = -0.5f * v.f32_Width;
  const std::vector<pcl::PointXYZI> ego{P(x_max, 0, 0), P(x_min, 0, 0),  P(2, y_max, 0),
                                        P(2, y_min, 0), P(2, 0, -1.9f), P(2, 0, 2.9f),
                                        P(x_max, y_max, 2.5f)};
  const std::vector<pcl::PointXYZI> kept{
      P(std::nextafter(x_max, kInf), 0, 0), P(std::nextafter(x_min, -kInf), 0, 0),
      P(2, std::nextafter(y_max, kInf), 0), P(2, std::nextafter(y_min, -kInf), 0)};
  std::vector<pcl::PointXYZI> all = ego;
  all.insert(all.end(), kept.begin(), kept.end());
  CloudPtr out(new Cloud);
  Roi(MakeCloud(all), out, RangeConfig{}, RoiConfig{}, v);
  ASSERT_EQ(out->size(), kept.size());
  for (const auto& p : kept) EXPECT_TRUE(Contains(*out, p)) << p;
}

TEST(Roi, EgoColumnUsesMountTranslation) {
  // Identity mount at (0.80, 0, 1.45): x_v in [-0.70, 3.845] is x_s in [-1.50, 3.045].
  const auto in = MakeCloud({P(-1.45f, 0, 0), P(-1.55f, 0, 0), P(3.0f, 0.5f, -1.9f),
                             P(3.1f, 0, 0), P(1.f, 0.9f, 2.f), P(1.f, 1.0f, 2.f)});
  CloudPtr out(new Cloud);
  Roi(in, out, RangeConfig{}, RoiConfig{}, MainVehicle());
  ASSERT_EQ(out->size(), 3u);
  EXPECT_TRUE(Contains(*out, P(-1.55f, 0, 0)));
  EXPECT_TRUE(Contains(*out, P(3.1f, 0, 0)));
  EXPECT_TRUE(Contains(*out, P(1.f, 1.0f, 2.f)));
}

TEST(Roi, EgoColumnUsesMountRotation) {
  VehicleSpec v = MainVehicle();
  v.st_LiDARPos = {0.f, 0.f, 0.f};
  v.st_LiDARRotationDeg = {0.f, 0.f, 90.f};  // sensor +x looks along vehicle +y
  // Sensor (0,-2,0) is vehicle (2,0,0): inside. Sensor (2,0,0) is vehicle (0,2,0): outside.
  const auto in = MakeCloud({P(0, -2.f, 0), P(2.f, 0, 0)});
  CloudPtr out(new Cloud);
  Roi(in, out, RangeConfig{}, RoiConfig{}, v);
  ASSERT_EQ(out->size(), 1u);
  EXPECT_TRUE(Contains(*out, P(2.f, 0, 0)));
}

TEST(Roi, OutputIsClearedOrderedAndMetadataSet) {
  CloudPtr out(new Cloud);
  for (int i = 0; i < 5; ++i) out->push_back(P(99, 99, 99, 9));
  out->is_dense = true;
  const auto in = MakeCloud({P(20, 1, 0, 3.f), P(kNaN, 0, 0), P(10, -1, 0, 7.f), P(-10, 4, 1, 1.f)});
  Roi(in, out, RangeConfig{}, RoiConfig{}, MainVehicle());
  ASSERT_EQ(out->points.size(), 3u);
  EXPECT_EQ(out->width, 3u);
  EXPECT_EQ(out->height, 1u);
  EXPECT_FALSE(out->is_dense);
  EXPECT_EQ(out->points[0].intensity, 3.f);
  EXPECT_EQ(out->points[1].intensity, 7.f);
  EXPECT_EQ(out->points[2].intensity, 1.f);
  EXPECT_EQ(out->points[2].x, -10.f);
}

// ---------------------------------------------------------------- Voxel
TEST(Voxel, AveragesPointsAndIntensityInOneLeaf) {
  const auto in = MakeCloud({P(10.12f, 5.03f, 0.51f, 2.f), P(10.18f, 5.07f, 0.59f, 4.f),
                             P(12.15f, 5.05f, 0.55f, 8.f)});
  CloudPtr out(new Cloud);
  Voxel(in, out);
  ASSERT_EQ(out->size(), 2u);
  EXPECT_EQ(out->width, 2u);
  EXPECT_EQ(out->height, 1u);
  EXPECT_TRUE(out->is_dense);
  EXPECT_FLOAT_EQ(out->points[0].x, 0.5f * (10.12f + 10.18f));
  EXPECT_FLOAT_EQ(out->points[0].y, 0.5f * (5.03f + 5.07f));
  EXPECT_FLOAT_EQ(out->points[0].z, 0.5f * (0.51f + 0.59f));
  EXPECT_FLOAT_EQ(out->points[0].intensity, 3.f);
  EXPECT_EQ(out->points[1].x, 12.15f);
  EXPECT_EQ(out->points[1].intensity, 8.f);
}

TEST(Voxel, LeafBoundariesAreAbsoluteMultiplesOfTheLeaf) {
  // 0.09 apart but on both sides of x = 0.1: two leaves (not anchored at the cloud min).
  const auto split = MakeCloud({P(0.05f, 5.05f, 0.55f), P(0.14f, 5.05f, 0.55f)});
  CloudPtr out(new Cloud);
  Voxel(split, out);
  EXPECT_EQ(out->size(), 2u);
  // 0.08 apart inside [0.1, 0.2): one leaf.
  const auto same = MakeCloud({P(0.11f, 5.05f, 0.55f), P(0.19f, 5.05f, 0.55f)});
  Voxel(same, out);
  EXPECT_EQ(out->size(), 1u);
  // A 0.2 leaf merges the first pair.
  Voxel(split, out, 0.2f);
  EXPECT_EQ(out->size(), 1u);
}

TEST(Voxel, OutputIsSortedByLeafIndexXFastest) {
  const pcl::PointXYZI a = P(0.15f, 0.15f, 0.15f, 1), b = P(0.55f, 0.15f, 0.15f, 2),
                       c = P(0.15f, 0.55f, 0.15f, 3), d = P(0.15f, 0.15f, 0.55f, 4);
  const auto in = MakeCloud({d, c, b, a});
  CloudPtr out(new Cloud);
  Voxel(in, out);
  ASSERT_EQ(out->size(), 4u);
  EXPECT_EQ(out->points[0].intensity, 1.f);
  EXPECT_EQ(out->points[1].intensity, 2.f);
  EXPECT_EQ(out->points[2].intensity, 3.f);
  EXPECT_EQ(out->points[3].intensity, 4.f);
}

TEST(Voxel, NonFiniteDroppedForNonDenseInput) {
  const auto in = MakeCloud({P(1.05f, 2.05f, 0.05f), P(kNaN, 1, 1), P(1, kInf, 1)}, false);
  CloudPtr out(new Cloud);
  Voxel(in, out);
  ASSERT_EQ(out->size(), 1u);
  EXPECT_TRUE(std::isfinite(out->points[0].x));
}

TEST(Voxel, DeterministicForIdenticalInput) {
  std::mt19937 rng(7);
  std::uniform_real_distribution<float> xy(-20.f, 20.f), z(-1.8f, 2.5f), in(0.f, 255.f);
  std::vector<pcl::PointXYZI> points;
  for (int i = 0; i < 20000; ++i) points.push_back(P(xy(rng), xy(rng), z(rng), in(rng)));
  const auto cloud = MakeCloud(points);
  CloudPtr first(new Cloud), second(new Cloud);
  Voxel(cloud, first);
  Voxel(cloud, second);
  ASSERT_EQ(first->size(), second->size());
  ASSERT_GT(first->size(), 1000u);
  for (std::size_t i = 0; i < first->size(); ++i) {
    const auto& a = first->points[i];
    const auto& b = second->points[i];
    EXPECT_EQ(0, std::memcmp(&a.x, &b.x, 3 * sizeof(float))) << i;
    EXPECT_EQ(a.intensity, b.intensity) << i;
  }
}

// ---------------------------------------------------------------- LevelPoints
Rot Rx(double r) {
  Rot m;
  m << 1, 0, 0, 0, std::cos(r), -std::sin(r), 0, std::sin(r), std::cos(r);
  return m;
}
Rot Ry(double p) {
  Rot m;
  m << std::cos(p), 0, std::sin(p), 0, 1, 0, -std::sin(p), 0, std::cos(p);
  return m;
}
Rot Rz(double y) {
  Rot m;
  m << std::cos(y), -std::sin(y), 0, std::sin(y), std::cos(y), 0, 0, 0, 1;
  return m;
}

CloudPtr LevelInput() {
  return MakeCloud({P(10.f, 2.f, -1.4f, 5.f), P(-7.5f, 11.f, 0.3f, 6.f), P(3.25f, -9.f, 1.7f, 7.f),
                    P(0.f, 0.f, 0.f, 8.f), P(-12.f, -4.5f, -1.9f, 9.f)});
}

void ExpectLeveled(const Cloud& in, const Cloud& out, const Rot& expected) {
  ASSERT_EQ(out.points.size(), in.points.size());
  for (std::size_t i = 0; i < in.points.size(); ++i) {
    const auto& p = in.points[i];
    const Eigen::Vector3d q = expected * Eigen::Vector3d(p.x, p.y, p.z);
    EXPECT_NEAR(out.points[i].x, q.x(), 1e-6) << i;
    EXPECT_NEAR(out.points[i].y, q.y(), 1e-6) << i;
    EXPECT_NEAR(out.points[i].z, q.z(), 1e-6) << i;
    EXPECT_EQ(out.points[i].intensity, p.intensity) << i;
  }
}

TEST(LevelPoints, IdentityMountIsRyPitchTimesRxRoll) {
  const float roll = 0.07f, pitch = -0.05f;
  const auto in = LevelInput();
  CloudPtr out(new Cloud);
  const Rot applied = LevelPoints(in, out, roll, pitch, Rot::Identity(), true);
  const Rot expected = Ry(pitch) * Rx(roll);
  EXPECT_LT((applied - expected).cwiseAbs().maxCoeff(), 1e-12);
  ExpectLeveled(*in, *out, expected);
}

TEST(LevelPoints, NonzeroMountIsMountTransposeRMount) {
  const float roll = -0.12f, pitch = 0.06f;
  const Rot mount = Rz(8. * kPi / 180.) * Ry(-3. * kPi / 180.) * Rx(4. * kPi / 180.);
  const auto in = LevelInput();
  CloudPtr out(new Cloud);
  const Rot applied = LevelPoints(in, out, roll, pitch, mount, true);
  const Rot expected = mount.transpose() * Ry(pitch) * Rx(roll) * mount;
  EXPECT_LT((applied - expected).cwiseAbs().maxCoeff(), 1e-12);
  ExpectLeveled(*in, *out, expected);
  // Pitch-only with the same mount.
  const Rot pitch_only = LevelPoints(in, out, roll, pitch, mount, false);
  EXPECT_LT((pitch_only - mount.transpose() * Ry(pitch) * mount).cwiseAbs().maxCoeff(), 1e-12);
  ExpectLeveled(*in, *out, pitch_only);
}

TEST(LevelPoints, PitchOnlyIgnoresRoll) {
  const float roll = 0.2f, pitch = 0.04f;
  const auto in = LevelInput();
  CloudPtr out(new Cloud);
  const Rot applied = LevelPoints(in, out, roll, pitch, Rot::Identity(), false);
  EXPECT_LT((applied - Ry(pitch)).cwiseAbs().maxCoeff(), 1e-12);
  ExpectLeveled(*in, *out, Ry(pitch));
}

TEST(LevelPoints, MatchesMainDoubleExpressionBitwise) {
  // Main: (exact * pt.cast<double>()).cast<float>() with pt the float xyz.
  const float roll = 0.031f, pitch = -0.027f;
  const auto in = LevelInput();
  CloudPtr out(new Cloud);
  const Rot applied = LevelPoints(in, out, roll, pitch, Rot::Identity(), true);
  for (std::size_t i = 0; i < in->size(); ++i) {
    const Eigen::Vector3f pt(in->points[i].x, in->points[i].y, in->points[i].z);
    const Eigen::Vector3f q = (applied * pt.cast<double>()).cast<float>().eval();
    EXPECT_EQ(out->points[i].x, q.x());
    EXPECT_EQ(out->points[i].y, q.y());
    EXPECT_EQ(out->points[i].z, q.z());
  }
}

TEST(LevelPoints, OnlyPointsAreWrittenLikeMain) {
  const auto in = LevelInput();
  CloudPtr fresh(new Cloud);
  LevelPoints(in, fresh, 0.01f, 0.02f, Rot::Identity(), true);
  EXPECT_EQ(fresh->points.size(), in->size());
  EXPECT_EQ(fresh->width, 0u);   // never set by Main CorrectingPoints
  EXPECT_EQ(fresh->height, 0u);
  EXPECT_TRUE(fresh->is_dense);  // PCL default kept
  // A reused cloud keeps its stale metadata (preserved quirk).
  CloudPtr reused(new Cloud);
  for (int i = 0; i < 9; ++i) reused->push_back(P(1, 1, 1));
  reused->is_dense = false;
  LevelPoints(in, reused, 0.01f, 0.02f, Rot::Identity(), true);
  EXPECT_EQ(reused->points.size(), in->size());
  EXPECT_EQ(reused->width, 9u);
  EXPECT_EQ(reused->height, 1u);
  EXPECT_FALSE(reused->is_dense);
  EXPECT_EQ(reused->points[0].x, fresh->points[0].x);
}

TEST(LevelPoints, NonFiniteAnglesOrMountThrowWithoutTouchingOutput) {
  const auto in = LevelInput();
  CloudPtr out(new Cloud);
  out->push_back(P(42, 42, 42, 42));
  EXPECT_THROW(LevelPoints(in, out, kNaN, 0.f, Rot::Identity(), true), std::invalid_argument);
  EXPECT_THROW(LevelPoints(in, out, 0.f, kInf, Rot::Identity(), true), std::invalid_argument);
  // Roll is validated even when it is not applied (asmc::lidar::leveling_rotation).
  EXPECT_THROW(LevelPoints(in, out, kNaN, 0.f, Rot::Identity(), false), std::invalid_argument);
  Rot scaled = 2. * Rot::Identity();
  EXPECT_THROW(LevelPoints(in, out, 0.f, 0.f, scaled, true), std::invalid_argument);
  ASSERT_EQ(out->size(), 1u);
  EXPECT_EQ(out->points[0].x, 42.f);
}

}  // namespace

int main(int argc, char** argv) { testing::InitGoogleTest(&argc, argv); return RUN_ALL_TESTS(); }
