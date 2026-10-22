// Experimental tunnel stage: OFF bypass, ON smoke on a synthetic tunnel and a
// roll/pitch smoke. Connection checks only; no filter performance is claimed.

#include <asmc_lidar/pipeline/pipeline.hpp>
#include <asmc_lidar/pipeline/tunnel_filter_stage.hpp>
#include <asmc_lidar/projection.hpp>

#include <Lidar/TunnelFilter/TunnelFilterCore.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

using namespace asmc::lidar::pipeline;

namespace {

// The stage API has no GPS (or any other mode) input: Apply() takes only the
// voxel cloud, roll/pitch and the ground output, and the config is three switches.
using ApplySignature = TunnelFilterStageReport (TunnelFilterStage::*)(
    const CloudPtr&, float32, float32, const CloudPtr&, std::vector<int>&);
static_assert(std::is_same<decltype(&TunnelFilterStage::Apply), ApplySignature>::value,
              "TunnelFilterStage::Apply must not take a GPS/mode input");
static_assert(sizeof(TunnelFilterStageConfig) == 3 * sizeof(bool),
              "TunnelFilterStageConfig is enabled/suppress_ceiling/suppress_wall only");
static_assert(!std::is_default_constructible<TunnelFilterStage>::value, "config required");
static_assert(!std::is_copy_constructible<TunnelFilterStage>::value, "stateful, not copyable");

constexpr float kGroundZ = -1.70f;  // leveled sensor frame, metres
constexpr float kCeilingZ = 4.0f;

enum Part : std::uint8_t { kPartGround, kPartWall, kPartCeiling, kPartBox };

pcl::PointXYZI MakePoint(float x, float y, float z) {
  pcl::PointXYZI p;
  p.x = x;
  p.y = y;
  p.z = z;
  p.intensity = 0.f;
  return p;
}

struct Scene {
  Cloud level;  // leveled frame, the scene as built
  std::vector<Part> part;
  void Add(float x, float y, float z, Part p) {
    level.points.push_back(MakePoint(x, y, z));
    part.push_back(p);
  }
};

// Flat ground, vertical walls at y=+-5 m up to just below a ceiling at z=+4 m,
// and a car-sized box in the lane.
Scene MakeTunnel() {
  Scene s;
  for (float x = -10.f; x <= 40.f; x += 0.5f)
    for (float y = -4.5f; y <= 4.5f; y += 0.5f) s.Add(x, y, kGroundZ, kPartGround);
  for (const float y : {5.f, -5.f})
    for (float x = -8.f; x <= 40.f; x += 0.4f)
      for (float h = 0.3f; h <= 5.5f; h += 0.2f) s.Add(x, y, kGroundZ + h, kPartWall);
  for (float x = -8.f; x <= 40.f; x += 0.5f)
    for (float y = -4.5f; y <= 4.5f; y += 0.5f) s.Add(x, y, kCeilingZ, kPartCeiling);
  for (float x = 12.f; x <= 16.4f; x += 0.2f)
    for (float y = -0.9f; y <= 0.9f; y += 0.2f)
      for (float z = kGroundZ + 0.3f; z <= kGroundZ + 1.5f; z += 0.2f) s.Add(x, y, z, kPartBox);
  s.level.width = static_cast<std::uint32_t>(s.level.points.size());
  s.level.height = 1;
  s.level.is_dense = true;
  return s;
}

// Stand-in for the common RANSAC output (leveled frame, input order): every
// non-ground point plus a far ground strip (x >= 35 m) that the core labels
// kGround and the stage must keep.
bool InCommonOutput(const Scene& s, std::size_t i) {
  return s.part[i] != kPartGround || s.level.points[i].x >= 35.f;
}
void CommonOutput(const Scene& s, const CloudPtr& out, std::vector<int>* indices) {
  out->clear();
  indices->clear();
  for (std::size_t i = 0; i < s.level.points.size(); ++i) {
    if (!InCommonOutput(s, i)) continue;
    out->push_back(s.level.points[i]);
    indices->push_back(static_cast<int>(i));
  }
}

// Sensor-frame scan whose common leveling (identity mount) gives the scene:
// p_level = L * p_sensor, so p_sensor = L^T * p_level.
CloudPtr SensorCloud(const Scene& s, float roll, float pitch) {
  CloudPtr out(new Cloud);
  if (roll == 0.f && pitch == 0.f) {
    *out = s.level;
    return out;
  }
  const asmc::lidar::Rot L = asmc::lidar::leveling_rotation(
      roll, pitch, asmc::lidar::Rot::Identity(), true, true);
  for (const auto& p : s.level.points) {
    const Eigen::Vector3d v = L.transpose() * Eigen::Vector3d(p.x, p.y, p.z);
    out->push_back(MakePoint(static_cast<float>(v.x()), static_cast<float>(v.y()),
                             static_cast<float>(v.z())));
  }
  return out;
}

struct Frame {
  TunnelFilterStageReport report;
  CloudPtr output{new Cloud};
  std::vector<int> indices;
};

Frame RunScan(TunnelFilterStage& stage, const Scene& s, const CloudPtr& voxel, float roll,
              float pitch) {
  Frame f;
  CommonOutput(s, f.output, &f.indices);
  f.report = stage.Apply(voxel, roll, pitch, f.output, f.indices);
  return f;
}

bool SameBits(const pcl::PointXYZI& a, const pcl::PointXYZI& b) {
  return std::memcmp(&a.x, &b.x, sizeof(float)) == 0 && std::memcmp(&a.y, &b.y, sizeof(float)) == 0 &&
         std::memcmp(&a.z, &b.z, sizeof(float)) == 0 &&
         std::memcmp(&a.intensity, &b.intensity, sizeof(float)) == 0;
}

// Survivors keep their order and their voxel-index pairing.
void ExpectConsistent(const Scene& s, const Frame& f) {
  ASSERT_EQ(f.output->points.size(), f.indices.size());
  EXPECT_EQ(f.output->points.size(), static_cast<std::size_t>(f.output->width));
  EXPECT_EQ(1u, f.output->height);
  for (std::size_t k = 0; k < f.indices.size(); ++k) {
    if (k > 0) {
      EXPECT_LT(f.indices[k - 1], f.indices[k]);
    }
    ASSERT_GE(f.indices[k], 0);
    ASSERT_LT(static_cast<std::size_t>(f.indices[k]), s.level.points.size());
    EXPECT_TRUE(SameBits(s.level.points[static_cast<std::size_t>(f.indices[k])], f.output->points[k]))
        << "k=" << k;
  }
}

int KeptPart(const Scene& s, const Frame& f, Part part) {
  int n = 0;
  for (const int i : f.indices)
    if (s.part[static_cast<std::size_t>(i)] == part) ++n;
  return n;
}

int CommonPart(const Scene& s, Part part) {
  int n = 0;
  for (std::size_t i = 0; i < s.part.size(); ++i)
    if (InCommonOutput(s, i) && s.part[i] == part) ++n;
  return n;
}

int Labelled(const tunnel_filter::FrameResult& r, std::uint8_t label) {
  int n = 0;
  for (const std::uint8_t l : r.labels)
    if (l == label) ++n;
  return n;
}

void Print(const char* name, int frame, float roll, float pitch, const TunnelFilterStageReport& r) {
  std::cout << "[tunnel-stage] " << name << " frame=" << frame << " roll=" << roll
            << " pitch=" << pitch << " ran=" << r.ran << " input=" << r.input_points
            << " ceiling=" << r.ceiling_labeled << " wall=" << r.wall_labeled
            << " ground=" << r.ground_labeled << " removed=" << r.removed_points
            << " core_ms=" << r.core_ms << std::endl;
}

}  // namespace

TEST(TunnelFilterStage, ConfigDefaultsOffIncludingPipelineConfig) {
  const TunnelFilterStageConfig config{};
  EXPECT_FALSE(config.enabled);
  EXPECT_TRUE(config.suppress_ceiling);
  EXPECT_TRUE(config.suppress_wall);
  const PipelineConfig pipeline{};
  EXPECT_FALSE(pipeline.tunnel.enabled);
  const TunnelFilterStageReport report{};
  EXPECT_FALSE(report.ran);
  EXPECT_EQ(0, report.removed_points);
}

TEST(TunnelFilterStage, OffReturnsNotRunAndLeavesOutputByteIdentical) {
  const Scene s = MakeTunnel();
  const CloudPtr voxel = SensorCloud(s, 0.f, 0.f);
  TunnelFilterStage stage(TunnelFilterStageConfig{});  // enabled=false
  Frame f;
  CommonOutput(s, f.output, &f.indices);
  f.output->header.frame_id = "lidar_leveled";
  f.output->header.stamp = 123456u;
  const Cloud before = *f.output;
  const std::vector<int> before_indices = f.indices;
  for (int frame = 0; frame < 3; ++frame) {
    const TunnelFilterStageReport r = stage.Apply(voxel, 0.03f, -0.02f, f.output, f.indices);
    EXPECT_FALSE(r.ran);
    EXPECT_EQ(0, r.input_points);
    EXPECT_EQ(0, r.ceiling_labeled);
    EXPECT_EQ(0, r.wall_labeled);
    EXPECT_EQ(0, r.ground_labeled);
    EXPECT_EQ(0, r.removed_points);
    EXPECT_EQ(0., r.core_ms);
  }
  ASSERT_EQ(before.points.size(), f.output->points.size());
  EXPECT_EQ(0, std::memcmp(before.points.data(), f.output->points.data(),
                           before.points.size() * sizeof(pcl::PointXYZI)));
  EXPECT_EQ(before.width, f.output->width);
  EXPECT_EQ(before.height, f.output->height);
  EXPECT_EQ(before.is_dense, f.output->is_dense);
  EXPECT_EQ(before.header.frame_id, f.output->header.frame_id);
  EXPECT_EQ(before.header.stamp, f.output->header.stamp);
  EXPECT_EQ(before_indices, f.indices);

  // OFF bypasses entirely: not even input validation runs.
  std::vector<int> mismatched{0, 1, 2};
  const float nan = std::numeric_limits<float>::quiet_NaN();
  TunnelFilterStageReport r;
  EXPECT_NO_THROW(r = stage.Apply(CloudPtr(), nan, nan, f.output, mismatched));
  EXPECT_FALSE(r.ran);
  EXPECT_EQ((std::vector<int>{0, 1, 2}), mismatched);
  EXPECT_EQ(before.points.size(), f.output->points.size());
}

TEST(TunnelFilterStage, OnRemovesCeilingAndWallKeepsBoxAndGroundLabels) {
  const Scene s = MakeTunnel();
  const CloudPtr voxel = SensorCloud(s, 0.f, 0.f);
  TunnelFilterStageConfig config;
  config.enabled = true;
  TunnelFilterStage stage(config);
  // Same core, same defaults, fed the same scans: the expected labels.
  tunnel_filter::TunnelPipeline reference;
  CloudPtr common(new Cloud);
  std::vector<int> common_indices;
  CommonOutput(s, common, &common_indices);
  const int box_points = CommonPart(s, kPartBox);
  const int strip_points = CommonPart(s, kPartGround);
  ASSERT_GT(box_points, 1000);
  ASSERT_GT(strip_points, 100);
  ASSERT_GT(CommonPart(s, kPartCeiling), 1000);
  ASSERT_GT(CommonPart(s, kPartWall), 1000);

  for (int frame = 1; frame <= 3; ++frame) {
    const Frame f = RunScan(stage, s, voxel, 0.f, 0.f);
    const tunnel_filter::FrameResult& ref =
        reference.Process(*voxel, 0.f, 0.f, nullptr, nullptr, false, nullptr);
    Print("level", frame, 0.f, 0.f, f.report);
    ASSERT_TRUE(f.report.ran);
    EXPECT_EQ(static_cast<int>(voxel->size()), f.report.input_points);
    EXPECT_EQ(Labelled(ref, tunnel_filter::kCeiling), f.report.ceiling_labeled);
    EXPECT_EQ(Labelled(ref, tunnel_filter::kWall), f.report.wall_labeled);
    EXPECT_EQ(Labelled(ref, tunnel_filter::kGround), f.report.ground_labeled);

    // Exactly the common-output points the core labelled ceiling/wall are gone.
    std::vector<int> expected;
    for (const int i : common_indices) {
      const std::uint8_t l = ref.labels[static_cast<std::size_t>(i)];
      if (l != tunnel_filter::kCeiling && l != tunnel_filter::kWall) expected.push_back(i);
    }
    EXPECT_EQ(expected, f.indices);
    EXPECT_EQ(static_cast<int>(common_indices.size() - f.indices.size()), f.report.removed_points);
    ExpectConsistent(s, f);

    EXPECT_GT(f.report.ceiling_labeled, 0);
    EXPECT_EQ(0, KeptPart(s, f, kPartCeiling)) << "frame " << frame;
    EXPECT_EQ(box_points, KeptPart(s, f, kPartBox)) << "frame " << frame;
    // kGround is only counted: the far ground strip stays in the output.
    EXPECT_GT(f.report.ground_labeled, 0);
    EXPECT_EQ(strip_points, KeptPart(s, f, kPartGround)) << "frame " << frame;
    // Wall points above the ceiling cut go every frame; the rest only after the
    // core's 3-frame wall persistence (no ego motion is passed).
    if (frame < 3) {
      EXPECT_EQ(0, f.report.wall_labeled) << "frame " << frame;
      EXPECT_GT(KeptPart(s, f, kPartWall), 0);
    } else {
      EXPECT_GT(f.report.wall_labeled, 0);
      EXPECT_EQ(0, KeptPart(s, f, kPartWall));
    }
  }
}

TEST(TunnelFilterStage, SameScansGiveSameOutputWithoutAnyGpsInput) {
  // Two independent stages fed the same scans (the API has no GPS input, see
  // the static_asserts above) produce byte-identical outputs on every frame.
  const Scene s = MakeTunnel();
  const CloudPtr voxel = SensorCloud(s, 0.f, 0.f);
  TunnelFilterStageConfig config;
  config.enabled = true;
  TunnelFilterStage a(config);
  TunnelFilterStage b(config);
  for (int frame = 1; frame <= 3; ++frame) {
    const Frame fa = RunScan(a, s, voxel, 0.f, 0.f);
    const Frame fb = RunScan(b, s, voxel, 0.f, 0.f);
    EXPECT_EQ(fa.indices, fb.indices);
    ASSERT_EQ(fa.output->points.size(), fb.output->points.size());
    for (std::size_t k = 0; k < fa.output->points.size(); ++k)
      EXPECT_TRUE(SameBits(fa.output->points[k], fb.output->points[k]));
    EXPECT_EQ(fa.report.ceiling_labeled, fb.report.ceiling_labeled);
    EXPECT_EQ(fa.report.wall_labeled, fb.report.wall_labeled);
    EXPECT_EQ(fa.report.ground_labeled, fb.report.ground_labeled);
    EXPECT_EQ(fa.report.removed_points, fb.report.removed_points);
  }
}

TEST(TunnelFilterStage, SuppressSwitchesSelectRemovedLabels) {
  const Scene s = MakeTunnel();
  const CloudPtr voxel = SensorCloud(s, 0.f, 0.f);
  TunnelFilterStageConfig keep_ceiling;
  keep_ceiling.enabled = true;
  keep_ceiling.suppress_ceiling = false;
  TunnelFilterStageConfig keep_wall;
  keep_wall.enabled = true;
  keep_wall.suppress_wall = false;
  TunnelFilterStage only_wall(keep_ceiling);
  TunnelFilterStage only_ceiling(keep_wall);
  tunnel_filter::TunnelPipeline reference;
  CloudPtr common(new Cloud);
  std::vector<int> common_indices;
  CommonOutput(s, common, &common_indices);
  for (int frame = 1; frame <= 3; ++frame) {
    const Frame fw = RunScan(only_wall, s, voxel, 0.f, 0.f);
    const Frame fc = RunScan(only_ceiling, s, voxel, 0.f, 0.f);
    const tunnel_filter::FrameResult& ref =
        reference.Process(*voxel, 0.f, 0.f, nullptr, nullptr, false, nullptr);
    int ceiling_in_common = 0;
    int wall_in_common = 0;
    for (const int i : common_indices) {
      const std::uint8_t l = ref.labels[static_cast<std::size_t>(i)];
      if (l == tunnel_filter::kCeiling) ++ceiling_in_common;
      if (l == tunnel_filter::kWall) ++wall_in_common;
    }
    // Labels are counted whatever the switches; only the selected ones are removed.
    EXPECT_EQ(Labelled(ref, tunnel_filter::kCeiling), fw.report.ceiling_labeled);
    EXPECT_EQ(Labelled(ref, tunnel_filter::kWall), fc.report.wall_labeled);
    EXPECT_EQ(wall_in_common, fw.report.removed_points);
    EXPECT_EQ(ceiling_in_common, fc.report.removed_points);
    EXPECT_GT(ceiling_in_common, 0);
    EXPECT_EQ(frame >= 3, wall_in_common > 0) << "frame " << frame;
    ExpectConsistent(s, fw);
    ExpectConsistent(s, fc);
  }
}

TEST(TunnelFilterStage, OnRejectsInconsistentInputsWithoutTouchingOutput) {
  const Scene s = MakeTunnel();
  const CloudPtr voxel = SensorCloud(s, 0.f, 0.f);
  TunnelFilterStageConfig config;
  config.enabled = true;
  TunnelFilterStage stage(config);
  Frame f;
  CommonOutput(s, f.output, &f.indices);
  const std::size_t size = f.indices.size();
  EXPECT_THROW(stage.Apply(CloudPtr(), 0.f, 0.f, f.output, f.indices), std::invalid_argument);
  EXPECT_THROW(stage.Apply(voxel, 0.f, 0.f, CloudPtr(), f.indices), std::invalid_argument);
  std::vector<int> short_indices(f.indices.begin(), f.indices.end() - 1);
  EXPECT_THROW(stage.Apply(voxel, 0.f, 0.f, f.output, short_indices), std::invalid_argument);
  std::vector<int> bad = f.indices;
  bad.back() = static_cast<int>(voxel->size());
  EXPECT_THROW(stage.Apply(voxel, 0.f, 0.f, f.output, bad), std::invalid_argument);
  bad.back() = -1;
  EXPECT_THROW(stage.Apply(voxel, 0.f, 0.f, f.output, bad), std::invalid_argument);
  EXPECT_EQ(size, f.output->points.size());
  EXPECT_EQ(size, f.indices.size());
  ExpectConsistent(s, f);
  // The rejected calls did not advance the core: three valid calls are still
  // needed before walls are removed.
  for (int frame = 1; frame <= 3; ++frame) {
    const Frame g = RunScan(stage, s, voxel, 0.f, 0.f);
    EXPECT_EQ(frame >= 3, g.report.wall_labeled > 0) << "frame " << frame;
  }
}

TEST(TunnelFilterStage, RollPitchSmokeRunsAndReportsCounts) {
  // The scan is tilted in the sensor frame; the core levels it with the given
  // roll/pitch. Withheld or swapped angles leave a residual tilt that moves wall
  // points across the 2.55 m ceiling cut, so the ceiling count changes.
  const Scene s = MakeTunnel();
  const float roll = 0.04f;
  const float pitch = 0.08f;
  const CloudPtr level_voxel = SensorCloud(s, 0.f, 0.f);
  const CloudPtr tilted_voxel = SensorCloud(s, roll, pitch);
  ASSERT_EQ(level_voxel->size(), tilted_voxel->size());
  TunnelFilterStageConfig config;
  config.enabled = true;
  TunnelFilterStage level_stage(config);
  TunnelFilterStage tilted_stage(config);
  TunnelFilterStage unleveled_stage(config);  // tilted scan, roll/pitch withheld
  TunnelFilterStage swapped_stage(config);    // tilted scan, roll and pitch swapped
  const int box_points = CommonPart(s, kPartBox);
  for (int frame = 1; frame <= 3; ++frame) {
    const Frame fl = RunScan(level_stage, s, level_voxel, 0.f, 0.f);
    const Frame ft = RunScan(tilted_stage, s, tilted_voxel, roll, pitch);
    const Frame fu = RunScan(unleveled_stage, s, tilted_voxel, 0.f, 0.f);
    const Frame fs = RunScan(swapped_stage, s, tilted_voxel, pitch, roll);
    Print("level", frame, 0.f, 0.f, fl.report);
    Print("tilted", frame, roll, pitch, ft.report);
    Print("tilted_unleveled", frame, 0.f, 0.f, fu.report);
    Print("tilted_swapped", frame, pitch, roll, fs.report);
    EXPECT_NE(fl.report.ceiling_labeled, fs.report.ceiling_labeled) << "frame " << frame;
    const std::string suffix = "_f" + std::to_string(frame);
    RecordProperty("tilted_ceiling" + suffix, ft.report.ceiling_labeled);
    RecordProperty("tilted_wall" + suffix, ft.report.wall_labeled);
    RecordProperty("tilted_ground" + suffix, ft.report.ground_labeled);
    RecordProperty("tilted_removed" + suffix, ft.report.removed_points);
    ASSERT_TRUE(ft.report.ran);
    EXPECT_EQ(static_cast<int>(tilted_voxel->size()), ft.report.input_points);
    ExpectConsistent(s, ft);
    EXPECT_EQ(fl.report.ceiling_labeled, ft.report.ceiling_labeled) << "frame " << frame;
    EXPECT_NE(fl.report.ceiling_labeled, fu.report.ceiling_labeled) << "frame " << frame;
    EXPECT_EQ(box_points, KeptPart(s, ft, kPartBox)) << "frame " << frame;
    EXPECT_EQ(0, KeptPart(s, ft, kPartCeiling)) << "frame " << frame;
    if (frame >= 3) {
      EXPECT_GT(ft.report.wall_labeled, 0);
    }
  }
}

int main(int argc, char** argv) { testing::InitGoogleTest(&argc, argv); return RUN_ALL_TESTS(); }
