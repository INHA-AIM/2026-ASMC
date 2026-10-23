// StageObserver hook of Pipeline::Process: zero effect on the results and
// consistent read-only stage views (monitoring contract).
#include <asmc_lidar/pipeline/pipeline.hpp>
#include <asmc_lidar/pipeline/stage_observer.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <set>
#include <string>
#include <vector>

using namespace asmc::lidar::pipeline;
namespace al = asmc::lidar;

namespace {

constexpr std::int64_t kT0 = 1700000000123456789LL;
constexpr std::int64_t kPeriod = 100000000LL;  // 10 Hz
constexpr float kGroundZ = -1.45f;

VehicleSpec Vehicle() {
  VehicleSpec v;
  v.f32_Width = 1.892f;
  v.f32_Length = 4.635f;
  v.f32_WheelBase = 3.f;
  v.f32_FrontOverhang = 0.845f;
  v.f32_RearOverhang = 0.700f;
  v.st_LiDARPos = {0.80f, 0.f, 1.45f};
  v.st_GPSPos = {0.35f, 0.f, 1.25f};
  return v;
}

PipelineConfig Config(StageObserver* observer, bool debug, bool tunnel = false) {
  PipelineConfig c;
  c.vehicle = Vehicle();
  c.calibration_id = "test_stage_observer";
  c.capture_debug = debug;
  c.observer = observer;
  c.tunnel.enabled = tunnel;
  return c;
}

pcl::PointXYZI Point(float x, float y, float z, float intensity) {
  pcl::PointXYZI p;
  p.x = x;
  p.y = y;
  p.z = z;
  p.intensity = intensity;
  return p;
}

struct Box {
  float cx, cy, length, width;
};

void AddBox(Cloud& cloud, const Box& b) {
  const float step = 0.15f;
  const int nl = static_cast<int>(std::lround(b.length / step));
  const int nw = static_cast<int>(std::lround(b.width / step));
  const int nh = 10;
  const float x0 = b.cx - 0.5f * b.length, y0 = b.cy - 0.5f * b.width;
  for (int k = 0; k <= nh; ++k) {
    const float z = kGroundZ + 0.05f + step * k;
    for (int i = 0; i <= nl; ++i) {
      const float x = x0 + step * i;
      cloud.points.push_back(Point(x, y0, z, 20.f + (i + k) % 7));
      cloud.points.push_back(Point(x, y0 + step * nw, z, 30.f + (i + k) % 5));
    }
    for (int j = 1; j < nw; ++j) {
      const float y = y0 + step * j;
      cloud.points.push_back(Point(x0, y, z, 40.f + (j + k) % 3));
      cloud.points.push_back(Point(x0 + step * nl, y, z, 50.f + (j + k) % 4));
    }
  }
}

CloudPtr Scene(int k) {
  CloudPtr cloud(new Cloud);
  for (int ix = 0; ix <= 120; ++ix) {
    for (int iy = 0; iy <= 60; ++iy) {
      const float x = -20.f + 0.5f * ix, y = -15.f + 0.5f * iy;
      cloud->points.push_back(Point(x, y, kGroundZ + 0.01f * std::sin(0.7f * x + 0.3f * y), float((7 * ix + 3 * iy) % 50)));
    }
  }
  AddBox(*cloud, {10.f + 0.5f * k, 3.f, 4.2f, 1.8f});
  if (k < 4) AddBox(*cloud, {16.f, -5.f, 2.1f, 2.1f});
  cloud->width = static_cast<std::uint32_t>(cloud->points.size());
  cloud->height = 1;
  cloud->is_dense = false;
  return cloud;
}

LocalState Local(int k) {
  LocalState s;
  s.st_IMU.f32_Roll_rad = 0.02f;
  s.st_IMU.f32_Pitch_rad = -0.015f;
  s.st_IMU.f32_Yaw_rad = 0.3f + 0.01f * k;
  s.st_IMU.b_ImuReady = true;
  s.st_CAN.f32_CurrentVel_MPS = 2.f;
  s.st_CAN.f32_CurrentVel_KPH = 7.2f;
  return s;
}

ScanInput Input(int k, CloudPtr cloud = nullptr) {
  ScanInput in;
  const std::int64_t stamp = kT0 + k * kPeriod;
  in.time = al::Observation{stamp, 1000 + k * 100, 1000 + k * 100, "ros", "test", "steady"};
  in.points = cloud ? cloud : Scene(k);
  in.scan_sequence = static_cast<std::uint64_t>(k + 1);
  in.generation = 1;
  return in;
}

// Records what the observer saw and checks the view invariants while it is valid.
class Recording : public StageObserver {
 public:
  explicit Recording(bool accept) : accept_(accept) {}
  bool Begin(std::int64_t stamp_ns) override {
    begins.push_back(stamp_ns);
    return accept_;
  }
  void OnScan(const StageView& view) override {
    Seen s;
    const auto& n = view.result.counts;
    s.stamp_ns = view.result.stamp_ns;
    s.raw = view.scan.points ? view.scan.points->size() : 0;
    s.roi = view.roi.size();
    s.voxel = view.voxel.size();
    s.leveled = view.leveled.size();
    s.cluster_input = view.cluster_input.size();
    s.cluster_input_indices = view.cluster_input_indices;
    s.ground_kept_indices = view.ground_kept_indices;
    s.clusters = view.clusters.size();
    s.detections = view.detections.size();
    s.leveled_digest = CloudDigest(view.leveled);
    s.counts = n;
    s.roll = view.local.st_IMU.f32_Roll_rad;
    s.tunnel_ran = view.result.tunnel.ran;
    // Every clustering input point is a leveled point (same coordinates).
    const std::size_t real = static_cast<std::size_t>(n.cluster_input_points);
    s.indices_consistent = real <= view.cluster_input.size() && real == view.cluster_input_indices.size();
    for (std::size_t i = 0; s.indices_consistent && i < real; ++i) {
      const int li = view.cluster_input_indices[i];
      if (li < 0 || static_cast<std::size_t>(li) >= view.leveled.size()) {
        s.indices_consistent = false;
        break;
      }
      const auto& a = view.cluster_input[i];
      const auto& b = view.leveled[static_cast<std::size_t>(li)];
      if (a.x != b.x || a.y != b.y || a.z != b.z) s.indices_consistent = false;
    }
    s.clusters_in_range = true;
    for (const auto& c : view.clusters)
      for (const int j : c.indices)
        if (j < 0 || static_cast<std::size_t>(j) >= real) s.clusters_in_range = false;
    if (!view.detections.empty()) s.first_detection_center_x = view.detections.front().detection.f32_OBB_CenterX;
    seen.push_back(s);
  }
  struct Seen {
    std::int64_t stamp_ns = 0;
    std::size_t raw = 0, roi = 0, voxel = 0, leveled = 0, cluster_input = 0, clusters = 0, detections = 0;
    std::vector<int> cluster_input_indices, ground_kept_indices;
    std::string leveled_digest;
    StageCounts counts;
    float roll = 0.f;
    bool tunnel_ran = false;
    bool indices_consistent = false, clusters_in_range = false;
    float first_detection_center_x = 0.f;
  };
  std::vector<std::int64_t> begins;
  std::vector<Seen> seen;

 private:
  bool accept_;
};

void ExpectSameResult(const ScanResult& a, const ScanResult& b) {
  ASSERT_EQ(a.processed, b.processed);
  EXPECT_EQ(a.reason, b.reason);
  EXPECT_EQ(a.stamp_ns, b.stamp_ns);
  EXPECT_EQ(a.dt_s, b.dt_s);
  EXPECT_EQ(a.digests.roi, b.digests.roi);
  EXPECT_EQ(a.digests.voxel, b.digests.voxel);
  EXPECT_EQ(a.digests.leveled, b.digests.leveled);
  EXPECT_EQ(a.digests.ground_kept, b.digests.ground_kept);
  EXPECT_EQ(a.counts.raw_points, b.counts.raw_points);
  EXPECT_EQ(a.counts.roi_points, b.counts.roi_points);
  EXPECT_EQ(a.counts.voxel_points, b.counts.voxel_points);
  EXPECT_EQ(a.counts.leveled_points, b.counts.leveled_points);
  EXPECT_EQ(a.counts.ground_kept_points, b.counts.ground_kept_points);
  EXPECT_EQ(a.counts.cluster_input_points, b.counts.cluster_input_points);
  EXPECT_EQ(a.counts.clusters, b.counts.clusters);
  EXPECT_EQ(a.counts.detections, b.counts.detections);
  EXPECT_EQ(a.counts.tracks, b.counts.tracks);
  EXPECT_EQ(a.counts.observed_tracks, b.counts.observed_tracks);
  EXPECT_EQ(a.counts.predicted_tracks, b.counts.predicted_tracks);
  EXPECT_EQ(a.plane.valid, b.plane.valid);
  EXPECT_EQ(a.plane.coefficients, b.plane.coefficients);
  EXPECT_TRUE(a.context.applied_leveling == b.context.applied_leveling);
  ASSERT_EQ(a.objects.size(), b.objects.size());
  for (std::size_t i = 0; i < a.objects.size(); ++i) {
    const auto& x = a.objects[i];
    const auto& y = b.objects[i];
    EXPECT_EQ(x.tracking.s32_Id, y.tracking.s32_Id);
    EXPECT_EQ(x.tracking.s32_Age, y.tracking.s32_Age);
    EXPECT_EQ(x.tracking.s32_MissCount, y.tracking.s32_MissCount);
    EXPECT_EQ(x.detection.u16_Idx, y.detection.u16_Idx);
    EXPECT_EQ(x.detection.f32_OBB_CenterX, y.detection.f32_OBB_CenterX);
    EXPECT_EQ(x.detection.f32_OBB_CenterY, y.detection.f32_OBB_CenterY);
    EXPECT_EQ(x.detection.f32_OBB_CenterZ, y.detection.f32_OBB_CenterZ);
    EXPECT_EQ(x.detection.f32_OBB_YawRad, y.detection.f32_OBB_YawRad);
    EXPECT_EQ(x.detection.obb_length_m, y.detection.obb_length_m);
    EXPECT_EQ(x.detection.obb_width_m, y.detection.obb_width_m);
    EXPECT_EQ(x.detection.pcl_ClusterPoints.size(), y.detection.pcl_ClusterPoints.size());
    for (unsigned c = 0; c < 8; ++c) {
      EXPECT_EQ(x.detection.obb_corners_level[c].x, y.detection.obb_corners_level[c].x);
      EXPECT_EQ(x.detection.obb_corners_level[c].y, y.detection.obb_corners_level[c].y);
      EXPECT_EQ(x.detection.obb_corners_level[c].z, y.detection.obb_corners_level[c].z);
    }
    EXPECT_TRUE(x.tracking.Eigen_KfState_CA == y.tracking.Eigen_KfState_CA);
  }
}

}  // namespace

TEST(StageObserver, NullRefusingAndAcceptingObserversGiveIdenticalResults) {
  Recording refusing(false), accepting(true);
  Pipeline none(Config(nullptr, true));
  Pipeline refused(Config(&refusing, true));
  Pipeline observed(Config(&accepting, true));
  for (int k = 0; k < 6; ++k) {
    const ScanInput in = Input(k);
    const LocalState local = Local(k);
    const ScanResult a = none.Process(in, local, al::PoseSelection{});
    const ScanResult b = refused.Process(in, local, al::PoseSelection{});
    const ScanResult c = observed.Process(in, local, al::PoseSelection{});
    ASSERT_TRUE(a.processed);
    ExpectSameResult(a, b);
    ExpectSameResult(a, c);
    EXPECT_GT(a.counts.tracks, 0);
  }
  EXPECT_EQ(refusing.begins.size(), 6u);
  EXPECT_TRUE(refusing.seen.empty());
  EXPECT_EQ(accepting.begins.size(), 6u);
  EXPECT_EQ(accepting.seen.size(), 6u);
}

TEST(StageObserver, ViewMatchesCountsAndIndicesWithoutCaptureDebug) {
  Recording accepting(true);
  Pipeline observed(Config(&accepting, false));
  for (int k = 0; k < 3; ++k) {
    const ScanResult r = observed.Process(Input(k), Local(k), al::PoseSelection{});
    ASSERT_TRUE(r.processed);
    EXPECT_TRUE(r.detections.empty());  // capture_debug stays off
    ASSERT_EQ(accepting.seen.size(), static_cast<std::size_t>(k + 1));
    const auto& s = accepting.seen.back();
    EXPECT_EQ(s.stamp_ns, r.stamp_ns);
    EXPECT_EQ(s.raw, static_cast<std::size_t>(r.counts.raw_points));
    EXPECT_EQ(s.roi, static_cast<std::size_t>(r.counts.roi_points));
    EXPECT_EQ(s.voxel, static_cast<std::size_t>(r.counts.voxel_points));
    EXPECT_EQ(s.leveled, static_cast<std::size_t>(r.counts.leveled_points));
    EXPECT_EQ(s.voxel, s.leveled);
    EXPECT_GE(s.cluster_input, static_cast<std::size_t>(r.counts.cluster_input_points));
    EXPECT_LE(s.cluster_input, static_cast<std::size_t>(r.counts.cluster_input_points) + 1);  // dummy point
    EXPECT_EQ(s.clusters, static_cast<std::size_t>(r.counts.clusters));
    EXPECT_EQ(s.detections, static_cast<std::size_t>(r.counts.detections));
    EXPECT_TRUE(s.indices_consistent);
    EXPECT_TRUE(s.clusters_in_range);
    EXPECT_EQ(s.ground_kept_indices, s.cluster_input_indices);  // tunnel OFF
    EXPECT_EQ(s.roll, 0.02f);
    EXPECT_FALSE(s.tunnel_ran);
    EXPECT_GT(s.detections, 0u);
  }
}

TEST(StageObserver, LeveledViewEqualsResultDigest) {
  Recording accepting(true);
  Pipeline observed(Config(&accepting, true));
  const ScanResult r = observed.Process(Input(0), Local(0), al::PoseSelection{});
  ASSERT_TRUE(r.processed);
  ASSERT_EQ(accepting.seen.size(), 1u);
  EXPECT_EQ(accepting.seen.front().leveled_digest, r.digests.leveled);
}

TEST(StageObserver, BeginOnlyForScansThatReachTheStages) {
  Recording accepting(true);
  Pipeline observed(Config(&accepting, false));
  CloudPtr empty(new Cloud);
  const ScanResult e = observed.Process(Input(0, empty), Local(0), al::PoseSelection{});
  EXPECT_FALSE(e.processed);
  EXPECT_EQ(e.reason, "empty_scan");
  EXPECT_TRUE(accepting.begins.empty());
  const ScanResult dup = observed.Process(Input(0), Local(0), al::PoseSelection{});
  EXPECT_FALSE(dup.processed);
  EXPECT_EQ(dup.reason, "duplicate_or_regressing_stamp");
  EXPECT_TRUE(accepting.begins.empty());
  const ScanResult ok = observed.Process(Input(1), Local(1), al::PoseSelection{});
  EXPECT_TRUE(ok.processed);
  ASSERT_EQ(accepting.begins.size(), 1u);
  EXPECT_EQ(accepting.begins.front(), ok.stamp_ns);
  EXPECT_EQ(accepting.seen.size(), 1u);
}

TEST(StageObserver, TunnelStageExposesPreAndPostIndices) {
  Recording accepting(true);
  Pipeline observed(Config(&accepting, true, true));
  Pipeline plain(Config(nullptr, true, true));
  for (int k = 0; k < 2; ++k) {
    const ScanResult a = plain.Process(Input(k), Local(k), al::PoseSelection{});
    const ScanResult b = observed.Process(Input(k), Local(k), al::PoseSelection{});
    ASSERT_TRUE(b.processed);
    ExpectSameResult(a, b);
    const auto& s = accepting.seen.back();
    EXPECT_TRUE(s.tunnel_ran);
    EXPECT_EQ(s.ground_kept_indices.size(), static_cast<std::size_t>(b.counts.ground_kept_points));
    EXPECT_EQ(s.cluster_input_indices.size(), static_cast<std::size_t>(b.counts.cluster_input_points));
    const std::set<int> pre(s.ground_kept_indices.begin(), s.ground_kept_indices.end());
    for (const int i : s.cluster_input_indices) EXPECT_TRUE(pre.count(i));
    EXPECT_EQ(static_cast<int>(s.ground_kept_indices.size() - s.cluster_input_indices.size()),
              b.counts.tunnel_removed_points);
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
