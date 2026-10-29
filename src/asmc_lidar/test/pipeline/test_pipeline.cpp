#include <asmc_lidar/tracking/track.hpp>
// Pipeline::Process orchestration and CloudDigest (Main LidarProcess common path).
#include <asmc_lidar/pipeline/pipeline.hpp>
#include <asmc_lidar/geometry/projection.hpp>

#include <gtest/gtest.h>
#include <ros/time.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

using namespace asmc::lidar::pipeline;
namespace al = asmc::lidar;

namespace {

constexpr std::int64_t kT0 = 1700000000123456789LL;
constexpr std::int64_t kPeriod = 100000000LL;  // 10 Hz
constexpr float kGroundZ = -1.45f;             // sensor frame, LiDAR 1.45 m above base_link
constexpr const char* kEmptySha = "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855";

VehicleSpec Vehicle() {
  VehicleSpec v;  // setting.yaml Vehicle.*
  v.f32_Width = 1.892f;
  v.f32_Length = 4.635f;
  v.f32_WheelBase = 3.f;
  v.f32_FrontOverhang = 0.845f;
  v.f32_RearOverhang = 0.700f;
  v.st_LiDARPos = {0.80f, 0.f, 1.45f};
  v.st_GPSPos = {0.35f, 0.f, 1.25f};
  return v;
}

PipelineConfig Config(bool use_roll = true, bool debug = false) {
  PipelineConfig c;
  c.vehicle = Vehicle();
  c.calibration_id = "test_autonav_v3";
  c.use_roll = use_roll;
  c.capture_debug = debug;
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

// Hollow box (four walls + roof) on a 0.15 m lattice from 0.05 m above ground.
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
  for (int i = 1; i < nl; ++i)
    for (int j = 1; j < nw; ++j)
      cloud.points.push_back(Point(x0 + step * i, y0 + step * j, kGroundZ + 0.05f + step * nh, 60.f));
}

CloudPtr Scene(const std::vector<Box>& boxes, bool ground = true) {
  CloudPtr cloud(new Cloud);
  if (ground) {
    for (int ix = 0; ix <= 120; ++ix) {
      for (int iy = 0; iy <= 60; ++iy) {
        const float x = -20.f + 0.5f * ix, y = -15.f + 0.5f * iy;
        cloud->points.push_back(
            Point(x, y, kGroundZ + 0.01f * std::sin(0.7f * x + 0.3f * y), float((7 * ix + 3 * iy) % 50)));
      }
    }
  }
  for (const auto& box : boxes) AddBox(*cloud, box);
  cloud->width = static_cast<std::uint32_t>(cloud->points.size());
  cloud->height = 1;
  cloud->is_dense = false;
  return cloud;
}

// Scan k of the standard sequence: A drives away along +x, B is static and
// leaves the scene from scan 4 on.
CloudPtr SequenceScene(int k) {
  std::vector<Box> boxes{{10.f + 0.5f * k, 3.f, 4.2f, 1.8f}};
  if (k < 4) boxes.push_back({16.f, -5.f, 2.1f, 2.1f});
  return Scene(boxes);
}

LocalState Local(int k, float roll = 0.02f, float pitch = -0.015f) {
  LocalState s;
  s.st_IMU.f32_Roll_rad = roll;
  s.st_IMU.f32_Pitch_rad = pitch;
  s.st_IMU.f32_Yaw_rad = 0.3f + 0.01f * k;
  s.st_IMU.f32_Roll_deg = roll * 57.29578f;
  s.st_IMU.f32_Pitch_deg = pitch * 57.29578f;
  s.st_IMU.f32_Yaw_deg = s.st_IMU.f32_Yaw_rad * 57.29578f;
  s.st_IMU.b_ImuReady = true;
  s.st_CAN.f32_CurrentVel_MPS = 2.f;
  s.st_CAN.f32_CurrentVel_KPH = 7.2f;
  return s;
}

// GPS fields as Main's TemporalThread leaves them for a valid / invalid map pose.
LocalState WithGps(LocalState s, bool valid) {
  if (valid) {
    s.st_GPS.f64_E = 101.25;
    s.st_GPS.f64_N = -204.5;
    s.st_GPS.f64_U = 1.25;
    s.st_GPS.b_isJamming = false;
    s.b_Jamming = false;
  } else {
    s.st_GPS.f64_E = s.st_GPS.f64_N = s.st_GPS.f64_U = std::numeric_limits<double>::quiet_NaN();
    s.st_GPS.b_isJamming = true;
    s.b_Jamming = true;
  }
  return s;
}

al::PoseSelection MapPose(bool valid, std::int64_t stamp) {
  al::PoseSelection m;
  m.query_stamp_ns = stamp;
  m.clock = "ros";
  if (!valid) {
    m.reason = "gps_missing";
    return m;
  }
  m.valid = true;
  m.reason = "";
  m.map_from_base = al::Pose::Identity();
  m.map_from_base.linear() = al::rpy(0.0, 0.0, 0.3);
  m.map_from_base.translation() = al::Vec(100.0, -205.0, 0.5);
  m.sample_stamps = {stamp - 20000000};
  m.sample_arrivals = {stamp - 15000000};
  return m;
}

ScanInput Scan(std::int64_t stamp, CloudPtr cloud, std::uint64_t scan_sequence) {
  ScanInput s;
  s.time.stamp_ns = stamp;
  s.time.receipt_ns = 5000000000LL + (stamp - kT0) + 2000000;
  s.time.processing_ns = s.time.receipt_ns + 1000000;
  s.time.clock = "ros";
  s.time.stamp_provenance = "bridge_receive_ns";
  s.time.receipt_clock = "steady";
  s.points = std::move(cloud);
  s.scan_sequence = scan_sequence;
  return s;
}

template <class T>
bool SameBits(const T& a, const T& b) {
  return std::memcmp(&a, &b, sizeof(T)) == 0;
}

template <class M>
bool SameMatrix(const M& a, const M& b) {
  if (a.rows() != b.rows() || a.cols() != b.cols()) return false;
  for (int r = 0; r < a.rows(); ++r)
    for (int c = 0; c < a.cols(); ++c)
      if (!SameBits(a(r, c), b(r, c))) return false;
  return true;
}

#define EXPECT_SAME_FIELD(a, b, f) \
  EXPECT_TRUE(SameBits((a).f, (b).f)) << where << " " #f ": " << (a).f << " vs " << (b).f

void ExpectSameObject(const TrackedObject& a, const TrackedObject& b, const std::string& where) {
  EXPECT_EQ(a.tracking.s32_Id, b.tracking.s32_Id) << where;
  EXPECT_EQ(a.tracking.s32_Age, b.tracking.s32_Age) << where;
  EXPECT_EQ(a.tracking.s32_MissCount, b.tracking.s32_MissCount) << where;
  EXPECT_EQ(a.tracking.s32_KfInitCount, b.tracking.s32_KfInitCount) << where;
  EXPECT_EQ(a.tracking.s32_DynamicCount, b.tracking.s32_DynamicCount) << where;
  EXPECT_EQ(a.tracking.s32_StaticCount, b.tracking.s32_StaticCount) << where;
  EXPECT_EQ(a.tracking.b_KfInitialized, b.tracking.b_KfInitialized) << where;
  EXPECT_EQ(a.detection.u16_Idx, b.detection.u16_Idx) << where;
  EXPECT_EQ(a.camera.str_ObjectClass, b.camera.str_ObjectClass) << where;
  EXPECT_EQ(a.camera.str_YoloClass, b.camera.str_YoloClass) << where;
  EXPECT_EQ(a.camera.continuity_counter, b.camera.continuity_counter) << where;
  EXPECT_EQ(a.camera.class_miss_counter, b.camera.class_miss_counter) << where;
  EXPECT_SAME_FIELD(a, b, detection.f32_CoreX);
  EXPECT_SAME_FIELD(a, b, detection.f32_CoreY);
  EXPECT_SAME_FIELD(a, b, detection.f32_CoreZ);
  EXPECT_SAME_FIELD(a, b, tracking.f32_PredCoreX);
  EXPECT_SAME_FIELD(a, b, tracking.f32_PredCoreY);
  EXPECT_SAME_FIELD(a, b, tracking.f32_PrevCoreX);
  EXPECT_SAME_FIELD(a, b, tracking.f32_PrevCoreY);
  EXPECT_SAME_FIELD(a, b, tracking.f32_PrevYaw);
  EXPECT_SAME_FIELD(a, b, detection.f32_Dist);
  EXPECT_SAME_FIELD(a, b, detection.f32_MinX);
  EXPECT_SAME_FIELD(a, b, detection.f32_MinY);
  EXPECT_SAME_FIELD(a, b, detection.f32_MinZ);
  EXPECT_SAME_FIELD(a, b, detection.f32_MaxX);
  EXPECT_SAME_FIELD(a, b, detection.f32_MaxY);
  EXPECT_SAME_FIELD(a, b, detection.f32_MaxZ);
  EXPECT_SAME_FIELD(a, b, detection.f32_OBB_CenterX);
  EXPECT_SAME_FIELD(a, b, detection.f32_OBB_CenterY);
  EXPECT_SAME_FIELD(a, b, detection.f32_OBB_CenterZ);
  EXPECT_SAME_FIELD(a, b, detection.obb_length_m);
  EXPECT_SAME_FIELD(a, b, detection.obb_width_m);
  EXPECT_SAME_FIELD(a, b, detection.f32_OBB_ZMin);
  EXPECT_SAME_FIELD(a, b, detection.f32_OBB_ZMax);
  EXPECT_SAME_FIELD(a, b, detection.f32_OBB_YawRad);
  EXPECT_SAME_FIELD(a, b, tracking.f32_Velocity);
  EXPECT_SAME_FIELD(a, b, tracking.f32_AbsVx);
  EXPECT_SAME_FIELD(a, b, tracking.f32_AbsVy);
  EXPECT_SAME_FIELD(a, b, tracking.f32_DirX);
  EXPECT_SAME_FIELD(a, b, tracking.f32_DirY);
  for (int i = 0; i < 8; ++i) {
    EXPECT_SAME_FIELD(a, b, detection.obb_corners_level[i].x);
    EXPECT_SAME_FIELD(a, b, detection.obb_corners_level[i].y);
    EXPECT_SAME_FIELD(a, b, detection.obb_corners_level[i].z);
  }
  EXPECT_TRUE(SameMatrix(a.tracking.Eigen_KfState_CA, b.tracking.Eigen_KfState_CA)) << where << " kf state";
  EXPECT_TRUE(SameMatrix(a.tracking.Eigen_KfP_CA, b.tracking.Eigen_KfP_CA)) << where << " kf P";
  EXPECT_EQ(a.tracking.tracking_frame_valid, b.tracking.tracking_frame_valid) << where;
  EXPECT_SAME_FIELD(a, b, tracking.tracking_height_input);
  EXPECT_TRUE(SameMatrix(a.tracking.tracking_yaw_from_input, b.tracking.tracking_yaw_from_input)) << where;
  EXPECT_TRUE(SameMatrix(a.tracking.tracking_origin_in_yaw, b.tracking.tracking_origin_in_yaw)) << where;
  EXPECT_EQ(a.detection.ground_evidence.valid, b.detection.ground_evidence.valid) << where;
  EXPECT_EQ(a.detection.ground_evidence.sample_count, b.detection.ground_evidence.sample_count) << where;
  EXPECT_SAME_FIELD(a, b, detection.ground_evidence.median_clearance_m);
  EXPECT_SAME_FIELD(a, b, detection.ground_evidence.rejection_band_m);
  EXPECT_EQ(a.detection.pcl_ClusterPoints.size(), b.detection.pcl_ClusterPoints.size()) << where;
  EXPECT_EQ(CloudDigest(a.detection.pcl_ClusterPoints), CloudDigest(b.detection.pcl_ClusterPoints)) << where;
}

void ExpectSameObjects(const std::vector<TrackedObject>& a, const std::vector<TrackedObject>& b,
                       const std::string& where) {
  ASSERT_EQ(a.size(), b.size()) << where;
  for (std::size_t i = 0; i < a.size(); ++i) ExpectSameObject(a[i], b[i], where + " object " + std::to_string(i));
}

void ExpectSameCounts(const StageCounts& a, const StageCounts& b, const std::string& where,
                      bool include_consolidated = true) {
  EXPECT_EQ(a.raw_points, b.raw_points) << where;
  EXPECT_EQ(a.roi_points, b.roi_points) << where;
  EXPECT_EQ(a.voxel_points, b.voxel_points) << where;
  EXPECT_EQ(a.leveled_points, b.leveled_points) << where;
  EXPECT_EQ(a.ground_kept_points, b.ground_kept_points) << where;
  EXPECT_EQ(a.tunnel_removed_points, b.tunnel_removed_points) << where;
  EXPECT_EQ(a.cluster_input_points, b.cluster_input_points) << where;
  EXPECT_EQ(a.clusters, b.clusters) << where;
  EXPECT_EQ(a.centroids, b.centroids) << where;
  EXPECT_EQ(a.detections, b.detections) << where;
  if (include_consolidated) {
    EXPECT_EQ(a.consolidated_detections, b.consolidated_detections) << where;
  }
  EXPECT_EQ(a.tracks, b.tracks) << where;
  EXPECT_EQ(a.observed_tracks, b.observed_tracks) << where;
  EXPECT_EQ(a.predicted_tracks, b.predicted_tracks) << where;
}

// Everything except the map pose and wall-clock timings.
void ExpectSameLocalResult(const ScanResult& a, const ScanResult& b, const std::string& where) {
  EXPECT_EQ(a.processed, b.processed) << where;
  EXPECT_EQ(a.reason, b.reason) << where;
  EXPECT_EQ(a.stamp_ns, b.stamp_ns) << where;
  EXPECT_EQ(a.previous_stamp_ns, b.previous_stamp_ns) << where;
  EXPECT_TRUE(SameBits(a.dt_s, b.dt_s)) << where;
  ExpectSameCounts(a.counts, b.counts, where);
  EXPECT_EQ(a.plane.valid, b.plane.valid) << where;
  for (int i = 0; i < 4; ++i) EXPECT_TRUE(SameBits(a.plane.coefficients[i], b.plane.coefficients[i])) << where;
  EXPECT_EQ(a.context.frame, b.context.frame) << where;
  EXPECT_EQ(a.context.scan_sequence, b.context.scan_sequence) << where;
  EXPECT_EQ(a.context.time.stamp_ns, b.context.time.stamp_ns) << where;
  EXPECT_EQ(a.context.calibration_source, b.context.calibration_source) << where;
  EXPECT_EQ(a.context.leveling_source, b.context.leveling_source) << where;
  EXPECT_TRUE(SameMatrix(a.context.applied_leveling, b.context.applied_leveling)) << where;
  EXPECT_TRUE(SameMatrix(a.context.base_from_sensor.matrix(), b.context.base_from_sensor.matrix())) << where;
  EXPECT_TRUE(SameMatrix(a.context.ego_linear_base, b.context.ego_linear_base)) << where;
  EXPECT_EQ(a.digests.roi, b.digests.roi) << where;
  EXPECT_EQ(a.digests.voxel, b.digests.voxel) << where;
  EXPECT_EQ(a.digests.leveled, b.digests.leveled) << where;
  EXPECT_EQ(a.digests.ground_kept, b.digests.ground_kept) << where;
  EXPECT_EQ(a.cluster_sizes, b.cluster_sizes) << where;
  ExpectSameObjects(a.detections, b.detections, where + " detections");
  ExpectSameObjects(a.objects, b.objects, where + " tracks");
}

void ExpectNotProcessed(const ScanResult& r, const std::string& reason, const std::string& where) {
  EXPECT_FALSE(r.processed) << where;
  EXPECT_EQ(r.reason, reason) << where;
  EXPECT_TRUE(r.objects.empty()) << where;
  EXPECT_EQ(r.counts.raw_points, 0) << where;
  EXPECT_EQ(r.counts.tracks, 0) << where;
}

std::vector<ScanResult> RunSequence(Pipeline& pipeline, int scans, bool map_valid = true) {
  std::vector<ScanResult> out;
  for (int k = 0; k < scans; ++k) {
    const std::int64_t stamp = kT0 + k * kPeriod;
    out.push_back(pipeline.Process(Scan(stamp, SequenceScene(k), k + 1), WithGps(Local(k), map_valid),
                                   MapPose(map_valid, stamp)));
  }
  return out;
}

bool IsLowerHex64(const std::string& s) {
  if (s.size() != 64) return false;
  for (char c : s)
    if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))) return false;
  return true;
}

}  // namespace

TEST(CloudDigest, KnownVectors) {
  Cloud cloud;
  EXPECT_EQ(CloudDigest(cloud), kEmptySha);
  cloud.points.push_back(Point(1.f, 2.f, 3.f, 4.f));
  EXPECT_EQ(CloudDigest(cloud), "ad73b9acd6e4a74b2f5bb5386658ce3bb146cd040a1867646ab3b973fb6632b1");
  cloud.points.push_back(Point(-0.5f, 1e-3f, 100.25f, 0.f));
  EXPECT_EQ(CloudDigest(cloud), "0be4876848d1a16b0b7be966d0c67ba75f10ea6b2cf08fffb5584c09a2eb8fae");
  Cloud reversed;
  reversed.points.push_back(cloud.points[1]);
  reversed.points.push_back(cloud.points[0]);
  EXPECT_EQ(CloudDigest(reversed), "a5b160a944dbf3633ca0fad6df4ff650b40ec6a4fb9cc73a325a121f9c74d25f");
}

TEST(CloudDigest, HashesOnlyXyzIntensityAcrossBufferChunks) {
  Cloud a, b;
  for (int i = 0; i < 1500; ++i) {  // spans several internal update chunks
    a.points.push_back(Point(0.01f * i, -0.02f * i, 0.5f, float(i % 17)));
    b.points.push_back(a.points.back());
    b.points.back().data[3] = 7.f;  // padding is not part of the digest
  }
  EXPECT_EQ(CloudDigest(a), CloudDigest(b));
  EXPECT_TRUE(IsLowerHex64(CloudDigest(a)));
  b.points[1499].intensity += 1.f;
  EXPECT_NE(CloudDigest(a), CloudDigest(b));
}

TEST(Pipeline, RejectsEmptyCalibrationId) {
  PipelineConfig config = Config();
  config.calibration_id.clear();
  EXPECT_THROW(Pipeline{config}, std::invalid_argument);
}

TEST(Pipeline, ProcessesSceneInMainOrder) {
  Pipeline pipeline(Config(true, true));
  const auto results = RunSequence(pipeline, 3);
  for (int k = 0; k < 3; ++k) {
    const auto& r = results[k];
    const std::string where = "scan " + std::to_string(k);
    ASSERT_TRUE(r.processed) << where << " " << r.reason;
    EXPECT_EQ(r.stamp_ns, kT0 + k * kPeriod);
    EXPECT_EQ(r.counts.raw_points, static_cast<int>(SequenceScene(k)->size()));
    EXPECT_GT(r.counts.roi_points, 0);
    EXPECT_LE(r.counts.roi_points, r.counts.raw_points);
    EXPECT_LE(r.counts.voxel_points, r.counts.roi_points);
    EXPECT_EQ(r.counts.leveled_points, r.counts.voxel_points);
    EXPECT_GT(r.counts.ground_kept_points, 0);
    EXPECT_LT(r.counts.ground_kept_points, r.counts.leveled_points);
    EXPECT_EQ(r.counts.cluster_input_points, r.counts.ground_kept_points);
    EXPECT_GE(r.counts.clusters, 2) << where;
    EXPECT_EQ(r.counts.centroids, r.counts.clusters);  // Main cluster_points = centroid count
    EXPECT_EQ(r.counts.detections, r.counts.clusters);
    EXPECT_LE(r.counts.consolidated_detections, r.counts.detections);
    EXPECT_EQ(r.counts.tracks, static_cast<int>(r.objects.size()));
    EXPECT_EQ(r.counts.observed_tracks + r.counts.predicted_tracks, r.counts.tracks);
    EXPECT_TRUE(r.plane.valid) << where;
    EXPECT_EQ(r.context.frame, al::Frame::LeveledSensor);
    EXPECT_EQ(r.context.time.stamp_ns, r.stamp_ns);
    EXPECT_EQ(r.context.scan_sequence, static_cast<std::uint64_t>(k + 1));
    EXPECT_EQ(r.context.calibration_source, "test_autonav_v3");
    EXPECT_TRUE(r.context.map_pose.valid);
    // Main sort by f32_Dist, then the frame record and ground evidence for observed tracks.
    for (std::size_t i = 1; i < r.objects.size(); ++i)
      EXPECT_LE(r.objects[i - 1].detection.f32_Dist, r.objects[i].detection.f32_Dist) << where;
    for (const auto& o : r.objects) {
      if (o.tracking.s32_MissCount != 0) continue;
      EXPECT_TRUE(o.tracking.tracking_frame_valid) << where;
      EXPECT_TRUE(SameBits(o.tracking.tracking_height_input, o.detection.f32_CoreZ)) << where;
      EXPECT_TRUE(o.detection.ground_evidence.valid) << where;
      EXPECT_GE(o.tracking.s32_Id, 0) << where;
    }
    for (const auto& d : r.detections) EXPECT_LT(d.tracking.s32_Id, 0) << where;
  }
  // One committed frame per processed scan; the last one = published objects.
  ASSERT_EQ(pipeline.tracker().History().size(), 3u);
  ExpectSameObjects(*pipeline.tracker().LastFrame(), results.back().objects, "committed frame");
  // dt follows Main ts_CurrTime - ts_PrevTime (ts_PrevTime = ros::Time(0,0) at start).
  EXPECT_EQ(results[0].previous_stamp_ns, 0);
  EXPECT_TRUE(SameBits(results[0].dt_s,
                       (ros::Time().fromNSec(static_cast<std::uint64_t>(kT0)) - ros::Time()).toSec()));
  EXPECT_EQ(results[1].previous_stamp_ns, kT0);
  EXPECT_TRUE(SameBits(results[1].dt_s, (ros::Time().fromNSec(static_cast<std::uint64_t>(kT0 + kPeriod)) -
                                         ros::Time().fromNSec(static_cast<std::uint64_t>(kT0)))
                                            .toSec()));
  EXPECT_DOUBLE_EQ(results[2].dt_s, 0.1);
  // Two separated boxes stay two observed tracks with stable ids (the tracker
  // receives this scan's dt, local state and frame).
  EXPECT_EQ(results[2].counts.observed_tracks, 2);
  std::vector<int> first_ids, last_ids;
  for (const auto& o : results[0].objects) first_ids.push_back(o.tracking.s32_Id);
  for (const auto& o : results[2].objects) {
    if (o.tracking.s32_MissCount != 0) continue;
    last_ids.push_back(o.tracking.s32_Id);
    EXPECT_EQ(o.tracking.s32_Age, 3);
    EXPECT_TRUE(o.tracking.b_KfInitialized);
  }
  std::sort(first_ids.begin(), first_ids.end());
  std::sort(last_ids.begin(), last_ids.end());
  EXPECT_EQ(first_ids, last_ids);
}

TEST(Pipeline, MissedTrackIsCarriedWithoutGroundEvidence) {
  Pipeline pipeline(Config());
  const auto results = RunSequence(pipeline, 6);
  ASSERT_TRUE(results[4].processed);
  EXPECT_GE(results[4].counts.predicted_tracks, 1);
  bool saw_missed = false;
  for (int k = 4; k < 6; ++k) {
    for (const auto& o : results[k].objects) {
      if (o.tracking.s32_MissCount == 0) continue;
      saw_missed = true;
      EXPECT_FALSE(o.detection.ground_evidence.valid);
      EXPECT_EQ(o.detection.ground_evidence.sample_count, 0);
      EXPECT_EQ(o.detection.ground_evidence.median_clearance_m, 0.0);
      EXPECT_EQ(o.detection.ground_evidence.rejection_band_m, 0.0);
    }
  }
  EXPECT_TRUE(saw_missed);
}

TEST(Pipeline, MapPoseAndGpsDoNotChangeLocalResults) {
  Pipeline with_map(Config(true, true)), without_map(Config(true, true)), alternating(Config(true, true));
  for (int k = 0; k < 6; ++k) {
    const std::int64_t stamp = kT0 + k * kPeriod;
    const auto a = with_map.Process(Scan(stamp, SequenceScene(k), k + 1), WithGps(Local(k), true),
                                    MapPose(true, stamp));
    const auto b = without_map.Process(Scan(stamp, SequenceScene(k), k + 1), WithGps(Local(k), false),
                                       MapPose(false, stamp));
    const bool alt = k % 2 == 0;
    const auto c = alternating.Process(Scan(stamp, SequenceScene(k), k + 1), WithGps(Local(k), alt),
                                       MapPose(alt, stamp));
    const std::string where = "scan " + std::to_string(k);
    ASSERT_TRUE(a.processed) << where;
    EXPECT_TRUE(a.context.map_pose.valid);
    EXPECT_FALSE(b.context.map_pose.valid);
    EXPECT_EQ(b.context.map_pose.reason, "gps_missing");
    EXPECT_EQ(c.context.map_pose.valid, alt);
    ExpectSameLocalResult(a, b, where + " valid/invalid");
    ExpectSameLocalResult(a, c, where + " valid/alternating");
  }
  EXPECT_EQ(with_map.tracker().NextTrackId(), without_map.tracker().NextTrackId());
}

TEST(Pipeline, DuplicateAndRegressingStampsDoNotAdvanceTracker) {
  Pipeline pipeline(Config(true, true)), reference(Config(true, true));
  auto run = [](Pipeline& p, int k, std::int64_t stamp, int scene) {
    return p.Process(Scan(stamp, SequenceScene(scene), k + 1), WithGps(Local(k), true), MapPose(true, stamp));
  };
  for (int k = 0; k < 2; ++k) {
    ASSERT_TRUE(run(pipeline, k, kT0 + k * kPeriod, k).processed);
    ASSERT_TRUE(run(reference, k, kT0 + k * kPeriod, k).processed);
  }
  const auto history = pipeline.tracker().History();
  const auto next_id = pipeline.tracker().NextTrackId();

  // Same stamp again (different content), then an older stamp.
  const auto duplicate = run(pipeline, 2, kT0 + kPeriod, 3);
  ExpectNotProcessed(duplicate, "duplicate_or_regressing_stamp", "duplicate");
  EXPECT_EQ(duplicate.stamp_ns, kT0 + kPeriod);
  EXPECT_EQ(duplicate.previous_stamp_ns, kT0 + kPeriod);
  const auto regressing = run(pipeline, 3, kT0 + kPeriod - 1, 4);
  ExpectNotProcessed(regressing, "duplicate_or_regressing_stamp", "regressing");
  ExpectNotProcessed(run(pipeline, 4, kT0, 0), "duplicate_or_regressing_stamp", "old");

  ASSERT_EQ(pipeline.tracker().History().size(), history.size());
  for (std::size_t i = 0; i < history.size(); ++i)
    ExpectSameObjects(pipeline.tracker().History()[i], history[i], "history " + std::to_string(i));
  EXPECT_EQ(pipeline.tracker().NextTrackId(), next_id);

  // The next valid scan sees dt from the last processed scan, as if the
  // rejected scans never arrived.
  for (int k = 2; k < 5; ++k) {
    const auto a = run(pipeline, k, kT0 + k * kPeriod, k);
    const auto b = run(reference, k, kT0 + k * kPeriod, k);
    ASSERT_TRUE(a.processed);
    EXPECT_EQ(a.previous_stamp_ns, kT0 + (k - 1) * kPeriod);
    ExpectSameLocalResult(a, b, "after rejection scan " + std::to_string(k));
    EXPECT_TRUE(a.context.map_pose.valid);
  }
}

TEST(Pipeline, EmptyScanRejectedAndConsumesItsStamp) {
  Pipeline pipeline(Config(true, true)), reference(Config(true, true));
  auto run = [](Pipeline& p, int k, std::int64_t stamp, CloudPtr cloud) {
    return p.Process(Scan(stamp, std::move(cloud), k + 1), WithGps(Local(k), true), MapPose(true, stamp));
  };
  for (int k = 0; k < 2; ++k) {
    ASSERT_TRUE(run(pipeline, k, kT0 + k * kPeriod, SequenceScene(k)).processed);
    ASSERT_TRUE(run(reference, k, kT0 + k * kPeriod, SequenceScene(k)).processed);
  }
  const auto history_size = pipeline.tracker().History().size();
  ExpectNotProcessed(run(pipeline, 2, kT0 + 2 * kPeriod, CloudPtr(new Cloud)), "empty_scan", "empty cloud");
  ExpectNotProcessed(run(pipeline, 2, kT0 + 2 * kPeriod + 1, nullptr), "empty_scan", "null cloud");
  // Main advances last_raw before the empty check: the same stamp with points
  // is now a duplicate (preserved quirk).
  ExpectNotProcessed(run(pipeline, 2, kT0 + 2 * kPeriod + 1, SequenceScene(2)),
                     "duplicate_or_regressing_stamp", "after empty");
  EXPECT_EQ(pipeline.tracker().History().size(), history_size);

  const auto a = run(pipeline, 3, kT0 + 3 * kPeriod, SequenceScene(3));
  const auto b = run(reference, 3, kT0 + 3 * kPeriod, SequenceScene(3));
  ASSERT_TRUE(a.processed);
  EXPECT_EQ(a.previous_stamp_ns, kT0 + kPeriod);
  EXPECT_NEAR(a.dt_s, 0.2, 1e-12);
  ExpectSameLocalResult(a, b, "after empty");
}

TEST(Pipeline, ZeroStampIsRejectedLikeMainLastRaw) {
  Pipeline pipeline(Config());
  ExpectNotProcessed(pipeline.Process(Scan(0, SequenceScene(0), 1), Local(0), MapPose(false, 0)),
                     "duplicate_or_regressing_stamp", "zero stamp");
  EXPECT_TRUE(pipeline.tracker().History().empty());
  const auto r = pipeline.Process(Scan(kT0, SequenceScene(0), 2), Local(0), MapPose(false, kT0));
  EXPECT_TRUE(r.processed);
  EXPECT_EQ(r.previous_stamp_ns, 0);
}

TEST(Pipeline, CaptureDebugDoesNotChangeObjects) {
  Pipeline off(Config(true, false)), on(Config(true, true));
  const auto a = RunSequence(off, 6);
  const auto b = RunSequence(on, 6);
  for (int k = 0; k < 6; ++k) {
    const std::string where = "scan " + std::to_string(k);
    ASSERT_TRUE(a[k].processed && b[k].processed) << where;
    ExpectSameCounts(a[k].counts, b[k].counts, where, false);
    ExpectSameObjects(a[k].objects, b[k].objects, where);
    EXPECT_TRUE(SameMatrix(a[k].context.applied_leveling, b[k].context.applied_leveling));
    // Off: nothing captured.
    EXPECT_TRUE(a[k].detections.empty());
    EXPECT_FALSE(a[k].centroids);
    EXPECT_TRUE(a[k].cluster_sizes.empty());
    EXPECT_TRUE(a[k].digests.roi.empty() && a[k].digests.voxel.empty() && a[k].digests.leveled.empty() &&
                a[k].digests.ground_kept.empty());
    // On: Objectinfo_for_km output, centroids and stage digests.
    EXPECT_EQ(static_cast<int>(b[k].detections.size()), b[k].counts.detections);
    ASSERT_TRUE(b[k].centroids);
    EXPECT_EQ(static_cast<int>(b[k].centroids->size()), b[k].counts.centroids);
    ASSERT_EQ(static_cast<int>(b[k].cluster_sizes.size()), b[k].counts.clusters);
    int clustered = 0;
    for (std::size_t i = 0; i < b[k].cluster_sizes.size(); ++i) {
      clustered += b[k].cluster_sizes[i];
      EXPECT_EQ(b[k].cluster_sizes[i], static_cast<int>(b[k].detections[i].detection.pcl_ClusterPoints.size()));
    }
    EXPECT_LE(clustered, b[k].counts.cluster_input_points);
    EXPECT_TRUE(IsLowerHex64(b[k].digests.roi));
    EXPECT_TRUE(IsLowerHex64(b[k].digests.voxel));
    EXPECT_TRUE(IsLowerHex64(b[k].digests.leveled));
    EXPECT_TRUE(IsLowerHex64(b[k].digests.ground_kept));
    EXPECT_NE(b[k].digests.voxel, b[k].digests.leveled);
    EXPECT_GE(b[k].counts.consolidated_detections, 1);
    EXPECT_LE(b[k].counts.consolidated_detections, b[k].counts.detections);
  }
}

TEST(Pipeline, TunnelOffKeepsGroundOutputForClustering) {
  PipelineConfig config = Config(true, true);
  EXPECT_FALSE(config.tunnel.enabled);  // default OFF
  Pipeline pipeline(config);
  for (const auto& r : RunSequence(pipeline, 3)) {
    ASSERT_TRUE(r.processed);
    EXPECT_FALSE(r.tunnel.ran);
    EXPECT_EQ(r.counts.tunnel_removed_points, 0);
    EXPECT_EQ(r.counts.cluster_input_points, r.counts.ground_kept_points);
    EXPECT_EQ(r.timings.tunnel_ms, 0.0);
  }
}

TEST(Pipeline, TunnelOnRunsBetweenGroundRemovalAndClustering) {
  PipelineConfig config = Config(true, true);
  Pipeline off(config);
  config.tunnel.enabled = true;
  Pipeline on(config);
  const auto a = RunSequence(off, 3, false);
  const auto b = RunSequence(on, 3, false);
  for (int k = 0; k < 3; ++k) {
    ASSERT_TRUE(b[k].processed);
    EXPECT_TRUE(b[k].tunnel.ran);
    EXPECT_EQ(b[k].tunnel.input_points, b[k].counts.voxel_points);
    // Stages up to RANSAC are unchanged; the stage only removes points after it.
    EXPECT_EQ(b[k].digests.voxel, a[k].digests.voxel);
    EXPECT_EQ(b[k].digests.ground_kept, a[k].digests.ground_kept);
    EXPECT_EQ(b[k].counts.ground_kept_points, a[k].counts.ground_kept_points);
    EXPECT_EQ(b[k].counts.tunnel_removed_points, b[k].tunnel.removed_points);
    EXPECT_EQ(b[k].counts.cluster_input_points,
              b[k].counts.ground_kept_points - b[k].counts.tunnel_removed_points);
  }
}

TEST(Pipeline, GroundOnlySceneCountsBeforeClusteringDummy) {
  Pipeline pipeline(Config(true, true));
  const auto r = pipeline.Process(Scan(kT0, Scene({}), 1), Local(0), MapPose(true, kT0));
  ASSERT_TRUE(r.processed);
  ASSERT_TRUE(r.plane.valid);
  // Every ground point is removed; Clustering then pushes its (0,0,0) dummy,
  // which must not appear in the RANSAC/cluster-input counts or digest.
  EXPECT_EQ(r.counts.ground_kept_points, 0);
  EXPECT_EQ(r.counts.cluster_input_points, 0);
  EXPECT_EQ(r.digests.ground_kept, kEmptySha);
  EXPECT_EQ(r.counts.clusters, 0);
  EXPECT_EQ(r.counts.centroids, 0);
  EXPECT_EQ(r.counts.detections, 0);
  EXPECT_TRUE(r.objects.empty());
  EXPECT_EQ(pipeline.tracker().History().size(), 1u);
}

TEST(Pipeline, LevelingMatrixRollPitchAndPitchOnly) {
  const float roll = 0.03f, pitch = -0.02f;
  for (const bool mounted : {false, true}) {
    PipelineConfig base = Config(true, true);
    if (mounted) base.vehicle.st_LiDARRotationDeg = {0.5f, 1.f, 2.f};
    const al::Rot mount = ScanFrame::SensorMount(base.vehicle).rotation();
    for (const bool use_roll : {true, false}) {
      PipelineConfig config = base;
      config.use_roll = use_roll;
      Pipeline pipeline(config);
      const CloudPtr scene = SequenceScene(0);
      const auto r = pipeline.Process(Scan(kT0, scene, 1), Local(0, roll, pitch), MapPose(true, kT0));
      ASSERT_TRUE(r.processed);
      const std::string where = std::string(mounted ? "mounted " : "") + (use_roll ? "roll+pitch" : "pitch-only");
      const al::Rot expected = al::leveling_rotation(roll, pitch, mount, use_roll, true);
      EXPECT_TRUE(SameMatrix(r.context.applied_leveling, expected)) << where;
      EXPECT_TRUE(SameMatrix(pipeline.frame().Current().applied_leveling, expected)) << where;
      if (!mounted) {
        const al::Rot plain = al::rpy(use_roll ? roll : 0.f, pitch, 0.0);
        EXPECT_LT((r.context.applied_leveling - plain).norm(), 1e-12) << where;
      } else {
        EXPECT_GT((r.context.applied_leveling - al::rpy(roll, pitch, 0.0)).norm(), 1e-6) << where;
      }
      const al::Rot other = al::leveling_rotation(roll, pitch, mount, !use_roll, true);
      EXPECT_GT((r.context.applied_leveling - other).norm(), 0.01) << where;

      // The leveled stage cloud is (L * p.cast<double>()).cast<float>() of the
      // voxel output with intensity kept.
      CloudPtr roi(new Cloud), voxel(new Cloud);
      Roi(scene, roi, config.range, config.roi, config.vehicle);
      Voxel(roi, voxel, config.voxel_leaf_m);
      EXPECT_EQ(r.digests.roi, CloudDigest(*roi)) << where;
      EXPECT_EQ(r.digests.voxel, CloudDigest(*voxel)) << where;
      Cloud leveled;
      for (const auto& p : voxel->points) {
        const Eigen::Vector3f q = (expected * Eigen::Vector3f(p.x, p.y, p.z).cast<double>()).cast<float>();
        leveled.points.push_back(Point(q.x(), q.y(), q.z(), p.intensity));
      }
      EXPECT_EQ(r.digests.leveled, CloudDigest(leveled)) << where;
    }
  }
}

TEST(Pipeline, RepeatedRunsAreDeterministic) {
  Pipeline first(Config(true, true)), second(Config(true, true));
  const auto a = RunSequence(first, 6);
  const auto b = RunSequence(second, 6);
  for (int k = 0; k < 6; ++k) {
    ExpectSameLocalResult(a[k], b[k], "scan " + std::to_string(k));
    EXPECT_TRUE(SameMatrix(a[k].context.map_pose.map_from_base.matrix(), b[k].context.map_pose.map_from_base.matrix()));
    ASSERT_TRUE(a[k].centroids && b[k].centroids);
    EXPECT_EQ(CloudDigest(*a[k].centroids), CloudDigest(*b[k].centroids));
  }
  EXPECT_EQ(first.tracker().NextTrackId(), second.tracker().NextTrackId());
  ASSERT_EQ(first.tracker().History().size(), 3u);  // Main MAX_FRAMES = 3
  ASSERT_EQ(first.tracker().History().size(), second.tracker().History().size());
  for (std::size_t i = 0; i < first.tracker().History().size(); ++i)
    ExpectSameObjects(first.tracker().History()[i], second.tracker().History()[i], "history");
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(PipelineSemantic, FinalizeOnceAfterFusionAndMapLossInvalidatesAllTracks) {
  auto cfg=Config();cfg.motion.semantic_static_enabled=true;Pipeline pipeline(cfg);
  for(int i=0;i<14;++i) {
    const auto stamp=kT0+i*kPeriod;const bool valid=i<12;
    auto local=WithGps(Local(0,0,0),valid);local.st_CAN.f32_CurrentVel_MPS=0;local.st_CAN.f32_CurrentVel_KPH=0;
    auto result=pipeline.Process(Scan(stamp,SequenceScene(0),i+1),local,MapPose(valid,stamp));
    ASSERT_TRUE(result.processed);ASSERT_FALSE(result.objects.empty());
    for(auto& o:result.objects) {
      o.camera.str_ObjectClass=o.camera.observed_class="static_obstacle";
      o.camera.camera_observation_ns=stamp;o.camera.class_miss_counter=0;
    }
    const auto corners=result.objects.front().detection.obb_corners_level;
    pipeline.FinalizeMotion(result);
    const auto motion=result.objects.front().tracking.motion;
    pipeline.FinalizeMotion(result); // cannot count the same scan twice
    EXPECT_EQ(motion.s32_MotionAcceptedSamples,result.objects.front().tracking.motion.s32_MotionAcceptedSamples);
    EXPECT_EQ(motion.semantic_static_applied,result.objects.front().tracking.motion.semantic_static_applied);
    EXPECT_FLOAT_EQ(corners[0].x,result.objects.front().detection.obb_corners_level[0].x);
    if(!valid)for(const auto& o:result.objects) {
      EXPECT_FALSE(o.tracking.motion.motion_valid);EXPECT_FALSE(o.tracking.motion.semantic_static_applied);
    }
  }
}

TEST(Pipeline, ScopedTunnelUpdatesDoNotChangeNormalRoadGeometry) {
  auto baseline=Config(true,true),scoped=baseline;
  baseline.clustering.s32_MinPoints=scoped.clustering.s32_MinPoints=5;
  baseline.ground.near_threshold_m=scoped.ground.near_threshold_m=.30;
  scoped.tunnel_only_updates=true;scoped.shadow_wall.enabled=true;
  scoped.shadow_wall.entry_m=1800;scoped.shadow_wall.exit_m=2000;
  scoped.shadow_wall.route={{{1700,0,0}},{{2100,400,0}}};
  scoped.camera_point_refinement.vehicle_history_enabled=true;
  scoped.camera_point_refinement.detached_wall_cleanup=true;
  Pipeline a(baseline),b(scoped);
  for(int k=0;k<6;++k){
    const auto stamp=kT0+k*kPeriod;auto pose=MapPose(true,stamp);
    auto x=a.Process(Scan(stamp,SequenceScene(k),k+1),WithGps(Local(k),true),pose);
    auto y=b.Process(Scan(stamp,SequenceScene(k),k+1),WithGps(Local(k),true),pose);
    EXPECT_FALSE(y.tunnel.updates_active);
    ExpectSameLocalResult(x,y,"normal road scoped update equivalence");
  }
}
TEST(Pipeline, TunnelScopeRequiresMapWindowBeforeGpsLossAndClearsOnExit) {
  auto c=Config(true,true);c.tunnel_only_updates=true;c.shadow_wall.enabled=true;
  c.shadow_wall.entry_m=100;c.shadow_wall.exit_m=200;
  c.shadow_wall.route={{{0,0,0}},{{300,300,0}}};
  Pipeline p(c);
  for(int k=0;k<6;++k){
    const auto stamp=kT0+k*kPeriod;auto pose=MapPose(true,stamp);
    pose.map_from_base.setIdentity();pose.map_from_base.translation().x()=k>=4?250:100;
    pose.valid=(k==1||k>=4);
    const auto r=p.Process(Scan(stamp,Scene({}),k+1),WithGps(Local(k),pose.valid),pose);
    EXPECT_EQ(k>=1&&k<4,r.tunnel.updates_active);
  }
}

// Offline sensitivity experiment: PR144's GENERAL-road defaults can change
// candidates even where the mapped tunnel gate is inactive. Not a contact test.
TEST(Pipeline, GlobalTunnelTuningChangesNormalRoadCandidateAdmission) {
  auto base=Config(true,true),global=base;
  base.clustering.s32_MinPoints=5;global.clustering.s32_MinPoints=3;
  base.ground.near_threshold_m=.30;global.ground.near_threshold_m=.23;
  for(int scene=0;scene<2;++scene) {
    auto cloud=Scene({});
    const int points=scene==0?3:6;
    const float z=scene==0?-.50f:kGroundZ+.27f;
    for(int j=0;j<points;++j)
      cloud->points.push_back(Point(10.f+.2f*j,3.f,z,50.f));
    cloud->width=cloud->points.size();
    Pipeline a(base),b(global);
    const auto x=a.Process(Scan(kT0,cloud,1),WithGps(Local(0,0,0),true),MapPose(true,kT0));
    const auto y=b.Process(Scan(kT0,cloud,1),WithGps(Local(0,0,0),true),MapPose(true,kT0));
    ASSERT_TRUE(x.processed&&y.processed);
    EXPECT_FALSE(x.tunnel.ran||y.tunnel.ran);
    std::cout << "NORMAL_SCOPE_AB scene=" << scene
      << " baseline_ground=" << x.counts.ground_kept_points
      << " global_ground=" << y.counts.ground_kept_points
      << " baseline_clusters=" << x.counts.clusters
      << " global_clusters=" << y.counts.clusters << std::endl;
    EXPECT_EQ(0,x.counts.clusters);
    EXPECT_EQ(1,y.counts.clusters);
    if(scene==1)EXPECT_GT(y.counts.ground_kept_points,x.counts.ground_kept_points);
  }
}
