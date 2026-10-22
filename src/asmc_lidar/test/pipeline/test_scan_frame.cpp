// ScanFrame port tests (Main Geometry/FrameBoundary.cpp at 8f93c22e) plus the
// tracker-fix Kalman test and the base re-expression test of Main
// test/test_frame_boundary_ros.cpp, with ScanFrame configured directly instead
// of ROS parameters. The world/scan model is computed independently:
//   p_scan = L * mount^-1 * (R_imu^T * w)   (zero ego translation),
// R_imu = rpy(roll, pitch, yaw) of the stored float IMU angles, mount from the
// float Deg2Rad of the vehicle angles, L = asmc::lidar::leveling_rotation.
#include <asmc_lidar/pipeline/kalman.hpp>
#include <asmc_lidar/pipeline/scan_frame.hpp>
#include <asmc_lidar/pipeline/tracker.hpp>
#include <asmc_lidar/projection.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace {
using namespace asmc::lidar::pipeline;
using asmc::lidar::Pose;
using asmc::lidar::Rot;
using asmc::lidar::Vec;

constexpr double kPosTol = 1e-5;  // m
constexpr double kAngTol = 1e-5;  // rad
const char* const kCalibration = "test-calibration";

VehicleSpec BaseVehicle() {
  VehicleSpec v;
  v.st_LiDARPos = {.8f, 0.f, 1.45f};
  v.st_LiDARRotationDeg = {0.f, 0.f, 0.f};
  v.st_GPSPos = {.35f, 0.f, 1.25f};
  return v;
}
VehicleSpec MountedVehicle() {
  auto v = BaseVehicle();
  v.st_LiDARRotationDeg = {4.f, -3.f, 8.f};
  return v;
}

LocalState Local(double yaw = 0.) {
  LocalState out{};
  out.st_IMU.b_ImuReady = true;
  out.st_IMU.f32_Yaw_rad = yaw;
  out.st_IMU.f32_Yaw_deg = yaw * 180 / M_PI;
  out.st_CAN.f32_CurrentVel_MPS = 2.;
  out.st_CAN.f32_CurrentVel_KPH = 7.2;
  return out;
}

// Independent model of Main SensorMount (float Deg2Rad) and the IMU attitude.
Pose MountOf(const VehicleSpec& v) {
  const auto rad = [](float deg) { return static_cast<float>(deg * M_PI / 180.0f); };
  Pose m = Pose::Identity();
  m.translation() = Vec(v.st_LiDARPos.f32_X, v.st_LiDARPos.f32_Y, v.st_LiDARPos.f32_Z);
  m.linear() = asmc::lidar::rpy(rad(v.st_LiDARRotationDeg.f32_X), rad(v.st_LiDARRotationDeg.f32_Y),
                                rad(v.st_LiDARRotationDeg.f32_Z));
  return m;
}
Rot Attitude(const LocalState& l) {
  return asmc::lidar::rpy(l.st_IMU.f32_Roll_rad, l.st_IMU.f32_Pitch_rad, l.st_IMU.f32_Yaw_rad);
}

asmc::lidar::Observation Time(double t) {
  asmc::lidar::Observation o;
  o.stamp_ns = o.receipt_ns = o.processing_ns = static_cast<asmc::lidar::Ns>(std::llround(t * 1e9));
  o.clock = "ros";
  o.stamp_provenance = "bridge_receive_ms";
  return o;
}

struct TrackScan {
  LocalState local;
  Pose mount;
  Rot level;
  Vec Point(const Vec& world) const { return level * (mount.inverse() * (Attitude(local).transpose() * world)); }
  Vec Direction(const Vec& world) const {
    return level * (mount.rotation().transpose() * (Attitude(local).transpose() * world));
  }
  Vec World(const Vec& point) const { return Attitude(local) * (mount * (level.transpose() * point)); }
  Vec WorldDirection(const Vec& d) const { return Attitude(local) * (mount.rotation() * (level.transpose() * d)); }
};

TrackScan BeginTrackScan(ScanFrame& frame, double time, double yaw, double roll, double pitch,
                         const VehicleSpec& vehicle, bool use_roll) {
  TrackScan scan;
  scan.local = Local(yaw);
  scan.local.st_IMU.f32_Roll_rad = roll;
  scan.local.st_IMU.f32_Pitch_rad = pitch;
  scan.mount = MountOf(vehicle);
  scan.level = asmc::lidar::leveling_rotation(scan.local.st_IMU.f32_Roll_rad, scan.local.st_IMU.f32_Pitch_rad,
                                              scan.mount.rotation(), use_roll);
  asmc::lidar::PoseSelection unavailable;
  unavailable.reason = "gps_missing";
  frame.Prepare(Time(time), scan.local, unavailable, 0);
  frame.StoreLeveling(scan.level);
  return scan;
}

struct WorldBox {
  Vec center;
  double heading, length, width, height;
};
Vec Heading(const WorldBox& box) { return {std::cos(box.heading), std::sin(box.heading), 0}; }
// computeOBBAtYaw order: 0->1 is the yaw (length) edge, 4..7 repeat 0..3 on top.
std::array<Vec, 8> WorldCorners(const WorldBox& box) {
  const auto u = Heading(box);
  const Vec v(-u.y(), u.x(), 0), up(0, 0, .5 * box.height);
  const double su[] = {-1, 1, 1, -1}, sv[] = {-1, -1, 1, 1};
  std::array<Vec, 8> out;
  for (unsigned i = 0; i < 4; ++i) {
    const auto xy = (box.center + .5 * su[i] * box.length * u + .5 * sv[i] * box.width * v).eval();
    out[i] = xy - up;
    out[i + 4] = xy + up;
  }
  return out;
}
// Main test_frame_boundary_ros.cpp:220-248 BoxDetection.
TrackedObject BoxDetection(const TrackScan& scan, const WorldBox& box) {
  TrackedObject out{};
  out.s32_Id = -1;
  out.str_ObjectClass = "car";
  out.str_YoloClass = "car";
  const auto world = WorldCorners(box);
  double low = 1e9, high = -1e9;
  out.f32_MinX = out.f32_MinY = 1e9;
  out.f32_MaxX = out.f32_MaxY = -1e9;
  for (unsigned i = 0; i < 8; ++i) {
    const auto p = scan.Point(world[i]);
    out.OBB_Corners_Body[i] = {float(p.x()), float(p.y()), float(p.z())};
    low = std::min(low, p.z());
    high = std::max(high, p.z());
    out.f32_MinX = std::min<float>(out.f32_MinX, p.x());
    out.f32_MaxX = std::max<float>(out.f32_MaxX, p.x());
    out.f32_MinY = std::min<float>(out.f32_MinY, p.y());
    out.f32_MaxY = std::max<float>(out.f32_MaxY, p.y());
  }
  static const int edges[12][2] = {{0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6},
                                   {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
  for (unsigned i = 0; i < 8; ++i) {
    pcl::PointXYZI p;
    const auto q = scan.Point(world[i]);
    p.x = q.x();
    p.y = q.y();
    p.z = q.z();
    p.intensity = 1;
    out.pcl_ClusterPoints.push_back(p);
  }
  for (const auto& e : edges) {
    pcl::PointXYZI p;
    const auto q = scan.Point(.5 * (world[e[0]] + world[e[1]]));
    p.x = q.x();
    p.y = q.y();
    p.z = q.z();
    p.intensity = 1;
    out.pcl_ClusterPoints.push_back(p);
  }
  const auto center = scan.Point(box.center), axis = scan.Direction(Heading(box));
  out.f32_CoreX = out.f32_OBB_CenterX = center.x();
  out.f32_CoreY = out.f32_OBB_CenterY = center.y();
  out.f32_CoreZ = out.f32_OBB_CenterZ = center.z();
  out.f32_OBB_Width = box.length;
  out.f32_OBB_Height = box.width;
  out.f32_OBB_ZMin = low;
  out.f32_OBB_ZMax = high;
  out.f32_OBB_YawRad = std::atan2(axis.y(), axis.x());
  return out;
}
// Main test_frame_boundary_ros.cpp:32-44 Detection.
TrackedObject Detection(double x, double y) {
  TrackedObject out{};
  out.s32_Id = -1;
  out.str_ObjectClass = "car";
  out.str_YoloClass = "car";
  out.f32_CoreX = out.f32_OBB_CenterX = x;
  out.f32_CoreY = out.f32_OBB_CenterY = y;
  out.f32_CoreZ = out.f32_OBB_CenterZ = -.2;
  out.f32_OBB_Width = 1.;
  out.f32_OBB_Height = .8;
  out.f32_OBB_ZMin = -.5;
  out.f32_OBB_ZMax = .5;
  out.f32_MinX = x - .5;
  out.f32_MaxX = x + .5;
  out.f32_MinY = y - .4;
  out.f32_MaxY = y + .4;
  for (unsigned i = 0; i < 8; ++i) {
    out.OBB_Corners_Body[i] = {float(x + ((i & 1) ? .5 : -.5)), float(y + ((i & 2) ? .4 : -.4)),
                               float((i & 4) ? .5 : -.5)};
    pcl::PointXYZI p;
    p.x = out.OBB_Corners_Body[i].x;
    p.y = out.OBB_Corners_Body[i].y;
    p.z = out.OBB_Corners_Body[i].z;
    p.intensity = 1;
    out.pcl_ClusterPoints.push_back(p);
  }
  return out;
}

double AxisError(double a, double b) { return std::abs(std::remainder(a - b, M_PI)); }
double CornerYaw(const TrackedObject& o) {
  const auto& a = o.OBB_Corners_Body[0];
  const auto& b = o.OBB_Corners_Body[1];
  return std::atan2(b.y - a.y, b.x - a.x);
}
double ScanYaw(const TrackScan& scan, const Vec& world_a, const Vec& world_b) {
  const auto d = (scan.Point(world_b) - scan.Point(world_a)).eval();
  return std::atan2(d.y(), d.x());
}
void ExpectObbSelfConsistent(const TrackedObject& o, const char* role) {
  EXPECT_LT(AxisError(o.f32_OBB_YawRad, CornerYaw(o)), kAngTol) << role << " yaw axis vs corner 0->1 edge";
  Vec mean = Vec::Zero();
  for (const auto& c : o.OBB_Corners_Body) mean += Vec(c.x, c.y, c.z) / 8.;
  EXPECT_NEAR(mean.x(), o.f32_OBB_CenterX, kPosTol) << role;
  EXPECT_NEAR(mean.y(), o.f32_OBB_CenterY, kPosTol) << role;
  EXPECT_NEAR(mean.z(), o.f32_OBB_CenterZ, kPosTol) << role;
}
void ExpectCarriedBox(const TrackedObject& o, const TrackScan& scan, const WorldBox& box, double z_extent,
                      bool kf_at_center = true) {
  EXPECT_NEAR(.5 * (o.f32_OBB_ZMin + o.f32_OBB_ZMax), o.f32_OBB_CenterZ, kPosTol);
  EXPECT_NEAR(o.f32_OBB_ZMax - o.f32_OBB_ZMin, z_extent, kPosTol);
  const auto world = WorldCorners(box);
  for (unsigned i = 0; i < 8; ++i) {
    const auto p = scan.Point(world[i]);
    EXPECT_NEAR(o.OBB_Corners_Body[i].x, p.x(), kPosTol) << "corner " << i;
    EXPECT_NEAR(o.OBB_Corners_Body[i].y, p.y(), kPosTol) << "corner " << i;
    EXPECT_NEAR(o.OBB_Corners_Body[i].z, p.z(), kPosTol) << "corner " << i;
  }
  const auto center = scan.Point(box.center);
  EXPECT_NEAR(o.f32_OBB_CenterX, center.x(), kPosTol);
  EXPECT_NEAR(o.f32_OBB_CenterY, center.y(), kPosTol);
  EXPECT_NEAR(o.f32_OBB_CenterZ, center.z(), kPosTol);
  if (kf_at_center) {
    EXPECT_NEAR(o.Eigen_KfState_CA(0), center.x(), kPosTol);
    EXPECT_NEAR(o.Eigen_KfState_CA(1), center.y(), kPosTol);
  }
  EXPECT_LT(AxisError(o.f32_OBB_YawRad, ScanYaw(scan, world[0], world[1])), kAngTol) << "carried yaw";
}
// Independent frame change previous scan -> current scan for direction vectors.
Rot Change(const TrackScan& previous, const TrackScan& now) {
  return now.level * now.mount.rotation().transpose() * Attitude(now.local).transpose() * Attitude(previous.local) *
         previous.mount.rotation() * previous.level.transpose();
}

const WorldBox kMatchedBox{{12, 2, .25}, .4, 4.5, 1.8, 1.5};

TEST(ScanFrame, SensorMountUsesFloatDeg2Rad) {
  const auto v = MountedVehicle();
  const auto mount = ScanFrame::SensorMount(v);
  const auto expected = MountOf(v);
  EXPECT_EQ((mount.matrix() - expected.matrix()).norm(), 0.) << "float Deg2Rad, rpy(roll, pitch, yaw)";
  EXPECT_TRUE(mount.translation() == Vec(.8f, 0.f, 1.45f));
  // Main's float overload differs from a double conversion (float rounding).
  const Rot double_rad = asmc::lidar::rpy(4. * M_PI / 180., -3. * M_PI / 180., 8. * M_PI / 180.);
  EXPECT_GT((mount.linear() - double_rad).norm(), 0.);
  EXPECT_LT((mount.linear() - double_rad).norm(), 1e-6);
}

TEST(ScanFrame, MapRotationIsImuRollPitchYaw) {
  auto local = Local(.7);
  local.st_IMU.f32_Roll_rad = .05f;
  local.st_IMU.f32_Pitch_rad = -.03f;
  EXPECT_EQ((ScanFrame::MapRotation(local) - asmc::lidar::rpy(.05f, -.03f, local.st_IMU.f32_Yaw_rad)).norm(), 0.);
}

TEST(ScanFrame, ConfigureRejectsEmptyCalibrationId) {
  ScanFrame frame;
  EXPECT_THROW(frame.Configure(BaseVehicle(), ""), std::invalid_argument);
  EXPECT_NO_THROW(frame.Configure(BaseVehicle(), kCalibration));
}

TEST(ScanFrame, PrepareResetsContextAndStoreLevelingSetsIt) {
  ScanFrame frame;
  frame.Configure(MountedVehicle(), kCalibration);
  auto local = Local(.3);
  local.st_CAN.f32_CurrentVel_MPS = 3.5f;
  asmc::lidar::PoseSelection map;
  map.valid = true;
  map.reason = "gps_bounded_prediction";
  map.map_from_base.translation() = Vec(1, 2, 3);
  frame.Prepare(Time(10.), local, map, 7);
  frame.StoreLeveling(asmc::lidar::rpy(.1, .2, 0));
  frame.Prepare(Time(10.1), local, map, 8);
  const auto& c = frame.Current();
  EXPECT_TRUE(c.applied_leveling == Rot::Identity()) << "Prepare resets the Context (Main current=Context{})";
  EXPECT_EQ(c.time.stamp_ns, 10100000000LL);
  EXPECT_EQ(c.input_generation, 8u);
  EXPECT_EQ(c.frame, asmc::lidar::Frame::LeveledSensor);
  EXPECT_EQ((c.base_from_sensor.matrix() - MountOf(MountedVehicle()).matrix()).norm(), 0.);
  EXPECT_EQ(c.calibration_source, kCalibration);
  EXPECT_EQ(c.leveling_source, "Main::CorrectingPoints");
  EXPECT_TRUE(c.map_pose.valid);
  EXPECT_TRUE(c.map_pose.map_from_base.translation() == Vec(1, 2, 3));
  EXPECT_TRUE(c.ego_linear_base == Vec(3.5f, 0, 0));
  EXPECT_FALSE(c.twist_valid);
  const Rot level = asmc::lidar::rpy(.01, -.02, 0);
  frame.StoreLeveling(level);
  EXPECT_TRUE(frame.Current().applied_leveling == level);
}

TEST(ScanFrame, RecordTrackingFrameStoresThisScanFrame) {
  for (bool use_roll : {false, true}) {
    ScanFrame frame;
    const auto vehicle = MountedVehicle();
    frame.Configure(vehicle, kCalibration);
    const auto scan = BeginTrackScan(frame, 1., .4, .05, -.02, vehicle, use_roll);
    TrackedObject o;
    o.f32_CoreZ = -.7f;
    frame.RecordTrackingFrame(o);
    EXPECT_TRUE(o.tracking_frame_valid);
    EXPECT_EQ(o.tracking_height_input, -.7f);
    const Rot yaw_t = asmc::lidar::rpy(0, 0, scan.local.st_IMU.f32_Yaw_rad).transpose();
    const Rot expected_y = yaw_t * Attitude(scan.local) * scan.mount.rotation() * scan.level.transpose();
    const Vec expected_o = yaw_t * Attitude(scan.local) * scan.mount.translation();
    EXPECT_LT((o.tracking_yaw_from_input - expected_y).norm(), 1e-12);
    EXPECT_LT((o.tracking_origin_in_yaw - expected_o).norm(), 1e-12);
  }
}

// Previous-scan track recorded in `before`, re-expressed in `now`.
struct TwoScans {
  ScanFrame frame;
  TrackScan before, now;
  TrackedObject track;
};
void MakeTwoScans(TwoScans& t, const VehicleSpec& vehicle, bool use_roll, double yaw0, double yaw1) {
  t.frame.Configure(vehicle, kCalibration);
  t.before = BeginTrackScan(t.frame, 90., yaw0, .05, -.03, vehicle, use_roll);
  t.track = BoxDetection(t.before, kMatchedBox);
  t.track.s32_Id = 42;
  t.track.b_KfInitialized = true;
  t.track.s32_KfInitCount = 2;
  t.track.Eigen_KfState_CA << t.track.f32_CoreX, t.track.f32_CoreY, 1.2f, -.4f, .3f, .1f;
  for (int r = 0; r < 6; ++r)
    for (int c = 0; c < 6; ++c) t.track.Eigen_KfP_CA(r, c) = (r == c ? 1.f : 0.f) + 0.01f * (r + 1) * (c + 1);
  t.track.f32_PrevYaw = t.before.local.st_IMU.f32_Yaw_rad;
  t.frame.RecordTrackingFrame(t.track);
  t.now = BeginTrackScan(t.frame, 90.1, yaw1, -.02, .04, vehicle, use_roll);
}

TEST(ScanFrame, AlignTrackingStateMovesOnlyTheCopiedFilterState) {
  for (const auto& vehicle : {BaseVehicle(), MountedVehicle()})
    for (bool use_roll : {false, true}) {
      SCOPED_TRACE(use_roll ? "roll_pitch" : "pitch_only");
      TwoScans t;
      MakeTwoScans(t, vehicle, use_roll, .3, .36);
      auto o = t.track;
      t.frame.AlignTrackingState(o, t.now.local.st_IMU.f32_Yaw_rad);
      // Position (x, y, recorded height) as a point.
      const Vec p_prev(t.track.Eigen_KfState_CA(0), t.track.Eigen_KfState_CA(1), t.track.tracking_height_input);
      const Vec p_now = t.now.Point(t.before.World(p_prev));
      EXPECT_NEAR(o.Eigen_KfState_CA(0), p_now.x(), kPosTol);
      EXPECT_NEAR(o.Eigen_KfState_CA(1), p_now.y(), kPosTol);
      EXPECT_NEAR(o.tracking_height_input, p_now.z(), kPosTol);
      // Velocity, acceleration and covariance: top-left 2x2 of the change.
      const Eigen::Matrix2d c2 = Change(t.before, t.now).topLeftCorner<2, 2>();
      Eigen::Matrix<double, 6, 6> j = Eigen::Matrix<double, 6, 6>::Zero();
      for (int b = 0; b < 3; ++b) j.block<2, 2>(2 * b, 2 * b) = c2;
      const Eigen::Matrix<double, 6, 1> x = j * t.track.Eigen_KfState_CA.cast<double>();
      for (int k = 2; k < 6; ++k) EXPECT_NEAR(o.Eigen_KfState_CA(k), x(k), 1e-6) << k;
      const Eigen::Matrix<double, 6, 6> p = j * t.track.Eigen_KfP_CA.cast<double>() * j.transpose();
      EXPECT_LT((o.Eigen_KfP_CA.cast<double>() - p).cwiseAbs().maxCoeff(), 1e-6);
      EXPECT_EQ(o.f32_PrevYaw, t.now.local.st_IMU.f32_Yaw_rad);
      // Re-recorded with this scan's frame.
      TrackedObject record;
      t.frame.RecordTrackingFrame(record);
      EXPECT_TRUE(o.tracking_yaw_from_input == record.tracking_yaw_from_input);
      EXPECT_TRUE(o.tracking_origin_in_yaw == record.tracking_origin_in_yaw);
      // The OBB (a current detection for matched tracks) is untouched.
      EXPECT_EQ(o.f32_OBB_YawRad, t.track.f32_OBB_YawRad);
      EXPECT_EQ(o.f32_OBB_CenterX, t.track.f32_OBB_CenterX);
      EXPECT_EQ(o.f32_OBB_ZMin, t.track.f32_OBB_ZMin);
      for (int i = 0; i < 8; ++i) EXPECT_EQ(o.OBB_Corners_Body[i].z, t.track.OBB_Corners_Body[i].z);
    }
}

TEST(ScanFrame, AlignTrackingFrameCarriesTheWholeBoxOnce) {
  for (const auto& vehicle : {BaseVehicle(), MountedVehicle()})
    for (bool use_roll : {false, true}) {
      SCOPED_TRACE(use_roll ? "roll_pitch" : "pitch_only");
      TwoScans t;
      MakeTwoScans(t, vehicle, use_roll, .3, .36);
      auto box = t.track;
      auto state = t.track;
      t.frame.AlignTrackingFrame(box, t.now.local.st_IMU.f32_Yaw_rad);
      t.frame.AlignTrackingState(state, t.now.local.st_IMU.f32_Yaw_rad);
      ExpectCarriedBox(box, t.now, kMatchedBox, t.track.f32_OBB_ZMax - t.track.f32_OBB_ZMin, false);
      ExpectObbSelfConsistent(box, "carried");
      EXPECT_NEAR(box.f32_OBB_ZMin - t.track.f32_OBB_ZMin, box.f32_OBB_CenterZ - t.track.f32_OBB_CenterZ, 1e-6)
          << "Z range follows the centre";
      // The filter state takes exactly the AlignTrackingState change.
      EXPECT_TRUE(box.Eigen_KfState_CA == state.Eigen_KfState_CA);
      EXPECT_TRUE(box.Eigen_KfP_CA == state.Eigen_KfP_CA);
      EXPECT_EQ(box.tracking_height_input, state.tracking_height_input);
      EXPECT_EQ(box.f32_PrevYaw, state.f32_PrevYaw);
      // Not re-expressed (preserved): W/H, AABB, points, CoreZ.
      EXPECT_EQ(box.f32_OBB_Width, t.track.f32_OBB_Width);
      EXPECT_EQ(box.f32_MinX, t.track.f32_MinX);
      EXPECT_EQ(box.pcl_ClusterPoints.points[0].x, t.track.pcl_ClusterPoints.points[0].x);
      EXPECT_EQ(box.f32_CoreZ, t.track.f32_CoreZ);
      // Yaw stays an atan2 angle in (-pi, pi], not axis-normalized.
      EXPECT_GT(box.f32_OBB_YawRad, -M_PI - 1e-6);
      EXPECT_LE(box.f32_OBB_YawRad, M_PI + 1e-6);
    }
}

TEST(ScanFrame, UnrecordedObjectIsLeftUnchanged) {
  TwoScans t;
  MakeTwoScans(t, BaseVehicle(), true, .3, .36);
  auto o = t.track;
  o.tracking_frame_valid = false;
  const auto before = o;
  t.frame.ResetEdgeFollowStats();
  t.frame.AlignTrackingFrame(o, .36);
  t.frame.AlignTrackingState(o, .36);
  EXPECT_TRUE(o.Eigen_KfState_CA == before.Eigen_KfState_CA);
  EXPECT_EQ(o.f32_PrevYaw, before.f32_PrevYaw);
  EXPECT_EQ(o.f32_OBB_YawRad, before.f32_OBB_YawRad);
  EXPECT_EQ(o.OBB_Corners_Body[3].x, before.OBB_Corners_Body[3].x);
  EXPECT_EQ(t.frame.edge_follow_stats().calls, 1u);
  EXPECT_EQ(t.frame.edge_follow_stats().not_recorded, 1u);
}

TEST(ScanFrame, EdgeFollowStatsCountEveryCall) {
  TwoScans t;
  MakeTwoScans(t, MountedVehicle(), false, .3, .36);  // tilted frame change
  t.frame.ResetEdgeFollowStats();
  // 1) Box in computeOBBAtYaw order: small non-zero correction applied.
  auto box = t.track;
  t.frame.AlignTrackingFrame(box, t.now.local.st_IMU.f32_Yaw_rad);
  // 2) Value-initialized yaw-prior copy (all corners at the origin): degenerate.
  TrackedObject prior{};
  prior.tracking_frame_valid = true;
  prior.tracking_yaw_from_input = t.track.tracking_yaw_from_input;
  prior.tracking_origin_in_yaw = t.track.tracking_origin_in_yaw;
  prior.f32_PrevYaw = t.track.f32_PrevYaw;
  prior.f32_OBB_YawRad = t.track.f32_OBB_YawRad;
  t.frame.AlignTrackingFrame(prior, t.now.local.st_IMU.f32_Yaw_rad);
  // 3) Foreign corner order (0->1 is the short edge): rejected, ~pi/2.
  auto foreign = t.track;
  std::rotate(foreign.OBB_Corners_Body.begin(), foreign.OBB_Corners_Body.begin() + 1,
              foreign.OBB_Corners_Body.begin() + 4);
  std::rotate(foreign.OBB_Corners_Body.begin() + 4, foreign.OBB_Corners_Body.begin() + 5,
              foreign.OBB_Corners_Body.end());
  t.frame.AlignTrackingFrame(foreign, t.now.local.st_IMU.f32_Yaw_rad);
  // 4) Unrecorded.
  auto unrecorded = t.track;
  unrecorded.tracking_frame_valid = false;
  t.frame.AlignTrackingFrame(unrecorded, t.now.local.st_IMU.f32_Yaw_rad);

  const auto& s = t.frame.edge_follow_stats();
  EXPECT_EQ(s.calls, 4u);
  EXPECT_EQ(s.applied, 1u);
  EXPECT_EQ(s.degenerate_edge, 1u);
  EXPECT_EQ(s.rejected, 1u);
  EXPECT_EQ(s.not_recorded, 1u);
  EXPECT_EQ(s.calls, s.applied + s.degenerate_edge + s.rejected + s.not_recorded);
  EXPECT_GT(s.max_applied_abs_rad, 1e-7) << "tilted change: the 2D axis differs from the edge";
  EXPECT_LT(s.max_applied_abs_rad, .1);
  EXPECT_NEAR(s.max_rejected_abs_rad, M_PI / 2, .05);
  // The degenerate prior is the 2D axis rotation only.
  const Vec a = Change(t.before, t.now) *
                Vec(std::cos(t.track.f32_OBB_YawRad), std::sin(t.track.f32_OBB_YawRad), 0);
  EXPECT_NEAR(prior.f32_OBB_YawRad, std::atan2(a.y(), a.x()), kAngTol);
  t.frame.ResetEdgeFollowStats();
  EXPECT_EQ(t.frame.edge_follow_stats().calls, 0u);
}

// Port of Main test_frame_boundary_ros.cpp:146-179.
TEST(FrameBoundary, TrackingReexpressesRollPitchAndNonzeroMountWithoutEgoDoubleCompensation) {
  VehicleSpec vehicle = BaseVehicle();
  vehicle.st_LiDARRotationDeg = {4.f, -3.f, 8.f};
  ScanFrame frame;
  frame.Configure(vehicle, kCalibration);
  const auto mount = ScanFrame::SensorMount(vehicle);
  const Vec fixed_in_world(12, 2, 1);
  auto local = Local(.2);
  local.st_IMU.f32_Roll_rad = .1;
  local.st_IMU.f32_Pitch_rad = -.08;
  asmc::lidar::PoseSelection unavailable;
  unavailable.reason = "gps_missing";
  frame.Prepare(Time(25.), local, unavailable, 0);
  const auto old_level = asmc::lidar::leveling_rotation(.1, -.08, mount.rotation());
  frame.StoreLeveling(old_level);
  const auto old_point =
      (old_level * mount.inverse() * (ScanFrame::MapRotation(local).transpose() * fixed_in_world)).eval();
  auto previous = Detection(old_point.x(), old_point.y());
  previous.f32_CoreZ = old_point.z();
  previous.s32_Id = 42;
  previous.b_KfInitialized = true;
  previous.s32_KfInitCount = 2;
  previous.Eigen_KfState_CA << old_point.x(), old_point.y(), 0, 0, 0, 0;
  previous.f32_PrevYaw = .2;
  frame.RecordTrackingFrame(previous);
  local = Local(.28);
  local.st_IMU.f32_Roll_rad = -.12;
  local.st_IMU.f32_Pitch_rad = .06;
  frame.Prepare(Time(25.1), local, unavailable, 0);
  const auto new_level = asmc::lidar::leveling_rotation(-.12, .06, mount.rotation());
  frame.StoreLeveling(new_level);
  const auto new_point =
      (new_level * mount.inverse() * (ScanFrame::MapRotation(local).transpose() * fixed_in_world)).eval();
  auto aligned = previous;
  frame.AlignTrackingFrame(aligned, .28);
  EXPECT_NEAR(aligned.Eigen_KfState_CA(0), new_point.x(), 1e-5);
  EXPECT_NEAR(aligned.Eigen_KfState_CA(1), new_point.y(), 1e-5);
  EXPECT_NEAR(aligned.tracking_height_input, new_point.z(), 1e-5);
  // CAN=2m/s does not subtract another .2m from the relative CA state.
  auto current = Detection(new_point.x(), new_point.y());
  current.f32_CoreZ = new_point.z();
  TrackingConfig config;
  config.f32_MergedTrackRetireDistance = .5f;  // Main test TrackingConfig()
  Tracker tracker(config);
  tracker.Commit({previous});
  std::vector<TrackedObject> detections{current};
  tracker.Update(detections, .1, local, frame);
  ASSERT_FALSE(detections.empty());
  EXPECT_EQ(detections.front().s32_Id, 42);
  EXPECT_NEAR(detections.front().f32_CoreX, new_point.x(), 1e-5);
}

// Port of Main test_frame_boundary_ros.cpp:357-413 (tracker fix 8f93c22e).
TEST(FrameBoundary, KalmanUpdateKeepsCurrentObbWhilePredictCarriesPreviousObb) {
  for (bool roll : {false, true}) {
    SCOPED_TRACE(roll ? "roll_pitch" : "pitch_only");
    ScanFrame frame;
    const auto vehicle = BaseVehicle();
    frame.Configure(vehicle, kCalibration);
    const auto before = BeginTrackScan(frame, 90., .3, .05, -.03, vehicle, roll);
    auto track = BoxDetection(before, kMatchedBox);
    track.s32_Id = 42;
    track.b_KfInitialized = true;
    track.s32_KfInitCount = 2;
    track.Eigen_KfState_CA << track.f32_CoreX, track.f32_CoreY, 0, 0, 0, 0;
    track.f32_PrevYaw = before.local.st_IMU.f32_Yaw_rad;
    frame.RecordTrackingFrame(track);
    const auto now = BeginTrackScan(frame, 90.1, .36, -.02, .04, vehicle, roll);
    // Matched: the OBB is this scan's detection; only the copied filter state is old.
    auto matched = BoxDetection(now, kMatchedBox);
    matched.Eigen_KfState_CA = track.Eigen_KfState_CA;
    matched.Eigen_KfP_CA = track.Eigen_KfP_CA;
    matched.b_KfInitialized = true;
    matched.s32_KfInitCount = 2;
    matched.f32_PrevYaw = track.f32_PrevYaw;
    matched.tracking_frame_valid = true;
    matched.tracking_height_input = track.tracking_height_input;
    matched.tracking_yaw_from_input = track.tracking_yaw_from_input;
    matched.tracking_origin_in_yaw = track.tracking_origin_in_yaw;
    const auto detected = matched;
    kalman_filter_update(matched, matched.f32_CoreX, matched.f32_CoreY, matched.f32_CoreX, matched.f32_CoreY, .1,
                         now.local.st_IMU.f32_Yaw_deg, frame);
    EXPECT_FLOAT_EQ(matched.f32_OBB_YawRad, detected.f32_OBB_YawRad) << "ego yaw applied again to a current OBB";
    EXPECT_FLOAT_EQ(matched.f32_OBB_CenterX, detected.f32_OBB_CenterX);
    EXPECT_FLOAT_EQ(matched.f32_OBB_CenterY, detected.f32_OBB_CenterY);
    EXPECT_FLOAT_EQ(matched.f32_OBB_CenterZ, detected.f32_OBB_CenterZ);
    EXPECT_FLOAT_EQ(matched.f32_OBB_ZMin, detected.f32_OBB_ZMin);
    EXPECT_FLOAT_EQ(matched.f32_OBB_ZMax, detected.f32_OBB_ZMax);
    for (unsigned i = 0; i < 8; ++i) {
      EXPECT_FLOAT_EQ(matched.OBB_Corners_Body[i].x, detected.OBB_Corners_Body[i].x);
      EXPECT_FLOAT_EQ(matched.OBB_Corners_Body[i].y, detected.OBB_Corners_Body[i].y);
      EXPECT_FLOAT_EQ(matched.OBB_Corners_Body[i].z, detected.OBB_Corners_Body[i].z);
    }
    ExpectObbSelfConsistent(matched, "matched");
    // The copied filter state is re-expressed once: its prior equals this scan's
    // measurement of the static box, so the update keeps it.
    EXPECT_NEAR(matched.Eigen_KfState_CA(0), detected.f32_CoreX, 1e-5);
    EXPECT_NEAR(matched.Eigen_KfState_CA(1), detected.f32_CoreY, 1e-5);
    // Predicted-only: the whole previous-scan OBB is carried by one frame change.
    frame.ResetEdgeFollowStats();
    auto predicted = track;
    kalman_filter_predict(predicted, .1, now.local.st_IMU.f32_Yaw_deg, frame);
    ExpectObbSelfConsistent(predicted, "predicted");
    ExpectCarriedBox(predicted, now, kMatchedBox, track.f32_OBB_ZMax - track.f32_OBB_ZMin);
    EXPECT_EQ(frame.edge_follow_stats().applied, 1u);
    // With a relative velocity only the KF state is motion-predicted; the OBB is
    // re-expressed, not moved, so the CA displacement is not applied to it.
    auto moving = track;
    moving.Eigen_KfState_CA(2) = 1.5;
    moving.Eigen_KfState_CA(3) = -.5;
    kalman_filter_predict(moving, .1, now.local.st_IMU.f32_Yaw_deg, frame);
    ExpectCarriedBox(moving, now, kMatchedBox, track.f32_OBB_ZMax - track.f32_OBB_ZMin, false);
    const auto center = now.Point(kMatchedBox.center);
    const double dx = moving.Eigen_KfState_CA(0) - center.x(), dy = moving.Eigen_KfState_CA(1) - center.y();
    // Main's tolerance (1e-4): the 2x2 Jacobian of a tilted change is not
    // orthonormal, so |v| itself changes by ~1e-3 relative.
    EXPECT_NEAR(std::hypot(dx, dy), .1 * std::hypot(1.5, .5), 1e-4) << "KF moves by v*dt once";
    // Exact: displacement = dt * C2 * v with the independently computed change.
    const Eigen::Vector2d v_now = Change(before, now).topLeftCorner<2, 2>() * Eigen::Vector2d(1.5, -.5);
    EXPECT_NEAR(dx, .1 * v_now.x(), kPosTol);
    EXPECT_NEAR(dy, .1 * v_now.y(), kPosTol);
    // A corner order other than computeOBBAtYaw's must not snap the yaw by 90 deg.
    frame.ResetEdgeFollowStats();
    auto foreign = track;
    std::rotate(foreign.OBB_Corners_Body.begin(), foreign.OBB_Corners_Body.begin() + 1,
                foreign.OBB_Corners_Body.begin() + 4);
    std::rotate(foreign.OBB_Corners_Body.begin() + 4, foreign.OBB_Corners_Body.begin() + 5,
                foreign.OBB_Corners_Body.end());
    kalman_filter_predict(foreign, .1, now.local.st_IMU.f32_Yaw_deg, frame);
    const auto heading = now.Direction(Heading(kMatchedBox));
    // Main's tolerances: without the edge follow the 2D axis rotation keeps the
    // tilt projection error (5e-3 pitch-only).
    EXPECT_LT(AxisError(foreign.f32_OBB_YawRad, std::atan2(heading.y(), heading.x())), roll ? 1e-5 : 5e-3)
        << "yaw follows the rotated axis, not the foreign 0->1 edge";
    const Vec axis = Change(before, now) * Vec(std::cos(track.f32_OBB_YawRad), std::sin(track.f32_OBB_YawRad), 0);
    EXPECT_NEAR(foreign.f32_OBB_YawRad, std::atan2(axis.y(), axis.x()), kAngTol) << "exact 2D axis rotation";
    EXPECT_EQ(frame.edge_follow_stats().rejected, 1u) << "corner-order violation is counted";
  }
}

}  // namespace

int main(int argc, char** argv) { testing::InitGoogleTest(&argc, argv); return RUN_ALL_TESTS(); }
