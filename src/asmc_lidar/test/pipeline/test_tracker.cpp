#include <asmc_lidar/detection/detections.hpp>
#include <asmc_lidar/tracking/track.hpp>
// Tracker port tests: Hungarian, Kalman, detections/consolidation, class carry,
// track lifecycle, the tracker-fix scenarios of Main test_frame_boundary_ros.cpp
// (8f93c22e), the applicable cases of Main test_tracking_association.cpp, and
// regression scenarios. Expected geometry is computed independently from a
// world model: p_scan = L * mount^-1 * (R_imu^T * w) (zero ego translation),
// R_imu = rpy of the stored float IMU angles, mount from the float Deg2Rad of
// the vehicle angles, L = asmc::lidar::leveling_rotation.
//
// ONE coordinate rule is checked for every output track (matched and carried):
// yaw parallel to corner edge 0->1 (mod pi), corner mean == centre, corners
// counter-clockwise from +z with 0-3 the bottom face, 0.5 (ZMin+ZMax) == centre
// z, and the whole box == the independently expected box in THIS scan.
// ZMin/ZMax equal the corner z for detected/stabilized boxes and for carried
// boxes in gravity-aligned frames; a carried box under a tilted frame change has
// tilted corners while ZMin/ZMax keep centre z -+ extent/2 (Main rule, see
// RunScenario).
#include <asmc_lidar/tracking/hungarian.hpp>
#include <asmc_lidar/tracking/kalman.hpp>
#include <asmc_lidar/detection/obb.hpp>
#include <asmc_lidar/geometry/scan_frame.hpp>
#include <asmc_lidar/tracking/tracker.hpp>
#include <asmc_lidar/geometry/projection.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <string>
#include <vector>

namespace {
using namespace asmc::lidar::pipeline;
using asmc::lidar::Pose;
using asmc::lidar::Rot;
using asmc::lidar::Vec;

constexpr double kPosTol = 1e-5;  // m
constexpr double kAngTol = 1e-5;  // rad
// Velocity of a static box in these synthetic scans: the only innovation is
// float storage of positions (~1e-6 m at 16 m) divided by dt = 0.1 s and
// scaled by the KF gain (< 10 /s), so |v| stays well below 1e-3 m/s.
constexpr double kStaticVelTol = 1e-3;  // m/s
const char* const kCalibration = "test-calibration";
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

// ------------------------------------------------------------------ model --
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
};
TrackScan BeginTrackScan(ScanFrame& frame, double time, double yaw, double roll, double pitch,
                         const VehicleSpec& vehicle, bool use_roll,
                         const asmc::lidar::PoseSelection* map = nullptr) {
  TrackScan scan;
  scan.local = Local(yaw);
  scan.local.st_IMU.f32_Roll_rad = roll;
  scan.local.st_IMU.f32_Pitch_rad = pitch;
  scan.mount = MountOf(vehicle);
  scan.level = asmc::lidar::leveling_rotation(scan.local.st_IMU.f32_Roll_rad, scan.local.st_IMU.f32_Pitch_rad,
                                              scan.mount.rotation(), use_roll);
  asmc::lidar::PoseSelection unavailable;
  unavailable.reason = "gps_missing";
  frame.Prepare(Time(time), scan.local, map ? *map : unavailable, 0);
  frame.StoreLeveling(scan.level);
  return scan;
}
// Frame change previous -> now for direction vectors; eps = PrevYaw - scan yaw
// of the previous record (Main mixes f32_Yaw_deg- and f32_Yaw_rad-derived yaws).
Rot Change(const TrackScan& previous, const TrackScan& now, double eps = 0.) {
  return now.level * now.mount.rotation().transpose() * Attitude(now.local).transpose() *
         asmc::lidar::rpy(0, 0, eps) * Attitude(previous.local) * previous.mount.rotation() *
         previous.level.transpose();
}

struct WorldBox {
  Vec center;
  double heading, length, width, height;
};
Vec Heading(const WorldBox& box) { return {std::cos(box.heading), std::sin(box.heading), 0}; }
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
// Main test_frame_boundary_ros.cpp:220-248 BoxDetection (raw yaw = projected heading).
TrackedObject BoxDetection(const TrackScan& scan, const WorldBox& box) {
  TrackedObject out{};
  out.tracking.s32_Id = -1;
  out.camera.str_ObjectClass = "car";
  out.camera.str_YoloClass = "car";
  const auto world = WorldCorners(box);
  double low = 1e9, high = -1e9;
  out.detection.f32_MinX = out.detection.f32_MinY = 1e9;
  out.detection.f32_MaxX = out.detection.f32_MaxY = -1e9;
  for (unsigned i = 0; i < 8; ++i) {
    const auto p = scan.Point(world[i]);
    out.detection.obb_corners_level[i] = {float(p.x()), float(p.y()), float(p.z())};
    low = std::min(low, p.z());
    high = std::max(high, p.z());
    out.detection.f32_MinX = std::min<float>(out.detection.f32_MinX, p.x());
    out.detection.f32_MaxX = std::max<float>(out.detection.f32_MaxX, p.x());
    out.detection.f32_MinY = std::min<float>(out.detection.f32_MinY, p.y());
    out.detection.f32_MaxY = std::max<float>(out.detection.f32_MaxY, p.y());
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
    out.detection.pcl_ClusterPoints.push_back(p);
  }
  for (const auto& e : edges) {
    pcl::PointXYZI p;
    const auto q = scan.Point(.5 * (world[e[0]] + world[e[1]]));
    p.x = q.x();
    p.y = q.y();
    p.z = q.z();
    p.intensity = 1;
    out.detection.pcl_ClusterPoints.push_back(p);
  }
  const auto center = scan.Point(box.center), axis = scan.Direction(Heading(box));
  out.detection.f32_CoreX = out.detection.f32_OBB_CenterX = center.x();
  out.detection.f32_CoreY = out.detection.f32_OBB_CenterY = center.y();
  out.detection.f32_CoreZ = out.detection.f32_OBB_CenterZ = center.z();
  out.detection.obb_length_m = box.length;
  out.detection.obb_width_m = box.width;
  out.detection.f32_OBB_ZMin = low;
  out.detection.f32_OBB_ZMax = high;
  out.detection.f32_OBB_YawRad = std::atan2(axis.y(), axis.x());
  return out;
}
// Main test_frame_boundary_ros.cpp:32-44 Detection.
TrackedObject Detection(double x, double y) {
  TrackedObject out{};
  out.tracking.s32_Id = -1;
  out.camera.str_ObjectClass = "car";
  out.camera.str_YoloClass = "car";
  out.detection.f32_CoreX = out.detection.f32_OBB_CenterX = x;
  out.detection.f32_CoreY = out.detection.f32_OBB_CenterY = y;
  out.detection.f32_CoreZ = out.detection.f32_OBB_CenterZ = -.2;
  out.detection.obb_length_m = 1.;
  out.detection.obb_width_m = .8;
  out.detection.f32_OBB_ZMin = -.5;
  out.detection.f32_OBB_ZMax = .5;
  out.detection.f32_MinX = x - .5;
  out.detection.f32_MaxX = x + .5;
  out.detection.f32_MinY = y - .4;
  out.detection.f32_MaxY = y + .4;
  for (unsigned i = 0; i < 8; ++i) {
    out.detection.obb_corners_level[i] = {float(x + ((i & 1) ? .5 : -.5)), float(y + ((i & 2) ? .4 : -.4)),
                               float((i & 4) ? .5 : -.5)};
    pcl::PointXYZI p;
    p.x = out.detection.obb_corners_level[i].x;
    p.y = out.detection.obb_corners_level[i].y;
    p.z = out.detection.obb_corners_level[i].z;
    p.intensity = 1;
    out.detection.pcl_ClusterPoints.push_back(p);
  }
  return out;
}
// Main test_tracking_association.cpp:10-37 MakeDetection.
TrackedObject MakeDetection(float center_x, float center_y, float width, float height, float z_offset) {
  TrackedObject object{};
  object.detection.f32_CoreX = center_x;
  object.detection.f32_CoreY = center_y;
  object.detection.f32_CoreZ = z_offset;
  object.detection.f32_MinX = center_x - width * 0.5F;
  object.detection.f32_MaxX = center_x + width * 0.5F;
  object.detection.f32_MinY = center_y - height * 0.5F;
  object.detection.f32_MaxY = center_y + height * 0.5F;
  object.detection.f32_MinZ = z_offset;
  object.detection.f32_MaxZ = z_offset + 0.2F;
  object.detection.obb_length_m = width;
  object.detection.obb_width_m = height;
  for (int x_index = 0; x_index < 4; ++x_index) {
    for (int y_index = 0; y_index < 3; ++y_index) {
      pcl::PointXYZI point;
      point.x = object.detection.f32_MinX + width * static_cast<float>(x_index) / 3.0F;
      point.y = object.detection.f32_MinY + height * static_cast<float>(y_index) / 2.0F;
      point.z = z_offset;
      object.detection.pcl_ClusterPoints.push_back(point);
    }
  }
  return object;
}
TrackingConfig MainTestConfig() {  // Main test_frame_boundary_ros.cpp:45-51
  TrackingConfig c;
  c.b_UseMapAssociation = true;
  c.f32_COST_W_DIST = 1;
  c.f32_COST_W_IOU = 1;
  c.f32_COST_THRESHOLD = 3;
  c.f32_COST_INF = 9999;
  c.f32_ContinuityBonus = .2f;
  c.f32_MergedTrackRetireDistance = .5f;
  c.f32_DuplicateIouThreshold = .35f;
  return c;
}

double AxisError(double a, double b) { return std::abs(std::remainder(a - b, M_PI)); }
double NormalizeAxis(double yaw) {
  while (yaw >= M_PI / 2) yaw -= M_PI;
  while (yaw < -M_PI / 2) yaw += M_PI;
  return yaw;
}
bool Finite(const TrackedObject& o) {
  bool ok = std::isfinite(o.detection.f32_OBB_CenterX) && std::isfinite(o.detection.f32_OBB_CenterY) && std::isfinite(o.detection.f32_OBB_CenterZ) &&
            std::isfinite(o.detection.obb_length_m) && std::isfinite(o.detection.obb_width_m) && std::isfinite(o.detection.f32_OBB_ZMin) &&
            std::isfinite(o.detection.f32_OBB_ZMax) && std::isfinite(o.detection.f32_OBB_YawRad) && std::isfinite(o.detection.f32_CoreX) &&
            std::isfinite(o.detection.f32_CoreY) && std::isfinite(o.detection.f32_CoreZ) && std::isfinite(o.detection.f32_Dist) &&
            std::isfinite(o.tracking.f32_AbsVx) && std::isfinite(o.tracking.f32_AbsVy) && std::isfinite(o.tracking.f32_Velocity) &&
            std::isfinite(o.tracking.f32_DirX) && std::isfinite(o.tracking.f32_DirY) && std::isfinite(o.tracking.f32_PrevYaw) &&
            o.tracking.Eigen_KfState_CA.allFinite() && o.tracking.Eigen_KfP_CA.allFinite();
  for (const auto& c : o.detection.obb_corners_level) ok = ok && std::isfinite(c.x) && std::isfinite(c.y) && std::isfinite(c.z);
  return ok;
}
const TrackedObject* FindId(const std::vector<TrackedObject>& objects, int id) {
  for (const auto& o : objects)
    if (o.tracking.s32_Id == id) return &o;
  return nullptr;
}

// Expected box (double).
struct Expected {
  std::array<Vec, 8> corners;
  Vec center = Vec::Zero();
  double yaw = 0, zmin = 0, zmax = 0, w = -1, h = -1;
};
Expected FromDetection(const TrackedObject& d) {
  Expected e;
  for (int i = 0; i < 8; ++i) e.corners[i] = Vec(d.detection.obb_corners_level[i].x, d.detection.obb_corners_level[i].y, d.detection.obb_corners_level[i].z);
  e.center = Vec(d.detection.f32_OBB_CenterX, d.detection.f32_OBB_CenterY, d.detection.f32_OBB_CenterZ);
  e.yaw = d.detection.f32_OBB_YawRad;
  e.zmin = d.detection.f32_OBB_ZMin;
  e.zmax = d.detection.f32_OBB_ZMax;
  return e;
}
// Definition of a refit box: the smallest box with axis `yaw` (already
// normalized) holding every finite point, sides floored at 0.05 m, corners
// (-,-) (+,-) (+,+) (-,+) along (u, v), 0-3 at the lowest z, 4-7 at the highest.
Expected ReferenceBoxAtYaw(const Cloud& cloud, double yaw) {
  const double c = std::cos(yaw), s = std::sin(yaw);
  double umin = 1e300, umax = -1e300, vmin = 1e300, vmax = -1e300, zmin = 1e300, zmax = -1e300;
  for (const auto& p : cloud) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
    const double u = p.x * c + p.y * s, v = -p.x * s + p.y * c;
    umin = std::min(umin, u);
    umax = std::max(umax, u);
    vmin = std::min(vmin, v);
    vmax = std::max(vmax, v);
    zmin = std::min<double>(zmin, p.z);
    zmax = std::max<double>(zmax, p.z);
  }
  Expected e;
  e.w = std::max(umax - umin, .05);
  e.h = std::max(vmax - vmin, .05);
  const double cu = .5 * (umin + umax), cv = .5 * (vmin + vmax);
  e.center = Vec(cu * c - cv * s, cu * s + cv * c, .5 * (zmin + zmax));
  e.yaw = yaw;
  e.zmin = zmin;
  e.zmax = zmax;
  const double su[] = {-1, 1, 1, -1}, sv[] = {-1, -1, 1, 1};
  for (int i = 0; i < 4; ++i) {
    const double x = e.center.x() + su[i] * .5 * e.w * c - sv[i] * .5 * e.h * s;
    const double y = e.center.y() + su[i] * .5 * e.w * s + sv[i] * .5 * e.h * c;
    e.corners[i] = Vec(x, y, zmin);
    e.corners[i + 4] = Vec(x, y, zmax);
  }
  return e;
}
Expected Carry(const Expected& last, const TrackScan& last_scan, const TrackScan& scan) {
  Expected e;
  for (int i = 0; i < 8; ++i) e.corners[i] = scan.Point(last_scan.World(last.corners[i]));
  e.center = scan.Point(last_scan.World(last.center));
  const double extent = last.zmax - last.zmin;
  e.zmin = e.center.z() - .5 * extent;
  e.zmax = e.center.z() + .5 * extent;
  const Vec edge = e.corners[1] - e.corners[0];
  e.yaw = std::atan2(edge.y(), edge.x());
  return e;
}

void ExpectCcwFromPlusZ(const TrackedObject& o, const std::string& role) {
  for (int face = 0; face < 2; ++face)
    for (int i = 0; i < 4; ++i) {
      const auto& a = o.detection.obb_corners_level[4 * face + i];
      const auto& b = o.detection.obb_corners_level[4 * face + (i + 1) % 4];
      const auto& c = o.detection.obb_corners_level[4 * face + (i + 2) % 4];
      EXPECT_GT((b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x), 0.) << role << " face " << face << " corner " << i;
    }
  for (int i = 0; i < 4; ++i) EXPECT_GT(o.detection.obb_corners_level[i + 4].z, o.detection.obb_corners_level[i].z) << role << " 0-3 bottom";
}
// The coordinate rule shared by every OBB in one scan frame.
void ExpectRule(const TrackedObject& o, const std::string& role, bool z_equals_corner_z) {
  const auto& c0 = o.detection.obb_corners_level[0];
  const auto& c1 = o.detection.obb_corners_level[1];
  EXPECT_LT(AxisError(o.detection.f32_OBB_YawRad, std::atan2(c1.y - c0.y, c1.x - c0.x)), kAngTol) << role << " yaw vs edge 0->1";
  Vec mean = Vec::Zero();
  for (const auto& c : o.detection.obb_corners_level) mean += Vec(c.x, c.y, c.z) / 8.;
  EXPECT_NEAR(mean.x(), o.detection.f32_OBB_CenterX, kPosTol) << role << " corner mean";
  EXPECT_NEAR(mean.y(), o.detection.f32_OBB_CenterY, kPosTol) << role << " corner mean";
  EXPECT_NEAR(mean.z(), o.detection.f32_OBB_CenterZ, kPosTol) << role << " corner mean";
  EXPECT_NEAR(.5 * (o.detection.f32_OBB_ZMin + o.detection.f32_OBB_ZMax), o.detection.f32_OBB_CenterZ, kPosTol) << role << " Z range centre";
  if (z_equals_corner_z) {
    for (int i = 0; i < 4; ++i) {
      EXPECT_NEAR(o.detection.obb_corners_level[i].z, o.detection.f32_OBB_ZMin, kPosTol) << role << " bottom corner " << i;
      EXPECT_NEAR(o.detection.obb_corners_level[i + 4].z, o.detection.f32_OBB_ZMax, kPosTol) << role << " top corner " << i;
    }
  }
  ExpectCcwFromPlusZ(o, role);
  EXPECT_TRUE(Finite(o)) << role;
}
void ExpectBox(const TrackedObject& o, const Expected& e, const std::string& role, int shift = 0) {
  for (int i = 0; i < 8; ++i) {
    const int k = (i / 4) * 4 + (i % 4 + shift) % 4;
    EXPECT_NEAR(o.detection.obb_corners_level[i].x, e.corners[k].x(), kPosTol) << role << " corner " << i;
    EXPECT_NEAR(o.detection.obb_corners_level[i].y, e.corners[k].y(), kPosTol) << role << " corner " << i;
    EXPECT_NEAR(o.detection.obb_corners_level[i].z, e.corners[k].z(), kPosTol) << role << " corner " << i;
  }
  EXPECT_NEAR(o.detection.f32_OBB_CenterX, e.center.x(), kPosTol) << role;
  EXPECT_NEAR(o.detection.f32_OBB_CenterY, e.center.y(), kPosTol) << role;
  EXPECT_NEAR(o.detection.f32_OBB_CenterZ, e.center.z(), kPosTol) << role;
  EXPECT_LT(AxisError(o.detection.f32_OBB_YawRad, e.yaw), kAngTol) << role << " yaw";
  EXPECT_NEAR(o.detection.f32_OBB_ZMin, e.zmin, kPosTol) << role;
  EXPECT_NEAR(o.detection.f32_OBB_ZMax, e.zmax, kPosTol) << role;
  if (e.w >= 0) {
    EXPECT_NEAR(o.detection.obb_length_m, e.w, kPosTol) << role;
    EXPECT_NEAR(o.detection.obb_width_m, e.h, kPosTol) << role;
  }
}

// --------------------------------------------------------------- scenario --
struct TrackSpec {
  int id;
  WorldBox box;
  unsigned seen_scans;          // detected in scans [0, seen_scans)
  unsigned redetect_from = 0;   // and again from this scan on (0: never)
  bool Seen(unsigned s) const { return s < seen_scans || (redetect_from && s >= redetect_from); }
};
struct Scenario {
  std::string name;
  bool use_roll;
  double yaw0, yaw_step;
  VehicleSpec vehicle;
  std::vector<TrackSpec> tracks;
  unsigned scans;
  bool gravity_aligned;  // Main's world_yaw flag; checked against the model
  double roll_scale = 1.;
};
struct ScenarioResult {
  ScanFrame::EdgeFollowStats stats;
  std::uint64_t matched_priors = 0, carried_predictions = 0;
  double max_carried_tilt_z = 0;  // max |corner z - ZMin/ZMax| of carried boxes
  std::map<int, std::vector<double>> yaw;  // output yaw per track per scan
};

ScenarioResult RunScenario(const Scenario& c) {
  SCOPED_TRACE(c.name);
  ScenarioResult result;
  ScanFrame frame;
  frame.Configure(c.vehicle, kCalibration);
  Tracker tracker(MainTestConfig());
  struct Model {
    bool observed_last = false;
    unsigned last_seen = 0;
    TrackScan last_scan;
    Expected last;
    Vec last_kf = Vec::Zero();
    double yaw = 0;
    float last_w = 0, last_h = 0;
    double prev_yaw_eps = 0;  // f32_PrevYaw - scan yaw stored with the last record
  };
  std::map<int, Model> models;
  for (unsigned s = 0; s < c.scans; ++s) {
    SCOPED_TRACE("scan " + std::to_string(s));
    const double yaw = std::remainder(c.yaw0 + c.yaw_step * s, 2 * M_PI);
    const auto scan = BeginTrackScan(frame, 80. + .1 * s, yaw, c.roll_scale * .06 * std::sin(.7 * s + .3),
                                     -.04 * std::cos(.5 * s), c.vehicle, c.use_roll);
    std::vector<TrackedObject> objects;
    std::map<int, TrackedObject> detections;
    for (const auto& t : c.tracks) {
      if (!t.Seen(s)) continue;
      detections[t.id] = BoxDetection(scan, t.box);
      objects.push_back(detections[t.id]);
      if (c.gravity_aligned) {
        EXPECT_LT(std::abs(scan.Direction(Heading(t.box)).z()), 1e-9) << "gravity-aligned frame";
      }
    }
    if (s == 0) {
      for (unsigned i = 0; i < objects.size(); ++i) {
        auto& o = objects[i];
        o.tracking.s32_Id = c.tracks[i].id;
        o.tracking.s32_Age = 5;
        o.tracking.b_KfInitialized = true;
        o.tracking.s32_KfInitCount = 2;
        o.tracking.Eigen_KfState_CA << o.detection.f32_CoreX, o.detection.f32_CoreY, 0, 0, 0, 0;
        o.tracking.f32_PrevYaw = scan.local.st_IMU.f32_Yaw_rad;
      }
    } else {
      tracker.Update(objects, .1, scan.local, frame);
    }
    // LidarProcess records the current frame for observed tracks after tracking.
    for (auto& o : objects)
      if (o.tracking.s32_MissCount == 0) frame.RecordTrackingFrame(o);
    EXPECT_EQ(objects.size(), c.tracks.size()) << "no births and no lost tracks";

    for (const auto& t : c.tracks) {
      const std::string role = "track " + std::to_string(t.id);
      const TrackedObject* o = FindId(objects, t.id);
      if (!o) {
        ADD_FAILURE() << role << " missing";
        continue;
      }
      auto& m = models[t.id];
      EXPECT_EQ(o->tracking.s32_Age, int(5 + s)) << role;
      result.yaw[t.id].push_back(o->detection.f32_OBB_YawRad);
      if (t.Seen(s)) {
        const auto& det = detections[t.id];
        EXPECT_EQ(o->tracking.s32_MissCount, 0) << role;
        const bool raw = s == 0 || !m.observed_last;  // birth frame or first frame after a miss
        Expected e;
        if (raw) {
          e = FromDetection(det);
          m.yaw = det.detection.f32_OBB_YawRad;
        } else {
          // Matched prior: previous yaw axis re-expressed once (zero-corner
          // copy, no edge follow), then the stabilizer rule.
          const Vec a = Change(m.last_scan, scan, m.prev_yaw_eps) * Vec(std::cos(m.yaw), std::sin(m.yaw), 0);
          const double ego_delta = std::remainder(
              double(scan.local.st_IMU.f32_Yaw_deg) * M_PI / 180. - double(scan.local.st_IMU.f32_Yaw_rad), 2 * M_PI);
          const double pred = NormalizeAxis(std::atan2(a.y(), a.x()) - ego_delta);
          const double raw_yaw = det.detection.f32_OBB_YawRad;
          const double aspect = det.detection.obb_width_m > 1e-6f ? det.detection.obb_length_m / det.detection.obb_width_m
                                                           : std::numeric_limits<double>::infinity();
          double filtered = pred;
          if (aspect >= 1.2) filtered = pred + .25 * std::remainder(raw_yaw - pred, M_PI);
          m.yaw = NormalizeAxis(filtered);
          e = ReferenceBoxAtYaw(det.detection.pcl_ClusterPoints, m.yaw);
          ++result.matched_priors;
        }
        if (s > 0 && !m.observed_last) ++result.matched_priors;  // aligned copy also made after a miss
        ExpectRule(*o, role + " observed", !raw || c.gravity_aligned);
        const int shift = (!raw && std::abs(o->detection.f32_OBB_YawRad - e.yaw) > M_PI / 2) ? 2 : 0;
        ExpectBox(*o, e, role + " observed", shift);
        if (!raw) {
          EXPECT_GE(o->detection.f32_OBB_YawRad, -M_PI / 2) << role;
          EXPECT_LT(o->detection.f32_OBB_YawRad, M_PI / 2) << role;
          for (int i = 0; i < 4; ++i) {
            EXPECT_EQ(o->detection.obb_corners_level[i + 4].x, o->detection.obb_corners_level[i].x);
            EXPECT_EQ(o->detection.obb_corners_level[i + 4].y, o->detection.obb_corners_level[i].y);
          }
        }
        if (c.gravity_aligned) {
          const auto axis = scan.Direction(Heading(t.box));
          EXPECT_LT(AxisError(o->detection.f32_OBB_YawRad, std::atan2(axis.y(), axis.x())), kAngTol) << role << " vs heading";
          EXPECT_NEAR(o->detection.obb_length_m, t.box.length, kPosTol) << role;
          EXPECT_NEAR(o->detection.obb_width_m, t.box.width, kPosTol) << role;
        }
        // KF: the measurement is the detection core; Core is the KF position.
        if (s > 0) {
          EXPECT_NEAR(o->tracking.Eigen_KfState_CA(0), det.detection.f32_CoreX, kPosTol) << role << " KF";
          EXPECT_NEAR(o->tracking.Eigen_KfState_CA(1), det.detection.f32_CoreY, kPosTol) << role << " KF";
          EXPECT_EQ(o->detection.f32_CoreX, o->tracking.Eigen_KfState_CA(0));
          EXPECT_EQ(o->detection.f32_CoreY, o->tracking.Eigen_KfState_CA(1));
          EXPECT_LT(std::hypot(o->tracking.Eigen_KfState_CA(2), o->tracking.Eigen_KfState_CA(3)), kStaticVelTol) << role;
          EXPECT_FLOAT_EQ(o->tracking.f32_AbsVx, o->tracking.Eigen_KfState_CA(2) + 7.2f / 3.6f);
          EXPECT_FLOAT_EQ(o->tracking.f32_AbsVy, o->tracking.Eigen_KfState_CA(3));
          m.prev_yaw_eps = double(static_cast<float>(scan.local.st_IMU.f32_Yaw_deg * M_PI / 180.0f)) -
                           double(scan.local.st_IMU.f32_Yaw_rad);
        } else {
          m.prev_yaw_eps = 0.;
        }
        m.observed_last = true;
        m.last_seen = s;
        m.last_scan = scan;
        m.last = e;
        m.last_kf = Vec(det.detection.f32_CoreX, det.detection.f32_CoreY, det.detection.f32_CoreZ);
        m.last_w = o->detection.obb_length_m;
        m.last_h = o->detection.obb_width_m;
      } else {
        // Carried (predicted-only): last observed box re-expressed into this scan.
        ++result.carried_predictions;
        EXPECT_EQ(o->tracking.s32_MissCount, int(s - m.last_seen)) << role;
        const Expected e = Carry(m.last, m.last_scan, scan);
        ExpectRule(*o, role + " carried", c.gravity_aligned);
        ExpectBox(*o, e, role + " carried");
        EXPECT_NEAR(o->detection.f32_OBB_ZMax - o->detection.f32_OBB_ZMin, m.last.zmax - m.last.zmin, kPosTol) << role << " Z extent";
        EXPECT_GT(o->detection.f32_OBB_YawRad, -M_PI - 1e-6) << role;
        EXPECT_LE(o->detection.f32_OBB_YawRad, M_PI + 1e-6) << role;
        EXPECT_EQ(o->detection.obb_length_m, m.last_w) << role << " W not re-expressed";
        EXPECT_EQ(o->detection.obb_width_m, m.last_h) << role;
        const Vec kf = scan.Point(m.last_scan.World(m.last_kf));
        EXPECT_NEAR(o->tracking.Eigen_KfState_CA(0), kf.x(), kPosTol) << role << " KF";
        EXPECT_NEAR(o->tracking.Eigen_KfState_CA(1), kf.y(), kPosTol) << role << " KF";
        EXPECT_EQ(o->detection.f32_CoreX, o->tracking.Eigen_KfState_CA(0));
        for (int i = 0; i < 4; ++i) {
          result.max_carried_tilt_z = std::max<double>(
              result.max_carried_tilt_z, std::abs(o->detection.obb_corners_level[i].z - o->detection.f32_OBB_ZMin));
          result.max_carried_tilt_z = std::max<double>(
              result.max_carried_tilt_z, std::abs(o->detection.obb_corners_level[i + 4].z - o->detection.f32_OBB_ZMax));
        }
        m.observed_last = false;
      }
    }
    tracker.Commit(objects);
  }
  result.stats = frame.edge_follow_stats();
  const auto& st = result.stats;
  EXPECT_EQ(st.calls, st.not_recorded + st.degenerate_edge + st.applied + st.rejected);
  EXPECT_EQ(st.not_recorded, 0u);
  EXPECT_EQ(st.rejected, 0u) << "corner-order violation";
  EXPECT_EQ(st.degenerate_edge, result.matched_priors) << "one zero-corner prior copy per matched pair";
  EXPECT_EQ(st.applied, result.carried_predictions) << "one edge follow per carried track and scan";
  EXPECT_LT(st.max_applied_abs_rad, .1);
  if (c.gravity_aligned) {
    EXPECT_LT(st.max_applied_abs_rad, 1e-6) << "no tilt: edge == rotated axis";
  }
  ::testing::Test::RecordProperty(c.name + "_max_carried_tilt_z", std::to_string(result.max_carried_tilt_z));
  return result;
}

const WorldBox kMatchedBox{{12, 2, .25}, .4, 4.5, 1.8, 1.5}, kPredictedBox{{7, -6, .1}, -.9, 3., 1.2, 1.},
    kHandoverBox{{16, -1, .2}, 1.2, 2., 1.1, 1.3};
constexpr unsigned kHandoverScans = 3;
// Main RunTrackCase layout: matched 42 every scan, handover 44 scans 0..2,
// predicted 43 scan 0 only (same push order).
std::vector<TrackSpec> MainTracks(const WorldBox& matched, const WorldBox& predicted, const WorldBox& handover,
                                  unsigned scans) {
  return {{42, matched, scans}, {44, handover, kHandoverScans}, {43, predicted, 1}};
}

// ------------------------------------------------------------- Hungarian --
double BruteForce(const std::vector<std::vector<double>>& m) {
  const int n = static_cast<int>(m.size()), k = n ? std::min<int>(n, static_cast<int>(m[0].size())) : 0;
  double best = std::numeric_limits<double>::infinity();
  std::vector<bool> used(n ? m[0].size() : 0, false);
  std::function<void(int, int, double)> rec = [&](int row, int assigned, double cost) {
    if (row == n) {
      if (assigned == k) best = std::min(best, cost);
      return;
    }
    if (n - row > k - assigned) rec(row + 1, assigned, cost);  // leave this row unassigned
    for (std::size_t col = 0; col < used.size(); ++col) {
      if (used[col]) continue;
      used[col] = true;
      rec(row + 1, assigned + 1, cost + m[row][col]);
      used[col] = false;
    }
  };
  rec(0, 0, 0.);
  return best;
}
double AssignmentCost(const std::vector<std::vector<double>>& m, const std::vector<int>& a) {
  double c = 0;
  for (std::size_t i = 0; i < a.size(); ++i)
    if (a[i] >= 0) c += m[i][a[i]];
  return c;
}

TEST(Hungarian, SquareKnownOptimum) {
  std::vector<std::vector<double>> m{{4, 1, 3}, {2, 0, 5}, {3, 2, 2}};
  const auto copy = m;
  AssignmentProblemSolver solver;
  std::vector<int> a;
  const double cost = solver.Solve(m, a);
  EXPECT_EQ(a, (std::vector<int>{1, 0, 2}));
  EXPECT_DOUBLE_EQ(cost, 5.);
  EXPECT_EQ(m, copy) << "input matrix unchanged";
}

TEST(Hungarian, RectangularMoreDetectionsThanTracks) {
  std::vector<std::vector<double>> m{{1, 2, 3}, {2, 4, 6}};
  AssignmentProblemSolver solver;
  std::vector<int> a;
  EXPECT_DOUBLE_EQ(solver.Solve(m, a), 4.);
  EXPECT_EQ(a, (std::vector<int>{1, 0}));
}

TEST(Hungarian, RectangularMoreTracksThanDetections) {
  std::vector<std::vector<double>> m{{1, 2}, {2, 4}, {3, 6}};
  AssignmentProblemSolver solver;
  std::vector<int> a;
  EXPECT_DOUBLE_EQ(solver.Solve(m, a), 4.);
  EXPECT_EQ(a, (std::vector<int>{1, 0, -1}));
}

TEST(Hungarian, GatedEntriesAreStillAssignedAndRejectedByTheCaller) {
  std::vector<std::vector<double>> m{{.5, 9999}, {9999, 9999}};
  AssignmentProblemSolver solver;
  std::vector<int> a;
  EXPECT_DOUBLE_EQ(solver.Solve(m, a), 9999.5);
  EXPECT_EQ(a, (std::vector<int>{0, 1}));
  // Tracker side: two tracks far from the only detection near track 0.
  ScanFrame frame;
  frame.Configure(BaseVehicle(), kCalibration);
  BeginTrackScan(frame, 1., 0, 0, 0, BaseVehicle(), true);
  std::vector<TrackedObject> previous{Detection(10, 0), Detection(-20, 5)};
  for (int i = 0; i < 2; ++i) {
    previous[i].tracking.s32_Id = 10 + i;
    previous[i].tracking.s32_Age = 3;
    previous[i].tracking.b_KfInitialized = true;
    previous[i].tracking.s32_KfInitCount = 2;
    previous[i].tracking.Eigen_KfState_CA << previous[i].detection.f32_CoreX, previous[i].detection.f32_CoreY, 0, 0, 0, 0;
    frame.RecordTrackingFrame(previous[i]);
  }
  Tracker tracker(MainTestConfig());
  tracker.Commit(previous);
  BeginTrackScan(frame, 1.1, 0, 0, 0, BaseVehicle(), true);
  std::vector<TrackedObject> objects{Detection(10.1, 0), Detection(40, 40)};
  tracker.Update(objects, .1, Local(), frame);
  ASSERT_EQ(objects.size(), 3u);
  EXPECT_EQ(objects[0].tracking.s32_Id, 10);
  EXPECT_EQ(objects[1].tracking.s32_Id, 0) << "gated pair rejected: birth";
  EXPECT_EQ(objects[2].tracking.s32_Id, 11);
  EXPECT_EQ(objects[2].tracking.s32_MissCount, 1);
}

TEST(Hungarian, NoDetectionsGivesAllUnassigned) {
  std::vector<std::vector<double>> m(3, std::vector<double>());
  AssignmentProblemSolver solver;
  std::vector<int> a;
  EXPECT_DOUBLE_EQ(solver.Solve(m, a), 0.);
  EXPECT_EQ(a, (std::vector<int>{-1, -1, -1}));
}

TEST(Hungarian, NoTracksGivesEmptyAssignment) {  // deviation: Main reads DistMatrix[0]
  std::vector<std::vector<double>> m;
  AssignmentProblemSolver solver;
  std::vector<int> a{7};
  EXPECT_DOUBLE_EQ(solver.Solve(m, a), 0.);
  EXPECT_TRUE(a.empty());
}

TEST(Hungarian, TiesKeepMunkresOrder) {
  AssignmentProblemSolver solver;
  std::vector<int> a;
  std::vector<std::vector<double>> square(3, std::vector<double>(3, 1.));
  solver.Solve(square, a);
  EXPECT_EQ(a, (std::vector<int>{0, 1, 2}));
  std::vector<std::vector<double>> tall(3, std::vector<double>(2, 1.));
  solver.Solve(tall, a);
  EXPECT_EQ(a, (std::vector<int>{0, 1, -1}));
  std::vector<std::vector<double>> wide(2, std::vector<double>(3, 1.));
  solver.Solve(wide, a);
  EXPECT_EQ(a, (std::vector<int>{0, 1}));
}

TEST(Hungarian, RandomMatricesReachTheBruteForceOptimum) {
  std::uint32_t state = 12345;
  const auto next = [&state]() {
    state = state * 1664525u + 1013904223u;
    return state >> 8;
  };
  AssignmentProblemSolver solver;
  for (int trial = 0; trial < 400; ++trial) {
    const int n = 1 + next() % 5, m = 1 + next() % 5;
    std::vector<std::vector<double>> cost(n, std::vector<double>(m));
    for (auto& row : cost)
      for (auto& v : row) v = (next() % 5 == 0) ? 9999. : (next() % 1000) / 100.;
    std::vector<int> a;
    const auto copy = cost;
    const double total = solver.Solve(cost, a);
    ASSERT_EQ(a.size(), std::size_t(n));
    std::vector<bool> used(m, false);
    int assigned = 0;
    for (int i = 0; i < n; ++i) {
      if (a[i] < 0) continue;
      ASSERT_LT(a[i], m);
      EXPECT_FALSE(used[a[i]]) << "column used twice";
      used[a[i]] = true;
      ++assigned;
    }
    EXPECT_EQ(assigned, std::min(n, m));
    EXPECT_NEAR(total, AssignmentCost(copy, a), 1e-9);
    EXPECT_NEAR(total, BruteForce(copy), 1e-9) << "trial " << trial;
  }
}

// ---------------------------------------------------------------- Kalman --
struct KfRef {
  Eigen::Matrix<double, 6, 1> x;
  Eigen::Matrix<double, 6, 6> P;
  static Eigen::Matrix<double, 6, 6> A(double dt) {
    Eigen::Matrix<double, 6, 6> a = Eigen::Matrix<double, 6, 6>::Identity();
    a(0, 2) = a(1, 3) = a(2, 4) = a(3, 5) = dt;
    a(0, 4) = a(1, 5) = .5 * dt * dt;
    return a;
  }
  void Predict(double dt) {
    x = A(dt) * x;
    P = A(dt) * P * A(dt).transpose() + .003 * Eigen::Matrix<double, 6, 6>::Identity();
  }
  void Update(double zx, double zy) {
    Eigen::Matrix<double, 2, 6> H = Eigen::Matrix<double, 2, 6>::Zero();
    H(0, 0) = H(1, 1) = 1;
    const Eigen::Vector2d y = Eigen::Vector2d(zx, zy) - H * x;
    const Eigen::Matrix2d S = H * P * H.transpose() + .01 * Eigen::Matrix2d::Identity();
    const Eigen::Matrix<double, 6, 2> K = P * H.transpose() * S.inverse();
    x += K * y;
    P = (Eigen::Matrix<double, 6, 6>::Identity() - K * H) * P;
  }
};

TEST(Kalman, InitialisesOnFirstObservation) {
  ScanFrame frame;
  frame.Configure(BaseVehicle(), kCalibration);
  const auto local = Local(.3);
  TrackedObject o;
  o.tracking.Eigen_KfP_CA *= 7.f;
  kalman_filter_update(o, 3.f, 4.f, 99.f, 99.f, 0.1, local.st_IMU.f32_Yaw_deg, frame);
  EXPECT_TRUE(o.tracking.b_KfInitialized);
  EXPECT_EQ(o.tracking.s32_KfInitCount, 1);
  EXPECT_TRUE(o.tracking.Eigen_KfState_CA == (Eigen::Matrix<float, 6, 1>() << 3.f, 4.f, 0, 0, 0, 0).finished());
  EXPECT_TRUE(o.tracking.Eigen_KfP_CA == (Eigen::Matrix<float, 6, 6>::Identity()));
  EXPECT_EQ(o.detection.f32_CoreX, 3.f);
  EXPECT_EQ(o.detection.f32_CoreY, 4.f);
  EXPECT_EQ(o.tracking.f32_PrevYaw, local.st_IMU.f32_Yaw_deg * 0.017453292519943295f) << "float deg->rad multiply";
}

TEST(Kalman, SecondObservationInitialisesVelocityThenUpdates) {
  ScanFrame frame;
  frame.Configure(BaseVehicle(), kCalibration);
  TrackedObject o;  // unrecorded, yaw 0: no frame change
  kalman_filter_update(o, 10.f, 1.f, 0, 0, 0.1, 0.f, frame);
  kalman_filter_update(o, 9.8f, 1.05f, 10.f, 1.f, 0.1, 0.f, frame);
  EXPECT_EQ(o.tracking.s32_KfInitCount, 2);
  KfRef ref;
  const float vx = (9.8f - 10.f) / 0.1, vy = (1.05f - 1.f) / 0.1;
  ref.x << 10., 1., vx, vy, 0, 0;
  ref.P = Eigen::Matrix<double, 6, 6>::Identity();
  ref.Predict(.1);
  ref.Update(9.8f, 1.05f);
  for (int k = 0; k < 6; ++k) EXPECT_NEAR(o.tracking.Eigen_KfState_CA(k), ref.x(k), 1e-5) << k;
  EXPECT_LT((o.tracking.Eigen_KfP_CA.cast<double>() - ref.P).cwiseAbs().maxCoeff(), 1e-6);
  EXPECT_EQ(o.detection.f32_CoreX, o.tracking.Eigen_KfState_CA(0));
}

TEST(Kalman, PredictAndUpdateMatchTheCaEquations) {
  ScanFrame frame;
  frame.Configure(BaseVehicle(), kCalibration);
  TrackedObject o;
  o.tracking.b_KfInitialized = true;
  o.tracking.s32_KfInitCount = 2;
  o.tracking.Eigen_KfState_CA << 12.f, -3.f, -1.5f, .4f, .2f, -.1f;
  for (int r = 0; r < 6; ++r)
    for (int c = 0; c < 6; ++c) o.tracking.Eigen_KfP_CA(r, c) = (r == c ? .5f : 0.f) + .02f * std::min(r, c);
  KfRef ref;
  ref.x = o.tracking.Eigen_KfState_CA.cast<double>();
  ref.P = o.tracking.Eigen_KfP_CA.cast<double>();
  auto predicted = o;
  kalman_filter_predict(predicted, .1, 0.f, frame);
  KfRef p = ref;
  p.Predict(.1);
  for (int k = 0; k < 6; ++k) EXPECT_NEAR(predicted.tracking.Eigen_KfState_CA(k), p.x(k), 1e-5) << k;
  EXPECT_LT((predicted.tracking.Eigen_KfP_CA.cast<double>() - p.P).cwiseAbs().maxCoeff(), 1e-6);
  EXPECT_EQ(predicted.detection.f32_CoreX, predicted.tracking.Eigen_KfState_CA(0));
  auto updated = o;
  kalman_filter_update(updated, 11.9f, -2.9f, 0, 0, .1, 0.f, frame);
  KfRef u = ref;
  u.Predict(.1);
  u.Update(11.9f, -2.9f);
  for (int k = 0; k < 6; ++k) EXPECT_NEAR(updated.tracking.Eigen_KfState_CA(k), u.x(k), 1e-5) << k;
  EXPECT_LT((updated.tracking.Eigen_KfP_CA.cast<double>() - u.P).cwiseAbs().maxCoeff(), 1e-6);
}

TEST(Kalman, UnrecordedTrackFallsBackToTheTwoDimensionalYawRotation) {
  ScanFrame frame;
  frame.Configure(BaseVehicle(), kCalibration);
  TrackedObject o;
  o.tracking.b_KfInitialized = true;
  o.tracking.s32_KfInitCount = 2;
  o.tracking.Eigen_KfState_CA << 10.f, 0.f, 1.f, .5f, 0.f, 0.f;
  o.tracking.f32_PrevYaw = 0.f;
  const float yaw_deg = static_cast<float>(.1 * 180 / M_PI);
  // Association prediction (float deg->rad multiply).
  const auto pp = predictKalmanPosition(o, .1, yaw_deg, frame);
  const double dpsi_p = yaw_deg * 0.017453292519943295f;
  const double c = std::cos(dpsi_p), s = std::sin(dpsi_p);
  EXPECT_NEAR(pp.first, c * 10 + s * 0 + .1 * (c * 1 + s * .5), 1e-5);
  EXPECT_NEAR(pp.second, -s * 10 + c * 0 + .1 * (-s * 1 + c * .5), 1e-5);
  // Predict-only: rotate by -dpsi then CA.
  auto predicted = o;
  kalman_filter_predict(predicted, .1, yaw_deg, frame);
  EXPECT_NEAR(predicted.detection.f32_CoreX, pp.first, 1e-5);
  EXPECT_NEAR(predicted.detection.f32_CoreY, pp.second, 1e-5);
  EXPECT_EQ(predicted.tracking.f32_PrevYaw, static_cast<float>(yaw_deg * M_PI / 180.0f));
  // |dpsi| < 1e-8: no rotation, PrevYaw updated.
  auto still = o;
  kalman_filter_predict(still, .1, 0.f, frame);
  EXPECT_NEAR(still.detection.f32_CoreX, 10.1, 1e-5);
  EXPECT_NEAR(still.detection.f32_CoreY, .05, 1e-5);
}

TEST(Kalman, PredictPositionDoesNotChangeTheTrack) {
  ScanFrame frame;
  frame.Configure(MountedVehicle(), kCalibration);
  BeginTrackScan(frame, 1., .2, .03, -.02, MountedVehicle(), true);
  TrackedObject o = Detection(10, 2);
  o.tracking.b_KfInitialized = true;
  o.tracking.s32_KfInitCount = 2;
  o.tracking.Eigen_KfState_CA << 10.f, 2.f, 1.f, -.3f, 0.f, 0.f;
  o.tracking.f32_PrevYaw = .2f;
  frame.RecordTrackingFrame(o);
  const auto now = BeginTrackScan(frame, 1.1, .26, -.01, .02, MountedVehicle(), true);
  const auto before = o;
  const auto pp = predictKalmanPosition(o, .1, now.local.st_IMU.f32_Yaw_deg, frame);
  EXPECT_TRUE(o.tracking.Eigen_KfState_CA == before.tracking.Eigen_KfState_CA);
  EXPECT_EQ(o.tracking.f32_PrevYaw, before.tracking.f32_PrevYaw);
  auto state = o;
  frame.AlignTrackingState(state, now.local.st_IMU.f32_Yaw_deg * 0.017453292519943295f);
  const auto& x = state.tracking.Eigen_KfState_CA;
  EXPECT_EQ(pp.first, static_cast<float>(x(0) + .1 * x(2) + .5 * .1 * .1 * x(4)));
  EXPECT_EQ(pp.second, static_cast<float>(x(1) + .1 * x(3) + .5 * .1 * .1 * x(5)));
}

// ------------------------------------------------- detections and classes --
TEST(TrackingAssociation, ConsolidatesVerticallySplitSameOccupancy) {
  TrackingConfig config;
  config.f32_DuplicateCenterDistance = 0.60F;
  config.f32_DuplicateIouThreshold = 0.35F;
  std::vector<TrackedObject> detections;
  detections.push_back(MakeDetection(10.0F, 1.0F, 1.0F, 0.8F, -0.5F));
  detections.push_back(MakeDetection(10.02F, 1.01F, 1.0F, 0.8F, 0.2F));
  ConsolidateCurrentDetections(detections, config);
  ASSERT_EQ(detections.size(), 1U);
  EXPECT_EQ(detections.front().detection.pcl_ClusterPoints.size(), 24U);
}

TEST(TrackingAssociation, KeepsNearbyNonOverlappingObjectsSeparate) {
  TrackingConfig config;
  config.f32_DuplicateCenterDistance = 0.60F;
  config.f32_DuplicateIouThreshold = 0.35F;
  std::vector<TrackedObject> detections;
  detections.push_back(MakeDetection(10.0F, 1.0F, 0.2F, 0.2F, 0.0F));
  detections.push_back(MakeDetection(10.4F, 1.0F, 0.2F, 0.2F, 0.0F));
  ConsolidateCurrentDetections(detections, config);
  EXPECT_EQ(detections.size(), 2U);
}

TEST(TrackingAssociation, ConsolidationOwnerAndRecomputedGeometry) {
  TrackingConfig config;
  // Equal point counts: the lower index owns the merge.
  {
    std::vector<TrackedObject> d{MakeDetection(10.F, 1.F, 1.F, .8F, -.5F), MakeDetection(10.F, 1.F, 1.F, .8F, .2F)};
    d[0].detection.u16_Idx = 3;
    d[1].detection.u16_Idx = 4;
    ConsolidateCurrentDetections(d, config);
    ASSERT_EQ(d.size(), 1u);
    EXPECT_EQ(d[0].detection.u16_Idx, 3);
  }
  // More points on the right: the right object owns the merge.
  {
    std::vector<TrackedObject> d{MakeDetection(10.F, 1.F, 1.F, .8F, -.5F), MakeDetection(10.F, 1.F, 1.F, .8F, .2F)};
    d[0].detection.u16_Idx = 3;
    d[1].detection.u16_Idx = 4;
    d[1].detection.pcl_ClusterPoints.push_back(d[1].detection.pcl_ClusterPoints.points[0]);
    pcl::PointXYZI bad;
    bad.x = bad.y = bad.z = std::numeric_limits<float>::quiet_NaN();
    d[1].detection.pcl_ClusterPoints.push_back(bad);
    ConsolidateCurrentDetections(d, config);
    ASSERT_EQ(d.size(), 1u);
    EXPECT_EQ(d[0].detection.u16_Idx, 4);
    const auto& o = d[0];
    double sx = 0, sy = 0, sz = 0;
    std::size_t n = 0;
    float xmin = 1e9f, zmax = -1e9f;
    for (const auto& p : o.detection.pcl_ClusterPoints) {
      if (!std::isfinite(p.x)) continue;
      sx += p.x;
      sy += p.y;
      sz += p.z;
      ++n;
      xmin = std::min(xmin, p.x);
      zmax = std::max(zmax, p.z);
    }
    EXPECT_EQ(n, 25u);
    EXPECT_EQ(o.detection.f32_CoreX, static_cast<float>(sx / n)) << "double mean";
    EXPECT_EQ(o.detection.f32_CoreY, static_cast<float>(sy / n));
    EXPECT_EQ(o.detection.f32_CoreZ, static_cast<float>(sz / n));
    EXPECT_EQ(o.detection.f32_MinX, xmin);
    EXPECT_EQ(o.detection.f32_MaxZ, zmax);
    EXPECT_EQ(o.detection.f32_Dist, std::hypot(o.detection.f32_CoreX, o.detection.f32_CoreY));
    EXPECT_EQ(o.detection.pcl_ClusterPoints.width, o.detection.pcl_ClusterPoints.size());
    EXPECT_EQ(o.detection.pcl_ClusterPoints.height, 1u);
    EXPECT_FALSE(o.detection.pcl_ClusterPoints.is_dense);
    EXPECT_GE(o.detection.obb_length_m, o.detection.obb_width_m);
    EXPECT_TRUE(Finite(o));
  }
  // Chain: three fragments merge into one (the scan restarts after each merge).
  {
    std::vector<TrackedObject> d{MakeDetection(10.F, 1.F, 1.F, .8F, -.5F), MakeDetection(10.F, 1.F, 1.F, .8F, .2F),
                                 MakeDetection(10.F, 1.F, 1.F, .8F, .9F)};
    for (auto& o : d) {  // MakeDetection leaves the OBB centre at 0; the merged owner's is recomputed
      o.detection.f32_OBB_CenterX = 10.F;
      o.detection.f32_OBB_CenterY = 1.F;
    }
    ConsolidateCurrentDetections(d, config);
    ASSERT_EQ(d.size(), 1u);
    EXPECT_EQ(d[0].detection.pcl_ClusterPoints.size(), 36u);
  }
}

TEST(TrackingAssociation, AabbIouAndShapeAgreement) {
  auto a = MakeDetection(0.F, 0.F, 2.F, 2.F, 0.F), b = MakeDetection(1.F, 0.F, 2.F, 2.F, 0.F);
  EXPECT_FLOAT_EQ(tracking_association::AabbIou(a, b), 2.F / 6.F);
  auto z = MakeDetection(0.F, 0.F, 0.F, 0.F, 0.F);
  EXPECT_EQ(tracking_association::AabbIou(z, z), 0.F) << "union <= 1e-6";
  TrackedObject l, r;
  l.detection.obb_length_m = 4.F;
  l.detection.obb_width_m = 2.F;
  r.detection.obb_length_m = 2.F;  // axis swapped
  r.detection.obb_width_m = 4.F;
  EXPECT_FLOAT_EQ(tracking_association::ShapeAgreement(l, r), 1.F);
  r.detection.obb_length_m = 1.F;
  r.detection.obb_width_m = 0.F;  // floored at 0.05
  EXPECT_FLOAT_EQ(tracking_association::ShapeAgreement(l, r), (1.F / 4.F) * (.05F / 2.F));
}

TEST(TrackingAssociation, ShouldCarryPreviousClassRules) {
  TrackedObject p, c;
  const auto carry = [&](const char* prev, const char* cur) {
    p.camera.str_ObjectClass = prev;
    c.camera.str_ObjectClass = cur;
    return tracking_association::ShouldCarryPreviousClass(p, c);
  };
  EXPECT_TRUE(carry("car", "unknown"));
  EXPECT_TRUE(carry("unknown", "noise"));
  EXPECT_TRUE(carry("car", "지면노이즈"));
  EXPECT_FALSE(carry("noise", "unknown"));
  EXPECT_FALSE(carry("지면노이즈", "car"));
  EXPECT_TRUE(carry("noise", "지면노이즈"));
  EXPECT_TRUE(tracking_association::IsGroundNoiseClass("noise"));
  EXPECT_FALSE(tracking_association::IsGroundNoiseClass("Noise"));
  TrackedObject src, dst;
  src.camera.str_ObjectClass = "car";
  src.camera.str_YoloClass = "truck";
  src.camera.last_seen_class = "car";
  src.camera.continuity_counter = 4;
  src.camera.class_miss_counter = 2;
  src.camera.camera_observation_ns = 11;
  src.camera.camera_observation_epoch = 12;
  src.camera.is_class_locked = true;
  tracking_association::CopyClassState(src, dst);
  EXPECT_EQ(dst.camera.str_ObjectClass, "car");
  EXPECT_EQ(dst.camera.str_YoloClass, "truck");
  EXPECT_EQ(dst.camera.last_seen_class, "car");
  EXPECT_EQ(dst.camera.continuity_counter, 4);
  EXPECT_EQ(dst.camera.class_miss_counter, 2);
  EXPECT_EQ(dst.camera.camera_observation_ns, 11u);
  EXPECT_EQ(dst.camera.camera_observation_epoch, 12u);
  EXPECT_TRUE(dst.camera.is_class_locked);
}

// Ports of Main test_tracking_association.cpp:99-171. Main associated these by
// map centre (legacy); on the common path the body-frame distance 0.1 m matches.
void RunClassCase(const char* previous_class, const char* current_class, float prev_z, float cur_z,
                  const char* expected) {
  ScanFrame frame;
  frame.Configure(BaseVehicle(), kCalibration);
  BeginTrackScan(frame, 1., 0, 0, 0, BaseVehicle(), true);
  auto previous = MakeDetection(10.0F, 0.0F, 1.0F, 0.8F, prev_z);
  previous.tracking.s32_Id = 42;
  previous.tracking.s32_Age = 10;
  previous.tracking.b_KfInitialized = true;
  previous.tracking.s32_KfInitCount = 2;
  previous.tracking.Eigen_KfState_CA << 10.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F;
  previous.camera.str_ObjectClass = previous_class;
  auto current = MakeDetection(9.9F, 0.0F, 1.0F, 0.8F, cur_z);
  current.tracking.s32_Id = -1;
  current.camera.str_ObjectClass = current_class;
  Tracker tracker(MainTestConfig());
  tracker.Commit({previous});
  std::vector<TrackedObject> detections{current};
  tracker.Update(detections, 0.1, LocalState{}, frame);
  ASSERT_FALSE(detections.empty());
  EXPECT_EQ(detections.front().tracking.s32_Id, 42);
  EXPECT_EQ(detections.front().tracking.s32_Age, 11);
  EXPECT_EQ(detections.front().camera.str_ObjectClass, expected);
}
TEST(TrackingAssociation, RecoversFromLegacyGroundNoiseOnCurrentObservation) {
  RunClassCase("지면노이즈", "unknown", -1.4F, -1.1F, "unknown");
}
TEST(TrackingAssociation, OneNoiseMeasurementDoesNotDowngradeValidTrack) {
  RunClassCase("unknown", "지면노이즈", -1.1F, -1.4F, "unknown");
}

// Replaces Main PreservesIdAcrossLargeBodyFrameShiftUsingMapCenter (legacy map
// association). Common path: association is body-frame distance to the KF
// prediction only; a 5 m body jump is followed only when the relative KF
// velocity predicts it, never through map centres.
TEST(TrackingAssociation, CommonPathLargeBodyShiftFollowsKfPredictionOnly) {
  for (float vx : {-50.f, 0.f}) {
    SCOPED_TRACE(vx);
    ScanFrame frame;
    frame.Configure(BaseVehicle(), kCalibration);
    asmc::lidar::PoseSelection map;
    map.valid = true;  // a valid map pose is irrelevant to the association
    map.reason = "gps_bounded_prediction";
    BeginTrackScan(frame, 1., 0, 0, 0, BaseVehicle(), true, &map);
    auto previous = MakeDetection(10.0F, 0.0F, 1.0F, 0.8F, 0.0F);
    previous.tracking.s32_Id = 42;
    previous.tracking.s32_Age = 20;
    previous.tracking.b_KfInitialized = true;
    previous.tracking.s32_KfInitCount = 2;
    previous.tracking.Eigen_KfState_CA << 10.0F, 0.0F, vx, 0.0F, 0.0F, 0.0F;
    frame.RecordTrackingFrame(previous);
    Tracker tracker(MainTestConfig());
    tracker.Commit({previous});
    map.map_from_base.translation().x() = 5.;
    const auto scan = BeginTrackScan(frame, 1.1, 0, 0, 0, BaseVehicle(), true, &map);
    std::vector<TrackedObject> detections{MakeDetection(5.0F, 0.0F, 1.0F, 0.8F, 0.0F)};
    tracker.Update(detections, 0.1, scan.local, frame);
    if (vx < 0) {
      ASSERT_EQ(detections.size(), 1u);
      EXPECT_EQ(detections.front().tracking.s32_Id, 42);
      EXPECT_EQ(detections.front().tracking.s32_Age, 21);
    } else {
      ASSERT_EQ(detections.size(), 2u);
      EXPECT_EQ(detections[0].tracking.s32_Id, 0) << "5 m without a predicting velocity: new track";
      EXPECT_EQ(detections[1].tracking.s32_Id, 42);
      EXPECT_EQ(detections[1].tracking.s32_MissCount, 1);
    }
  }
}

// ------------------------------------------------------------- lifecycle --
TEST(TrackerLifecycle, FirstFrameBirthsWithIdsFromZero) {
  ScanFrame frame;
  frame.Configure(BaseVehicle(), kCalibration);
  const auto scan = BeginTrackScan(frame, 1., .3, 0, 0, BaseVehicle(), true);
  Tracker tracker;
  EXPECT_EQ(tracker.NextTrackId(), 0);
  EXPECT_EQ(tracker.LastFrame(), nullptr);
  EXPECT_DOUBLE_EQ(Tracker::kBirthInitDt, 0.1);
  std::vector<TrackedObject> objects{Detection(10, 1), Detection(-5, 3), Detection(20, -4)};
  tracker.Update(objects, 123.0, scan.local, frame);  // dt unused on the first frame
  ASSERT_EQ(objects.size(), 3u);
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(objects[i].tracking.s32_Id, i);
    EXPECT_EQ(objects[i].tracking.s32_Age, 1);
    EXPECT_EQ(objects[i].tracking.s32_MissCount, 0);
    EXPECT_TRUE(objects[i].tracking.b_KfInitialized);
    EXPECT_EQ(objects[i].tracking.s32_KfInitCount, 1);
    EXPECT_EQ(objects[i].tracking.Eigen_KfState_CA(0), objects[i].detection.f32_CoreX);
    EXPECT_EQ(objects[i].tracking.Eigen_KfState_CA(2), 0.f);
    EXPECT_EQ(objects[i].tracking.f32_PrevYaw, scan.local.st_IMU.f32_Yaw_deg * 0.017453292519943295f);
    EXPECT_FALSE(objects[i].tracking.tracking_frame_valid) << "the pipeline records after tracking";
  }
  EXPECT_EQ(tracker.NextTrackId(), 3);
}

TEST(TrackerLifecycle, FirstFrameStopsAtTheFirstZeroCentroid) {  // preserved quirk
  ScanFrame frame;
  frame.Configure(BaseVehicle(), kCalibration);
  const auto scan = BeginTrackScan(frame, 1., 0, 0, 0, BaseVehicle(), true);
  Tracker tracker;
  TrackedObject dummy{};  // centroid exactly (0,0,0)
  std::vector<TrackedObject> objects{Detection(10, 1), dummy, Detection(20, -4)};
  tracker.Update(objects, .1, scan.local, frame);
  ASSERT_EQ(objects.size(), 3u);
  EXPECT_EQ(objects[0].tracking.s32_Id, 0);
  EXPECT_EQ(objects[1].tracking.s32_Id, -1);
  EXPECT_EQ(objects[2].tracking.s32_Id, -1) << "loop breaks at the dummy";
  EXPECT_FALSE(objects[2].tracking.b_KfInitialized);
}

TEST(TrackerLifecycle, AllDummyOrEmptyPreviousFrameOnlyBirths) {
  for (bool empty : {true, false}) {
    ScanFrame frame;
    frame.Configure(BaseVehicle(), kCalibration);
    const auto scan = BeginTrackScan(frame, 1., 0, 0, 0, BaseVehicle(), true);
    Tracker tracker;
    std::vector<TrackedObject> previous;
    if (!empty) {
      TrackedObject dummy{};
      dummy.tracking.s32_Id = 7;
      dummy.tracking.b_KfInitialized = true;
      previous.push_back(dummy);
    }
    tracker.Commit(previous);
    std::vector<TrackedObject> objects{Detection(10, 1), Detection(-5, 3)};
    objects[1].tracking.s32_Id = 9;  // already identified: kept
    tracker.Update(objects, .1, scan.local, frame);
    ASSERT_EQ(objects.size(), 2u) << "no association, previous dummy not carried";
    EXPECT_EQ(objects[0].tracking.s32_Id, 0);
    EXPECT_EQ(objects[0].tracking.s32_KfInitCount, 1);
    EXPECT_EQ(objects[1].tracking.s32_Id, 9);
  }
}

TrackedObject RecordedTrack(const ScanFrame& frame, int id, int age, double x, double y) {
  TrackedObject o = Detection(x, y);
  o.tracking.s32_Id = id;
  o.tracking.s32_Age = age;
  o.tracking.b_KfInitialized = true;
  o.tracking.s32_KfInitCount = 2;
  o.tracking.Eigen_KfState_CA << o.detection.f32_CoreX, o.detection.f32_CoreY, 0, 0, 0, 0;
  frame.RecordTrackingFrame(o);
  return o;
}

TEST(TrackerLifecycle, MissedTrackIsDeletedAtThe25thConsecutiveMiss) {
  ScanFrame frame;
  frame.Configure(BaseVehicle(), kCalibration);
  auto scan = BeginTrackScan(frame, 1., 0, 0, 0, BaseVehicle(), true);
  Tracker tracker;
  tracker.Commit({RecordedTrack(frame, 5, 3, 10, 1)});
  float last_p = 1.f;
  for (int miss = 1; miss <= 25; ++miss) {
    scan = BeginTrackScan(frame, 1. + .1 * miss, 0, 0, 0, BaseVehicle(), true);
    std::vector<TrackedObject> objects;
    tracker.Update(objects, .1, scan.local, frame);
    if (miss < Tracker::kMaxMissCount) {
      ASSERT_EQ(objects.size(), 1u) << miss;
      EXPECT_EQ(objects[0].tracking.s32_MissCount, miss);
      EXPECT_EQ(objects[0].tracking.s32_Age, 3 + miss);
      EXPECT_GT(objects[0].tracking.Eigen_KfP_CA(0, 0), last_p);
      last_p = objects[0].tracking.Eigen_KfP_CA(0, 0);
    } else {
      EXPECT_TRUE(objects.empty()) << "deleted at the 25th miss";
    }
    tracker.Commit(objects);
  }
}

TEST(TrackerLifecycle, UninitialisedPreviousTrackIsNotCarried) {
  ScanFrame frame;
  frame.Configure(BaseVehicle(), kCalibration);
  const auto scan = BeginTrackScan(frame, 1., 0, 0, 0, BaseVehicle(), true);
  auto previous = RecordedTrack(frame, 5, 3, 10, 1);
  previous.tracking.b_KfInitialized = false;
  Tracker tracker;
  tracker.Commit({previous});
  std::vector<TrackedObject> objects;
  tracker.Update(objects, .1, scan.local, frame);
  EXPECT_TRUE(objects.empty());
}

TEST(TrackerLifecycle, NoDetectionsCarriesEveryTrack) {
  ScanFrame frame;
  frame.Configure(BaseVehicle(), kCalibration);
  BeginTrackScan(frame, 1., 0, 0, 0, BaseVehicle(), true);
  auto a = RecordedTrack(frame, 1, 4, 10, 1), b = RecordedTrack(frame, 2, 9, -3, 5);
  a.tracking.Eigen_KfState_CA(2) = -1.f;
  Tracker tracker;
  tracker.Commit({a, b});
  const auto scan = BeginTrackScan(frame, 1.1, 0, 0, 0, BaseVehicle(), true);
  std::vector<TrackedObject> objects;
  tracker.Update(objects, .1, scan.local, frame);
  ASSERT_EQ(objects.size(), 2u);
  EXPECT_EQ(objects[0].tracking.s32_Id, 1);
  EXPECT_EQ(objects[1].tracking.s32_Id, 2);
  EXPECT_EQ(objects[0].tracking.s32_MissCount, 1);
  EXPECT_NEAR(objects[0].detection.f32_CoreX, 9.9, 1e-5);
  EXPECT_NEAR(objects[1].detection.f32_CoreX, -3., 1e-5);
}

TEST(TrackerLifecycle, ContinuityBonusPrefersTheOlderTrack) {
  for (int older : {1, 2}) {
    ScanFrame frame;
    frame.Configure(BaseVehicle(), kCalibration);
    BeginTrackScan(frame, 1., 0, 0, 0, BaseVehicle(), true);
    Tracker tracker;
    tracker.Commit({RecordedTrack(frame, 1, older == 1 ? 30 : 1, 9.8, 0),
                    RecordedTrack(frame, 2, older == 2 ? 30 : 1, 10.2, 0)});
    const auto scan = BeginTrackScan(frame, 1.1, 0, 0, 0, BaseVehicle(), true);
    std::vector<TrackedObject> objects{Detection(10, 0)};
    tracker.Update(objects, .1, scan.local, frame);
    ASSERT_GE(objects.size(), 1u);
    EXPECT_EQ(objects[0].tracking.s32_Id, older);
  }
}

// Retire: the loser of a re-merged split track is dropped when the matched
// KF-filtered centre is within MergedTrackRetireDistance of its prediction.
TEST(TrackerLifecycle, RetireUsesTheKfFilteredMatchedCentre) {
  for (bool tight : {false, true}) {
    SCOPED_TRACE(tight ? "tight prior" : "loose prior");
    ScanFrame frame;
    frame.Configure(BaseVehicle(), kCalibration);
    BeginTrackScan(frame, 1., 0, 0, 0, BaseVehicle(), true);
    auto a = RecordedTrack(frame, 1, 30, 10, 0), b = RecordedTrack(frame, 2, 0, 10.6, 0);
    if (tight) a.tracking.Eigen_KfP_CA = Eigen::Matrix<float, 6, 6>::Identity() * 1e-4f;
    TrackingConfig config;  // retire 0.45, shape >= 0.28
    Tracker tracker(config);
    tracker.Commit({a, b});
    const auto scan = BeginTrackScan(frame, 1.1, 0, 0, 0, BaseVehicle(), true);
    std::vector<TrackedObject> objects{Detection(10.3, 0)};
    tracker.Update(objects, .1, scan.local, frame);
    EXPECT_EQ(objects[0].tracking.s32_Id, 1);
    const double filtered_gap = std::abs(objects[0].detection.f32_CoreX - 10.6);
    if (tight) {
      // Raw detection is 0.3 m from b, but the filtered centre stays near a.
      EXPECT_GT(filtered_gap, .45);
      ASSERT_EQ(objects.size(), 2u);
      EXPECT_EQ(objects[1].tracking.s32_Id, 2);
      EXPECT_EQ(objects[1].tracking.s32_MissCount, 1);
    } else {
      EXPECT_LE(filtered_gap, .45);
      EXPECT_EQ(objects.size(), 1u) << "duplicate retired";
    }
  }
}

TEST(TrackerLifecycle, RetireNeedsShapeAgreement) {
  ScanFrame frame;
  frame.Configure(BaseVehicle(), kCalibration);
  BeginTrackScan(frame, 1., 0, 0, 0, BaseVehicle(), true);
  auto a = RecordedTrack(frame, 1, 30, 10, 0), b = RecordedTrack(frame, 2, 0, 10.2, 0);
  b.detection.obb_length_m = 6.f;
  b.detection.obb_width_m = 4.f;  // agreement with 1 x 0.8: (1/6)(0.8/4) < 0.28
  Tracker tracker;
  tracker.Commit({a, b});
  const auto scan = BeginTrackScan(frame, 1.1, 0, 0, 0, BaseVehicle(), true);
  std::vector<TrackedObject> objects{Detection(10.1, 0)};
  tracker.Update(objects, .1, scan.local, frame);
  ASSERT_EQ(objects.size(), 2u);
  EXPECT_EQ(objects[0].tracking.s32_Id, 1);
  EXPECT_EQ(objects[1].tracking.s32_Id, 2);
  EXPECT_EQ(objects[1].tracking.s32_MissCount, 1);
}

TEST(TrackerLifecycle, VelocityBookkeepingAddsCanSpeedOnX) {
  for (float kph : {7.2f, 0.f}) {
    ScanFrame frame;
    frame.Configure(BaseVehicle(), kCalibration);
    BeginTrackScan(frame, 1., 0, 0, 0, BaseVehicle(), true);
    auto a = RecordedTrack(frame, 1, 5, 10, 0);
    if (kph > 0) a.tracking.Eigen_KfState_CA(2) = -1.f;
    Tracker tracker;
    tracker.Commit({a});
    const auto scan = BeginTrackScan(frame, 1.1, 0, 0, 0, BaseVehicle(), true);
    auto local = scan.local;
    local.st_CAN.f32_CurrentVel_KPH = kph;
    std::vector<TrackedObject> objects{Detection(kph > 0 ? 9.9 : 10., kph > 0 ? .02 : 0.)};
    tracker.Update(objects, .1, local, frame);
    ASSERT_EQ(objects.size(), 1u);
    const auto& o = objects[0];
    const float vx = o.tracking.Eigen_KfState_CA(2), vy = o.tracking.Eigen_KfState_CA(3);
    EXPECT_EQ(o.tracking.f32_AbsVx, vx + kph / 3.6f);
    EXPECT_EQ(o.tracking.f32_AbsVy, vy);
    EXPECT_EQ(o.tracking.f32_Velocity, std::hypot(o.tracking.f32_AbsVx, o.tracking.f32_AbsVy));
    if (o.tracking.f32_Velocity > 1e-3f) {
      EXPECT_EQ(o.tracking.f32_DirX, o.tracking.f32_AbsVx / o.tracking.f32_Velocity);
      EXPECT_EQ(o.tracking.f32_DirY, o.tracking.f32_AbsVy / o.tracking.f32_Velocity);
    } else {
      EXPECT_EQ(o.tracking.f32_DirX, 0.f);
      EXPECT_EQ(o.tracking.f32_DirY, 0.f);
    }
    EXPECT_EQ(o.tracking.f32_PrevCoreX, a.detection.f32_CoreX);
    EXPECT_FLOAT_EQ(o.tracking.f32_PredCoreX, 10.f + .1f * a.tracking.Eigen_KfState_CA(2));
  }
}

// Preserved quirk: the second-observation velocity uses only the last dt; a
// miss in between doubles the initial speed estimate.
TEST(TrackerLifecycle, VelocityInitAfterAMissUsesOnlyTheLastInterval) {
  ScanFrame frame;
  frame.Configure(BaseVehicle(), kCalibration);
  auto scan = BeginTrackScan(frame, 1., 0, 0, 0, BaseVehicle(), true);
  Tracker tracker;
  std::vector<TrackedObject> objects{Detection(10, 0)};
  tracker.Update(objects, .1, scan.local, frame);  // birth, KfInitCount 1
  for (auto& o : objects) frame.RecordTrackingFrame(o);
  tracker.Commit(objects);
  scan = BeginTrackScan(frame, 1.1, 0, 0, 0, BaseVehicle(), true);
  objects.clear();
  tracker.Update(objects, .1, scan.local, frame);  // missed: zero-velocity prediction
  ASSERT_EQ(objects.size(), 1u);
  tracker.Commit(objects);
  scan = BeginTrackScan(frame, 1.2, 0, 0, 0, BaseVehicle(), true);
  objects = {Detection(9.8, 0)};  // true relative speed -1 m/s over 0.2 s
  tracker.Update(objects, .1, scan.local, frame);
  ASSERT_EQ(objects.size(), 1u);
  EXPECT_EQ(objects[0].tracking.s32_Id, 0);
  EXPECT_EQ(objects[0].tracking.s32_KfInitCount, 2);
  EXPECT_NEAR(objects[0].tracking.Eigen_KfState_CA(2), -2.0, 1e-4) << "(z - pred) / dt with dt = last interval";
}

TEST(TrackerLifecycle, CommitKeepsThreeFrames) {
  Tracker tracker;
  for (int k = 0; k < 5; ++k) {
    std::vector<TrackedObject> f(k + 1);
    tracker.Commit(f);
    EXPECT_EQ(tracker.History().size(), std::min<std::size_t>(k + 1, 3));
    ASSERT_NE(tracker.LastFrame(), nullptr);
    EXPECT_EQ(tracker.LastFrame()->size(), std::size_t(k + 1));
  }
  EXPECT_EQ(tracker.History().front().size(), 3u);
}

// --------------------------------------- base frame-boundary tracking tests --
// Replaces Main LocalAssociationIgnoresMapCoordinatesIncludingNaN: TrackedObject
// carries no map coordinates; the scan map pose (valid near, valid far, or
// invalid with NaN) must not change the local association.
TEST(FrameBoundary, LocalAssociationIgnoresMapPoseIncludingNaN) {
  for (bool recorded : {false, true}) {
    SCOPED_TRACE(recorded ? "recorded" : "unrecorded (2D fallback as in Main)");
    std::vector<int> ids;
    for (double map_x : {100., 1000000., kNaN}) {
      ScanFrame frame;
      frame.Configure(BaseVehicle(), kCalibration);
      asmc::lidar::PoseSelection map;
      map.valid = std::isfinite(map_x);
      map.reason = map.valid ? "gps_bounded_prediction" : "gps_invalid";
      map.map_from_base.translation() = Vec(map_x, map_x, 0);
      BeginTrackScan(frame, 1., 0, 0, 0, BaseVehicle(), true, &map);
      auto previous = Detection(10, 1);
      previous.tracking.s32_Id = 42;
      previous.tracking.s32_Age = 20;
      previous.tracking.b_KfInitialized = true;
      previous.tracking.s32_KfInitCount = 2;
      previous.tracking.Eigen_KfState_CA << 10, 1, -2, 0, 0, 0;
      if (recorded) frame.RecordTrackingFrame(previous);
      Tracker tracker(MainTestConfig());
      tracker.Commit({previous});
      BeginTrackScan(frame, 1.1, 0, 0, 0, BaseVehicle(), true, &map);
      std::vector<TrackedObject> detections{Detection(9.8, 1)};
      tracker.Update(detections, .1, Local(), frame);
      ASSERT_FALSE(detections.empty());
      ids.push_back(detections.front().tracking.s32_Id);
      EXPECT_EQ(detections.front().camera.str_YoloClass, "car");
    }
    EXPECT_EQ(ids, std::vector<int>({42, 42, 42}));
  }
}

// Port of Main test_frame_boundary_ros.cpp:133-145 (unrecorded: 2D fallback).
TEST(FrameBoundary, LocalAssociationYawPredictionUsesCurrentBodyFrame) {
  ScanFrame frame;
  frame.Configure(BaseVehicle(), kCalibration);
  BeginTrackScan(frame, 1., .1, 0, 0, BaseVehicle(), true);
  auto previous = Detection(10, 0);
  previous.tracking.s32_Id = 42;
  previous.tracking.s32_Age = 20;
  previous.tracking.b_KfInitialized = true;
  previous.tracking.s32_KfInitCount = 2;
  previous.tracking.Eigen_KfState_CA << 10, 0, 0, 0, 0, 0;
  auto current = Detection(10 * std::cos(.1), -10 * std::sin(.1));
  Tracker tracker(MainTestConfig());
  tracker.Commit({previous});
  std::vector<TrackedObject> detections{current};
  tracker.Update(detections, .1, Local(.1), frame);
  ASSERT_FALSE(detections.empty());
  EXPECT_EQ(detections.front().tracking.s32_Id, 42);
}

// Port of Main test_frame_boundary_ros.cpp:456-492.
TEST(FrameBoundary, ContinuousLocalTrackingStateIsIndependentOfGpsHistory) {
  auto simulate = [](bool denial) {
    ScanFrame frame;
    frame.Configure(BaseVehicle(), kCalibration);
    Tracker tracker(MainTestConfig());
    std::vector<std::vector<double>> states;
    asmc::lidar::PoseSelection map;
    for (unsigned f = 0; f < 40; ++f) {
      const double time = 26. + f * .1;
      auto local = Local(.002 * f);
      local.st_IMU.f32_Roll_rad = .03 * std::sin(f * .1);
      local.st_IMU.f32_Pitch_rad = .02 * std::cos(f * .1);
      map.valid = !denial || f < 10 || f >= 30;
      map.reason = map.valid ? "gps_bounded_prediction" : "gps_stale";
      map.query_stamp_ns = Time(time).stamp_ns;
      map.clock = "ros";
      map.map_from_base = Pose::Identity();
      map.map_from_base.translation().x() = f * .2;
      frame.Prepare(Time(time), local, map, f + 1);
      frame.StoreLeveling(asmc::lidar::leveling_rotation(local.st_IMU.f32_Roll_rad, local.st_IMU.f32_Pitch_rad,
                                                         Rot::Identity()));
      std::vector<TrackedObject> objects{Detection(12. - .2 * f, 1.)};
      if (tracker.History().empty()) {
        objects[0].tracking.s32_Id = 42;
        objects[0].tracking.s32_Age = 1;
        objects[0].tracking.b_KfInitialized = true;
        objects[0].tracking.s32_KfInitCount = 2;
        objects[0].tracking.Eigen_KfState_CA << 12, 1, -2, 0, 0, 0;
        objects[0].tracking.f32_PrevYaw = 0;
      } else {
        tracker.Update(objects, .1, local, frame);
      }
      EXPECT_EQ(objects.front().tracking.s32_Id, 42);
      frame.RecordTrackingFrame(objects.front());
      const auto& o = objects.front();
      states.push_back({double(o.tracking.s32_Id), double(o.tracking.s32_Age), double(o.tracking.s32_MissCount), double(o.tracking.Eigen_KfState_CA(0)),
                        double(o.tracking.Eigen_KfState_CA(1)), double(o.camera.continuity_counter)});
      tracker.Commit(objects);
    }
    return states;
  };
  const auto normal = simulate(false), denied = simulate(true);
  EXPECT_EQ(normal, denied);
  EXPECT_EQ(normal.back()[1], 40.);
}

// ------------------------------------- tracker-fix tests (Main 8f93c22e) --
TEST(FrameBoundary, TrackerObbSingleScanMatchedAndPredicted) {
  RunScenario({"pitch_only", false, .2, .04, BaseVehicle(), MainTracks(kMatchedBox, kPredictedBox, kHandoverBox, 2),
               2, false});
  RunScenario({"roll_pitch", true, .2, .04, BaseVehicle(), MainTracks(kMatchedBox, kPredictedBox, kHandoverBox, 2),
               2, true});
}
TEST(FrameBoundary, TrackerObbYawDoesNotAccumulateOverScans) {
  const auto t = MainTracks(kMatchedBox, kPredictedBox, kHandoverBox, 8);
  RunScenario({"pitch_only", false, .2, .04, BaseVehicle(), t, 8, false});
  RunScenario({"pitch_only_zero_roll", false, .2, .04, BaseVehicle(), t, 8, true, 0.});
  RunScenario({"roll_pitch", true, .2, .04, BaseVehicle(), t, 8, true});
  RunScenario({"roll_pitch_nonzero_mount", true, .2, .04, MountedVehicle(), t, 8, false});
  RunScenario({"pitch_only_nonzero_mount", false, .2, -.05, MountedVehicle(), t, 8, false});
}
TEST(FrameBoundary, TrackerObbNearSquareMatchedYawDoesNotDrift) {
  // Aspect < 1.2 skips the yaw EMA, so a twice-rotated prior would drift every scan.
  const WorldBox square{{11, 3, .2}, .5, 1., .9, 1.2};
  const auto t = MainTracks(square, kPredictedBox, kHandoverBox, 8);
  RunScenario({"near_square_roll_pitch", true, .2, .04, BaseVehicle(), t, 8, true});
  RunScenario({"near_square_pitch_only_zero_roll", false, .2, .04, BaseVehicle(), t, 8, true, 0.});
}
TEST(FrameBoundary, TrackerObbYawWrapAround) {
  const double yaw0 = M_PI - .06;
  const WorldBox matched{{12, 2, .25}, yaw0 + M_PI / 2. + .06, 4.5, 1.8, 1.5},
      predicted{{7, -6, .1}, M_PI / 2. + .03, 3., 1.2, 1.}, handover{{16, -1, .2}, .03, 2., 1.1, 1.3};
  const auto t = MainTracks(matched, predicted, handover, 6);
  RunScenario({"wrap_pitch_only", false, yaw0, .04, BaseVehicle(), t, 6, false});
  RunScenario({"wrap_pitch_only_zero_roll", false, yaw0, .04, BaseVehicle(), t, 6, true, 0.});
  RunScenario({"wrap_roll_pitch", true, yaw0, .04, BaseVehicle(), t, 6, true});
}

// --------------------------------------------------- regression scenarios --
TEST(TrackerRegression, MatchedTrackWithEgoYawChange) {
  const std::vector<TrackSpec> t{{42, kMatchedBox, 6}};
  RunScenario({"matched_gravity", true, .2, .08, BaseVehicle(), t, 6, true});
  RunScenario({"matched_tilted", false, .2, .08, BaseVehicle(), t, 6, false});
}
TEST(TrackerRegression, PredictedOnlyTrackWithEgoYawChange) {
  const std::vector<TrackSpec> t{{43, kPredictedBox, 1}};
  const auto g = RunScenario({"predicted_gravity", true, .2, .08, BaseVehicle(), t, 8, true});
  const auto k = RunScenario({"predicted_tilted", false, .2, .08, BaseVehicle(), t, 8, false});
  EXPECT_EQ(g.carried_predictions, 7u);
  EXPECT_LT(g.max_carried_tilt_z, 1e-5) << "gravity-aligned: corner z == Z range";
  // Main rule under tilt: ZMin/ZMax stay centre z -+ extent/2 while the corners
  // are tilted (preserved, documented).
  EXPECT_GT(k.max_carried_tilt_z, 1e-4);
}
TEST(TrackerRegression, MatchedAndPredictedCoexistInOneScan) {
  const std::vector<TrackSpec> t{{42, kMatchedBox, 4}, {43, kPredictedBox, 1}};
  for (bool roll : {true, false}) {
    const auto r = RunScenario({roll ? "coexist_gravity" : "coexist_tilted", roll, .3, .06, BaseVehicle(), t, 4, roll});
    EXPECT_EQ(r.matched_priors, 3u);
    EXPECT_EQ(r.carried_predictions, 3u);
  }
}
TEST(TrackerRegression, MatchedToMissedTransition) {
  const std::vector<TrackSpec> t{{44, kHandoverBox, 3}};
  RunScenario({"handover_gravity", true, .2, .05, BaseVehicle(), t, 7, true});
  RunScenario({"handover_tilted", false, .2, .05, BaseVehicle(), t, 7, false});
  RunScenario({"handover_mount", true, .2, .05, MountedVehicle(), t, 7, false});
}
TEST(TrackerRegression, MissedThenRedetectedTrackRestartsFromRawObb) {
  // Seen 0..2, missed 3..4, seen again from 5: the first frame after the miss
  // keeps the raw detection OBB, the next one is stabilized again.
  const std::vector<TrackSpec> t{{44, kHandoverBox, 3, 5}, {42, kMatchedBox, 8}};
  RunScenario({"redetect_gravity", true, .2, .05, BaseVehicle(), t, 8, true});
  RunScenario({"redetect_tilted", false, .2, .05, BaseVehicle(), t, 8, false});
}
TEST(TrackerRegression, MultiScanAccumulation) {
  const auto t = MainTracks(kMatchedBox, kPredictedBox, kHandoverBox, 12);
  const auto g = RunScenario({"accumulate_gravity", true, .1, .05, BaseVehicle(), t, 12, true});
  RunScenario({"accumulate_tilted", false, .1, .05, BaseVehicle(), t, 12, false});
  RunScenario({"accumulate_mount", true, .1, .05, MountedVehicle(), t, 12, false});
  // Gravity-aligned: matched yaw equals the box heading at every scan (no drift).
  for (unsigned s = 0; s < 12; ++s)
    EXPECT_LT(AxisError(g.yaw.at(42)[s], NormalizeAxis(kMatchedBox.heading - (.1 + .05 * s))), kAngTol) << s;
}
TEST(TrackerRegression, YawWrapAroundCrossesBothBranches) {
  // Ego yaw crosses +pi; the matched axis crosses the +-pi/2 normalization branch
  // between scans 1 and 2; the predicted raw yaw crosses +-pi.
  const double yaw0 = M_PI - .1;
  const WorldBox matched{{12, 2, .25}, yaw0 + M_PI / 2 + .07, 4.5, 1.8, 1.5},
      predicted{{7, -6, .1}, -M_PI + .02 + yaw0, 3., 1.2, 1.};
  const std::vector<TrackSpec> t{{42, matched, 8}, {43, predicted, 1}};
  for (bool roll : {true, false}) {
    const auto r = RunScenario({roll ? "wrap2_gravity" : "wrap2_tilted", roll, yaw0, .05, BaseVehicle(), t, 8, roll});
    const auto& y = r.yaw.at(42);
    EXPECT_GT(y[0], M_PI / 2) << "scan 0: raw detection yaw (atan2 range)";
    EXPECT_LT(y[1], -1.4) << "first stabilized scan on the -pi/2 side";
    EXPECT_GT(y[7], 1.2) << "ends on the +pi/2 side";
  }
}
TEST(TrackerRegression, PitchOnlyLevelingWithRoll) {
  const auto t = MainTracks(kMatchedBox, kPredictedBox, kHandoverBox, 8);
  const auto r = RunScenario({"pitch_only_with_roll", false, .2, .04, BaseVehicle(), t, 8, false});
  EXPECT_GT(r.stats.max_applied_abs_rad, 1e-7) << "tilted change: edge follow corrects the 2D axis";
}
TEST(TrackerRegression, RollPitchLeveling) {
  const auto t = MainTracks(kMatchedBox, kPredictedBox, kHandoverBox, 8);
  RunScenario({"roll_pitch_leveled", true, .2, .04, BaseVehicle(), t, 8, true});
}
TEST(TrackerRegression, NonzeroSensorMount) {
  const auto t = MainTracks(kMatchedBox, kPredictedBox, kHandoverBox, 8);
  RunScenario({"mount_roll_pitch", true, .2, .04, MountedVehicle(), t, 8, false});
  RunScenario({"mount_pitch_only", false, -.3, -.05, MountedVehicle(), t, 8, false});
}
TEST(TrackerRegression, NearSquareObb) {
  const WorldBox square{{11, 3, .2}, .5, 1., .9, 1.2};  // aspect 1.11
  const auto t = MainTracks(square, kPredictedBox, kHandoverBox, 8);
  RunScenario({"square_gravity", true, .2, .06, BaseVehicle(), t, 8, true});
  RunScenario({"square_tilted", false, .2, .06, BaseVehicle(), t, 8, false});
  RunScenario({"square_mount", true, .2, .06, MountedVehicle(), t, 8, false});
}

TEST(TrackerRegression, InvalidGeometryNeverProducesNaN) {
  ScanFrame frame;
  frame.Configure(BaseVehicle(), kCalibration);
  Tracker tracker;
  for (int s = 0; s < 4; ++s) {
    SCOPED_TRACE(s);
    const auto scan = BeginTrackScan(frame, 1. + .1 * s, .02 * s, .01, -.01, BaseVehicle(), true);
    CloudPtr points(new Cloud), centroids(new Cloud);
    std::vector<pcl::PointIndices> clusters(4);
    const auto add = [&](int k, double x, double y, double z) {
      pcl::PointXYZI p;
      p.x = x;
      p.y = y;
      p.z = z;
      clusters[k].indices.push_back(static_cast<int>(points->size()));
      points->push_back(p);
    };
    // 0: box points + NaN points; 1: three points; 2: empty; 3: one point.
    for (int i = 0; i < 12; ++i) add(0, 10 + .2 * (i % 6), 1 + .5 * (i / 6), -.5 + .1 * (i % 3));
    add(0, kNaN, kNaN, kNaN);
    add(0, kNaN, 1.2, -.4);
    for (int i = 0; i < 3; ++i) add(1, 20 + .1 * i, -5 + .05 * i, -1);
    add(3, -8, 4, .2);
    const double cx[] = {10.5, 20.1, 30, -8}, cy[] = {1.25, -4.95, 30, 4};
    for (int k = 0; k < 4; ++k) {
      pcl::PointXYZI c;
      c.x = cx[k];
      c.y = cy[k];
      c.z = 0;
      centroids->push_back(c);
    }
    std::vector<TrackedObject> objects;
    BuildDetections(objects, points, centroids, clusters);
    ASSERT_EQ(objects.size(), 4u);
    tracker.Update(objects, .1, scan.local, frame);
    for (auto& o : objects)
      if (o.tracking.s32_MissCount == 0) frame.RecordTrackingFrame(o);
    ASSERT_EQ(objects.size(), 4u);
    for (const auto& o : objects) {
      EXPECT_TRUE(Finite(o)) << "id " << o.tracking.s32_Id;
      EXPECT_EQ(o.tracking.s32_MissCount, 0);
      EXPECT_EQ(o.tracking.s32_Id, static_cast<int>(o.detection.u16_Idx)) << "IDs stable across scans";
      if (!o.detection.pcl_ClusterPoints.empty()) {
        EXPECT_GE(o.detection.obb_length_m, .05f - 1e-7f);
        EXPECT_GE(o.detection.obb_width_m, .05f - 1e-7f);
      }
    }
    tracker.Commit(objects);
  }
}

}  // namespace

int main(int argc, char** argv) { testing::InitGoogleTest(&argc, argv); return RUN_ALL_TESTS(); }
