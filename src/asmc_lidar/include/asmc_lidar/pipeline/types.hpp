#pragma once
// Data records of the asmc_lidar shadow pipeline.
//
// Ported from Main's Global.hpp (src/lidar+sensorfusion/src/Main/src/Global/Global.hpp
// at 8f93c22e). Only the fields used by Main's common-geometry LiDAR path are kept.
// Field names are kept on purpose so every ported function can be compared line by
// line with the original. Planning, legacy-map, UDP and MotionClassifier fields are
// not part of this package (see docs/shadow-pipeline.md, "port boundary").

#include <asmc_lidar/frame_time.hpp>
#include <asmc_lidar/pipeline/motion_display.hpp>

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <array>
#include <cstdint>
#include <string>

namespace asmc {
namespace lidar {
namespace pipeline {

using float32 = float;
using float64 = double;
using int32 = std::int32_t;
using uint16 = std::uint16_t;
using uint64 = std::uint64_t;
using Cloud = pcl::PointCloud<pcl::PointXYZI>;
using CloudPtr = Cloud::Ptr;

struct Point3D {
  float32 f32_X = 0.f;
  float32 f32_Y = 0.f;
  float32 f32_Z = 0.f;
};

// Main ::P3 (float corner).
struct P3 {
  float x = 0.f, y = 0.f, z = 0.f;
};

// Main VehicleSpec (setting.yaml Vehicle.*). Rear-axle base_link origin.
struct VehicleSpec {
  float32 f32_Width = 0.f;
  float32 f32_Length = 0.f;
  float32 f32_WheelBase = 0.f;
  float32 f32_FrontOverhang = 0.f;
  float32 f32_RearOverhang = 0.f;
  Point3D st_LiDARPos;
  Point3D st_LiDARRotationDeg;
  Point3D st_GPSPos;
};

// Main GPS_DATA. E/N are UTM minus the GPSMessage offsets; U is altitude minus
// the map altitude origin (common path).
struct GpsData {
  float64 f64_Latitude = 0.;
  float64 f64_Longitude = 0.;
  float64 f64_Altitude = 0.;
  float64 f64_E = 0.;
  float64 f64_N = 0.;
  float64 f64_U = 0.;
  bool b_isJamming = false;
};

// Main IMU_DATA. Angles are computed in double from the quaternion and stored
// as float exactly as Main's IMU callback does.
struct ImuData {
  float32 f32_AccelX = 0.f, f32_AccelY = 0.f, f32_AccelZ = 0.f;
  float32 f32_QuatX = 0.f, f32_QuatY = 0.f, f32_QuatZ = 0.f, f32_QuatW = 0.f;
  float32 f32_AngularVelX = 0.f, f32_AngularVelY = 0.f, f32_AngularVelZ = 0.f;
  float32 f32_Roll_rad = 0.f, f32_Pitch_rad = 0.f, f32_Yaw_rad = 0.f;
  float32 f32_Roll_deg = 0.f, f32_Pitch_deg = 0.f, f32_Yaw_deg = 0.f;
  bool b_ImuReady = false;
};

// Main CAN_DATA (vehicle speed from EgoVehicleStatus).
struct CanData {
  float32 f32_CurrentVel_KPH = 0.f;
  float32 f32_CurrentVel_MPS = 0.f;
  float32 f32_Steer_deg = 0.f;
  float32 f32_Steer_rad = 0.f;
};

// Main GPS_IMU: the local inputs selected for one scan (plus the map-derived
// antenna position written back after the map pose lookup).
struct LocalState {
  GpsData st_GPS;
  ImuData st_IMU;
  CanData st_CAN;
  bool b_Jamming = false;
};

// Main GroundEvidence.hpp. Coordinates are the leveled LiDAR frame.
struct GroundPlaneSnapshot {
  bool valid = false;
  std::array<double, 4> coefficients{{0.0, 0.0, 1.0, 0.0}};
  double near_threshold_m = 0.30;
  double far_threshold_m = 0.40;
  double far_range_m = 50.0;
};

struct ObjectGroundEvidence {
  bool valid = false;
  int sample_count = 0;
  double median_clearance_m = 0.0;
  double rejection_band_m = 0.0;
};

// Main LiDARObjectInfo, reduced to the common-geometry path. One record is a
// detection (Id < 0) or a track. All geometry is in the scan's leveled LiDAR
// frame ("lidar_leveled"); typed base_link/map output is produced only by
// asmc::lidar::transform with the scan Context.
struct TrackedObject {
  // Main decision algorithm, monitoring-only. Not a public velocity validity.
  motion_display::MotionObject display_motion;
  // Common tracking frame record (Global.hpp:522-529).
  bool tracking_frame_valid = false;
  float tracking_height_input = 0.f;
  asmc::lidar::Rot tracking_yaw_from_input = asmc::lidar::Rot::Identity();
  asmc::lidar::Vec tracking_origin_in_yaw = asmc::lidar::Vec::Zero();

  // Camera class state (Global.hpp:530-541). str_ObjectClass is the consumer
  // group, str_YoloClass the original detector label.
  std::string str_ObjectClass = "unknown";
  std::string str_YoloClass = "unknown";
  std::string last_seen_class = "unknown";
  int continuity_counter = 0;
  int class_miss_counter = 0;
  uint64 camera_observation_ns = 0;
  uint64 camera_observation_epoch = 0;
  bool is_class_locked = false;

  Cloud pcl_ClusterPoints;
  ObjectGroundEvidence ground_evidence;

  uint16 u16_Idx = 0;
  float32 f32_CoreX = 0.f, f32_CoreY = 0.f, f32_CoreZ = 0.f;
  float32 f32_PredCoreX = 0.f, f32_PredCoreY = 0.f;
  float32 f32_PrevCoreX = 0.f, f32_PrevCoreY = 0.f;
  float32 f32_PrevYaw = 0.f;
  float32 f32_Dist = 0.f;
  float32 f32_MinX = 0.f, f32_MinY = 0.f, f32_MinZ = 0.f;
  float32 f32_MaxX = 0.f, f32_MaxY = 0.f, f32_MaxZ = 0.f;

  // L-shape OBB. Width is the extent along yaw, Height the perpendicular XY
  // extent (not the vertical size). Corners 0-3 at ZMin, 4-7 at ZMax, counter-
  // clockwise from +z; edge 0->1 is parallel to +-yaw.
  float f32_OBB_CenterX = 0.f, f32_OBB_CenterY = 0.f, f32_OBB_CenterZ = 0.f;
  float f32_OBB_Width = 0.f, f32_OBB_Height = 0.f;
  float f32_OBB_ZMin = 0.f, f32_OBB_ZMax = 0.f;
  float f32_OBB_YawRad = 0.f;
  std::array<P3, 8> OBB_Corners_Body{};

  // Internal velocity bookkeeping kept for behaviour parity. f32_AbsVx/Vy is
  // Main's "KF relative velocity + CAN speed on the leveled x axis"; its frame is
  // not validated, so it is never published as an absolute velocity.
  float32 f32_Velocity = 0.f;
  float32 f32_AbsVx = 0.f, f32_AbsVy = 0.f;
  float32 f32_DirX = 0.f, f32_DirY = 0.f;

  int32 s32_Id = -1;
  int32 s32_Age = 0;
  int32 s32_MissCount = 0;
  int32 s32_KfInitCount = 0;
  int32 s32_DynamicCount = 0;
  int32 s32_StaticCount = 0;

  // Relative constant-acceleration state [x, y, vx, vy, ax, ay] in the leveled frame.
  Eigen::Matrix<float, 6, 1> Eigen_KfState_CA = Eigen::Matrix<float, 6, 1>::Zero();
  Eigen::Matrix<float, 6, 6> Eigen_KfP_CA = Eigen::Matrix<float, 6, 6>::Identity();
  bool b_KfInitialized = false;
};

}  // namespace pipeline
}  // namespace lidar
}  // namespace asmc
