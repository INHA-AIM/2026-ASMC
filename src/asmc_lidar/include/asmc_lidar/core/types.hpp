#pragma once
// Data records of the asmc_lidar shadow pipeline.
//
// Ported from Main's Global.hpp (src/lidar+sensorfusion/src/Main/src/Global/Global.hpp
// at 8f93c22e). Only the fields used by Main's common-geometry LiDAR path are kept.
// Field names are kept on purpose so every ported function can be compared line by
// line with the original. Planning, legacy-map, UDP and MotionClassifier fields are
// not part of this package (see docs/pipeline.md, "port boundary").

#include <asmc_lidar/core/frame_time.hpp>

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


}  // namespace pipeline
}  // namespace lidar
}  // namespace asmc
