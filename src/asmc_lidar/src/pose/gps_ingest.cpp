// GPS ingestion/sanity and IMU/CAN message conversion.
// Ported from Main src/Parser/GpsSanityFilter.cpp and the common-geometry
// branches of GPSParser_Callback / IMUParser_Callback / CANParser_Callback in
// src/Parser/Parser.cpp at 8f93c22e, without the ROS transport. Legacy-only
// branches (enable_common_geometry=false) are dropped.
#include <asmc_lidar/pipeline/pose_history.hpp>

#include <GeographicLib/UTMUPS.hpp>

#include <algorithm>
#include <cmath>

namespace asmc {
namespace lidar {
namespace pipeline {
namespace {

// Ported from Main Global.cpp:10-13 (8f93c22e): float overload.
float32 Deg2Rad(const float32& f32_Deg) { return f32_Deg * M_PI / 180.0f; }

// Ported from Main Global.cpp:20-23 (8f93c22e): float overload.
float32 Rad2Deg(const float32& f32_Rad) { return f32_Rad * 180.0f / M_PI; }

// ros::Time(stamp).toSec(): double(sec) + 1e-9 * double(nsec).
double RosToSec(std::uint64_t stamp_ns) {
  const auto sec = static_cast<std::uint32_t>(stamp_ns / 1000000000ULL);
  const auto nsec = static_cast<std::uint32_t>(stamp_ns % 1000000000ULL);
  return static_cast<double>(sec) + 1e-9 * static_cast<double>(nsec);
}

}  // namespace

// Ported from Main GpsSanityFilter.cpp:6-7 (8f93c22e)
GpsSanityFilter::GpsSanityFilter(const GpsSanityConfig& config) : config_(config) {}

// Ported from Main GpsSanityFilter.cpp:9-60 (8f93c22e)
GpsSanityDecision GpsSanityFilter::Evaluate(double map_x, double map_y, double source_stamp_s) {
  GpsSanityDecision decision;
  if (!std::isfinite(map_x) || !std::isfinite(map_y) || !std::isfinite(source_stamp_s) || source_stamp_s <= 0.0) {
    decision.reason = "non-finite coordinate or stamp";
    return decision;
  }
  if (map_x < config_.min_x || map_x > config_.max_x || map_y < config_.min_y || map_y > config_.max_y) {
    decision.reason = "outside configured local-map bounds";
    return decision;
  }
  if (!has_accepted_) {
    has_accepted_ = true;
    accepted_x_ = map_x;
    accepted_y_ = map_y;
    accepted_stamp_s_ = source_stamp_s;
    decision.accepted = true;
    decision.reason = "initial valid sample";
    return decision;
  }

  const double elapsed_s = source_stamp_s - accepted_stamp_s_;
  decision.jump_m = std::hypot(map_x - accepted_x_, map_y - accepted_y_);
  if (elapsed_s < -1e-3) {
    decision.reason = "source timestamp moved backwards";
    return decision;
  }
  if (elapsed_s >= config_.reanchor_after_s) {
    accepted_x_ = map_x;
    accepted_y_ = map_y;
    accepted_stamp_s_ = source_stamp_s;
    decision.accepted = true;
    decision.reason = "reanchored after input gap";
    return decision;
  }
  decision.allowed_jump_m = config_.base_jump_allowance_m + config_.maximum_speed_mps * std::max(0.0, elapsed_s);
  if (decision.jump_m > decision.allowed_jump_m) {
    decision.reason = "implausible position jump";
    return decision;
  }

  accepted_x_ = map_x;
  accepted_y_ = map_y;
  accepted_stamp_s_ = source_stamp_s;
  decision.accepted = true;
  decision.reason = "continuous sample";
  return decision;
}

// Ported from Main GpsSanityFilter.cpp:62-67 (8f93c22e)
void GpsSanityFilter::Reset() {
  has_accepted_ = false;
  accepted_x_ = 0.0;
  accepted_y_ = 0.0;
  accepted_stamp_s_ = 0.0;
}

// Ported from Main Parser.cpp:566-598 (8f93c22e). The Euler products are float
// (IMU_DATA fields) promoted to double by the 2.0 factor, stored as float.
ImuData ImuFromInput(const ImuInput& input) {
  ImuData st_IMUData;
  st_IMUData.f32_AccelX = input.ax;
  st_IMUData.f32_AccelY = input.ay;
  st_IMUData.f32_AccelZ = input.az;

  st_IMUData.f32_AngularVelX = input.wx;
  st_IMUData.f32_AngularVelY = input.wy;
  st_IMUData.f32_AngularVelZ = input.wz;

  st_IMUData.f32_QuatX = input.qx;
  st_IMUData.f32_QuatY = input.qy;
  st_IMUData.f32_QuatZ = input.qz;
  st_IMUData.f32_QuatW = input.qw;

  st_IMUData.f32_Roll_rad =
      std::atan2(2.0 * (st_IMUData.f32_QuatW * st_IMUData.f32_QuatX + st_IMUData.f32_QuatY * st_IMUData.f32_QuatZ),
                 1.0 - 2.0 * (st_IMUData.f32_QuatX * st_IMUData.f32_QuatX + st_IMUData.f32_QuatY * st_IMUData.f32_QuatY));
  st_IMUData.f32_Pitch_rad =
      std::asin(2.0 * (st_IMUData.f32_QuatW * st_IMUData.f32_QuatY - st_IMUData.f32_QuatZ * st_IMUData.f32_QuatX));
  st_IMUData.f32_Yaw_rad =
      std::atan2(2.0 * (st_IMUData.f32_QuatW * st_IMUData.f32_QuatZ + st_IMUData.f32_QuatX * st_IMUData.f32_QuatY),
                 1.0 - 2.0 * (st_IMUData.f32_QuatY * st_IMUData.f32_QuatY + st_IMUData.f32_QuatZ * st_IMUData.f32_QuatZ));

  st_IMUData.f32_Roll_deg = Rad2Deg(st_IMUData.f32_Roll_rad);
  st_IMUData.f32_Pitch_deg = Rad2Deg(st_IMUData.f32_Pitch_rad);
  st_IMUData.f32_Yaw_deg = Rad2Deg(st_IMUData.f32_Yaw_rad);
  // Common branch: the norm uses the message (double) quaternion.
  const double norm = input.qx * input.qx + input.qy * input.qy + input.qz * input.qz + input.qw * input.qw;
  st_IMUData.b_ImuReady = input.stamp_ns != 0 && std::isfinite(norm) && std::abs(norm - 1.) <= .001;
  return st_IMUData;
}

// Ported from Main Parser.cpp:519-529 (8f93c22e)
CanData CanFromInput(const CanInput& input) {
  CanData st_CANData;
  st_CANData.f32_CurrentVel_KPH =
      3.6 * std::sqrt(std::pow(input.vx, 2) + std::pow(input.vy, 2) + std::pow(input.vz, 2));
  st_CANData.f32_CurrentVel_MPS = std::sqrt(std::pow(input.vx, 2) + std::pow(input.vy, 2) + std::pow(input.vz, 2));
  if (input.stamp_ns == 0) {
    st_CANData.f32_CurrentVel_MPS = NAN;
    st_CANData.f32_CurrentVel_KPH = NAN;
  }
  // EgoVehicleStatus.wheel_angle is float32: Main selects the float Deg2Rad.
  const float32 wheel_angle = static_cast<float32>(input.wheel_angle_deg);
  st_CANData.f32_Steer_deg = wheel_angle;
  st_CANData.f32_Steer_rad = Deg2Rad(wheel_angle);
  return st_CANData;
}

GpsIngest::GpsIngest(const GpsIngestConfig& config) : config_(config), sanity_(config.sanity) {}

// Ported from Main FrameBoundary.cpp:37-40 (8f93c22e)
bool GpsIngest::MapOffsetsValid(double east, double north) const {
  return std::isfinite(east) && std::isfinite(north) && std::abs(east - config_.map_origin.x()) <= 1e-6 &&
         std::abs(north - config_.map_origin.y()) <= 1e-6;
}

// Ported from Main Parser.cpp:398-468 (8f93c22e), common branch.
GpsIngestResult GpsIngest::Evaluate(const GpsInput& input, std::uint64_t ros_now_ns) {
  GpsIngestResult result;
  GpsData st_GPSData;
  st_GPSData.f64_Latitude = input.latitude;
  st_GPSData.f64_Longitude = input.longitude;
  st_GPSData.f64_Altitude = input.altitude;
  st_GPSData.b_isJamming = (input.status == 0);
  // Main invalidate(): AddGpsInvalidAt(original_stamp.toNSec(), receipt) - may be 0.
  result.source_ns = input.stamp_ns;
  result.data = st_GPSData;
  if (input.status == 0 || input.stamp_ns == 0 || !std::isfinite(input.latitude) ||
      !std::isfinite(input.longitude) || !std::isfinite(input.altitude) || !std::isfinite(input.east_offset) ||
      !std::isfinite(input.north_offset) || !MapOffsetsValid(input.east_offset, input.north_offset) ||
      (input.latitude == 0. && input.longitude == 0.)) {
    // Main logs one "invalid status" message for this whole group; the suffix
    // names the first failing term in Main's evaluation order (diagnostic only).
    if (input.status == 0)
      result.reason = "invalid status: status 0";
    else if (input.stamp_ns == 0)
      result.reason = "invalid status: zero stamp";
    else if (!std::isfinite(input.latitude) || !std::isfinite(input.longitude) || !std::isfinite(input.altitude) ||
             !std::isfinite(input.east_offset) || !std::isfinite(input.north_offset))
      result.reason = "invalid status: non-finite field";
    else if (!MapOffsetsValid(input.east_offset, input.north_offset))
      result.reason = "invalid status: map offsets mismatch";
    else
      result.reason = "invalid status: zero latitude/longitude";
    return result;
  }

  int zone = 52;  // overwritten: STANDARD zone selection (no forced zone)
  bool north = true;
  double east_m, north_m;
  // Deliberate deviation: Main lets GeographicErr (|latitude| > 90, |longitude|
  // > 540) escape the GPS callback and terminate the process. Here such a sample
  // takes the invalid path like any other unusable GPS message.
  try {
    GeographicLib::UTMUPS::Forward(st_GPSData.f64_Latitude, st_GPSData.f64_Longitude, zone, north, east_m,
                                   north_m);
  } catch (const GeographicLib::GeographicErr&) {
    result.reason = "invalid status: latitude/longitude out of range";
    return result;
  }
  st_GPSData.f64_E = east_m - input.east_offset;
  st_GPSData.f64_N = north_m - input.north_offset;
  st_GPSData.f64_U = input.altitude - config_.map_altitude_origin_m;
  result.data = st_GPSData;

  // Unreachable substitution in the common branch (zero stamps are rejected above).
  const std::uint64_t source_stamp_ns = input.stamp_ns == 0 ? ros_now_ns : input.stamp_ns;
  result.sanity = sanity_.Evaluate(st_GPSData.f64_E, st_GPSData.f64_N, RosToSec(source_stamp_ns));
  if (!result.sanity.accepted) {
    result.reason = result.sanity.reason;
    return result;
  }
  result.accepted = true;
  result.stamp_ms = static_cast<std::uint64_t>(source_stamp_ns / 1000000ULL);
  result.source_ns = source_stamp_ns;
  return result;
}

// Ported from Main Parser.cpp:411-415,467-468 (8f93c22e): invalidate() / AddGps.
void GpsIngest::Apply(const GpsIngestResult& result, std::uint64_t receipt_ns, PoseHistory& history) {
  if (result.accepted)
    history.AddGps(result.stamp_ms, result.data, result.source_ns, receipt_ns);
  else
    history.AddGpsInvalidAt(result.source_ns, receipt_ns);
}

}  // namespace pipeline
}  // namespace lidar
}  // namespace asmc
