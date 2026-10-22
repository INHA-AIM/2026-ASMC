#pragma once
// Scan-time local input selection (IMU/CAN), GPS ingestion and bounded map pose.
// Ported from Main Parser.cpp (common-geometry branches of the GPS/IMU/CAN
// callbacks), Parser/SensorPoseHistory.cpp and Parser/GpsSanityFilter.cpp at
// 8f93c22e. Only the nanosecond (common) lookups are ported; the legacy GPS-gated
// lookup and dead-reckoning extrapolation are not.

#include <asmc_lidar/bounded_pose.hpp>
#include <asmc_lidar/pipeline/types.hpp>

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>

namespace asmc {
namespace lidar {
namespace pipeline {

// Main SensorPoseHistoryConfig. The struct default of imu_yaw_is_true_enu is
// false like Main's; the shadow configuration sets true like main.launch.
struct PoseHistoryConfig {
  bool imu_yaw_is_true_enu = false;
  double gps_max_age_s = 0.30;
  double imu_max_age_s = 0.12;
  double can_max_age_s = 0.12;
  double maximum_extrapolation_speed_mps = 35.0;
  double imu_base_yaw_allowance_rad = 0.0872664626;  // 5 deg
  double imu_maximum_yaw_rate_radps = 2.0943951024;  // 120 deg/s
  double can_base_speed_allowance_mps = 1.0;
  double can_maximum_acceleration_mps2 = 15.0;
  double stream_reanchor_after_s = 2.0;
  double history_duration_s = 5.0;
  // Main st_VehicleSpec.st_GPSPos (float, setting.yaml GPS.*): the antenna lever
  // removed by the bounded map pose. The runtime copies the vehicle block here.
  Point3D gps_in_base{0.35f, 0.f, 1.25f};
};

struct LocalInputSelection {
  std::uint64_t imu_stamp_ns = 0, imu_receipt_ns = 0;
  std::uint64_t can_stamp_ns = 0, can_receipt_ns = 0;
};

// Main SensorPoseHistory, common path. receipt_ns is the arrival clock of the
// caller ("steady" on the node, the logical bag clock in the replay).
class PoseHistory {
 public:
  explicit PoseHistory(const PoseHistoryConfig& config = {});
  void Configure(const PoseHistoryConfig& config);
  const PoseHistoryConfig& config() const { return config_; }

  // Callback contract (Main common branch): stamp_ms = SourceStampMs(header ns,
  // ros now), source_ns = the original header ns (0 when missing), receipt_ns =
  // arrival clock. Rejections of IMU/CAN raise the newer-invalid barrier.
  bool AddImu(std::uint64_t stamp_ms, const ImuData& imu, std::string* reason,
              std::uint64_t source_ns, std::uint64_t receipt_ns);
  bool AddCan(std::uint64_t stamp_ms, const CanData& can, std::string* reason,
              std::uint64_t source_ns, std::uint64_t receipt_ns);
  // ros_now_ns is Main's ros::Time::now() arrival fallback, used only when
  // receipt_ns is 0 (never on the runtime path).
  void AddGps(std::uint64_t stamp_ms, const GpsData& gps, std::uint64_t source_ns,
              std::uint64_t receipt_ns, std::uint64_t ros_now_ns = 0);
  void AddGpsInvalidAt(std::uint64_t source_ns, std::uint64_t receipt_ns);

  // Main WaitForLocalLookupNs: latest IMU and CAN at or before stamp_ns within
  // their max ages, no interpolation. The newer-invalid barrier is Main's single
  // latest-rejected stamp per stream (max of rejected IMU/CAN stamps): it blocks
  // falling back to an older valid sample only while that latest rejection lies
  // between the selected sample and the scan. Waits at most timeout_s for
  // samples that can still satisfy the stamp.
  bool WaitForLocalLookupNs(std::uint64_t stamp_ns, double timeout_s, LocalState* local,
                            LocalInputSelection* selection, std::string* reason) const;

  // Main MapAt(Observation, local): BoundedPoseHistory (300 ms causal GPS age,
  // 3 consecutive normal samples to recover, lever arm removed).
  asmc::lidar::PoseSelection MapAt(const asmc::lidar::Observation& observation,
                                   const LocalState& local) const;

 private:
  struct TimedGps {
    std::uint64_t stamp_ms = 0;
    GpsData data{};
    std::uint64_t source_ns = 0;
  };
  struct TimedImu {
    std::uint64_t stamp_ms = 0;
    ImuData data{};
    std::uint64_t source_ns = 0, receipt_ns = 0;
  };
  struct TimedCan {
    std::uint64_t stamp_ms = 0;
    CanData data{};
    std::uint64_t source_ns = 0, receipt_ns = 0;
  };
  bool LookupLocalLocked(std::uint64_t source_ns, LocalState* local, std::string* reason,
                         LocalInputSelection* selection) const;
  void PruneGpsLocked();
  void PruneImuLocked();
  void PruneCanLocked();

  asmc::lidar::BoundedPoseHistory bounded_gps_;
  mutable std::mutex mutex_;
  mutable std::condition_variable sample_cv_;
  PoseHistoryConfig config_;
  std::deque<TimedGps> gps_;
  std::deque<TimedImu> imu_;
  std::deque<TimedCan> can_;
  std::uint64_t invalid_imu_stamp_ms_ = 0, invalid_can_stamp_ms_ = 0;
  std::uint64_t invalid_imu_stamp_ns_ = 0, invalid_can_stamp_ns_ = 0;
  bool has_accepted_imu_ = false;
  double accepted_imu_yaw_rad_ = 0.0;
  std::uint64_t accepted_imu_stamp_ms_ = 0;
  std::uint64_t last_seen_imu_stamp_ms_ = 0;
  bool has_accepted_can_ = false;
  double accepted_can_speed_mps_ = 0.0;
  std::uint64_t accepted_can_stamp_ms_ = 0;
  std::uint64_t last_seen_can_stamp_ms_ = 0;
};

// Main GridImu (SensorPoseHistory.cpp): true-ENU IMU yaw -> UTM grid yaw using
// the meridian convergence at the GPS lat/lon (zone 52 forced, lat +-1e-5).
// Identity when !true_enu. Roll/pitch unchanged.
ImuData GridImu(const ImuData& imu, const GpsData& gps, bool true_enu);

// Main callbacks: ms stamp of the header, or of ros::Time::now() when the
// header stamp is zero (toNSec()/1000000).
std::uint64_t SourceStampMs(std::uint64_t header_stamp_ns, std::uint64_t ros_now_ns);

// Main GpsSanityFilter (defaults = main.launch values).
struct GpsSanityConfig {
  double min_x = -454.5159;
  double min_y = -825.0458;
  double max_x = 162.4841;
  double max_y = 621.3542;
  double base_jump_allowance_m = 3.0;
  double maximum_speed_mps = 80.0;
  double reanchor_after_s = 2.0;
};
struct GpsSanityDecision {
  bool accepted = false;
  std::string reason;
  double jump_m = 0.0;
  double allowed_jump_m = 0.0;
};
class GpsSanityFilter {
 public:
  explicit GpsSanityFilter(const GpsSanityConfig& config = {});
  GpsSanityDecision Evaluate(double map_x, double map_y, double source_stamp_s);
  void Reset();

 private:
  GpsSanityConfig config_;
  bool has_accepted_ = false;
  double accepted_x_ = 0.0;
  double accepted_y_ = 0.0;
  double accepted_stamp_s_ = 0.0;
};

// Sensor message contents, independent of ROS message types.
struct ImuInput {
  std::uint64_t stamp_ns = 0;  // header stamp (0 = missing)
  double qx = 0, qy = 0, qz = 0, qw = 1;
  double ax = 0, ay = 0, az = 0;
  double wx = 0, wy = 0, wz = 0;
};
struct CanInput {
  std::uint64_t stamp_ns = 0;
  double vx = 0, vy = 0, vz = 0;
  double wheel_angle_deg = 0;
};
struct GpsInput {
  std::uint64_t stamp_ns = 0;
  double latitude = 0, longitude = 0, altitude = 0;
  double east_offset = 0, north_offset = 0;
  int status = 0;
};

// Main IMUParser_Callback math (common branch: ready requires a non-zero stamp
// and |q|^2 within 1e-3 of 1).
ImuData ImuFromInput(const ImuInput& input);
// Main CANParser_Callback math (common branch: zero stamp -> NaN speed).
CanData CanFromInput(const CanInput& input);

// Main GPSParser_Callback, common branch, without the ROS transport.
struct GpsIngestConfig {
  GpsSanityConfig sanity;
  asmc::lidar::Vec map_origin = asmc::lidar::Vec::Zero();  // HD-map origin (UTM E/N, z)
  double map_altitude_origin_m = 0.0;                        // must equal map_origin.z()
};
struct GpsIngestResult {
  bool accepted = false;
  std::string reason;  // empty when accepted
  GpsData data{};
  std::uint64_t source_ns = 0;  // header stamp, 0 when missing
  std::uint64_t stamp_ms = 0;
  GpsSanityDecision sanity;     // set once the sample reached the sanity gate
};
// Not internally synchronized (Main guards its filter with g_GpsSanityMutex):
// callers serialize GPS callbacks.
class GpsIngest {
 public:
  explicit GpsIngest(const GpsIngestConfig& config = {});
  // ros_now_ns substitutes a zero header stamp exactly where Main does.
  GpsIngestResult Evaluate(const GpsInput& input, std::uint64_t ros_now_ns);
  // Feeds the result into the history: AddGps on accept, AddGpsInvalidAt otherwise.
  static void Apply(const GpsIngestResult& result, std::uint64_t receipt_ns, PoseHistory& history);
  bool MapOffsetsValid(double east, double north) const;

 private:
  GpsIngestConfig config_;
  GpsSanityFilter sanity_;
};

}  // namespace pipeline
}  // namespace lidar
}  // namespace asmc
