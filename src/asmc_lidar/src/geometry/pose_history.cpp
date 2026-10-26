// Scan-time local inputs (IMU/CAN) and bounded map pose.
// Ported from Main src/Parser/SensorPoseHistory.cpp at 8f93c22e, common-geometry
// branch only (frame_boundary::Enabled() == true). Not ported: Lookup,
// WaitForLookup, LookupLocked (legacy GPS-gated lookup), WaitForLocalLookup and
// MapAt(uint64) (ms API), AddGpsInvalid (legacy barrier), Reset (never called).
#include <asmc_lidar/geometry/pose_history.hpp>

#include <GeographicLib/UTMUPS.hpp>
#include <asmc_geometry/geodesy.hpp>
#include <asmc_geometry/time.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iterator>

namespace asmc {
namespace lidar {
namespace pipeline {
namespace {

constexpr double kPi = 3.14159265358979323846;

// Main's ordering key: source ns, or ms * 1e6 when the source stamp is missing.
template <typename Sample>
std::uint64_t SampleNs(const Sample& s) {
  return s.source_ns ? s.source_ns : s.stamp_ms * 1000000ULL;
}

// Ported from Main SensorPoseHistory.cpp:32-45 (8f93c22e)
template <typename Sample>
void InsertOrdered(std::deque<Sample>* samples, const Sample& sample) {
  const auto it = std::lower_bound(samples->begin(), samples->end(), SampleNs(sample),
                                   [](const Sample& lhs, std::uint64_t ns) { return SampleNs(lhs) < ns; });
  if (it != samples->end() && SampleNs(*it) == SampleNs(sample)) {
    *it = sample;
  } else {
    samples->insert(it, sample);
  }
}

// Ported from Main SensorPoseHistory.cpp:47-56 (8f93c22e). Used only by AddGps
// for a sample without a source stamp.
template <typename Sample>
typename std::deque<Sample>::const_iterator LatestAtOrBefore(const std::deque<Sample>& samples,
                                                             std::uint64_t stamp_ms) {
  const auto upper = std::upper_bound(samples.begin(), samples.end(), stamp_ms,
                                      [](std::uint64_t value, const Sample& rhs) { return value < rhs.stamp_ms; });
  return upper == samples.begin() ? samples.end() : std::prev(upper);
}

// Ported from Main SensorPoseHistory.cpp:58-60 (8f93c22e)
void SetReason(std::string* reason, const char* value) {
  if (reason != nullptr) *reason = value;
}

// Ported from Main FrameBoundary.cpp:51-53 (8f93c22e): float angles promoted to double.
asmc::lidar::Rot MapRotation(const LocalState& pose) {
  return asmc::lidar::rpy(pose.st_IMU.f32_Roll_rad, pose.st_IMU.f32_Pitch_rad, pose.st_IMU.f32_Yaw_rad);
}

}  // namespace

// Ported from Main SensorPoseHistory.cpp:15-30 (8f93c22e)
namespace {
double GridOffset(const GpsData& gps) {
  return asmc::geometry::GridYawOffset(gps.f64_Latitude,gps.f64_Longitude,
      [](double lat,double lon,double& e,double& n){
        int zone=52;bool north=true;
        GeographicLib::UTMUPS::Forward(lat,lon,zone,north,e,n,52);
      });
}
ImuData ApplyGridOffset(const ImuData& imu,double offset) {
  ImuData out=imu;
  out.f32_Yaw_rad=std::remainder(double(imu.f32_Yaw_rad)+offset,2.*kPi);
  out.f32_Yaw_deg=out.f32_Yaw_rad*180./kPi;
  return out;
}
template<class Range>
auto LatestSource(const Range& samples,std::uint64_t ns) -> decltype(samples.begin()) {
  return asmc::geometry::LatestAtOrBefore(samples,ns,[](const auto& s){return SampleNs(s);});
}
}  // namespace
ImuData GridImu(const ImuData& imu,const GpsData& gps,bool true_enu) {
  return true_enu?ApplyGridOffset(imu,GridOffset(gps)):imu;
}

// Ported from Main Parser.cpp:531-534/600-603 (8f93c22e): ros::Time toNSec()/1000000.
std::uint64_t SourceStampMs(std::uint64_t header_stamp_ns, std::uint64_t ros_now_ns) {
  const std::uint64_t source_ns = header_stamp_ns == 0 ? ros_now_ns : header_stamp_ns;
  return static_cast<std::uint64_t>(source_ns / 1000000ULL);
}

// Ported from Main SensorPoseHistory.cpp:64-67 (8f93c22e)
PoseHistory::PoseHistory(const PoseHistoryConfig& config)
    : bounded_gps_(static_cast<asmc::lidar::Ns>(std::min(.30, std::max(.001, config.gps_max_age_s)) * 1e9)),
      config_(config) {}

// Ported from Main SensorPoseHistory.cpp:69-86 (8f93c22e)
void PoseHistory::Configure(const PoseHistoryConfig& config) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    config_ = config;
    bounded_gps_ = asmc::lidar::BoundedPoseHistory(
        static_cast<asmc::lidar::Ns>(std::min(.30, std::max(.001, config_.gps_max_age_s)) * 1e9));
    gps_.clear();
    imu_.clear();
    can_.clear();
    invalid_imu_stamp_ms_ = invalid_can_stamp_ms_ = 0;
    invalid_imu_stamp_ns_ = invalid_can_stamp_ns_ = 0;
    imu_latest_seen_ns_ = imu_latest_receipt_ns_ = 0;
    imu_rejection_reason_.clear();
    has_accepted_imu_ = false;
    has_accepted_can_ = false;
    last_seen_imu_stamp_ms_ = 0;
    last_seen_can_stamp_ms_ = 0;
  }
  sample_cv_.notify_all();
}

// Ported from Main SensorPoseHistory.cpp:88-134 (8f93c22e)
void PoseHistory::AddGps(std::uint64_t stamp_ms, const GpsData& gps, std::uint64_t source_ns,
                         std::uint64_t receipt_ns, std::uint64_t ros_now_ns) {
  if (stamp_ms == 0 || !std::isfinite(gps.f64_E) || !std::isfinite(gps.f64_N) || !std::isfinite(gps.f64_U)) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (config_.imu_yaw_is_true_enu &&
        (!std::isfinite(gps.f64_Latitude) || !std::isfinite(gps.f64_Longitude) || gps.f64_Latitude < 0. ||
         gps.f64_Latitude > 84. || gps.f64_Longitude < 126. || gps.f64_Longitude > 132. || gps.b_isJamming)) {
      return;  // This deployment's map contract is UTM zone 52N.
    }
    const double grid_offset=config_.imu_yaw_is_true_enu?GridOffset(gps):0.;
    InsertOrdered(&gps_, TimedGps{stamp_ms, gps, source_ns, grid_offset});
    // Common branch (frame_boundary::Enabled()); the legacy path stores only gps_.
    asmc::lidar::PoseSample sample;
    sample.stamp_ns = source_ns ? source_ns : static_cast<asmc::lidar::Ns>(stamp_ms) * 1000000LL;
    sample.arrival_ns = receipt_ns ? receipt_ns : static_cast<asmc::lidar::Ns>(ros_now_ns);
    if (receipt_ns) sample.arrival_clock = "steady";
    sample.clock = "ros";
    sample.valid = !gps.b_isJamming;
    sample.reference_is_gps = true;
    sample.gps_in_base = asmc::lidar::Vec(config_.gps_in_base.f32_X, config_.gps_in_base.f32_Y,
                                          config_.gps_in_base.f32_Z);
    sample.map_from_reference.translation() = asmc::lidar::Vec(gps.f64_E, gps.f64_N, gps.f64_U);
    auto attitude = LatestAtOrBefore(imu_, stamp_ms);
    if (source_ns) {
      attitude = LatestSource(imu_,source_ns);
    }
    const auto attitude_ns = attitude == imu_.end() ? 0 : SampleNs(*attitude);
    if (!asmc::geometry::WithinAge(attitude_ns,sample.stamp_ns,config_.imu_max_age_s) ||
        (invalid_imu_stamp_ns_ <= static_cast<std::uint64_t>(sample.stamp_ns) &&
         invalid_imu_stamp_ns_ >= attitude_ns)) {
      sample.valid = false;
    } else {
      LocalState at_gps{};
      at_gps.st_IMU = config_.imu_yaw_is_true_enu?ApplyGridOffset(attitude->data,grid_offset):attitude->data;
      sample.map_from_reference.linear() = MapRotation(at_gps);
    }
    bounded_gps_.append(sample);  // Main ignores the append result.
    PruneGpsLocked();
  }
  sample_cv_.notify_all();
}

// Ported from Main SensorPoseHistory.cpp:136-186 (8f93c22e)
bool PoseHistory::AddImu(std::uint64_t stamp_ms, const ImuData& imu, std::string* reason,
                         std::uint64_t source_ns, std::uint64_t receipt_ns) {
  std::lock_guard<std::mutex> lock(mutex_);
  imu_latest_seen_ns_=source_ns?source_ns:stamp_ms*1000000ULL;
  imu_latest_receipt_ns_=receipt_ns;
  if (stamp_ms == 0 || !imu.b_ImuReady || !std::isfinite(imu.f32_Yaw_rad) || !std::isfinite(imu.f32_Roll_rad) ||
      !std::isfinite(imu.f32_Pitch_rad)) {
    {
      imu_rejection_reason_="invalid IMU sample";
      invalid_imu_stamp_ms_ = std::max(invalid_imu_stamp_ms_, stamp_ms);
      invalid_imu_stamp_ns_ =
          std::max<std::uint64_t>(invalid_imu_stamp_ns_, source_ns ? source_ns : stamp_ms * 1000000ULL);
    }
    SetReason(reason, "invalid IMU sample");
    return false;
  }
  const double yaw = std::remainder(static_cast<double>(imu.f32_Yaw_rad), 2.0 * kPi);
  if (last_seen_imu_stamp_ms_ != 0 && stamp_ms < last_seen_imu_stamp_ms_) {
    imu_rejection_reason_="IMU timestamp moved backwards";
    SetReason(reason, "IMU timestamp moved backwards");
    return false;
  }
  const double seen_gap_s =
      last_seen_imu_stamp_ms_ == 0 ? 0.0 : static_cast<double>(stamp_ms - last_seen_imu_stamp_ms_) / 1000.0;
  last_seen_imu_stamp_ms_ = stamp_ms;
  if (has_accepted_imu_ && seen_gap_s < config_.stream_reanchor_after_s) {
    if (stamp_ms < accepted_imu_stamp_ms_) {
      imu_rejection_reason_="IMU timestamp moved backwards";
      SetReason(reason, "IMU timestamp moved backwards");
      return false;
    }
    const double elapsed_s = static_cast<double>(stamp_ms - accepted_imu_stamp_ms_) / 1000.0;
    const double allowed = config_.imu_base_yaw_allowance_rad + config_.imu_maximum_yaw_rate_radps * elapsed_s;
    const double jump = std::abs(std::remainder(yaw - accepted_imu_yaw_rad_, 2.0 * kPi));
    if (jump > allowed) {
      invalid_imu_stamp_ms_ = std::max(invalid_imu_stamp_ms_, stamp_ms);
      invalid_imu_stamp_ns_ =
          std::max<std::uint64_t>(invalid_imu_stamp_ns_, source_ns ? source_ns : stamp_ms * 1000000ULL);
      imu_rejection_reason_="implausible IMU yaw jump";
      SetReason(reason, "implausible IMU yaw jump");
      return false;
    }
  }
  InsertOrdered(&imu_, TimedImu{stamp_ms, imu, source_ns, receipt_ns});
  has_accepted_imu_ = true;
  accepted_imu_yaw_rad_ = yaw;
  accepted_imu_stamp_ms_ = stamp_ms;
  PruneImuLocked();
  SetReason(reason, "accepted");
  sample_cv_.notify_all();
  return true;
}

// Ported from Main SensorPoseHistory.cpp:188-233 (8f93c22e)
bool PoseHistory::AddCan(std::uint64_t stamp_ms, const CanData& can, std::string* reason,
                         std::uint64_t source_ns, std::uint64_t receipt_ns) {
  const double speed_mps = can.f32_CurrentVel_MPS;
  if (stamp_ms == 0 || !std::isfinite(speed_mps) || speed_mps < 0.0) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      invalid_can_stamp_ms_ = std::max(invalid_can_stamp_ms_, stamp_ms);
      invalid_can_stamp_ns_ =
          std::max<std::uint64_t>(invalid_can_stamp_ns_, source_ns ? source_ns : stamp_ms * 1000000ULL);
    }
    SetReason(reason, "invalid vehicle-speed sample");
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (last_seen_can_stamp_ms_ != 0 && stamp_ms < last_seen_can_stamp_ms_) {
    SetReason(reason, "vehicle-speed timestamp moved backwards");
    return false;
  }
  const double seen_gap_s =
      last_seen_can_stamp_ms_ == 0 ? 0.0 : static_cast<double>(stamp_ms - last_seen_can_stamp_ms_) / 1000.0;
  last_seen_can_stamp_ms_ = stamp_ms;
  if (has_accepted_can_ && seen_gap_s < config_.stream_reanchor_after_s) {
    if (stamp_ms < accepted_can_stamp_ms_) {
      SetReason(reason, "vehicle-speed timestamp moved backwards");
      return false;
    }
    const double elapsed_s = static_cast<double>(stamp_ms - accepted_can_stamp_ms_) / 1000.0;
    const double allowed =
        config_.can_base_speed_allowance_mps + config_.can_maximum_acceleration_mps2 * elapsed_s;
    if (std::abs(speed_mps - accepted_can_speed_mps_) > allowed) {
      invalid_can_stamp_ms_ = std::max(invalid_can_stamp_ms_, stamp_ms);
      invalid_can_stamp_ns_ =
          std::max<std::uint64_t>(invalid_can_stamp_ns_, source_ns ? source_ns : stamp_ms * 1000000ULL);
      SetReason(reason, "implausible vehicle-speed jump");
      return false;
    }
  }
  InsertOrdered(&can_, TimedCan{stamp_ms, can, source_ns, receipt_ns});
  has_accepted_can_ = true;
  accepted_can_speed_mps_ = speed_mps;
  accepted_can_stamp_ms_ = stamp_ms;
  PruneCanLocked();
  SetReason(reason, "accepted");
  sample_cv_.notify_all();
  return true;
}

// Ported from Main SensorPoseHistory.cpp:333-384 (8f93c22e), source_ns != 0
// branch only (the ms LatestAtOrBefore result is overwritten there in Main).
bool PoseHistory::LookupLocalLocked(std::uint64_t source_ns, LocalState* local, std::string* reason,
                                    LocalInputSelection* selection) const {
  const auto imu_latest=LatestSource(imu_,source_ns);
  const auto can_latest=LatestSource(can_,source_ns);
  if(selection) {
    *selection={};
    selection->imu_latest_seen_ns=imu_latest_seen_ns_;
    selection->imu_latest_receipt_ns=imu_latest_receipt_ns_;
    selection->imu_invalid_ns=invalid_imu_stamp_ns_;
    selection->imu_rejection_reason=imu_rejection_reason_;
    if(imu_latest!=imu_.end()) {
      selection->imu_stamp_ns=SampleNs(*imu_latest);
      selection->imu_receipt_ns=imu_latest->receipt_ns;
    }
    if(can_latest!=can_.end()) {
      selection->can_stamp_ns=SampleNs(*can_latest);
      selection->can_receipt_ns=can_latest->receipt_ns;
    }
  }
  if (imu_latest == imu_.end()) {
    SetReason(reason, "no IMU at or before LiDAR scan");
    return false;
  }
  // A fresh-but-rejected sample is not a sensor Hz outage. Report its barrier
  // before age; never use the rejected orientation or loosen the 120ms bound.
  if(invalid_imu_stamp_ns_<=source_ns && invalid_imu_stamp_ns_>=SampleNs(*imu_latest)) {
    SetReason(reason,"imu_input_invalid"); return false;
  }
  if (!asmc::geometry::WithinAge(SampleNs(*imu_latest),source_ns,config_.imu_max_age_s)) {
    SetReason(reason, "IMU is stale for LiDAR scan");
    return false;
  }

  if (can_latest == can_.end()) {
    SetReason(reason, "no vehicle speed at or before LiDAR scan");
    return false;
  }
  if(invalid_can_stamp_ns_<=source_ns && invalid_can_stamp_ns_>=SampleNs(*can_latest)) {
    SetReason(reason,"speed_input_invalid"); return false;
  }
  if (!asmc::geometry::WithinAge(SampleNs(*can_latest),source_ns,config_.can_max_age_s)) {
    SetReason(reason, "vehicle speed is stale for LiDAR scan");
    return false;
  }

  local->st_IMU = imu_latest->data;
  local->st_CAN = can_latest->data;
  SetReason(reason, "local_ready");
  return true;
}

// Ported from Main SensorPoseHistory.cpp:396-406 (8f93c22e)
bool PoseHistory::WaitForLocalLookupNs(std::uint64_t stamp_ns, double timeout_s, LocalState* local,
                                       LocalInputSelection* selection, std::string* reason) const {
  if (!local || !stamp_ns) {
    SetReason(reason, "invalid local timestamp");
    return false;
  }
  std::unique_lock<std::mutex> lock(mutex_);
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                            std::chrono::duration<double>(std::max(0., timeout_s)));
  while (!LookupLocalLocked(stamp_ns, local, reason, selection)) {
    if (timeout_s <= 0. || sample_cv_.wait_until(lock, deadline) == std::cv_status::timeout)
      return LookupLocalLocked(stamp_ns, local, reason, selection);
  }
  return true;
}

// Ported from Main SensorPoseHistory.cpp:407-413 (8f93c22e)
void PoseHistory::AddGpsInvalidAt(std::uint64_t source_ns, std::uint64_t receipt_ns) {
  std::lock_guard<std::mutex> lock(mutex_);
  asmc::lidar::PoseSample sample;
  sample.stamp_ns = source_ns;
  sample.arrival_ns = receipt_ns;
  sample.clock = "ros";
  sample.arrival_clock = "steady";
  sample.valid = false;
  bounded_gps_.append(sample);  // Main ignores the append result (and does not notify).
}

// Ported from Main SensorPoseHistory.cpp:429-449 (8f93c22e)
asmc::lidar::PoseSelection PoseHistory::MapAt(const asmc::lidar::Observation& t, const LocalState& local) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!std::isfinite(local.st_CAN.f32_CurrentVel_MPS) ||
      local.st_CAN.f32_CurrentVel_MPS > config_.maximum_extrapolation_speed_mps) {
    asmc::lidar::PoseSelection invalid;
    invalid.reason = "map_speed_invalid";
    return invalid;
  }
  LocalState map_local = local;
  if (config_.imu_yaw_is_true_enu) {
    const auto gps=LatestSource(gps_,t.stamp_ns);
    if (gps == gps_.end()) {
      asmc::lidar::PoseSelection invalid;
      invalid.reason = "gps_missing";
      return invalid;
    }
    map_local.st_IMU = ApplyGridOffset(local.st_IMU,gps->grid_yaw_offset);
  }
  return bounded_gps_.at(t, MapRotation(map_local), asmc::lidar::Vec(local.st_CAN.f32_CurrentVel_MPS, 0, 0));
}

// Ported from Main SensorPoseHistory.cpp:468-476 (8f93c22e)
void PoseHistory::PruneGpsLocked() {
  if (gps_.empty()) return;
  const std::uint64_t keep_ms = static_cast<std::uint64_t>(std::max(1.0, config_.history_duration_s) * 1000.0);
  const std::uint64_t newest = gps_.back().stamp_ms;
  while (gps_.size() > 2 && gps_.front().stamp_ms + keep_ms < newest) {
    gps_.pop_front();
  }
}

// Ported from Main SensorPoseHistory.cpp:478-486 (8f93c22e)
void PoseHistory::PruneImuLocked() {
  if (imu_.empty()) return;
  const std::uint64_t keep_ms = static_cast<std::uint64_t>(std::max(1.0, config_.history_duration_s) * 1000.0);
  const std::uint64_t newest = imu_.back().stamp_ms;
  while (imu_.size() > 2 && imu_.front().stamp_ms + keep_ms < newest) {
    imu_.pop_front();
  }
}

// Ported from Main SensorPoseHistory.cpp:488-496 (8f93c22e)
void PoseHistory::PruneCanLocked() {
  if (can_.empty()) return;
  const std::uint64_t keep_ms = static_cast<std::uint64_t>(std::max(1.0, config_.history_duration_s) * 1000.0);
  const std::uint64_t newest = can_.back().stamp_ms;
  while (can_.size() > 2 && can_.front().stamp_ms + keep_ms < newest) {
    can_.pop_front();
  }
}

}  // namespace pipeline
}  // namespace lidar
}  // namespace asmc
