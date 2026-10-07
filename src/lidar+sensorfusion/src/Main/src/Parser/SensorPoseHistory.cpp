#include <Parser/SensorPoseHistory.hpp>
#include <Geometry/FrameBoundary.hpp>
extern VehicleSpec st_VehicleSpec;

#include <algorithm>
#include <chrono>
#include <cmath>

namespace {

constexpr double kPi = 3.14159265358979323846;

template <typename Sample>
void InsertOrdered(std::deque<Sample>* samples, const Sample& sample) {
    const auto it = std::lower_bound(
        samples->begin(), samples->end(), sample.stamp_ms,
        [](const Sample& lhs, std::uint64_t stamp_ms) {
            return lhs.stamp_ms < stamp_ms;
        });
    if (it != samples->end() && it->stamp_ms == sample.stamp_ms) {
        *it = sample;
    } else {
        samples->insert(it, sample);
    }
}

template <typename Sample>
typename std::deque<Sample>::const_iterator LatestAtOrBefore(
    const std::deque<Sample>& samples, std::uint64_t stamp_ms) {
    const auto upper = std::upper_bound(
        samples.begin(), samples.end(), stamp_ms,
        [](std::uint64_t value, const Sample& rhs) {
            return value < rhs.stamp_ms;
        });
    return upper == samples.begin() ? samples.end() : std::prev(upper);
}

void SetReason(std::string* reason, const char* value) {
    if (reason != nullptr) *reason = value;
}

}  // namespace

SensorPoseHistory::SensorPoseHistory(
    const SensorPoseHistoryConfig& config)
    : bounded_gps_(static_cast<asmc::lidar::Ns>(std::min(.30,std::max(.001,config.gps_max_age_s))*1e9)),
      config_(config) {}

void SensorPoseHistory::Configure(const SensorPoseHistoryConfig& config) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        config_ = config;
        bounded_gps_=asmc::lidar::BoundedPoseHistory(static_cast<asmc::lidar::Ns>(
            std::min(.30,std::max(.001,config_.gps_max_age_s))*1e9));
        gps_.clear();
        imu_.clear();
        can_.clear();
        invalid_imu_stamp_ms_=invalid_can_stamp_ms_=0;
        has_accepted_imu_ = false;
        has_accepted_can_ = false;
        last_seen_imu_stamp_ms_ = 0;
        last_seen_can_stamp_ms_ = 0;
    }
    sample_cv_.notify_all();
}

void SensorPoseHistory::AddGps(
    std::uint64_t stamp_ms, const GPS_DATA& gps) {
    if (stamp_ms == 0 || !std::isfinite(gps.f64_E) ||
        !std::isfinite(gps.f64_N) || !std::isfinite(gps.f64_U)) {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        InsertOrdered(&gps_, TimedGps{stamp_ms, gps});
        if(frame_boundary::Enabled()) {
        asmc::lidar::PoseSample sample;
        sample.stamp_ns=static_cast<asmc::lidar::Ns>(stamp_ms)*1000000LL;
        sample.arrival_ns=static_cast<asmc::lidar::Ns>(ros::Time::now().toNSec());
        sample.clock="ros"; sample.valid=!gps.b_isJamming;
        sample.reference_is_gps=true;
        sample.gps_in_base=asmc::lidar::Vec(st_VehicleSpec.st_GPSPos.f32_X,
            st_VehicleSpec.st_GPSPos.f32_Y,st_VehicleSpec.st_GPSPos.f32_Z);
        sample.map_from_reference.translation()=asmc::lidar::Vec(gps.f64_E,gps.f64_N,gps.f64_U);
        const auto attitude=LatestAtOrBefore(imu_,stamp_ms);
        if(attitude==imu_.end() || double(stamp_ms-attitude->stamp_ms)/1000.>config_.imu_max_age_s ||
            (invalid_imu_stamp_ms_<=stamp_ms && invalid_imu_stamp_ms_>=attitude->stamp_ms)) {
            sample.valid=false;
        } else {
            GPS_IMU at_gps{};at_gps.st_IMU=attitude->data;
            sample.map_from_reference.linear()=frame_boundary::MapRotation(at_gps);
        }
        bounded_gps_.append(sample);
        }
        PruneGpsLocked();
    }
    sample_cv_.notify_all();
}

bool SensorPoseHistory::AddImu(
    std::uint64_t stamp_ms, const IMU_DATA& imu, std::string* reason) {
    if (stamp_ms == 0 || !imu.b_ImuReady ||
        !std::isfinite(imu.f32_Yaw_rad) || !std::isfinite(imu.f32_Roll_rad) ||
        !std::isfinite(imu.f32_Pitch_rad)) {
        if(frame_boundary::Enabled()) {
            std::lock_guard<std::mutex> lock(mutex_);
            invalid_imu_stamp_ms_=std::max(invalid_imu_stamp_ms_,stamp_ms);
        }
        SetReason(reason, "invalid IMU sample");
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    const double yaw = std::remainder(
        static_cast<double>(imu.f32_Yaw_rad), 2.0 * kPi);
    if (last_seen_imu_stamp_ms_ != 0 && stamp_ms < last_seen_imu_stamp_ms_) {
        SetReason(reason, "IMU timestamp moved backwards");
        return false;
    }
    const double seen_gap_s = last_seen_imu_stamp_ms_ == 0 ? 0.0 :
        static_cast<double>(stamp_ms - last_seen_imu_stamp_ms_) / 1000.0;
    last_seen_imu_stamp_ms_ = stamp_ms;
    if (has_accepted_imu_ && seen_gap_s < config_.stream_reanchor_after_s) {
        if (stamp_ms < accepted_imu_stamp_ms_) {
            SetReason(reason, "IMU timestamp moved backwards");
            return false;
        }
        const double elapsed_s = static_cast<double>(
            stamp_ms - accepted_imu_stamp_ms_) / 1000.0;
        const double allowed = config_.imu_base_yaw_allowance_rad +
            config_.imu_maximum_yaw_rate_radps * elapsed_s;
        const double jump = std::abs(std::remainder(
            yaw - accepted_imu_yaw_rad_, 2.0 * kPi));
        if (jump > allowed) {
            if(frame_boundary::Enabled()) invalid_imu_stamp_ms_=std::max(invalid_imu_stamp_ms_,stamp_ms);
            SetReason(reason, "implausible IMU yaw jump");
            return false;
        }
    }
    InsertOrdered(&imu_, TimedImu{stamp_ms, imu});
    has_accepted_imu_ = true;
    accepted_imu_yaw_rad_ = yaw;
    accepted_imu_stamp_ms_ = stamp_ms;
    PruneImuLocked();
    SetReason(reason, "accepted");
    sample_cv_.notify_all();
    return true;
}

bool SensorPoseHistory::AddCan(
    std::uint64_t stamp_ms, const CAN_DATA& can, std::string* reason) {
    const double speed_mps = can.f32_CurrentVel_MPS;
    if (stamp_ms == 0 || !std::isfinite(speed_mps) || speed_mps < 0.0) {
        if(frame_boundary::Enabled()) {
            std::lock_guard<std::mutex> lock(mutex_);
            invalid_can_stamp_ms_=std::max(invalid_can_stamp_ms_,stamp_ms);
        }
        SetReason(reason, "invalid vehicle-speed sample");
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (last_seen_can_stamp_ms_ != 0 && stamp_ms < last_seen_can_stamp_ms_) {
        SetReason(reason, "vehicle-speed timestamp moved backwards");
        return false;
    }
    const double seen_gap_s = last_seen_can_stamp_ms_ == 0 ? 0.0 :
        static_cast<double>(stamp_ms - last_seen_can_stamp_ms_) / 1000.0;
    last_seen_can_stamp_ms_ = stamp_ms;
    if (has_accepted_can_ && seen_gap_s < config_.stream_reanchor_after_s) {
        if (stamp_ms < accepted_can_stamp_ms_) {
            SetReason(reason, "vehicle-speed timestamp moved backwards");
            return false;
        }
        const double elapsed_s = static_cast<double>(
            stamp_ms - accepted_can_stamp_ms_) / 1000.0;
        const double allowed = config_.can_base_speed_allowance_mps +
            config_.can_maximum_acceleration_mps2 * elapsed_s;
        if (std::abs(speed_mps - accepted_can_speed_mps_) > allowed) {
            if(frame_boundary::Enabled()) invalid_can_stamp_ms_=std::max(invalid_can_stamp_ms_,stamp_ms);
            SetReason(reason, "implausible vehicle-speed jump");
            return false;
        }
    }
    InsertOrdered(&can_, TimedCan{stamp_ms, can});
    has_accepted_can_ = true;
    accepted_can_speed_mps_ = speed_mps;
    accepted_can_stamp_ms_ = stamp_ms;
    PruneCanLocked();
    SetReason(reason, "accepted");
    sample_cv_.notify_all();
    return true;
}

bool SensorPoseHistory::Lookup(
    std::uint64_t lidar_stamp_ms, GPS_IMU* pose,
    std::string* reason) const {
    if (pose == nullptr || lidar_stamp_ms == 0) {
        SetReason(reason, "invalid LiDAR timestamp or output pose");
        return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    return LookupLocked(lidar_stamp_ms, pose, reason);
}

bool SensorPoseHistory::WaitForLookup(
    std::uint64_t lidar_stamp_ms, double timeout_s, GPS_IMU* pose,
    std::string* reason) const {
    if (pose == nullptr || lidar_stamp_ms == 0) {
        SetReason(reason, "invalid LiDAR timestamp or output pose");
        return false;
    }

    std::unique_lock<std::mutex> lock(mutex_);
    if (LookupLocked(lidar_stamp_ms, pose, reason)) return true;

    const auto timeout = std::chrono::duration<double>(
        std::max(0.0, timeout_s));
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            timeout);
    while (timeout_s > 0.0) {
        if (sample_cv_.wait_until(lock, deadline) ==
            std::cv_status::timeout) {
            break;
        }
        if (LookupLocked(lidar_stamp_ms, pose, reason)) return true;
    }
    // Refresh the final reason after timeout in case the last arriving stream
    // changed which input is missing/stale.
    return LookupLocked(lidar_stamp_ms, pose, reason);
}

bool SensorPoseHistory::LookupLocked(
    std::uint64_t lidar_stamp_ms, GPS_IMU* pose,
    std::string* reason) const {
    if (!LookupLocalLocked(lidar_stamp_ms,pose,reason)) return false;
    const auto imu_latest = LatestAtOrBefore(imu_,lidar_stamp_ms);
    const auto can_latest = LatestAtOrBefore(can_,lidar_stamp_ms);
    const auto gps_latest = LatestAtOrBefore(gps_, lidar_stamp_ms);
    if (gps_latest == gps_.end()) {
        SetReason(reason, "no accepted GPS at or before LiDAR scan");
        return false;
    }
    const double gps_age_s =
        static_cast<double>(lidar_stamp_ms - gps_latest->stamp_ms) / 1000.0;
    if (gps_age_s > config_.gps_max_age_s) {
        SetReason(reason, "GPS is stale for LiDAR scan");
        return false;
    }

    GPS_DATA resolved_gps = gps_latest->data;
    if (gps_latest != gps_.begin()) {
        const auto gps_previous = std::prev(gps_latest);
        const double sample_dt_s = static_cast<double>(
            gps_latest->stamp_ms - gps_previous->stamp_ms) / 1000.0;
        if (sample_dt_s > 1e-3 && sample_dt_s <= 1.0) {
            const double velocity_e =
                (gps_latest->data.f64_E - gps_previous->data.f64_E) /
                sample_dt_s;
            const double velocity_n =
                (gps_latest->data.f64_N - gps_previous->data.f64_N) /
                sample_dt_s;
            const double speed_mps = std::hypot(velocity_e, velocity_n);
            if (!std::isfinite(speed_mps) ||
                speed_mps > config_.maximum_extrapolation_speed_mps) {
                SetReason(reason, "GPS history implies impossible speed");
                return false;
            }
            resolved_gps.f64_U +=
                (gps_latest->data.f64_U - gps_previous->data.f64_U) /
                sample_dt_s * gps_age_s;
        }
    }
    // Position is advanced from the latest GPS using the independently
    // sanity-checked vehicle speed and heading. This avoids 1--2 m map jumps
    // when a 5 Hz GPS callback arrives just before/after LiDAR processing.
    const double speed_mps = can_latest->data.f32_CurrentVel_MPS;
    const double yaw_rad = imu_latest->data.f32_Yaw_rad;
    resolved_gps.f64_E += speed_mps * std::cos(yaw_rad) * gps_age_s;
    resolved_gps.f64_N += speed_mps * std::sin(yaw_rad) * gps_age_s;

    pose->st_GPS = resolved_gps;
    pose->st_IMU = imu_latest->data;
    pose->st_CAN = can_latest->data;
    pose->b_Jamming = resolved_gps.b_isJamming;
    SetReason(reason, "ready");
    return true;
}

bool SensorPoseHistory::LookupLocalLocked(std::uint64_t lidar_stamp_ms,
    GPS_IMU* pose,std::string* reason) const {
    const auto imu_latest = LatestAtOrBefore(imu_, lidar_stamp_ms);
    if (imu_latest == imu_.end()) {
        SetReason(reason, "no IMU at or before LiDAR scan");
        return false;
    }
    const double imu_age_s =
        static_cast<double>(lidar_stamp_ms - imu_latest->stamp_ms) / 1000.0;
    if (imu_age_s > config_.imu_max_age_s) {
        SetReason(reason, "IMU is stale for LiDAR scan");
        return false;
    }

    const auto can_latest = LatestAtOrBefore(can_, lidar_stamp_ms);
    if (can_latest == can_.end()) {
        SetReason(reason, "no vehicle speed at or before LiDAR scan");
        return false;
    }
    const double can_age_s =
        static_cast<double>(lidar_stamp_ms - can_latest->stamp_ms) / 1000.0;
    if (can_age_s > config_.can_max_age_s) {
        SetReason(reason, "vehicle speed is stale for LiDAR scan");
        return false;
    }

    if(frame_boundary::Enabled() && invalid_imu_stamp_ms_<=lidar_stamp_ms &&
        invalid_imu_stamp_ms_>=imu_latest->stamp_ms) {SetReason(reason,"imu_input_invalid");return false;}
    if(frame_boundary::Enabled() && invalid_can_stamp_ms_<=lidar_stamp_ms &&
        invalid_can_stamp_ms_>=can_latest->stamp_ms) {SetReason(reason,"speed_input_invalid");return false;}
    pose->st_IMU=imu_latest->data;
    pose->st_CAN=can_latest->data;
    SetReason(reason,"local_ready");
    return true;
}
bool SensorPoseHistory::WaitForLocalLookup(std::uint64_t stamp_ms,double timeout_s,
    GPS_IMU* pose,std::string* reason) const {
    if(!pose || !stamp_ms) {SetReason(reason,"invalid local timestamp"); return false;}
    std::unique_lock<std::mutex> lock(mutex_);
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(std::max(0.,timeout_s)));
    while(!LookupLocalLocked(stamp_ms,pose,reason)) {
        if(timeout_s<=0. || sample_cv_.wait_until(lock,deadline)==std::cv_status::timeout)
            return LookupLocalLocked(stamp_ms,pose,reason);
    }
    return true;
}
void SensorPoseHistory::AddGpsInvalid(std::uint64_t stamp_ms,std::uint64_t arrival_ms) {
    std::lock_guard<std::mutex> lock(mutex_);
    asmc::lidar::PoseSample sample;
    sample.stamp_ns=static_cast<asmc::lidar::Ns>(stamp_ms)*1000000LL;
    sample.arrival_ns=static_cast<asmc::lidar::Ns>(arrival_ms)*1000000LL;
    sample.clock="ros"; sample.valid=false;
    bounded_gps_.append(sample);
}
asmc::lidar::PoseSelection SensorPoseHistory::MapAt(std::uint64_t stamp_ms,
    const GPS_IMU& local) const {
    std::lock_guard<std::mutex> lock(mutex_);
    asmc::lidar::Observation t{static_cast<asmc::lidar::Ns>(stamp_ms)*1000000LL,
        static_cast<asmc::lidar::Ns>(stamp_ms)*1000000LL,
        static_cast<asmc::lidar::Ns>(ros::Time::now().toNSec()),"ros","bridge_receive_ms"};
    if(!std::isfinite(local.st_CAN.f32_CurrentVel_MPS) ||
        local.st_CAN.f32_CurrentVel_MPS>config_.maximum_extrapolation_speed_mps) {
        asmc::lidar::PoseSelection invalid;invalid.reason="map_speed_invalid";return invalid;
    }
    return bounded_gps_.at(t,frame_boundary::MapRotation(local),
        asmc::lidar::Vec(local.st_CAN.f32_CurrentVel_MPS,0,0));
}

void SensorPoseHistory::Reset() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        bounded_gps_.clear();
        gps_.clear();
        imu_.clear();
        can_.clear();
        invalid_imu_stamp_ms_=invalid_can_stamp_ms_=0;
        has_accepted_imu_ = false;
        has_accepted_can_ = false;
        last_seen_imu_stamp_ms_ = 0;
        last_seen_can_stamp_ms_ = 0;
    }
    sample_cv_.notify_all();
}

void SensorPoseHistory::PruneGpsLocked() {
    if (gps_.empty()) return;
    const std::uint64_t keep_ms = static_cast<std::uint64_t>(
        std::max(1.0, config_.history_duration_s) * 1000.0);
    const std::uint64_t newest = gps_.back().stamp_ms;
    while (gps_.size() > 2 && gps_.front().stamp_ms + keep_ms < newest) {
        gps_.pop_front();
    }
}

void SensorPoseHistory::PruneImuLocked() {
    if (imu_.empty()) return;
    const std::uint64_t keep_ms = static_cast<std::uint64_t>(
        std::max(1.0, config_.history_duration_s) * 1000.0);
    const std::uint64_t newest = imu_.back().stamp_ms;
    while (imu_.size() > 2 && imu_.front().stamp_ms + keep_ms < newest) {
        imu_.pop_front();
    }
}

void SensorPoseHistory::PruneCanLocked() {
    if (can_.empty()) return;
    const std::uint64_t keep_ms = static_cast<std::uint64_t>(
        std::max(1.0, config_.history_duration_s) * 1000.0);
    const std::uint64_t newest = can_.back().stamp_ms;
    while (can_.size() > 2 && can_.front().stamp_ms + keep_ms < newest) {
        can_.pop_front();
    }
}
