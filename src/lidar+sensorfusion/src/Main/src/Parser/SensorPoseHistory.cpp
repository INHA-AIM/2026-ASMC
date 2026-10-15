#include <Parser/SensorPoseHistory.hpp>
#include <Geometry/FrameBoundary.hpp>
extern VehicleSpec st_VehicleSpec;

#include <algorithm>
#include <chrono>
#include <cmath>

namespace {

constexpr double kPi = 3.14159265358979323846;

// Convert only map-facing copies. Local LiDAR/camera geometry and raw IMU
// history retain their original basis, including during GPS jamming.
IMU_DATA GridImu(const IMU_DATA& imu, const GPS_DATA& gps, bool true_enu) {
    if (!true_enu) return imu;
    constexpr double step = 1e-5;
    double e0, n0, e1, n1;
    int zone = 52;
    bool north = true;
    GeographicLib::UTMUPS::Forward(gps.f64_Latitude - step,
        gps.f64_Longitude, zone, north, e0, n0, 52);
    GeographicLib::UTMUPS::Forward(gps.f64_Latitude + step,
        gps.f64_Longitude, zone, north, e1, n1, 52);
    const double offset = std::atan2(n1 - n0, e1 - e0) - kPi / 2.;
    IMU_DATA out = imu;
    out.f32_Yaw_rad = std::remainder(double(imu.f32_Yaw_rad) + offset, 2. * kPi);
    out.f32_Yaw_deg = out.f32_Yaw_rad * 180. / kPi;
    return out;
}

template <typename Sample>
void InsertOrdered(std::deque<Sample>* samples, const Sample& sample) {
    const auto it = std::lower_bound(
        samples->begin(), samples->end(), sample.source_ns?sample.source_ns:sample.stamp_ms*1000000ULL,
        [](const Sample& lhs, std::uint64_t ns) {
            return (lhs.source_ns?lhs.source_ns:lhs.stamp_ms*1000000ULL) < ns;
        });
    if (it != samples->end() && (it->source_ns?it->source_ns:it->stamp_ms*1000000ULL)==
        (sample.source_ns?sample.source_ns:sample.stamp_ms*1000000ULL)) {
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

// Invalid IMU / vehicle-speed evidence (common geometry only). Every invalid
// source stamp is kept in ascending order. A lookup is invalid when ANY invalid
// stamp lies in [selected valid sample, query]; samples stamped after the query
// never change the answer, whenever they arrive (header contract: online and
// bag replay independent of callback timing).
constexpr std::size_t kMaxInvalidStamps = 1024;

void RecordInvalid(std::deque<std::uint64_t>* stamps, std::uint64_t ns) {
    const auto it = std::lower_bound(stamps->begin(), stamps->end(), ns);
    if (it == stamps->end() || *it != ns) stamps->insert(it, ns);
    while (stamps->size() > kMaxInvalidStamps) stamps->pop_front();
}

bool InvalidWithin(const std::deque<std::uint64_t>& stamps,
                   std::uint64_t from_ns, std::uint64_t to_ns) {
    const auto it = std::lower_bound(stamps.begin(), stamps.end(), from_ns);
    return it != stamps.end() && *it <= to_ns;
}

// Same horizon as PruneImuLocked/PruneCanLocked (history_duration_s behind the
// newest accepted valid sample).
void PruneInvalid(std::deque<std::uint64_t>* stamps, std::uint64_t newest_valid_ns,
                  double history_duration_s) {
    const auto keep_ns = static_cast<std::uint64_t>(
        std::max(1.0, history_duration_s) * 1e9);
    while (!stamps->empty() && newest_valid_ns > keep_ns &&
           stamps->front() < newest_valid_ns - keep_ns) {
        stamps->pop_front();
    }
}

template <typename Sample>
std::uint64_t NewestNs(const std::deque<Sample>& samples) {
    if (samples.empty()) return 0;
    const auto& s = samples.back();
    return s.source_ns ? s.source_ns : s.stamp_ms * 1000000ULL;
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
        invalid_imu_ns_.clear();
        invalid_can_ns_.clear();
        has_accepted_imu_ = false;
        has_accepted_can_ = false;
        last_seen_imu_stamp_ms_ = 0;
        last_seen_can_stamp_ms_ = 0;
    }
    sample_cv_.notify_all();
}

void SensorPoseHistory::AddGps(
    std::uint64_t stamp_ms, const GPS_DATA& gps,
    std::uint64_t source_ns, std::uint64_t receipt_steady_ns) {
    if (stamp_ms == 0 || !std::isfinite(gps.f64_E) ||
        !std::isfinite(gps.f64_N) || !std::isfinite(gps.f64_U)) {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (config_.imu_yaw_is_true_enu &&
            (!std::isfinite(gps.f64_Latitude) || !std::isfinite(gps.f64_Longitude) ||
             gps.f64_Latitude < 0. || gps.f64_Latitude > 84. ||
             gps.f64_Longitude < 126. || gps.f64_Longitude > 132. || gps.b_isJamming)) {
            return;  // This deployment's map contract is UTM zone 52N.
        }
        InsertOrdered(&gps_, TimedGps{stamp_ms, gps, source_ns});
        if(frame_boundary::Enabled()) {
        asmc::lidar::PoseSample sample;
        sample.stamp_ns=source_ns?source_ns:static_cast<asmc::lidar::Ns>(stamp_ms)*1000000LL;
        sample.arrival_ns=receipt_steady_ns?receipt_steady_ns:static_cast<asmc::lidar::Ns>(ros::Time::now().toNSec());
        if(receipt_steady_ns) sample.arrival_clock="steady";
        sample.clock="ros"; sample.valid=!gps.b_isJamming;
        sample.reference_is_gps=true;
        sample.gps_in_base=asmc::lidar::Vec(st_VehicleSpec.st_GPSPos.f32_X,
            st_VehicleSpec.st_GPSPos.f32_Y,st_VehicleSpec.st_GPSPos.f32_Z);
        sample.map_from_reference.translation()=asmc::lidar::Vec(gps.f64_E,gps.f64_N,gps.f64_U);
        auto attitude=LatestAtOrBefore(imu_,stamp_ms);
        if(source_ns) {
            attitude=imu_.end();
            for(auto it=imu_.begin();it!=imu_.end();++it)
                if((it->source_ns?it->source_ns:it->stamp_ms*1000000ULL)<=source_ns) attitude=it;
        }
        const auto attitude_ns=attitude==imu_.end()?0:(attitude->source_ns?attitude->source_ns:attitude->stamp_ms*1000000ULL);
        if(!attitude_ns || double(sample.stamp_ns-attitude_ns)*1e-9>config_.imu_max_age_s ||
            InvalidWithin(invalid_imu_ns_,attitude_ns,static_cast<std::uint64_t>(sample.stamp_ns))) {
            sample.valid=false;
        } else {
            GPS_IMU at_gps{};
            at_gps.st_IMU=GridImu(attitude->data,gps,config_.imu_yaw_is_true_enu);
            sample.map_from_reference.linear()=frame_boundary::MapRotation(at_gps);
        }
        bounded_gps_.append(sample);
        }
        PruneGpsLocked();
    }
    sample_cv_.notify_all();
}

bool SensorPoseHistory::AddImu(
    std::uint64_t stamp_ms, const IMU_DATA& imu, std::string* reason,
    std::uint64_t source_ns, std::uint64_t receipt_steady_ns) {
    if (stamp_ms == 0 || !imu.b_ImuReady ||
        !std::isfinite(imu.f32_Yaw_rad) || !std::isfinite(imu.f32_Roll_rad) ||
        !std::isfinite(imu.f32_Pitch_rad)) {
        if(frame_boundary::Enabled()) {
            std::lock_guard<std::mutex> lock(mutex_);
            invalid_imu_stamp_ms_=std::max(invalid_imu_stamp_ms_,stamp_ms);
            RecordInvalid(&invalid_imu_ns_,source_ns?source_ns:stamp_ms*1000000ULL);
            // Zero header: no source time (stamp_ms is the callback's receipt time). The
            // sample still arrived after every valid sample already held, so the newest
            // one must not be reused (receipt time vs scan source stamp alone raced).
            if(!source_ns && !imu_.empty()) RecordInvalid(&invalid_imu_ns_,NewestNs(imu_));
            PruneInvalid(&invalid_imu_ns_,NewestNs(imu_),config_.history_duration_s);
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
            if(frame_boundary::Enabled()) {
                invalid_imu_stamp_ms_=std::max(invalid_imu_stamp_ms_,stamp_ms);
                RecordInvalid(&invalid_imu_ns_,source_ns?source_ns:stamp_ms*1000000ULL);
                PruneInvalid(&invalid_imu_ns_,NewestNs(imu_),config_.history_duration_s);
            }
            SetReason(reason, "implausible IMU yaw jump");
            return false;
        }
    }
    InsertOrdered(&imu_, TimedImu{stamp_ms, imu, source_ns, receipt_steady_ns});
    has_accepted_imu_ = true;
    accepted_imu_yaw_rad_ = yaw;
    accepted_imu_stamp_ms_ = stamp_ms;
    PruneImuLocked();
    PruneInvalid(&invalid_imu_ns_,NewestNs(imu_),config_.history_duration_s);
    SetReason(reason, "accepted");
    sample_cv_.notify_all();
    return true;
}

bool SensorPoseHistory::AddCan(
    std::uint64_t stamp_ms, const CAN_DATA& can, std::string* reason,
    std::uint64_t source_ns, std::uint64_t receipt_steady_ns) {
    const double speed_mps = can.f32_CurrentVel_MPS;
    if (stamp_ms == 0 || !std::isfinite(speed_mps) || speed_mps < 0.0) {
        if(frame_boundary::Enabled()) {
            std::lock_guard<std::mutex> lock(mutex_);
            invalid_can_stamp_ms_=std::max(invalid_can_stamp_ms_,stamp_ms);
            RecordInvalid(&invalid_can_ns_,source_ns?source_ns:stamp_ms*1000000ULL);
            // Zero header (CANParser_Callback: NaN speed, receipt time): see AddImu.
            if(!source_ns && !can_.empty()) RecordInvalid(&invalid_can_ns_,NewestNs(can_));
            PruneInvalid(&invalid_can_ns_,NewestNs(can_),config_.history_duration_s);
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
            if(frame_boundary::Enabled()) {
                invalid_can_stamp_ms_=std::max(invalid_can_stamp_ms_,stamp_ms);
                RecordInvalid(&invalid_can_ns_,source_ns?source_ns:stamp_ms*1000000ULL);
                PruneInvalid(&invalid_can_ns_,NewestNs(can_),config_.history_duration_s);
            }
            SetReason(reason, "implausible vehicle-speed jump");
            return false;
        }
    }
    InsertOrdered(&can_, TimedCan{stamp_ms, can, source_ns, receipt_steady_ns});
    has_accepted_can_ = true;
    accepted_can_speed_mps_ = speed_mps;
    accepted_can_stamp_ms_ = stamp_ms;
    PruneCanLocked();
    PruneInvalid(&invalid_can_ns_,NewestNs(can_),config_.history_duration_s);
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
    const auto grid_imu = GridImu(imu_latest->data, resolved_gps, config_.imu_yaw_is_true_enu);
    const double yaw_rad = grid_imu.f32_Yaw_rad;
    resolved_gps.f64_E += speed_mps * std::cos(yaw_rad) * gps_age_s;
    resolved_gps.f64_N += speed_mps * std::sin(yaw_rad) * gps_age_s;

    pose->st_GPS = resolved_gps;
    pose->st_IMU = grid_imu;
    pose->st_CAN = can_latest->data;
    pose->b_Jamming = resolved_gps.b_isJamming;
    SetReason(reason, "ready");
    return true;
}

bool SensorPoseHistory::LookupLocalLocked(std::uint64_t lidar_stamp_ms,
    GPS_IMU* pose,std::string* reason,std::uint64_t source_ns,
    LocalInputSelection* selection) const {
    auto imu_latest = LatestAtOrBefore(imu_, lidar_stamp_ms);
    if(source_ns) {
        imu_latest=imu_.end();
        for(auto it=imu_.begin();it!=imu_.end();++it)
            if((it->source_ns?it->source_ns:it->stamp_ms*1000000ULL)<=source_ns) imu_latest=it;
    }
    if (imu_latest == imu_.end()) {
        SetReason(reason, "no IMU at or before LiDAR scan");
        return false;
    }
    const double imu_age_s =
        (source_ns?double(source_ns-(imu_latest->source_ns?imu_latest->source_ns:imu_latest->stamp_ms*1000000ULL))*1e-9:
        static_cast<double>(lidar_stamp_ms - imu_latest->stamp_ms) / 1000.0);
    if (imu_age_s > config_.imu_max_age_s) {
        SetReason(reason, "IMU is stale for LiDAR scan");
        return false;
    }

    auto can_latest = LatestAtOrBefore(can_, lidar_stamp_ms);
    if(source_ns) {
        can_latest=can_.end();
        for(auto it=can_.begin();it!=can_.end();++it)
            if((it->source_ns?it->source_ns:it->stamp_ms*1000000ULL)<=source_ns) can_latest=it;
    }
    if (can_latest == can_.end()) {
        SetReason(reason, "no vehicle speed at or before LiDAR scan");
        return false;
    }
    const double can_age_s =
        (source_ns?double(source_ns-(can_latest->source_ns?can_latest->source_ns:can_latest->stamp_ms*1000000ULL))*1e-9:
        static_cast<double>(lidar_stamp_ms - can_latest->stamp_ms) / 1000.0);
    if (can_age_s > config_.can_max_age_s) {
        SetReason(reason, "vehicle speed is stale for LiDAR scan");
        return false;
    }

    const auto query_ns=source_ns?source_ns:lidar_stamp_ms*1000000ULL;
    const auto imu_ns=imu_latest->source_ns?imu_latest->source_ns:imu_latest->stamp_ms*1000000ULL;
    const auto can_ns=can_latest->source_ns?can_latest->source_ns:can_latest->stamp_ms*1000000ULL;
    if(frame_boundary::Enabled() && InvalidWithin(invalid_imu_ns_,imu_ns,query_ns))
        {SetReason(reason,"imu_input_invalid");return false;}
    if(frame_boundary::Enabled() && InvalidWithin(invalid_can_ns_,can_ns,query_ns))
        {SetReason(reason,"speed_input_invalid");return false;}
    if(selection) *selection={imu_ns,imu_latest->receipt_steady_ns,can_ns,can_latest->receipt_steady_ns};
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
bool SensorPoseHistory::WaitForLocalLookupNs(std::uint64_t source_ns,double timeout_s,
    GPS_IMU* pose,LocalInputSelection* selection,std::string* reason) const {
    if(!pose || !source_ns) {SetReason(reason,"invalid local timestamp");return false;}
    std::unique_lock<std::mutex> lock(mutex_);
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(std::max(0.,timeout_s)));
    while(!LookupLocalLocked(source_ns/1000000ULL,pose,reason,source_ns,selection)) {
        if(timeout_s<=0. || sample_cv_.wait_until(lock,deadline)==std::cv_status::timeout)
            return LookupLocalLocked(source_ns/1000000ULL,pose,reason,source_ns,selection);
    }
    return true;
}
void SensorPoseHistory::AddGpsInvalidAt(std::uint64_t source_ns,std::uint64_t receipt_steady_ns) {
    std::lock_guard<std::mutex> lock(mutex_);
    asmc::lidar::PoseSample sample;
    sample.stamp_ns=source_ns;sample.arrival_ns=receipt_steady_ns;
    sample.clock="ros";sample.arrival_clock="steady";sample.valid=false;
    bounded_gps_.append(sample);
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
    asmc::lidar::Observation t{static_cast<asmc::lidar::Ns>(stamp_ms)*1000000LL,
        static_cast<asmc::lidar::Ns>(stamp_ms)*1000000LL,
        static_cast<asmc::lidar::Ns>(ros::Time::now().toNSec()),"ros","bridge_receive_ms"};
    return MapAt(t,local);
}
asmc::lidar::PoseSelection SensorPoseHistory::MapAt(const asmc::lidar::Observation& t,
    const GPS_IMU& local) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!std::isfinite(local.st_CAN.f32_CurrentVel_MPS) ||
        local.st_CAN.f32_CurrentVel_MPS>config_.maximum_extrapolation_speed_mps) {
        asmc::lidar::PoseSelection invalid;invalid.reason="map_speed_invalid";return invalid;
    }
    GPS_IMU map_local = local;
    if (config_.imu_yaw_is_true_enu) {
        auto gps = gps_.end();
        for (auto it = gps_.begin(); it != gps_.end(); ++it)
            if (static_cast<asmc::lidar::Ns>(it->source_ns ? it->source_ns : it->stamp_ms*1000000ULL) <= t.stamp_ns)
                gps = it;
        if (gps == gps_.end()) {
            asmc::lidar::PoseSelection invalid; invalid.reason="gps_missing"; return invalid;
        }
        map_local.st_IMU = GridImu(local.st_IMU, gps->data, true);
    }
    return bounded_gps_.at(t,frame_boundary::MapRotation(map_local),
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
        invalid_imu_ns_.clear();
        invalid_can_ns_.clear();
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
