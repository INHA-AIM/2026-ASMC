#ifndef SENSOR_POSE_HISTORY_HPP
#define SENSOR_POSE_HISTORY_HPP

#include <Global/Global.hpp>
#include <asmc_lidar/geometry/bounded_pose.hpp>

#include <cstdint>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <string>

struct SensorPoseHistoryConfig {
    // MORAI reports true ENU yaw; map positions use UTM grid axes.
    // The standalone struct defaults false; main.launch explicitly enables true.
    // Match planning_inputs and do not rotate an already-grid-yaw producer twice.
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
};
struct LocalInputSelection {
    std::uint64_t imu_stamp_ns=0, imu_receipt_steady_ns=0;
    std::uint64_t can_stamp_ns=0, can_receipt_steady_ns=0;
};

// Resolves the vehicle pose at a LiDAR source timestamp. GPS is only 5 Hz in
// MORAI, so the two latest accepted samples at or before the scan are used for
// bounded constant-velocity extrapolation. IMU is about 40 Hz and the latest
// sample at or before the scan is used. Future samples are deliberately not
// used, making online and bag-replay behavior independent of callback timing.
class SensorPoseHistory {
public:
    explicit SensorPoseHistory(
        const SensorPoseHistoryConfig& config = {});

    void Configure(const SensorPoseHistoryConfig& config);
    void AddGps(std::uint64_t stamp_ms, const GPS_DATA& gps,
                std::uint64_t source_ns=0, std::uint64_t receipt_steady_ns=0);
    bool AddImu(std::uint64_t stamp_ms, const IMU_DATA& imu,
                std::string* reason = nullptr, std::uint64_t source_ns=0,
                std::uint64_t receipt_steady_ns=0);
    bool AddCan(std::uint64_t stamp_ms, const CAN_DATA& can,
                std::string* reason = nullptr, std::uint64_t source_ns=0,
                std::uint64_t receipt_steady_ns=0);

    bool Lookup(std::uint64_t lidar_stamp_ms, GPS_IMU* pose,
                std::string* reason = nullptr) const;
    // Waits only for callbacks that can satisfy this source timestamp.  It
    // never rewrites the stamp or substitutes a stale pose.
    bool WaitForLookup(std::uint64_t lidar_stamp_ms, double timeout_s,
                       GPS_IMU* pose, std::string* reason = nullptr) const;
    bool WaitForLocalLookup(std::uint64_t stamp_ms, double timeout_s,
                            GPS_IMU* local, std::string* reason = nullptr) const;
    bool WaitForLocalLookupNs(std::uint64_t stamp_ns, double timeout_s,
                            GPS_IMU* local, LocalInputSelection* selection,
                            std::string* reason = nullptr) const;
    void AddGpsInvalid(std::uint64_t stamp_ms, std::uint64_t arrival_ms);
    void AddGpsInvalidAt(std::uint64_t source_ns, std::uint64_t receipt_steady_ns);
    asmc::lidar::PoseSelection MapAt(std::uint64_t stamp_ms,
                                    const GPS_IMU& local) const;
    asmc::lidar::PoseSelection MapAt(const asmc::lidar::Observation& observation,
                                    const GPS_IMU& local) const;
    void Reset();

private:
    struct TimedGps {
        std::uint64_t stamp_ms = 0;
        GPS_DATA data{};
        std::uint64_t source_ns=0;
    };
    struct TimedImu {
        std::uint64_t stamp_ms = 0;
        IMU_DATA data{};
        std::uint64_t source_ns=0, receipt_steady_ns=0;
    };
    struct TimedCan {
        std::uint64_t stamp_ms = 0;
        CAN_DATA data{};
        std::uint64_t source_ns=0, receipt_steady_ns=0;
    };

    void PruneGpsLocked();
    void PruneImuLocked();
    void PruneCanLocked();
    bool LookupLocked(std::uint64_t lidar_stamp_ms, GPS_IMU* pose,
                      std::string* reason) const;

    bool LookupLocalLocked(std::uint64_t stamp_ms, GPS_IMU* local,
                           std::string* reason, std::uint64_t source_ns=0,
                           LocalInputSelection* selection=nullptr) const;
    asmc::lidar::BoundedPoseHistory bounded_gps_;
    mutable std::mutex mutex_;
    mutable std::condition_variable sample_cv_;
    SensorPoseHistoryConfig config_;
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

#endif
