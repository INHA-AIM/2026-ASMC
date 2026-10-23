#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>

// Opt-in tracking trace: one JSON object per line, default OFF.
// Disabled tracing formats nothing (every hook checks Enabled() before building a record).
// Memory and disk are bounded: records are buffered (flushed every 256 KiB, never per scan)
// and every record beyond max_bytes is dropped and counted instead of growing the file.
// Track selection (ids) limits per-track records; scan-level records are always written.
namespace track_trace {
class Recorder {
public:
    void Configure(const std::string& path, const std::string& run_id,
                   const std::string& ids_csv, double max_mb) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (path.empty()) return;
        output_.open(path, std::ios::out | std::ios::trunc);
        if (!output_) throw std::runtime_error("cannot open lidar track trace: " + path);
        run_id_ = run_id;
        max_bytes_ = max_mb > 0. ? static_cast<std::uint64_t>(max_mb * 1e6) : 0;
        std::stringstream in(ids_csv); std::string cell;
        while (std::getline(in, cell, ',')) if (!cell.empty()) ids_.insert(std::stoi(cell));
        enabled_ = true;
    }
    bool Enabled() const { return enabled_; }
    bool WantsTrack(int id) const { return enabled_ && (ids_.empty() || ids_.count(id) > 0); }
    void SetScan(std::int64_t scan, std::uint64_t stamp_ns) { scan_ = scan; stamp_ns_ = stamp_ns; }
    std::int64_t Scan() const { return scan_; }
    std::uint64_t StampNs() const { return stamp_ns_; }
    const std::string& RunId() const { return run_id_; }
    void Append(const std::string& record) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (max_bytes_ && written_ + buffer_.size() + record.size() + 1 > max_bytes_) { ++dropped_; return; }
        buffer_ += record; buffer_ += '\n'; ++records_;
        if (buffer_.size() >= 262144) FlushLocked();
    }
    void Flush() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!enabled_) return;
        FlushLocked();
        output_.flush();
    }
    std::uint64_t Dropped() const { return dropped_; }
    std::uint64_t Records() const { return records_; }
private:
    void FlushLocked() {
        if (buffer_.empty()) return;
        output_ << buffer_;
        if (!output_) throw std::runtime_error("failed writing lidar track trace");
        written_ += buffer_.size();
        buffer_.clear();
    }
    bool enabled_ = false;
    std::ofstream output_;
    std::string buffer_, run_id_;
    std::set<int> ids_;
    std::uint64_t max_bytes_ = 0, written_ = 0, dropped_ = 0, records_ = 0;
    std::int64_t scan_ = -1;
    std::uint64_t stamp_ns_ = 0;
    std::mutex mutex_;
};

inline Recorder& Sink() { static Recorder recorder; return recorder; }

inline std::uint64_t MonoNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

// JSON number, null for NaN/Inf.
struct N { double v; };
inline std::ostream& operator<<(std::ostream& out, N n) {
    if (std::isfinite(n.v)) out << n.v; else out << "null";
    return out;
}
template <class Vec> inline void Arr(std::ostream& out, const Vec& v, int n) {
    out << '[';
    for (int i = 0; i < n; ++i) { if (i) out << ','; out << N{static_cast<double>(v(i))}; }
    out << ']';
}
template <class Mat> inline void Mat2(std::ostream& out, const Mat& m, int rows, int cols) {
    out << '[';
    for (int r = 0; r < rows; ++r) for (int c = 0; c < cols; ++c) {
        if (r || c) out << ',';
        out << N{static_cast<double>(m(r, c))};
    }
    out << ']';
}

// Every record: ev, run, scan (harness frame index or runtime generation), t_ns (LiDAR source stamp,
// ROS clock of the bridge), mono_ns (CLOCK_MONOTONIC of this process; never subtracted from t_ns).
template <typename Fields>
inline void Event(const char* kind, Fields fields) {
    auto& sink = Sink();
    if (!sink.Enabled()) return;
    std::ostringstream out;
    out << std::setprecision(9);
    out << "{\"ev\":\"" << kind << "\",\"run\":\"" << sink.RunId() << "\",\"scan\":" << sink.Scan()
        << ",\"t_ns\":" << sink.StampNs() << ",\"mono_ns\":" << MonoNs();
    fields(out);
    out << '}';
    sink.Append(out.str());
}
}  // namespace track_trace

// Experiment switches for the tracking state (all default false = P0 behaviour).
//   reacquire_elapsed_init: second-observation velocity uses the time elapsed since the first
//     observation (sum of scan dt over missed scans) instead of the last scan dt only.
//   ego_translation_comp: the common-geometry frame alignment also removes the base_link
//     translation (arc from vel_x and IMU yaw change), so the KF velocity is ground-referenced
//     and the published absolute velocity no longer adds the CAN speed.
namespace tracking_options {
struct Options {
    bool reacquire_elapsed_init = false;
    bool ego_translation_comp = false;
    bool observation_quality_r = false;
    bool observation_obb_center = false;
    bool jerk_q = false;
    double jerk_spectral_density = 1.0; // m^2/s^5, continuous white jerk (per axis).
    bool ego_gap_reset = false; // explicit reset instead of silently skipping unsupported B translation.
    bool two_point_init_cov = false;
    bool point_support_ownership = false; // Sparse initial-fragment containment with competing owner.
    bool association_innovation_gate = false; // Experimental 2D 99% prior gate; no noise/model change.
    bool obb_long_axis_state = false; // Candidate: canonicalize representation, no shape history.
    bool yaw_ema_enabled = true; // Preserve baseline; OFF is a diagnostic only.
};
inline Options& Get() { static Options options; return options; }
}  // namespace tracking_options
