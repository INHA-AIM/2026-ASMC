#pragma once

#include <chrono>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>

// Opt-in observation only. Configure before workers, flush after joining them.
// Disabled tracing does not format records or take the file mutex. Writes are
// buffered, including the partial final interval, rather than flushed per scan.
namespace fusion_trace {
class Recorder {
public:
    void Configure(const std::string& path) {
        if (path.empty()) return;
        output_.open(path, std::ios::out | std::ios::trunc);
        if (!output_) throw std::runtime_error("cannot open fusion_trace_path: " + path);
        enabled_ = true;
    }
    bool Enabled() const { return enabled_; }
    void Append(const std::string& record) {
        std::lock_guard<std::mutex> lock(mutex_);
        buffer_ += record;
        buffer_ += '\n';
        if (buffer_.size() >= 65536) FlushLocked();
    }
    void Flush() {
        std::lock_guard<std::mutex> lock(mutex_);
        FlushLocked();
        output_.flush();
    }
private:
    void FlushLocked() {
        if (buffer_.empty()) return;
        output_ << buffer_;
        if (!output_) throw std::runtime_error("failed writing fusion trace");
        buffer_.clear();
    }
    bool enabled_ = false;
    std::ofstream output_;
    std::string buffer_;
    std::mutex mutex_;
};

inline Recorder& Sink() { static Recorder recorder; return recorder; }

inline std::string Quote(const std::string& value) {
    std::ostringstream out;
    out << '"';
    for (unsigned char c : value) {
        if (c == '"' || c == '\\') out << '\\' << c;
        else if (c < 0x20) out << "\\u" << std::hex << std::setw(4)
                              << std::setfill('0') << static_cast<unsigned>(c) << std::dec;
        else out << c;
    }
    out << '"';
    return out.str();
}

template <typename Fields>
inline void Event(const char* kind, std::uint64_t stamp_ns, Fields fields) {
    if (!Sink().Enabled()) return;
    std::ostringstream out;
    out << std::setprecision(17);
    const auto steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    out << "{\"event\":" << Quote(kind) << ",\"stamp_ns\":" << stamp_ns
        << ",\"steady_ns\":" << steady_ns;
    fields(out);
    out << '}';
    Sink().Append(out.str());
}
}  // namespace fusion_trace
