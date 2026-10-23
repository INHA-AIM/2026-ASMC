#pragma once
#include <array>
#include <cstdint>
#include <iomanip>
#include <random>
#include <sstream>
#include <string>

namespace asmc::lidar::runtime {
// Caller holds its lifecycle lock across Reset and the final publish check.
// Neither sensor loss nor camera state changes call Reset.
class ProcessingEpoch {
 public:
  ProcessingEpoch() {
    std::random_device random;
    std::array<unsigned char, 16> bytes{};
    for (auto& byte : bytes) byte = static_cast<unsigned char>(random());
    bytes[6] = (bytes[6] & 0x0f) | 0x40;
    bytes[8] = (bytes[8] & 0x3f) | 0x80;
    std::ostringstream out;
    for (std::size_t i = 0; i < bytes.size(); ++i) {
      if (i == 4 || i == 6 || i == 8 || i == 10) out << '-';
      out << std::hex << std::setw(2) << std::setfill('0') << unsigned(bytes[i]);
    }
    session_ = out.str();
  }
  const std::string& session() const { return session_; }
  std::uint64_t generation() const { return generation_; }
  bool IsCurrent(std::uint64_t generation) const { return generation == generation_; }
  void Reset() { ++generation_; }
 private:
  std::string session_;
  std::uint64_t generation_ = 1;
};
}  // namespace asmc::lidar::runtime
