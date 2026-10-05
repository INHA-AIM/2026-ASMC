#pragma once
#include <cstdint>
#include <deque>

namespace asmc_lidar {
struct ImuSample {
  std::uint64_t stamp_ns = 0;
  double roll_rad = 0.0;
  double pitch_rad = 0.0;
  bool valid = false;
  double yaw_rad = 0.0;
  double yaw_rate_rad_s = 0.0;
};
enum class ImuSelectionStatus { Valid, Missing, FutureOnly, Invalid, Stale };
struct ImuSelection {
  ImuSelectionStatus status = ImuSelectionStatus::Missing;
  ImuSample sample;
  bool newer_invalid_seen = false;
};
inline ImuSelection SelectImu(const std::deque<ImuSample>& arrived,
                              std::uint64_t scan_ns,
                              std::uint64_t max_age_ns = 100000000ULL) {
  const ImuSample* latest_valid = nullptr;
  const ImuSample* latest_invalid = nullptr;
  bool stamped_arrived = false;
  for(const auto& sample : arrived) {
    if(!sample.stamp_ns) continue;
    stamped_arrived = true;
    if(sample.stamp_ns > scan_ns) continue;
    const ImuSample*& latest = sample.valid ? latest_valid : latest_invalid;
    if(!latest || sample.stamp_ns >= latest->stamp_ns) latest = &sample;
  }
  if(!latest_valid) {
    if(latest_invalid) return {ImuSelectionStatus::Invalid,*latest_invalid,false};
    return {stamped_arrived?ImuSelectionStatus::FutureOnly:ImuSelectionStatus::Missing,{},false};
  }
  const bool newer_invalid = latest_invalid &&
                             latest_invalid->stamp_ns > latest_valid->stamp_ns;
  if(scan_ns-latest_valid->stamp_ns > max_age_ns)
    return {ImuSelectionStatus::Stale,*latest_valid,newer_invalid};
  return {ImuSelectionStatus::Valid,*latest_valid,newer_invalid};
}
inline const char* ImuSelectionName(ImuSelectionStatus status) {
  switch(status) {
    case ImuSelectionStatus::Valid: return "valid";
    case ImuSelectionStatus::Missing: return "missing";
    case ImuSelectionStatus::FutureOnly: return "future_only";
    case ImuSelectionStatus::Invalid: return "invalid";
    case ImuSelectionStatus::Stale: return "stale";
  }
  return "unknown";
}
}  // namespace asmc_lidar
