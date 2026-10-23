#pragma once
#include <cstdint>
#include <stdexcept>

namespace asmc::lidar::pipeline {
// Exact ROS1 Duration::toSec arithmetic without a ROS dependency. Do not cast
// epoch nanoseconds to double before subtraction: the subsecond bits matter.
inline double SourceDeltaSeconds(std::int64_t current, std::int64_t previous) {
  constexpr std::int64_t billion = 1000000000;
  if (current < 0 || previous < 0 || current / billion > UINT32_MAX || previous / billion > UINT32_MAX)
    throw std::out_of_range("source stamp outside ROS1 Time range");
  auto sec = current / billion - previous / billion;
  auto nsec = current % billion - previous % billion;
  if (nsec < 0) { --sec; nsec += billion; }
  if (sec < INT32_MIN || sec > INT32_MAX)
    throw std::out_of_range("source delta outside ROS1 Duration range");
  return static_cast<double>(sec) + 1e-9 * static_cast<double>(nsec);
}
}  // namespace asmc::lidar::pipeline
