#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iterator>

namespace asmc { namespace geometry {
// Only compare stamps in an already-validated common clock domain. Receipt
// clocks measure delivery age; they are never subtracted from sensor stamps.
template<class Range, class Key>
auto LatestAtOrBefore(const Range& samples, std::uint64_t stamp, Key key)
    -> decltype(samples.begin()) {
    const auto upper = std::upper_bound(samples.begin(), samples.end(), stamp,
        [&](std::uint64_t value, const auto& sample) { return value < key(sample); });
    return upper == samples.begin() ? samples.end() : std::prev(upper);
}
inline bool WithinAge(std::uint64_t sample, std::uint64_t query, double maximum_seconds) {
    return sample > 0 && query >= sample && std::isfinite(maximum_seconds) &&
        maximum_seconds >= 0 && maximum_seconds <= 1e6 &&
        query - sample <= static_cast<std::uint64_t>(std::llround(maximum_seconds * 1e9));
}
// ROS-stamped observations may have a small, explicit future tolerance.
// Use integer ns before subtracting: Unix-epoch doubles lose ns precision.
// This is NOT a mapping between sensor, ROS and monotonic clock domains.
inline bool WithinStampWindow(std::uint64_t sample, std::uint64_t query,
                              double maximum_age, double future_tolerance) {
    if (!sample || !query || !std::isfinite(maximum_age) || maximum_age < 0 ||
        maximum_age > 1e6 || !std::isfinite(future_tolerance) ||
        future_tolerance < 0 || future_tolerance > 1e6) return false;
    return sample <= query ? WithinAge(sample, query, maximum_age)
                           : WithinAge(query, sample, future_tolerance);
}
} }  // namespace asmc::geometry
