#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace fusion_sync {

enum class MatchAction {
    Wait,
    Match,
    Drop,
};

struct MatchDecision {
    MatchAction action = MatchAction::Wait;
    std::size_t yolo_index = std::numeric_limits<std::size_t>::max();
    std::int64_t offset_ns = std::numeric_limits<std::int64_t>::max();
};

// Decide only after a YOLO frame at/after the LiDAR stamp has arrived.  At
// that point no later frame can be closer than the first future frame.  The
// wall-clock timeout prevents a stopped camera/YOLO stream from retaining a
// LiDAR snapshot forever; it does not relax max_offset_ns.
inline MatchDecision DecideMatch(
    std::int64_t lidar_stamp_ns,
    const std::vector<std::int64_t>& yolo_stamps_ns,
    std::int64_t waited_ns,
    std::int64_t max_offset_ns,
    std::int64_t wait_timeout_ns) {
    MatchDecision decision;
    bool has_future_frame = false;

    for (std::size_t index = 0; index < yolo_stamps_ns.size(); ++index) {
        const std::int64_t stamp_ns = yolo_stamps_ns[index];
        if (stamp_ns >= lidar_stamp_ns) has_future_frame = true;
        const std::int64_t offset_ns = stamp_ns >= lidar_stamp_ns
            ? stamp_ns - lidar_stamp_ns
            : lidar_stamp_ns - stamp_ns;
        if (offset_ns < decision.offset_ns) {
            decision.offset_ns = offset_ns;
            decision.yolo_index = index;
        }
    }

    const bool timed_out = waited_ns >= wait_timeout_ns;
    if (!has_future_frame && !timed_out) return decision;

    if (decision.yolo_index != std::numeric_limits<std::size_t>::max() &&
        decision.offset_ns <= max_offset_ns) {
        decision.action = MatchAction::Match;
    } else {
        decision.action = MatchAction::Drop;
    }
    return decision;
}

}  // namespace fusion_sync
