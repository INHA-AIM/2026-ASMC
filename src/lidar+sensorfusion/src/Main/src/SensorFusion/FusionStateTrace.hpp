#pragma once

#include <SensorFusion/FusionTrace.hpp>
#include <Global/Global.hpp>
#include <map>
#include <vector>

namespace fusion_state_trace {
struct Origin { std::uint64_t lidar = 0, yolo = 0, applied_at = 0; };
inline std::map<int, Origin>& Origins() { static std::map<int, Origin> origins; return origins; }

inline void Object(std::ostream& out, const LiDARObjectInfo& object) {
    out << "{\"id\":" << object.s32_Id
        << ",\"class\":" << fusion_trace::Quote(object.str_ObjectClass)
        << ",\"raw_class\":" << fusion_trace::Quote(object.str_YoloClass)
        << ",\"candidate\":" << fusion_trace::Quote(object.last_seen_class)
        << ",\"confirmations\":" << object.continuity_counter
        << ",\"class_misses\":" << object.class_miss_counter
        << ",\"lidar_misses\":" << object.s32_MissCount
        << ",\"locked\":" << (object.is_class_locked ? "true" : "false")
        << ",\"center\":[" << object.f32_CoreX << ',' << object.f32_CoreY << ',' << object.f32_CoreZ << "]}";
}

inline void Initial(const char* event, const LiDARObjectInfo& object,
                    const ros::Time& stamp) {
    if (!fusion_trace::Sink().Enabled()) return;
    Origins().erase(object.s32_Id);
    fusion_trace::Event(event, stamp.toNSec(), [&](std::ostream& out) {
        out << ",\"object\":"; Object(out, object);
    });
}

// The provenance is the historical state transferred to the live track,
// including a transfer that leaves its semantic label unchanged. It is not
// a fabricated current observation or a substitute for source timestamps.
inline void Transferred(int id, const ros::Time& historical,
                        const ros::Time& yolo, const ros::Time& current) {
    if (!fusion_trace::Sink().Enabled()) return;
    Origins()[id] = {historical.toNSec(), yolo.toNSec(), current.toNSec()};
}

inline void Snapshot(const char* event, const ros::Time& stamp,
                     const ros::Time& yolo, const std::vector<LiDARObjectInfo>& objects,
                     std::uint64_t generation = 0) {
    fusion_trace::Event(event, stamp.toNSec(), [&](std::ostream& out) {
        out << ",\"yolo_stamp_ns\":" << yolo.toNSec()
            << ",\"generation\":" << generation << ",\"objects\":[";
        bool first = true;
        for (const auto& object : objects) {
            if (!first) out << ',';
            first = false;
            Object(out, object);
        }
        out << "]";
        if (generation) {
            out << ",\"origins\":[";
            first = true;
            for (const auto& object : objects) {
                const auto it = Origins().find(object.s32_Id);
                const Origin origin = it == Origins().end() ? Origin{} : it->second;
                if (!first) out << ',';
                first = false;
                out << "{\"id\":" << object.s32_Id
                    << ",\"source_lidar_stamp_ns\":" << origin.lidar
                    << ",\"source_yolo_stamp_ns\":" << origin.yolo
                    << ",\"applied_at_lidar_stamp_ns\":" << origin.applied_at
                    << ",\"updated_in_frame\":" << (origin.applied_at == stamp.toNSec() ? "true" : "false") << '}';
            }
            out << ']';
        }
    });
}
}  // namespace fusion_state_trace
