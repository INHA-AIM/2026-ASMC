#ifndef FRENET_MANEUVER_MERGE_INTERNAL_HPP
#define FRENET_MANEUVER_MERGE_INTERNAL_HPP

#include "frenet_planner/global/data_logger.hpp"

// package-internal(frenet_planner.cpp/Maneuver/Merge.cpp 전용) 고주로 합류
// zone/checkpoint 조회 helper. 둘 다에서 필요해 익명 namespace 밖으로 뺐다 -
// public API가 아니므로 frenet_planner.hpp에는 선언하지 않는다. 로직은 원래
// frenet_planner.cpp 익명 namespace에 있던 것과 동일하다.

inline const HighwayMergeZone* HighwayZoneAt(const HighwayMergeConfig& cfg, int index) {
    if (index < 0 || static_cast<size_t>(index) >= cfg.zones.size()) return nullptr;
    return &cfg.zones[static_cast<size_t>(index)];
}

inline const HighwayMergeCheckpoint* HighwayCheckpointAt(
        const HighwayMergeConfig& cfg, int zone_index, int checkpoint_index) {
    const HighwayMergeZone* zone = HighwayZoneAt(cfg, zone_index);
    if (!zone || checkpoint_index < 0 ||
        static_cast<size_t>(checkpoint_index) >= zone->checkpoints.size()) return nullptr;
    return &zone->checkpoints[static_cast<size_t>(checkpoint_index)];
}

inline int FindHighwayMergeZone(const HighwayMergeConfig& cfg, double s) {
    if (!cfg.enabled) return -1;
    for (size_t i = 0; i < cfg.zones.size(); ++i) {
        const auto& zone = cfg.zones[i];
        if (s >= zone.start_s && s <= zone.completion_s)
            return static_cast<int>(i);
    }
    return -1;
}

#endif
