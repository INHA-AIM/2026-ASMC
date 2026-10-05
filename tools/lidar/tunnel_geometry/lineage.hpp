#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <map>
#include <stdexcept>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace l1b4 {
using Cloud = pcl::PointCloud<pcl::PointXYZI>;
struct Lineage { std::vector<std::int32_t> raw_to_voxel; int errors = 0; };
inline bool SamePoint(const pcl::PointXYZI& a, const pcl::PointXYZI& b) {
    const float av[] = {a.x, a.y, a.z, a.intensity}, bv[] = {b.x, b.y, b.z, b.intensity};
    return std::memcmp(av, bv, sizeof(av)) == 0;
}
inline Lineage Trace(const Cloud& raw, const Cloud& filtered, const Cloud& voxel, bool preprocessed) {
    Lineage result; result.raw_to_voxel.assign(raw.size(), -1);
    if (preprocessed) {
        if (raw.size() != voxel.size()) throw std::runtime_error("preprocessed index count mismatch");
        for (std::size_t i = 0; i < raw.size(); ++i) {
            if (!SamePoint(raw[i], voxel[i])) throw std::runtime_error("preprocessed point fields changed");
            result.raw_to_voxel[i] = i;
        }
        return result;
    }
    std::vector<int> filtered_to_raw;
    std::size_t cursor = 0;
    for (const auto& p : filtered) {
        while (cursor < raw.size() && !SamePoint(raw[cursor], p)) ++cursor;
        if (cursor == raw.size()) throw std::runtime_error("ROI lost ordered raw-point lineage");
        filtered_to_raw.push_back(cursor++);
    }
    using Cell = std::array<int, 3>;
    auto cell = [](const auto& p) -> Cell {
        return {static_cast<int>(std::floor(p.x * 10.f)), static_cast<int>(std::floor(p.y * 10.f)),
                static_cast<int>(std::floor(p.z * 10.f))};
    };
    std::map<Cell, std::vector<int>> groups;
    for (std::size_t i = 0; i < filtered.size(); ++i) groups[cell(filtered[i])].push_back(i);
    std::map<Cell, bool> used;
    for (std::size_t vi = 0; vi < voxel.size(); ++vi) {
        auto key = cell(voxel[vi]);
        auto group = groups.find(key);
        // A rounded centroid can fall on the adjacent leaf boundary. Admit a
        // fallback only when exactly one nearby original group reproduces it.
        if (group == groups.end() || used[key]) {
            std::vector<Cell> matches;
            for (int x = -1; x <= 1; ++x) for (int y = -1; y <= 1; ++y) for (int z = -1; z <= 1; ++z) {
                const Cell neighbor{key[0] + x, key[1] + y, key[2] + z};
                auto it = groups.find(neighbor);
                if (it == groups.end() || used[neighbor]) continue;
                std::array<double, 4> sum{};
                for (int index : it->second) {
                    const auto& p = filtered[index]; sum[0] += p.x; sum[1] += p.y;
                    sum[2] += p.z; sum[3] += p.intensity;
                }
                const float values[] = {voxel[vi].x, voxel[vi].y, voxel[vi].z, voxel[vi].intensity};
                bool exact = true;
                for (int axis = 0; axis < 4; ++axis)
                    exact &= std::abs(sum[axis] / it->second.size() - values[axis]) <= 1e-5;
                if (exact) matches.push_back(neighbor);
            }
            if (matches.size() != 1) { ++result.errors; continue; }
            key = matches[0]; group = groups.find(key);
        }
        used[key] = true;
        for (int index : group->second) result.raw_to_voxel[filtered_to_raw[index]] = vi;
    }
    for (int raw_index : filtered_to_raw) if (result.raw_to_voxel[raw_index] < 0) ++result.errors;
    return result;
}
}  // namespace l1b4
