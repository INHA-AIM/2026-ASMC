#pragma once
#include <cassert>
#include <cmath>
#include <map>
#include <limits>
#include <set>
#include <travel/aos.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

namespace l1b5 {
struct Params {
    int rows = 16, columns = 1200, vertical_window = 1, horizontal_window = 5;
    int extension = 5, skip = 2, min_voxels = 10, max_voxels = 30000;
    float min_angle = -15, max_angle = 15, min_range = .1, max_range = 100;
    float horizontal_merge = 1, vertical_merge = 1, tolerance = 1, angle_uncertainty = 1;
};
inline Params LoadParams(const YAML::Node& config) {
    Params p; const auto a = config["aos"], c = config["comparison"];
    p.rows = a["vertical_channels"].as<int>(); p.columns = a["horizontal_columns"].as<int>();
    p.vertical_window = a["vertical_scan_size"].as<int>(); p.horizontal_window = a["horizontal_scan_size"].as<int>();
    p.extension = a["horizontal_extension_size"].as<int>(); p.skip = a["horizontal_skip_size"].as<int>();
    p.min_angle = a["min_vertical_angle_deg"].as<float>(); p.max_angle = a["max_vertical_angle_deg"].as<float>();
    p.min_range = a["min_range_m"].as<float>(); p.max_range = a["max_range_m"].as<float>();
    p.horizontal_merge = a["horizontal_merge_m"].as<float>(); p.vertical_merge = a["vertical_merge_m"].as<float>();
    p.angle_uncertainty = a["angle_uncertainty_deg"].as<float>();
    p.tolerance = c["euclidean_tolerance_m"].as<float>();
    p.min_voxels = c["cluster_min_unique_voxels"].as<int>(); p.max_voxels = c["cluster_max_unique_voxels"].as<int>();
    if (p.rows < 2 || p.columns < 4 || p.columns % 2 || p.min_voxels < 1 || p.max_voxels < p.min_voxels)
        throw std::runtime_error("invalid clustering configuration");
    return p;
}
struct Point {
    float x = 0, y = 0, z = 0, intensity = 0;
    std::uint16_t label = 0, id = 0;
    std::int32_t raw_index = -1;
};
enum ProjectionReason { NotInput = 0, Returned = 1, Nonfinite = 2, Range = 3,
                        AmbiguousChannel = 4, CellCollision = 5, CoreUnassigned = 6, ObservationGate = 7 };
struct Partition {
    std::vector<pcl::PointIndices> clusters;
    std::vector<int> reasons, pre_gate_cluster_id, projection_row, projection_col, rejected_gate_sizes;
    int input_points = 0, projected_points = 0;
    double projection_ms = 0, linkage_ms = 0, observation_gate_ms = 0;
};
inline void Gate(Partition& output, std::vector<pcl::PointIndices> components,
                 const std::vector<std::int32_t>& mapping, const Params& p) {
    const auto start = Clock::now();
    std::sort(components.begin(), components.end(), [](const auto& a, const auto& b) {
        return *std::min_element(a.indices.begin(), a.indices.end()) < *std::min_element(b.indices.begin(), b.indices.end());
    });
    output.pre_gate_cluster_id.assign(mapping.size(), -1);
    for (std::size_t i = 0; i < components.size(); ++i) {
        auto& component = components[i]; std::sort(component.indices.begin(), component.indices.end());
        std::set<std::int32_t> voxels;
        for (int index : component.indices) {
            if (index < 0 || std::size_t(index) >= mapping.size() || mapping[index] < 0 || output.pre_gate_cluster_id[index] != -1)
                throw std::runtime_error("cluster/raw/voxel partition is inconsistent");
            voxels.insert(mapping[index]); output.pre_gate_cluster_id[index] = i;
        }
        if (voxels.size() < std::size_t(p.min_voxels) || voxels.size() > std::size_t(p.max_voxels)) {
            output.rejected_gate_sizes.push_back(voxels.size());
            for (int index : component.indices) output.reasons[index] = ObservationGate;
        } else output.clusters.push_back(component);
    }
    output.observation_gate_ms = Ms(start);
}
inline Partition Euclidean(const Cloud::Ptr& leveled, const std::vector<std::uint8_t>& labels,
                           const std::vector<std::int32_t>& mapping, const Params& p) {
    Partition result; result.reasons.assign(labels.size(), NotInput);
    pcl::IndicesPtr kept(new std::vector<int>());
    for (std::size_t i = 0; i < labels.size(); ++i) if (labels[i] == tunnel_filter::kKept) {
        kept->push_back(i); result.reasons[i] = Returned;
    }
    result.input_points = kept->size(); result.projected_points = kept->size();
    auto start = Clock::now(); std::vector<pcl::PointIndices> components;
    if (!kept->empty()) {
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> extract;
        extract.setClusterTolerance(p.tolerance); extract.setMinClusterSize(1);
        extract.setMaxClusterSize(std::numeric_limits<int>::max());
        extract.setSearchMethod(tree); extract.setInputCloud(leveled); extract.setIndices(kept);
        extract.extract(components);
    }
    result.linkage_ms = Ms(start); Gate(result, std::move(components), mapping, p); return result;
}
inline Partition Aos(const Cloud& sensor, const std::vector<std::uint8_t>& labels,
                     const std::vector<std::int32_t>& mapping, const Params& p) {
    const auto projection_start = Clock::now();
    Partition result; result.reasons.assign(sensor.size(), NotInput);
    result.projection_row.assign(sensor.size(), -1); result.projection_col.assign(sensor.size(), -1);
    auto input = std::make_shared<travel::PointCloud<Point>>();
    auto output = std::make_shared<travel::PointCloud<Point>>();
    travel::ObjectCluster<Point> aos;
    aos.setParams(p.rows, p.columns, p.min_range, p.max_range, p.min_angle, p.max_angle,
                  p.horizontal_merge, p.vertical_merge, p.vertical_window, p.horizontal_window,
                  p.extension, p.skip, 1);
    aos.setSeed(0);
    std::map<int, int> first;
    for (std::size_t i = 0; i < sensor.size(); ++i) if (labels[i] == tunnel_filter::kKept) {
        ++result.input_points; const auto& q = sensor[i];
        result.reasons[i] = CoreUnassigned;
        if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z)) { result.reasons[i] = Nonfinite; continue; }
        const float range = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z);
        if (range < p.min_range || range > p.max_range) { result.reasons[i] = Range; continue; }
        const float el = std::atan2(q.z, std::sqrt(q.x*q.x + q.y*q.y)) * 180 / M_PI;
        Point point; point.x = q.x; point.y = q.y; point.z = q.z;
        point.intensity = q.intensity; point.raw_index = i;
        const float step = (p.max_angle - p.min_angle) / (p.rows - 1);
        // Use the generated core's exact floating-point bin decision. A
        // re-expression with float overloads differs at half-column edges.
        const int row = aos.getRowIdx(point);
        if (std::abs(el - (p.min_angle + row * step)) > p.angle_uncertainty) {
            result.reasons[i] = AmbiguousChannel; continue;
        }
        const int col = aos.getColIdx(point);
        result.projection_row[i] = row; result.projection_col[i] = col;
        const int cell = row * p.columns + col;
        if (col < 0 || col >= p.columns) throw std::runtime_error("unexpected AOS column bounds");
        if (!first.emplace(cell, i).second) result.reasons[i] = CellCollision;
        input->push_back(point);
    }
    result.projection_ms = Ms(projection_start);
    const auto linkage_start = Clock::now(); aos.segmentObjects(input, output);
    result.linkage_ms = Ms(linkage_start) - aos.last_projection_ms;
    result.projection_ms += aos.last_projection_ms; result.projected_points = input->size();
    // The core keeps first-in-cell and may change cluster IDs. raw_index is a
    // private passthrough field, not intensity/semantic/instance truth.
    std::set<int> projected;
    for (const auto& point : input->points) projected.insert(point.raw_index);
    for (std::size_t i = 0; i < result.reasons.size(); ++i)
        if (result.reasons[i] == CellCollision && projected.count(i))
            throw std::runtime_error("projection diagnostic disagrees with pinned core");
    std::map<int, pcl::PointIndices> components;
    for (const auto& point : output->points) {
        int index = point.raw_index;
        if (index < 0 || std::size_t(index) >= sensor.size() || !projected.count(index) ||
            labels[index] != tunnel_filter::kKept || result.reasons[index] == Returned)
            throw std::runtime_error("AOS returned invalid or duplicate original index");
        const auto& original = sensor[index];
        if (point.x != original.x || point.y != original.y || point.z != original.z ||
            point.intensity != original.intensity) throw std::runtime_error("AOS modified input point fields");
        result.reasons[index] = Returned; components[point.id].indices.push_back(index);
    }
    std::vector<pcl::PointIndices> clusters;
    for (const auto& pair : components) clusters.push_back(pair.second);
    Gate(result, std::move(clusters), mapping, p); return result;
}
}
