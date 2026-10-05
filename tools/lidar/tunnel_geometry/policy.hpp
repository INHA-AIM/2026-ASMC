#pragma once
#include <algorithm>
#include <stdexcept>
#include <Lidar/TunnelFilter/TunnelFilterCore.hpp>

namespace l1b4 {
struct Params {
    double band_min = .05, band_max = .15, mad_scale = 3., mad_normal_scale = 1.4826;
    double connection_radius = .20, expansion_max = .30;
};
enum Reason : std::uint8_t {
    Upstream = 0, OutsideProposal = 1, WallConforming = 2, Foreground = 3,
    CameraProtected = 4, Unsupported = 5, Uncertain = 6, BehindWall = 7,
    LocalExpansion = 8
};
struct Model {
    tunnel_filter::WallSegment segment;
    double band = .05;
    int residual_samples = 0;
    double median = 0, mad = 0;
};
struct Decision {
    std::vector<std::uint8_t> labels, reasons, geometry_protected, uncertain;
    std::vector<int> component_id;
    std::vector<Model> models;
    int components = 0;
};
std::vector<std::uint8_t> PreWallLabels(const tunnel_filter::FrameResult& reference);
Decision Apply(const tunnel_filter::FrameResult& reference,
               const std::vector<std::uint8_t>& camera_protection,
               const tunnel_filter::Params& legacy, const Params& params = Params(), bool enabled = true);
}  // namespace l1b4

namespace tunnel_filter {
// Friendship exists ONLY in the generated BUILD-directory header.
class L1B4Harness {
public:
    static FrameResult Recluster(TunnelPipeline& pipeline, const FrameResult& frozen,
                                const std::vector<std::uint8_t>& labels,
                                const std::vector<std::uint8_t>& camera_protection) {
        if (labels.size() != frozen.labels.size()) throw std::runtime_error("label lineage mismatch");
        pipeline.result_ = frozen;
        auto& result = pipeline.result_;
        result.labels = labels;
        result.kept_indices.clear(); result.clusters.clear(); result.objects.clear();
        result.wall_removed = std::count(labels.begin(), labels.end(), kWall);
        pipeline.ClusterAndFilter(&camera_protection);
        FrameResult output = result;
        pipeline.result_ = frozen;
        return output;
    }
};
}  // namespace tunnel_filter
