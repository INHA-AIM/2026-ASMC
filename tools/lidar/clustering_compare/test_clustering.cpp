#include "evaluation_helpers.hpp"
#include "clustering.hpp"
#include "cluster_harness.hpp"

void Require(bool condition, const char* message) { if (!condition) throw std::runtime_error(message); }
pcl::PointXYZI Ray(float elevation, float azimuth, float radius) {
    const float e = elevation * M_PI / 180, a = azimuth * M_PI / 180;
    pcl::PointXYZI p; p.x = radius * std::cos(e) * std::cos(a);
    p.y = radius * std::cos(e) * std::sin(a); p.z = radius * std::sin(e); p.intensity = 73;
    return p;
}
int main() try {
    using namespace l1b5;
    Params p; p.min_voxels = 1;
    Cloud::Ptr points(new Cloud);
    points->push_back(Ray(15.2, 0, 10)); points->push_back(Ray(15.2, 0, 11));
    points->push_back(Ray(1, 179.999, 10)); points->push_back(Ray(1, -179.999, 10));
    points->push_back(Ray(20, 30, 10));
    const std::vector<std::uint8_t> labels(points->size(), tunnel_filter::kKept);
    const std::vector<std::int32_t> mapping{0,1,2,3,4};
    auto a = Aos(*points, labels, mapping, p), b = Aos(*points, labels, mapping, p);
    Require(a.pre_gate_cluster_id == b.pre_gate_cluster_id, "AOS repeat partition changed");
    Require(a.reasons[0] == Returned && a.reasons[1] == CellCollision, "collision lineage lost");
    Require(a.reasons[4] == AmbiguousChannel, "uncertain channel silently included");
    Require(a.projected_points == 2, "azimuth seam/duplicate cell handling changed");
    auto width = p; width.columns = 1440;
    auto c = Aos(*points, labels, mapping, width);
    Require(c.projection_col[2] >= 0 && c.projection_col[2] < 1440, "per-instance columns changed");
    Cloud::Ptr repeated(new Cloud);
    for (int i = 0; i < 20; ++i) { auto q = Ray(1, i*.3, 10); repeated->push_back(q); }
    auto gate = p; gate.min_voxels = 10;
    auto duplicates = Euclidean(repeated, std::vector<std::uint8_t>(20, 0), std::vector<std::int32_t>(20, 0), gate);
    Require(duplicates.clusters.empty() && duplicates.rejected_gate_sizes == std::vector<int>{1}, "raw count weakened voxel gate");
    std::vector<std::int32_t> unique(20); std::iota(unique.begin(), unique.end(), 0);
    auto accepted = Euclidean(repeated, std::vector<std::uint8_t>(20, 0), unique, gate);
    Require(accepted.clusters.size() == 1, "well-observed cluster rejected");
    tunnel_filter::FrameResult frozen; frozen.leveled = repeated; frozen.labels.assign(20, 0);
    frozen.ground_fallback_used = true; frozen.ground_reference_height = -1.45;
    tunnel_filter::TunnelPipeline pipeline;
    auto result = tunnel_filter::L1B5Harness::Finish(pipeline, frozen, repeated, frozen.labels,
        accepted.clusters, std::vector<std::uint8_t>(20, 0));
    Require(result.clusters.size() == 1 && result.objects.size() == 1, "OBB/cluster pairing differs");
    Require(pipeline.LastResult().leveled == frozen.leveled, "harness did not restore frozen state");
    return 0;
} catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
