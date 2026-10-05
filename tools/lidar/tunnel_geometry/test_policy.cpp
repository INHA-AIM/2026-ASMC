#include "policy.hpp"
#include "lineage.hpp"
#include <iostream>

using namespace tunnel_filter;
void Check(bool value, const char* message) { if (!value) throw std::runtime_error(message); }
FrameResult Fixture() {
    FrameResult frame; frame.leveled.reset(new pcl::PointCloud<pcl::PointXYZI>);
    for (int i = 0; i < 20; ++i) {
        pcl::PointXYZI p; p.x = i * .1f; p.y = 3.6f; p.z = 1; p.intensity = i;
        frame.leveled->push_back(p);
    }
    frame.labels.assign(20, kWall); frame.ground_reference_height = -1.45;
    frame.ground.source = PlaneSource::kUnavailable;
    WallSegment segment; segment.valid = true; segment.plane = {0, -1, 0, 3.6};
    segment.intervals = {{0, 2}}; frame.walls[0].segments.push_back(segment);
    frame.walls[0].removal_allowed = true;
    return frame;
}

int main() try {
    auto frame = Fixture(); Params legacy; legacy.wall_remove_distance_m = .25;
    std::vector<std::uint8_t> camera(frame.labels.size(), 0);
    auto result = l1b4::Apply(frame, camera, legacy);
    Check(l1b4::Apply(frame, camera, legacy, l1b4::Params(), false).labels == frame.labels,
          "candidate OFF did not preserve reference labels");
    Check(std::count(result.labels.begin(), result.labels.end(), kWall) == 20, "wall skin not removed");
    Check(result.models[0].band == .05, "minimum residual band wrong");
    frame.walls[0].segments[0].plane = {0, 1, 0, -3.6};
    Check(l1b4::Apply(frame, camera, legacy).labels == result.labels, "plane sign changed decision");
    auto point = frame.leveled->points[0]; point.x = 1; point.y = 3.5;
    frame.leveled->push_back(point); frame.labels.push_back(kWall); camera.push_back(0);
    result = l1b4::Apply(frame, camera, legacy);
    Check(result.labels.back() == kKept && result.geometry_protected.back(), "single protrusion lost");
    Check(result.components == 1, "component lineage missing");
    Check(std::count(result.labels.begin(), result.labels.begin() + 20, kWall) == 20, "protection flooded wall skin");
    point.x = 2.5; point.y = 3.6;
    frame.leveled->push_back(point); frame.labels.push_back(kWall); camera.push_back(0);
    result = l1b4::Apply(frame, camera, legacy);
    Check(result.labels.back() == kKept && result.uncertain.back(), "support extrapolated beyond interval");
    frame.labels.back() = kGround;
    Check(l1b4::Apply(frame, camera, legacy).labels.back() == kGround, "upstream label changed");
    camera[0] = kProtectPerson;
    Check(l1b4::Apply(frame, camera, legacy).labels[0] == kKept, "camera protection lost");
    Check(frame.leveled->points[7].intensity == 7, "intensity changed");
    auto missing = Fixture(); missing.walls[0].removal_allowed = false;
    Check(l1b4::Apply(missing, std::vector<std::uint8_t>(20, 0), legacy).models.empty(), "unconfirmed wall used");
    l1b4::Cloud raw, filtered, voxel;
    auto p = frame.leveled->points[0]; p.x = .01; p.y = .01; p.z = .01;
    raw.push_back(p); p.x = 10; raw.push_back(p); p.x = .09; raw.push_back(p);
    filtered.push_back(raw[0]); filtered.push_back(raw[2]);
    p.x = .05; voxel.push_back(p);
    auto lineage = l1b4::Trace(raw, filtered, voxel, false);
    Check(lineage.raw_to_voxel == std::vector<std::int32_t>({0, -1, 0}) && lineage.errors == 0,
          "raw-to-voxel groups or ROI rejection lost");
    Check(l1b4::Trace(raw, raw, raw, true).raw_to_voxel == std::vector<std::int32_t>({0, 1, 2}),
          "preprocessed lineage changed");
    std::cout << "geometry policy: orientation, singleton, no flooding, support, labels, camera, fields passed\n";
    return 0;
} catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
