#!/usr/bin/env python3
"""Generate isolated adapters from SHA-verified sources; never edit a checkout."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tunnel_geometry"))
from prepare import generate as prepare_l1b4
sys.path.insert(0, str(ROOT / "camera_sync"))
from prepare_reference import block

TRAVEL_REVISION = "95dc2fbd66a343efd9060c45a5711b6307a950a4"
TRAVEL_SHA = {"aos.hpp": "afde8f19e5e74acd78eb88dff52a8b660a1ca8dfb79967ea48286d48dfcb3a6c",
              "types.hpp": "d21302a7dd5bfa2cfa7890b9a30ae9f28c9e6b27002a3756f84398d8430cba82",
              "logging.hpp": "df1e336571450972a9e7f7e802b113c989da991b373a74bb883d7237e746779d"}


def digest(text):
    return hashlib.sha256(text.encode()).hexdigest()


def generate(source, travel, output):
    prepare_l1b4(source, output)
    header = output / "pinned/Lidar/TunnelFilter/TunnelFilterCore.hpp"
    header.write_text(header.read_text().replace("friend class L1B4Harness;",
                                                "friend class L1B4Harness;\n    friend class L1B5Harness;"))
    core = (source / "src/Lidar/TunnelFilter/TunnelFilterCore.cpp").read_text()
    original = block(core, "void TunnelPipeline::ClusterAndFilter(")
    marker = "    stage_start = Clock::now();\n    std::vector<Obb> obbs;"
    tail = original[original.index(marker):].rsplit("}", 1)[0]
    # The OBB body is verbatim; only the local timing helper name changes.
    tail = tail.replace("ElapsedMs(", "Ms(")
    harness = '''#pragma once
namespace tunnel_filter {
class L1B5Harness {
public:
    static FrameResult Finish(TunnelPipeline& pipeline, const FrameResult& frozen,
                             const Cloud::Ptr& points, const std::vector<std::uint8_t>& labels,
                             const std::vector<pcl::PointIndices>& clusters,
                             const std::vector<std::uint8_t>& protection) {
        if (points->size() != labels.size() || labels.size() != protection.size())
            throw std::runtime_error("raw OBB lineage mismatch");
        pipeline.result_ = frozen;
        FrameResult& r = pipeline.result_;
        r.leveled = points; r.labels = labels; r.clusters = clusters;
        r.objects.clear(); r.kept_indices.clear(); r.cluster_id.assign(points->size(), -1);
        r.ground_removed = r.ceiling_removed = r.wall_removed = r.protected_points = 0;
        for (std::size_t i = 0; i < labels.size(); ++i) {
            if (labels[i] == kKept) r.kept_indices.push_back(i);
            r.ground_removed += labels[i] == kGround; r.ceiling_removed += labels[i] == kCeiling;
            r.wall_removed += labels[i] == kWall; r.protected_points += protection[i] != kProtectNone;
        }
        r.kept_points = r.kept_indices.size();
        for (std::size_t c = 0; c < clusters.size(); ++c) for (int index : clusters[c].indices) {
            if (index < 0 || std::size_t(index) >= labels.size() || labels[index] != kKept || r.cluster_id[index] != -1)
                throw std::runtime_error("invalid supplied cluster partition");
            r.cluster_id[index] = c;
        }
        const Params& params_ = pipeline.params_;
        const auto& obb_function_ = pipeline.obb_function_;
        const auto* protect_class = &protection;
        auto GroundHeightAt = [&](float x, float y) { return pipeline.GroundHeightAt(x, y); };
        auto DegToRad = [](float x) { return x * 3.14159265358979323846f / 180.0f; };
        auto stage_start = Clock::now();
TAIL
        FrameResult result = r;
        pipeline.result_ = frozen;
        return result;
    }
};
}
'''.replace("TAIL", tail)
    (output / "cluster_harness.hpp").write_text(harness)
    destination = output / "travel"
    destination.mkdir(exist_ok=True)
    generated = {}
    for name, expected in TRAVEL_SHA.items():
        contents = (travel / name).read_bytes()
        if hashlib.sha256(contents).hexdigest() != expected:
            raise ValueError("pinned TRAVEL source changed: " + name)
        text = contents.decode()
        if name == "aos.hpp":
            # Both overloads dereference lower_bound(end). Bound the last row
            # without changing any graph linkage/metric/threshold behaviour.
            for needle, replacement in (
                ("if (iter_geq == vert_angles_.begin()) {", "if (iter_geq == vert_angles_.end()) {\n                    return int(vert_angles_.size()) - 1;\n                } else if (iter_geq == vert_angles_.begin()) {"),
                ("if (iter_geq == vert_angles.begin()) {", "if (iter_geq == vert_angles.end()) {\n                    return int(vert_angles.size()) - 1;\n                } else if (iter_geq == vert_angles.begin()) {"),
                ("static float ang_res_x = 360.0/float(HORZ_SCAN);", "const float ang_res_x = 360.0/float(HORZ_SCAN);")):
                if text.count(needle) != 1:
                    raise ValueError("TRAVEL boundary adapter marker changed")
                text = text.replace(needle, replacement)
            text = text.replace("            ObjectCluster() {", "            double last_projection_ms = 0;\n            ObjectCluster() {")
            text = text.replace("sphericalProjection(cloud_in);\n                auto end = chrono::steady_clock::now();",
                "sphericalProjection(cloud_in);\n                auto end = chrono::steady_clock::now();\n                last_projection_ms = chrono::duration<double, std::milli>(end-start).count();")
        (destination / name).write_text(text)
        generated[name] = digest(text)
    evaluation = (ROOT / "tunnel_geometry/evaluate.cpp").read_text()
    prefix = evaluation.split("        const auto off = l1b4::Apply(", 1)[0]
    if not prefix.endswith("        result[\"baseline\"] = Serialize(reference, protect, ambiguous, available, protected_boxes, protection_ms, 0, 0);\n"):
        raise ValueError("frozen evaluation prefix changed")
    prefix = prefix.replace('#include "lineage.hpp"', '#include "lineage.hpp"\n#include "clustering.hpp"\n#include "cluster_harness.hpp"')
    prefix = prefix.replace('const auto config = YAML::LoadFile(argv[4])["geometry"];',
                            'const auto full_config = YAML::LoadFile(argv[4]);\n    const auto config = full_config["geometry"];\n    const auto aos_params = l1b5::LoadParams(full_config);')
    # Keep the legacy warm-up state, but avoid serializing/reclustering four
    # large masks for unmeasured calls. No stateful AOS/tracker is introduced.
    needle = '        const auto pre = l1b4::PreWallLabels(reference);'
    prefix = prefix.replace(needle, '''        if (!result["measurement"].as<bool>()) {
            YAML::Node warm;
            warm["input_id"] = result["input_id"]; warm["measurement"] = false;
            warm["scan_stamp_ns"] = result["scan_stamp_ns"];
            Json(out, warm); out << '\\n'; continue;
        }
''' + needle)
    suffix = (Path(__file__).parent / "evaluate_tail.inc").read_text()
    (output / "cluster_evaluate.cpp").write_text(prefix + suffix)
    helper = output / "evaluation_helpers.hpp"
    helper.write_text(helper.read_text().replace('key=="input_id"||key=="scene"',
                     'key=="status"||key=="reason"||key=="raw_path"||key=="input_id"||key=="scene"'))
    provenance = json.loads((output / "reference_provenance.json").read_text())
    provenance.update({"travel_revision": TRAVEL_REVISION, "travel_source_sha256": TRAVEL_SHA,
                       "travel_generated_sha256": generated,
                       "obb_filter_original_body_sha256": digest(original[original.index(marker):]),
                       "cluster_harness_sha256": digest(harness),
                       "generated_evaluator_sha256": digest(prefix + suffix),
                       "adaptations_l1b5a": [
                           "Private generated friend harness calls the exact pinned L-shape OBB and OBB filter body",
                           "TRAVEL getRowIdx upper-end bounds in both overloads; no graph metric changes",
                           "TRAVEL angular column resolution becomes per-instance rather than function-static",
                           "Generated core exposes existing monotonic projection timer, without modifying graph decisions",
                           "Unique pre-existing voxel counts gate both raw algorithms; no GT classes are input",
                           "Projection exclusions remain visible; no collision-point label propagation",
                           "Warmup still runs original preprocessing/persistence; only measured calls compare clusters"]})
    (output / "reference_provenance.json").write_text(json.dumps(provenance, sort_keys=True, indent=2) + "\n")


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for name in ("source", "travel", "output"):
        p.add_argument("--" + name, type=Path, required=True)
    a = p.parse_args()
    generate(a.source, a.travel, a.output)
