import copy
import math
import os
from pathlib import Path
import tempfile
import unittest

import yaml

from asmc_hd_map.runtime_bundle import (
    RUNTIME_PINS_REL,
    RuntimeBundleError,
    build_runtime_pins,
    choose_label_position,
    dump_runtime_pins,
    geometry_sha256,
    lateral_point_at_station,
    load_runtime_bundle,
    marking_role_name,
    point_at_station,
    polyline_slice,
    polyline_stations,
    short_link_id,
    source_type_name,
)


REPO_ROOT = Path(__file__).resolve().parents[3]
MAP_PATH = (
    REPO_ROOT
    / "src/asmc_hd_map/reports/kcity2025_route_curation_20260917/route_curated_candidate_map.json"
)
METADATA_PATH = REPO_ROOT / "src/asmc_hd_map/config/kcity2025_competition_metadata.yaml"
BASELINE_PATH = REPO_ROOT / RUNTIME_PINS_REL
RVIZ_PATH = REPO_ROOT / "src/asmc_hd_map/rviz/competition_map_debug.rviz"


class RuntimeBundleTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        data_root = Path(os.environ.get("ASMC_DATA", str(REPO_ROOT.parent / "data")))
        cls.bundle = load_runtime_bundle(MAP_PATH, METADATA_PATH, BASELINE_PATH, data_root)

    def test_frozen_artifacts_load_without_geometry_mutation(self):
        self.assertEqual(
            self.bundle["geometry_sha256"],
            "01ec671fe402baf339f8f6bcfb83a97213d560f7311961c5f072a47d654bcfbf",
        )
        self.assertEqual(self.bundle["feature_counts"], {"lane_marking": 121, "stopline": 7})
        self.assertEqual(len(self.bundle["checkpoints"]), 15)
        self.assertEqual(len(self.bundle["mission_zones"]), 15)
        self.assertAlmostEqual(self.bundle["route_stations"][-1], 2184.611723336, places=6)
        self.assertEqual(geometry_sha256(self.bundle["map"]), self.bundle["geometry_sha256"])

    def test_runtime_pins_match_committed_artifacts(self):
        self.assertEqual(
            BASELINE_PATH.read_text(encoding="utf-8"),
            dump_runtime_pins(build_runtime_pins(REPO_ROOT)),
        )

    def test_station_interpolation_and_slice_preserve_endpoints(self):
        points = [[0.0, 0.0, 1.0], [3.0, 0.0, 2.0], [3.0, 4.0, 3.0]]
        stations = polyline_stations(points)
        self.assertEqual(stations, [0.0, 3.0, 7.0])
        self.assertEqual(point_at_station(points, stations, 1.5), [1.5, 0.0, 1.5])
        self.assertEqual(
            polyline_slice(points, stations, 1.5, 5.0),
            [[1.5, 0.0, 1.5], [3.0, 0.0, 2.0], [3.0, 2.0, 2.5]],
        )

    def test_label_layout_uses_route_normal_and_avoids_occupied_position(self):
        points = [[0.0, 0.0, 1.0], [10.0, 0.0, 1.0]]
        stations = polyline_stations(points)
        self.assertEqual(lateral_point_at_station(points, stations, 5.0, 4.0), [5.0, 4.0, 1.0])
        position = choose_label_position(
            points,
            stations,
            5.0,
            [4.0, -4.0],
            [[5.0, 4.0]],
            3.0,
        )
        self.assertEqual(position, [5.0, -4.0, 1.0])

    def test_short_link_id_preserves_unrecognised_ids(self):
        self.assertEqual(short_link_id("A2256W000846"), "A846")
        self.assertEqual(short_link_id("A2256W000083"), "A083")
        self.assertEqual(short_link_id("custom-link"), "custom-link")

    def test_rviz_overview_uses_upright_top_down_orientation(self):
        config = yaml.safe_load(RVIZ_PATH.read_text(encoding="utf-8"))
        view = config["Visualization Manager"]["Views"]["Current"]
        self.assertEqual(view["Class"], "rviz/TopDownOrtho")
        self.assertAlmostEqual(float(view["Angle"]), math.pi, places=12)

    def test_rviz_defaults_to_markers_without_optional_local_cache(self):
        for filename in ("competition_map_debug.rviz", "competition_map_follow_ego.rviz"):
            with self.subTest(preset=filename):
                config = yaml.safe_load(RVIZ_PATH.with_name(filename).read_text(encoding="utf-8"))
                displays = config["Visualization Manager"]["Displays"]
                maps = {display.get("Topic"): display for display in displays
                        if display.get("Class") == "rviz/Map"}
                # Local crops are opt-in, so neither preset subscribes to their
                # absent default output. Use vector outlines by default and
                # keep the heavy whole-map raster available for manual review.
                self.assertNotIn("/hd_map/local/drivable_grid", maps)
                self.assertFalse(maps["/hd_map/global/drivable_grid"]["Enabled"])
                self.assertNotIn("/hd_map/debug/drivable_grid_viz", maps)
                markers = [display for display in displays
                           if display.get("Marker Topic") == "/hd_map/debug/markers"]
                self.assertEqual(len(markers), 1)
                self.assertTrue(markers[0]["Enabled"])

    def test_role_and_source_mapping_do_not_infer_ambiguous_boundary(self):
        ambiguous = {
            "feature_type": "lane_marking",
            "attributes": {
                "functional_role": "lane_boundary_paint",
                "source_display": "legacy_origin_aligned",
            },
        }
        self.assertEqual(marking_role_name(ambiguous), "unknown")
        self.assertEqual(source_type_name(ambiguous), "legacy_map")
        stopline = copy.deepcopy(ambiguous)
        stopline["feature_type"] = "stopline"
        self.assertEqual(marking_role_name(stopline), "stop_line")

    def test_metadata_sha_drift_is_rejected_before_publication(self):
        document = yaml.safe_load(METADATA_PATH.read_text(encoding="utf-8"))
        document["mission_zones"][0]["description"] += " changed"
        with tempfile.TemporaryDirectory() as directory:
            changed = Path(directory) / "metadata.yaml"
            changed.write_text(
                yaml.safe_dump(document, allow_unicode=True, sort_keys=False), encoding="utf-8"
            )
            with self.assertRaisesRegex(RuntimeBundleError, "baseline metadata SHA mismatch"):
                load_runtime_bundle(
                    MAP_PATH,
                    changed,
                    BASELINE_PATH,
                    Path(os.environ.get("ASMC_DATA", str(REPO_ROOT.parent / "data"))),
                )


if __name__ == "__main__":
    unittest.main()
