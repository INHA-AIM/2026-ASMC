import json
from pathlib import Path
import unittest

import yaml

from asmc_hd_map.review_bundle import load_review_layers


class DevelopmentVisualizationContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.repo_root = Path(__file__).resolve().parents[3]
        cls.package_root = cls.repo_root / "src/asmc_hd_map"
        cls.config_path = cls.package_root / "config/kcity2025_hd_map_source_comparison.yaml"
        cls.config = yaml.safe_load(cls.config_path.read_text(encoding="utf-8"))
        cls.layers = load_review_layers(cls.config_path, cls.repo_root)

    def test_three_sources_are_independent_and_final_is_primary(self):
        comparison = self.config["comparison"]
        self.assertTrue(comparison["sources_are_independent"])
        self.assertTrue(comparison["source_merging_forbidden"])
        self.assertTrue(comparison["runtime_publication_forbidden"])
        self.assertEqual(comparison["primary_source"], "final_curated")

    def test_latest_and_legacy_drivable_sources_are_explicit(self):
        latest = self.layers["source_paths"]["compact_drivable_grid"]
        self.assertEqual(
            latest.relative_to(self.package_root).as_posix(),
            "config/kcity2025_compact_drivable_grid.npz",
        )
        roles = self.config["source_roles"]
        self.assertTrue(roles["legacy_h5_bake"]["comparison_only"])
        self.assertIn("build_id", roles["latest_drivable"])

    def test_final_reviewed_marking_counts_are_frozen(self):
        path = self.layers["source_paths"]["curated_candidate_map"]
        document = json.loads(path.read_text(encoding="utf-8"))
        counts = {
            kind: sum(feature.get("feature_type") == kind for feature in document["features"])
            for kind in ("lane_marking", "stopline")
        }
        self.assertEqual(counts, {"lane_marking": 121, "stopline": 7})

    def test_won_tae_source_is_archived_inside_package(self):
        path = self.layers["source_paths"]["won_tae_h5"]
        self.assertEqual(
            path.relative_to(self.package_root).as_posix(),
            "development/won_tae_link_node/out/link_node_map.h5",
        )
        self.assertEqual(
            self.layers["source_hashes"]["won_tae_h5"],
            "e8ce47f504cd0387686718dfe72f12d17e8c684d0cc23fe41d9938171d896e86",
        )

    def test_legacy_layer_is_corridor_filtered(self):
        self.assertEqual(len(self.layers["legacy_references"]), 127)
        self.assertTrue(self.config["comparison"]["legacy_current_corridor_only"])

    def test_launch_and_rviz_use_development_only_topics(self):
        launch = (self.package_root / "launch/hd_map_source_comparison.launch").read_text(
            encoding="utf-8"
        )
        self.assertIn('<param name="comparison_primary" value="curated"/>', launch)
        self.assertIn('<param name="compare_drivable_sources" value="true"/>', launch)
        self.assertIn('<param name="topic_root" value="/asmc_hd_map/development"/>', launch)
        rviz = (self.package_root / "rviz/hd_map_source_comparison.rviz").read_text(
            encoding="utf-8"
        )
        for suffix in ("final_curated", "legacy_origin_aligned", "won_tae_h5"):
            self.assertIn("/asmc_hd_map/development/hd_map_sources/lanes/" + suffix, rviz)
        for suffix in ("latest", "legacy_h5_bake"):
            self.assertIn("/asmc_hd_map/development/hd_map_sources/drivable/" + suffix, rviz)
        self.assertIn('Name: "1 Latest compact drivable"', rviz)
        self.assertIn('Name: "2 Legacy H5 road bake"', rviz)

    def test_old_lane_only_entrypoint_was_replaced(self):
        self.assertFalse((self.package_root / "launch/lane_source_comparison.launch").exists())
        self.assertFalse((self.package_root / "rviz/lane_source_comparison.rviz").exists())


if __name__ == "__main__":
    unittest.main()
