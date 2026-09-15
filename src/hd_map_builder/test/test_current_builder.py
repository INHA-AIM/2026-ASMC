import json
import csv
from pathlib import Path
import tempfile
import unittest

import yaml

from hd_map_builder.current_builder import build_current_corridor


class CurrentBuilderIntegrationTest(unittest.TestCase):
    def test_current_baseline_rejects_observed_or_legacy_source_key(self):
        repo_root = Path(__file__).resolve().parents[3]
        config_path = repo_root / "src/hd_map_builder/config/kcity2025_competition_corridor.yaml"
        config = yaml.safe_load(config_path.read_text())
        config["sources"]["legacy_lane_boundary"] = "not_allowed.json"
        with tempfile.TemporaryDirectory() as raw_output:
            temporary = Path(raw_output)
            modified_config = temporary / "config.yaml"
            modified_config.write_text(yaml.safe_dump(config), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "current-only source contract"):
                build_current_corridor(modified_config, repo_root, temporary / "output")

    def test_workspace_current_only_baseline(self):
        repo_root = Path(__file__).resolve().parents[3]
        config = repo_root / "src/hd_map_builder/config/kcity2025_competition_corridor.yaml"
        with tempfile.TemporaryDirectory() as raw_output:
            output = Path(raw_output)
            summary = build_current_corridor(config, repo_root, output)
            self.assertEqual(summary["route"]["link_count"], 39)
            self.assertEqual(summary["corridor"]["high_speed_parallel_link_count"], 32)
            self.assertEqual(summary["corridor"]["high_speed_same_direction_link_count"], 30)
            self.assertEqual(summary["corridor"]["high_speed_opposite_direction_link_count"], 2)
            self.assertEqual(summary["stoplines"]["route_stop_node_count"], 13)
            self.assertFalse(summary["source_contract"]["observed_or_legacy_inputs_used"])
            self.assertEqual(
                set(summary["source_contract"]["source_records"]),
                {
                    "current_global_info", "current_link_set", "current_node_set",
                    "route_links", "organizer_route",
                },
            )
            candidate = json.loads((output / "nominal_candidate_map.json").read_text())
            self.assertFalse(candidate["build"]["operational"])
            self.assertFalse(candidate["build"]["observed_or_legacy_inputs_used"])
            feature_ids = [feature["id"] for feature in candidate["features"]]
            self.assertEqual(len(feature_ids), len(set(feature_ids)))
            self.assertTrue(any(
                feature["feature_type"] == "lane_boundary"
                and feature["attributes"]["geometry_status"] == "link_nominal"
                for feature in candidate["features"]
            ))
            self.assertTrue(all(
                feature["status"] == "candidate" for feature in candidate["features"]
            ))
            self.assertTrue(all(
                feature["provenance"]["source_kind"] in {
                    "current_mgeo", "link_width_proxy",
                }
                for feature in candidate["features"]
            ))
            with (output / "corridor_links.csv").open(newline="") as stream:
                rows = list(csv.DictReader(stream))
            high_speed_only = [
                row for row in rows
                if row["geometry_scope"] == "high_speed_route_corridor_clip"
            ]
            self.assertTrue(high_speed_only)
            self.assertTrue(all(
                float(row["route_distance_p95_m"]) <= 15.0
                for row in high_speed_only
            ))


if __name__ == "__main__":
    unittest.main()
