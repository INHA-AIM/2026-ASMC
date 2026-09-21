import csv
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from asmc_hd_map.review_bundle import build_candidate_review, load_review_layers


def _hash(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class CandidateReviewIntegrationTest(unittest.TestCase):
    def setUp(self):
        self.repo_root = Path(__file__).resolve().parents[3]
        self.config = (
            self.repo_root / "src/asmc_hd_map/config/kcity2025_candidate_review.yaml"
        )

    def test_high_speed_lane_change_inventory_is_retained(self):
        layers = load_review_layers(self.config, self.repo_root)
        with tempfile.TemporaryDirectory() as raw_output:
            output = Path(raw_output)
            summary = build_candidate_review(self.config, self.repo_root, output)
            inventory = summary["high_speed_inventory"]
            self.assertEqual(inventory["parallel_current_link_count"], 32)
            self.assertEqual(inventory["same_direction_count"], 30)
            self.assertEqual(inventory["opposite_direction_count"], 2)
            self.assertEqual(inventory["lane_change_reachable_link_count"], 25)
            self.assertTrue(inventory["all_lane_change_reachable_links_present"])
            self.assertEqual(
                inventory["centerline_only_boundary_review_link_ids"],
                ["A2256W000010"],
            )
            self.assertEqual(
                inventory["lane_change_reachable_with_observed_calibration_count"], 21,
            )
            self.assertEqual(
                set(inventory["lane_change_reachable_without_observed_calibration_link_ids"]),
                {"A2256W000179", "A2256W000424", "A2256W000426", "A2256W000427"},
            )
            self.assertEqual(inventory["legacy_high_speed_candidate_count"], 64)
            self.assertEqual(inventory["legacy_high_speed_tiers"], {"A": 56, "B": 8})
            self.assertFalse(summary["operational"])
            self.assertEqual(len(summary["review_config_sha256"]), 64)
            self.assertTrue(summary["geometry_usage"]["legacy_geometry_merged_into_candidate_map"])
            self.assertTrue(summary["geometry_usage"]["legacy_geometry_shown_as_review_reference"])
            self.assertTrue(summary["geometry_usage"]["reviewed_legacy_adoption_candidate_available"])
            self.assertTrue(summary["geometry_usage"]["current_reconstructed_geometry_used"])
            self.assertEqual(summary["geometry_usage"]["painted_stoplines_after_driving_review"], 10)
            self.assertTrue(summary["geometry_usage"]["observed_geometry_demoted_to_validation_reference"])
            self.assertFalse(summary["geometry_usage"]["observed_calibrated_geometry_used"])
            self.assertTrue(summary["geometry_usage"]["observed_calibrated_geometry_shown_as_validation_reference"])
            self.assertTrue(summary["transfer_recommendation"]["current_topology_remains_authoritative"])
            self.assertEqual(
                summary["transfer_recommendation"]["stable_high_speed_legacy_tier_a"],
                "adopted_in_non_operational_hybrid_candidate",
            )
            self.assertGreater(len(layers["legacy_references"]), 64)
            self.assertGreater(len(layers["adopted_legacy_boundaries"]), 100)
            self.assertTrue(layers["changed_legacy_candidates"])

            with (output / "high_speed_inventory.csv").open(newline="") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 32)
            self.assertEqual(sum(row["lane_change_reachable"] == "True" for row in rows), 25)
            self.assertTrue(all(0.0 <= float(row["observed_boundary_coverage_ratio"]) <= 1.0 for row in rows))

    def test_review_outputs_are_deterministic(self):
        with tempfile.TemporaryDirectory() as first_raw, tempfile.TemporaryDirectory() as second_raw:
            first, second = Path(first_raw), Path(second_raw)
            build_candidate_review(self.config, self.repo_root, first)
            build_candidate_review(self.config, self.repo_root, second)
            files = ["summary.json", "high_speed_inventory.csv"] + [
                f"overlays/{name}.png" for name in (
                    "competition_corridor", "high_speed_01_entry", "high_speed_02_north",
                    "high_speed_03_middle_north", "high_speed_04_middle_south",
                    "high_speed_05_south", "high_speed_06_exit", "shadow",
                    "static_obstacle",
                )
            ]
            for filename in files:
                self.assertEqual(_hash(first / filename), _hash(second / filename), filename)

            summary = json.loads((first / "summary.json").read_text())
            agreement = summary["observed_legacy_high_speed_agreement"]
            self.assertFalse(agreement["promotion_gate"])
            self.assertGreater(agreement["comparable_interval_count"], 0)
            self.assertGreater(agreement["matched_within_3m_ratio"], 0.0)

    def test_three_source_comparison_is_independent_and_corridor_filtered(self):
        config = (
            self.repo_root
            / "src/asmc_hd_map/config/kcity2025_three_source_compare.yaml"
        )
        layers = load_review_layers(config, self.repo_root)
        self.assertEqual(
            layers["source_hashes"]["legacy_adopted_map"],
            "b424c56c310557fd8fbe6388c9320eae8dff0cf545d4e67e998a803239328be4",
        )
        self.assertEqual(
            layers["source_hashes"]["won_tae_h5"],
            "e8ce47f504cd0387686718dfe72f12d17e8c684d0cc23fe41d9938171d896e86",
        )
        self.assertEqual(len(layers["adopted_legacy_boundaries"]), 164)
        self.assertEqual(len(layers["legacy_references"]), 127)
        corridor_ids = {
            str(feature["topology"]["current_link_ids"][0])
            for feature in layers["current_links"]
        }
        self.assertEqual(len(corridor_ids), 70)
        self.assertTrue(all(
            set(feature["target_current_link_ids"]) & corridor_ids
            for feature in layers["legacy_references"]
        ))
        high_speed = json.loads(
            layers["source_paths"]["high_speed_legacy_index"].read_text()
        )
        high_speed_ab_ids = {
            str(feature["legacy_id"])
            for feature in high_speed["features"]
            if feature.get("tier") in {"A", "B"}
        }
        selected_ids = {feature["id"] for feature in layers["legacy_references"]}
        self.assertEqual(len(high_speed_ab_ids), 64)
        self.assertTrue(high_speed_ab_ids.issubset(selected_ids))
        self.assertEqual(
            sum(feature["lane_type"] == "525" for feature in layers["legacy_references"]),
            3,
        )
        self.assertNotIn(
            "kcity2025_operational_map_20260916",
            str(layers["source_paths"]["legacy_adopted_map"]),
        )


if __name__ == "__main__":
    unittest.main()
