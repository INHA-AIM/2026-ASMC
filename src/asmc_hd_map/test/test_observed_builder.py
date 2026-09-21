import csv
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from asmc_hd_map.observed_builder import (
    _combine_profiles,
    _connected_boundary_pairs,
    build_observed_calibration,
)


def _hash(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class ObservedBuilderIntegrationTest(unittest.TestCase):
    def test_topology_transition_preserves_boundary_side(self):
        parent = {"features": [
            {
                "feature_type": "link_centerline", "topology": {
                    "current_link_ids": ["a"], "predecessor_link_ids": [],
                    "successor_link_ids": ["b"],
                },
            },
            {
                "feature_type": "link_centerline", "topology": {
                    "current_link_ids": ["b"], "predecessor_link_ids": ["a"],
                    "successor_link_ids": [],
                },
            },
        ]}
        boundaries = [
            {"id": "a_left", "topology": {"current_link_ids": ["a"]}, "attributes": {"link_sides": {"a": "left"}}},
            {"id": "b_left", "topology": {"current_link_ids": ["b"]}, "attributes": {"link_sides": {"b": "left"}}},
            {"id": "b_right", "topology": {"current_link_ids": ["b"]}, "attributes": {"link_sides": {"b": "right"}}},
        ]
        pairs = _connected_boundary_pairs(parent, boundaries)
        self.assertIn(frozenset(("a_left", "b_left")), pairs)
        self.assertNotIn(frozenset(("a_left", "b_right")), pairs)

    def test_disagreeing_sources_are_not_automatically_applied(self):
        config = {"profile": {"station_bin_m": 1.0, "maximum_source_disagreement_p95_m": 0.30}}
        reports = [
            {"match_status": "automatic", "reason": "passed"},
            {"match_status": "automatic", "reason": "passed"},
        ]
        candidates = [
            {
                "parent_boundary_id": "boundary", "report_index": index,
                "source_file": f"source_{index}.csv", "source_kind": "current_gps_direct",
                "lane_color": "WHITE", "profile": [
                    {"bin_index": bin_id, "station_m": float(bin_id), "offset_m": offset, "support": 4}
                    for bin_id in range(6)
                ],
            }
            for index, offset in enumerate((0.0, 0.5))
        ]
        combined, rows = _combine_profiles(candidates, config, reports)
        self.assertEqual(combined["boundary"], [])
        self.assertTrue(all(row["conflict"] and not row["applied"] for row in rows))
        self.assertTrue(all(row["match_status"] == "review_required" for row in reports))

    def test_workspace_observed_calibration_contract_and_determinism(self):
        repo_root = Path(__file__).resolve().parents[3]
        config = repo_root / "src/asmc_hd_map/config/kcity2025_observed_calibration.yaml"
        with tempfile.TemporaryDirectory() as first_raw, tempfile.TemporaryDirectory() as second_raw:
            first = Path(first_raw)
            second = Path(second_raw)
            summary = build_observed_calibration(config, repo_root, first)
            build_observed_calibration(config, repo_root, second)
            self.assertEqual(summary["input_contract"]["source_count"], 19)
            self.assertEqual(summary["input_contract"]["total_rows"], 26511)
            self.assertEqual(summary["input_contract"]["direct_rows"], 25414)
            self.assertEqual(summary["input_contract"]["current_lidar_semantic_rows"], 13679)
            self.assertEqual(summary["input_contract"]["current_gps_direct_rows"], 11735)
            self.assertEqual(summary["input_contract"]["generated_audit_only_rows"], 958)
            self.assertEqual(summary["input_contract"]["interpolated_audit_only_rows"], 139)
            self.assertFalse(summary["input_contract"]["manual_bridges_used"])
            self.assertEqual(summary["input_contract"]["raster_inward_offset_m"], 0.0)
            self.assertTrue(summary["candidate_map"]["route_link_count_unchanged"])
            self.assertTrue(summary["candidate_map"]["stopline_count_unchanged"])
            self.assertGreater(summary["candidate_map"]["calibrated_interval_count"], 0)

            candidate = json.loads((first / "observed_candidate_map.json").read_text())
            self.assertFalse(candidate["build"]["operational"])
            self.assertFalse(candidate["build"]["legacy_katri_inputs_used"])
            self.assertFalse(candidate["build"]["manual_bridges_used"])
            self.assertEqual(len([f for f in candidate["features"] if f["feature_type"] == "link_centerline"]), 70)
            self.assertEqual(len([f for f in candidate["features"] if f["feature_type"] == "stopline"]), 13)
            self.assertTrue(all(feature["status"] == "candidate" for feature in candidate["features"]))
            self.assertTrue(all(feature["provenance"]["source_kind"] in {
                "current_mgeo", "link_width_proxy", "current_lidar_semantic", "current_gps_direct",
            } for feature in candidate["features"]))

            with (first / "source_audit.csv").open(newline="") as stream:
                source_rows = list(csv.DictReader(stream))
            self.assertEqual(len(source_rows), 19)
            self.assertTrue(all(int(row["fit_support_rows"]) == int(row["direct_rows"]) for row in source_rows))
            self.assertTrue(all(int(row["manual_bridge_rows"]) == 0 for row in source_rows))
            self.assertTrue(all(float(row["canonical_inward_offset_m"]) == 0.0 for row in source_rows))

            for filename in (
                "summary.json", "observed_candidate_map.json", "source_audit.csv",
                "segment_matches.csv", "boundary_offset_profiles.csv", "link_width_profiles.csv",
                "overlays/full_corridor.png", "overlays/high_speed.png",
                "overlays/shadow_and_static.png", "overlays/review_required.png",
            ):
                self.assertEqual(_hash(first / filename), _hash(second / filename), filename)

    def test_parent_station_intervals_cover_without_length_overlap(self):
        repo_root = Path(__file__).resolve().parents[3]
        config = repo_root / "src/asmc_hd_map/config/kcity2025_observed_calibration.yaml"
        with tempfile.TemporaryDirectory() as raw_output:
            output = Path(raw_output)
            build_observed_calibration(config, repo_root, output)
            candidate = json.loads((output / "observed_candidate_map.json").read_text())
            parent = json.loads((
                repo_root / "src/asmc_hd_map/reports/kcity2025_current_corridor_20260916/nominal_candidate_map.json"
            ).read_text())
            parent_lengths = {}
            for feature in parent["features"]:
                if feature["feature_type"] != "lane_boundary":
                    continue
                points = feature["geometry"]["coordinates"]
                parent_lengths[feature["id"]] = sum(
                    ((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2) ** 0.5
                    for a, b in zip(points[:-1], points[1:])
                )
            intervals = {}
            for feature in candidate["features"]:
                if feature["feature_type"] != "lane_boundary":
                    continue
                attributes = feature["attributes"]
                intervals.setdefault(attributes["parent_boundary_id"], []).append((
                    float(attributes["parent_station_start_m"]),
                    float(attributes["parent_station_end_m"]),
                ))
            self.assertEqual(len(intervals), 85)
            for parent_id, values in intervals.items():
                ordered = sorted(values)
                self.assertAlmostEqual(ordered[0][0], 0.0, places=6, msg=parent_id)
                for first, second in zip(ordered[:-1], ordered[1:]):
                    self.assertAlmostEqual(first[1], second[0], places=6, msg=parent_id)
                self.assertAlmostEqual(ordered[-1][1], parent_lengths[parent_id], places=6, msg=parent_id)


if __name__ == "__main__":
    unittest.main()
