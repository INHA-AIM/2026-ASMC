import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from asmc_hd_map.geometry import resample_polyline
from asmc_hd_map.legacy_adoption import (
    _close_short_false_runs,
    _station_is_approved,
    _true_runs,
    build_legacy_adoption,
)
from asmc_hd_map.spatial import GridIndex


def _hash(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class LegacyAdoptionTest(unittest.TestCase):
    def test_mask_helpers_are_conservative(self):
        mask = np.asarray([True, False, False, True, False, False, False, True])
        closed = _close_short_false_runs(mask, 2)
        self.assertEqual(closed.tolist(), [True, True, True, True, False, False, False, True])
        self.assertEqual(_true_runs(closed), [(0, 4), (7, 8)])
        self.assertEqual(
            _station_is_approved(5.0, [{"name": "stable", "start_s_m": 0, "end_s_m": 10}]),
            (True, "stable"),
        )
        self.assertEqual(
            _station_is_approved(12.0, [{"name": "stable", "start_s_m": 0, "end_s_m": 10}]),
            (False, None),
        )

    def test_workspace_adoption_contract_and_determinism(self):
        repo_root = Path(__file__).resolve().parents[3]
        config = repo_root / "src/asmc_hd_map/config/kcity2025_legacy_adoption.yaml"
        with tempfile.TemporaryDirectory() as first_raw, tempfile.TemporaryDirectory() as second_raw:
            first, second = Path(first_raw), Path(second_raw)
            summary = build_legacy_adoption(config, repo_root, first)
            build_legacy_adoption(config, repo_root, second)
            self.assertFalse(summary["operational"])
            self.assertEqual(summary["counts"]["current_link_center_count"], 70)
            self.assertEqual(summary["counts"]["current_stopline_count"], 13)
            self.assertGreater(summary["counts"]["adopted_legacy_source_lanes"], 100)
            self.assertGreater(summary["counts"]["rescued_tier_d_source_lanes"], 0)
            self.assertGreater(summary["counts"]["route_context_source_lanes"], 0)

            document = json.loads((first / "legacy_adopted_candidate_map.json").read_text())
            adopted = [f for f in document["features"] if f["feature_type"] == "lane_boundary"]
            self.assertTrue(adopted)
            self.assertTrue(all(f["status"] == "reviewed" for f in adopted))
            self.assertTrue(all(f["provenance"]["source_kind"] == "legacy_katri" for f in adopted))
            self.assertTrue(all(f["attributes"]["geometry_status"].startswith("legacy_validated") for f in adopted))
            self.assertTrue(all(f["attributes"]["lane_color"] != "blue" for f in adopted))
            self.assertTrue(all(530 not in f["attributes"]["mgeo"]["lane_type"] for f in adopted))
            context = [
                f for f in adopted
                if f["topology"]["binding_status"] == "route_context_only"
            ]
            self.assertTrue(context)
            self.assertTrue(all(not f["topology"]["current_link_ids"] for f in context))
            self.assertTrue(all(f["topology"]["route_context_link_ids"] for f in context))
            self.assertEqual(
                len([f for f in document["features"] if f["feature_type"] == "stopline"]),
                13,
            )
            self.assertTrue(document["review_layers"]["changed_region_legacy_candidates"])

            # Every high-speed lane-change centerline remains bracketed by adopted
            # physical boundaries even when ownership is assigned to its neighbor.
            inventory = json.loads((
                repo_root / "src/asmc_hd_map/reports/kcity2025_candidate_review_20260916/summary.json"
            ).read_text())
            self.assertTrue(inventory["high_speed_inventory"]["all_lane_change_reachable_links_present"])
            lane_points, lane_tangents = [], []
            for feature in adopted:
                points, tangents = resample_polyline(feature["geometry"]["coordinates"], 0.25)
                lane_points.extend(points)
                lane_tangents.extend(tangents)
            lane_index = GridIndex(np.asarray(lane_points), np.asarray(lane_tangents), cell_size_m=3.0)
            high_links = [
                f for f in document["features"]
                if f["feature_type"] == "link_centerline"
                and "high_speed_parallel" in f.get("attributes", {}).get("corridor_roles", [])
                and f.get("attributes", {}).get("relative_direction") == "same_route_direction"
            ]
            self.assertEqual(len(high_links), 30)
            for feature in high_links:
                points, tangents = resample_polyline(feature["geometry"]["coordinates"], 2.0)
                distances = [lane_index.query(point, tangent, 3.0, 30.0)[0] for point, tangent in zip(points, tangents)]
                # This is a coverage guard, not a fixed-width assertion. The
                # reviewed high-speed geometry includes wider taper sections.
                self.assertLessEqual(float(np.quantile(distances, 0.95)), 2.8, feature["id"])

            for filename in (
                "summary.json", "legacy_adopted_candidate_map.json", "adoption_inventory.csv",
                "overlays/full_corridor.png", "overlays/high_speed_to_tollgate.png",
                "overlays/shadow_and_tail.png",
            ):
                self.assertEqual(_hash(first / filename), _hash(second / filename), filename)


if __name__ == "__main__":
    unittest.main()
