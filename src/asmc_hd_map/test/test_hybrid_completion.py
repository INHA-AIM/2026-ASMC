from __future__ import annotations

import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from asmc_hd_map.geometry import cumulative_distance
from asmc_hd_map.hybrid_completion import (
    _feature_points,
    _route_samples,
    build_hybrid_completion,
)


def _hash(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _route_cross_section(document: dict, route_path: Path, station_m: float) -> list[dict]:
    route, tangents, stations = _route_samples(route_path, 0.25)
    index = int(np.argmin(np.abs(stations - station_m)))
    point = route[index, :2]
    tangent = tangents[index]
    normal = np.asarray([-tangent[1], tangent[0]])
    rows = []
    for feature in document["features"]:
        if feature["feature_type"] != "lane_boundary":
            continue
        points = _feature_points(feature)[:, :2]
        nearest = int(np.argmin(np.linalg.norm(points - point, axis=1)))
        delta = points[nearest] - point
        lateral = float(delta @ normal)
        longitudinal = float(delta @ tangent)
        if abs(lateral) <= 3.0 and abs(longitudinal) <= 1.0:
            rows.append({
                "lateral": lateral,
                "color": feature["attributes"]["lane_color"],
                "status": feature["attributes"]["geometry_status"],
            })
    return rows


class HybridCompletionTest(unittest.TestCase):
    def test_workspace_hybrid_contract_and_determinism(self):
        repo_root = Path(__file__).resolve().parents[3]
        config = repo_root / "src/asmc_hd_map/config/kcity2025_hybrid_completion.yaml"
        with tempfile.TemporaryDirectory() as first_raw, tempfile.TemporaryDirectory() as second_raw:
            first, second = Path(first_raw), Path(second_raw)
            summary = build_hybrid_completion(config, repo_root, first)
            build_hybrid_completion(config, repo_root, second)

            self.assertFalse(summary["operational"])
            self.assertEqual(summary["counts"]["current_link_center_count"], 70)
            self.assertEqual(summary["counts"]["route_reconstructed_boundary_count"], 12)
            self.assertEqual(summary["counts"]["high_speed_missing_shared_boundary_count"], 6)
            self.assertEqual(summary["counts"]["stopline_input_count"], 13)
            self.assertEqual(summary["counts"]["stopline_kept_count"], 10)
            self.assertGreater(summary["clipping"]["legacy_sample_points_removed"], 0)

            document = json.loads((first / "hybrid_candidate_map.json").read_text())
            self.assertFalse(document["build"]["operational"])
            stoplines = [feature for feature in document["features"] if feature["feature_type"] == "stopline"]
            rejected = {"A1256W000742", "A1256W000831", "A1256W000842"}
            self.assertEqual(len(stoplines), 10)
            self.assertFalse(any(
                rejected.intersection(feature["attributes"]["route_node_ids"])
                for feature in stoplines
            ))

            rebuilt = [
                feature for feature in document["features"]
                if feature["feature_type"] == "lane_boundary"
                and str(feature.get("attributes", {}).get("geometry_status", "")).startswith("current_reconstructed")
            ]
            route = [feature for feature in rebuilt if feature["attributes"]["geometry_status"] == "current_reconstructed_route_gap"]
            shared = [feature for feature in rebuilt if feature["attributes"]["geometry_status"] == "current_reconstructed_high_speed_lane1_shared"]
            self.assertEqual(len(route), 12)
            self.assertEqual(len(shared), 6)
            self.assertEqual(
                {frozenset(feature["topology"]["current_link_ids"]) for feature in shared},
                {
                    frozenset(("A2256W000418", "A2256W000429")),
                    frozenset(("A2256W000431", "A2256W000433")),
                    frozenset(("A2256W000435", "A2256W000421")),
                    frozenset(("A2256W000423", "A2256W000425")),
                    frozenset(("A2256W000432", "A2256W000424")),
                    frozenset(("A2256W000427", "A2256W000426")),
                },
            )
            obstacle = [feature for feature in route if feature["attributes"]["route_interval_name"] == "obstacle_bypass_2025"]
            self.assertEqual({feature["attributes"]["lane_color"] for feature in obstacle}, {"white", "yellow"})
            self.assertLessEqual(
                max(feature["attributes"]["route_s_max_m"] for feature in obstacle),
                436.80,
            )
            pre_roundabout = [
                feature for feature in route
                if feature["attributes"]["route_interval_name"] == "pre_roundabout_observed_right_2025"
            ]
            self.assertEqual(len(pre_roundabout), 1)
            self.assertEqual(pre_roundabout[0]["attributes"]["side"], "right")
            self.assertEqual(
                pre_roundabout[0]["attributes"]["width_source"],
                "won_tae_direct_outer_edges_midpoint",
            )
            self.assertEqual(
                pre_roundabout[0]["provenance"]["source_kind"],
                "current_lidar_semantic",
            )
            self.assertGreater(pre_roundabout[0]["attributes"]["observation_common_bin_count"], 40)
            self.assertGreater(pre_roundabout[0]["attributes"]["derived_two_lane_width_median_m"], 7.0)
            self.assertLess(pre_roundabout[0]["attributes"]["derived_two_lane_width_median_m"], 8.0)
            self.assertAlmostEqual(
                pre_roundabout[0]["attributes"]["driving_review_inward_correction_m"],
                0.25,
            )
            self.assertAlmostEqual(
                pre_roundabout[0]["attributes"]["derived_shared_lateral_median_m"]
                - pre_roundabout[0]["attributes"]["raw_derived_shared_lateral_median_m"],
                0.25,
                delta=0.01,
            )
            high_entry = [
                feature for feature in route
                if feature["attributes"]["route_interval_name"] == "high_speed_entry_straight_2025"
            ]
            self.assertEqual(len(high_entry), 2)
            self.assertEqual({feature["attributes"]["lane_color"] for feature in high_entry}, {"white"})
            right_turn = [
                feature for feature in route
                if feature["attributes"]["route_interval_name"] == "post_dynamic_right_turn_broken_2025"
            ]
            self.assertEqual(len(right_turn), 2)
            self.assertEqual({feature["attributes"]["lane_color"] for feature in right_turn}, {"white"})
            self.assertEqual({feature["attributes"]["lane_shape"] for feature in right_turn}, {"broken"})
            self.assertEqual({feature["attributes"]["dash_length_m"] for feature in right_turn}, {3.0})
            self.assertEqual({feature["attributes"]["dash_gap_m"] for feature in right_turn}, {3.0})
            safety_zone = [
                feature for feature in route
                if feature["attributes"]["route_interval_name"]
                == "high_speed_safety_zone_observed_right_2025"
            ]
            self.assertEqual(len(safety_zone), 1)
            self.assertEqual(safety_zone[0]["attributes"]["side"], "right")
            self.assertEqual(safety_zone[0]["attributes"]["lane_color"], "white")

            route_path = repo_root / "R_KR_PG_KATRI/2026_molit_comp_global_path.txt"
            dynamic = _route_cross_section(document, route_path, 390.0)
            self.assertEqual([row["color"] for row in dynamic].count("yellow"), 1)
            self.assertEqual([row["color"] for row in dynamic].count("white"), 1)
            before_roundabout = _route_cross_section(document, route_path, 500.0)
            self.assertEqual([row["color"] for row in before_roundabout].count("yellow"), 2)
            self.assertEqual([row["color"] for row in before_roundabout].count("white"), 1)
            self.assertEqual(
                [row for row in before_roundabout if row["color"] == "white"][0]["status"],
                "current_reconstructed_route_gap",
            )
            roundabout_profile_start = _route_cross_section(document, route_path, 479.0)
            start_right = [row for row in roundabout_profile_start if row["color"] == "white"]
            self.assertEqual(len(start_right), 1)
            self.assertGreaterEqual(abs(start_right[0]["lateral"]), 1.39)
            after_stopline = _route_cross_section(document, route_path, 570.0)
            self.assertEqual(len(after_stopline), 2)
            self.assertEqual({row["color"] for row in after_stopline}, {"white"})
            self.assertEqual({row["status"] for row in after_stopline}, {"current_reconstructed_route_gap"})
            after_roundabout = _route_cross_section(document, route_path, 940.0)
            self.assertEqual(len(after_roundabout), 2)
            self.assertEqual({row["color"] for row in after_roundabout}, {"white"})
            safety_zone_section = _route_cross_section(document, route_path, 1650.0)
            route_right = [row for row in safety_zone_section if row["lateral"] < 0.0]
            self.assertEqual(len(route_right), 1)
            self.assertAlmostEqual(route_right[0]["lateral"], -1.71, delta=0.08)
            # Smooth reconstruction may curve, but it must never contain a
            # large endpoint jump caused by joining old and current maps.
            for feature in route:
                points = np.asarray(feature["geometry"]["coordinates"], dtype=np.float64)
                self.assertGreater(float(cumulative_distance(points)[-1]), 40.0)
                self.assertLess(float(np.max(np.linalg.norm(np.diff(points[:, :2], axis=0), axis=1))), 1.0)

            for filename in (
                "summary.json", "hybrid_candidate_map.json", "reconstruction_audit.csv",
                "link_width_prior_audit.csv", "stopline_review.csv",
                "overlays/obstacle_bypass_2025.png",
                "overlays/pre_roundabout_observed_right_2025.png",
                "overlays/post_dynamic_right_turn_broken_2025.png",
                "overlays/roundabout_exit_to_high_speed_2025.png",
                "overlays/high_speed_entry_straight_2025.png",
                "overlays/high_speed_03_middle_south.png",
                "overlays/high_speed_safety_zone_observed_right_2025.png",
                "overlays/tollgate_to_shadow_2025.png",
            ):
                self.assertEqual(_hash(first / filename), _hash(second / filename), filename)


if __name__ == "__main__":
    unittest.main()
