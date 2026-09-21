import csv
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from asmc_hd_map.operational_map import (
    _anchor_registry,
    _lane_semantics,
    _verify_file,
    build_lane_areas,
    build_observation_anchored_boundaries,
    build_operational_map,
    build_reviewed_stoplines,
)


def _hash(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _nominal_document() -> dict:
    return {
        "features": [{
            "id": "nominal:lane_boundary:L0:left",
            "feature_type": "lane_boundary",
            "geometry": {"type": "LineString", "coordinates": [[0, 1.5, 0], [10, 1.5, 0]]},
            "attributes": {
                "link_sides": {"L0": "left"},
                "nominal_width_start_m": [3.0],
                "nominal_width_end_m": [3.0],
            },
            "topology": {"current_link_ids": ["L0"]},
            "provenance": {"source_sha256": "nominal-sha"},
        }],
    }


class OperationalMapGeometryTest(unittest.TestCase):
    def test_observation_offset_is_normal_and_continuous(self):
        profile = {
            "nominal:lane_boundary:L0:left": [
                {"station_m": float(value), "offset_m": 0.2, "support": 5,
                 "lane_color": "yellow", "source_files": "direct.csv",
                 "source_kinds": "current_gps_direct"}
                for value in range(11)
            ],
        }
        boundaries, _ = build_observation_anchored_boundaries(
            _nominal_document(), profile, map_id="candidate", sample_step_m=0.5,
            max_gap_m=2.0, observed_source_hash="observed-sha",
        )
        points = np.asarray(boundaries[0]["geometry"]["coordinates"])
        np.testing.assert_allclose(points[:, 1], 1.7, atol=1e-9)
        self.assertEqual(boundaries[0]["attributes"]["link_sides"], {"L0": "left"})
        self.assertGreater(boundaries[0]["attributes"]["observed_fraction"], 0.99)

    def test_collection_side_never_becomes_canonical_side(self):
        nominal = _nominal_document()
        snapshot = {"files": [{
            "path": "x/direct.csv", "sha256": "abc", "role": "direct_observation",
        }]}
        with tempfile.TemporaryDirectory() as raw:
            path = Path(raw) / "matches.csv"
            with path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=[
                    "segment_id", "run_index", "source_file", "source_kind",
                    "collection_side", "lane_color", "parent_boundary_id",
                    "station_start_m", "station_end_m", "observed_length_m",
                    "coverage", "nominal_distance_p95_m", "fit_residual_p95_m", "match_status",
                ])
                writer.writeheader()
                writer.writerow({
                    "segment_id": "s", "run_index": 0, "source_file": "direct.csv",
                    "source_kind": "current_gps_direct", "collection_side": "RIGHT",
                    "lane_color": "YELLOW", "parent_boundary_id": "nominal:lane_boundary:L0:left",
                    "station_start_m": 0, "station_end_m": 10, "observed_length_m": 10,
                    "coverage": 1, "nominal_distance_p95_m": 0.1,
                    "fit_residual_p95_m": 0.02, "match_status": "automatic",
                })
            row = _anchor_registry(path, nominal, snapshot)[0]
        self.assertEqual(row["collection_side_audit_only"], "RIGHT")
        self.assertEqual(json.loads(row["canonical_link_sides"]), {"L0": "left"})

    def test_broken_paint_is_a_semantic_not_a_geometry_gap(self):
        feature = {"attributes": {"lane_color": "white", "lane_shape": "Broken"}}
        self.assertEqual(_lane_semantics(feature), ("white", "broken", "single"))

    def test_variable_three_to_four_meter_lane_area_does_not_cross(self):
        link = {
            "id": "map:link_centerline:L", "feature_type": "link_centerline",
            "geometry": {"type": "LineString", "coordinates": [[0, 0, 0], [10, 0, 0]]},
            "attributes": {
                "nominal_width_start_m": 3.5, "corridor_roles": ["official_route"],
                "route_s_min_m": 0.0, "route_s_max_m": 10.0,
            },
            "topology": {"current_link_ids": ["L"], "is_merge_target": False, "is_split_source": False},
        }
        boundaries = [{
            "id": "map:lane_boundary:L:left", "feature_type": "lane_boundary",
            "geometry": {"type": "LineString", "coordinates": [[0, 1.5, 0], [10, 2.0, 0]]},
            "attributes": {"link_sides": {"L": "left"}}, "topology": {"current_link_ids": ["L"]},
        }, {
            "id": "map:lane_boundary:L:right", "feature_type": "lane_boundary",
            "geometry": {"type": "LineString", "coordinates": [[0, -1.5, 0], [10, -2.0, 0]]},
            "attributes": {"link_sides": {"L": "right"}}, "topology": {"current_link_ids": ["L"]},
        }]
        widths = {"L": [
            {"station_m": 0.0, "width_m": 3.0, "status": "both_sides_observed_calibrated"},
            {"station_m": 10.0, "width_m": 4.0, "status": "both_sides_observed_calibrated"},
        ]}
        config = {"geometry": {
            "sample_step_m": 0.5, "fallback_width_m": 3.5,
            "sanity_width_min_m": 2.5, "sanity_width_max_m": 4.5,
            "width_smoothing_length_m": 5.0,
        }}
        areas, profiles, _ = build_lane_areas([link], boundaries, widths, config, "map", "hash")
        self.assertEqual(len(areas), 1)
        self.assertAlmostEqual(profiles[0]["raw_width_m"], 3.0, delta=0.02)
        self.assertAlmostEqual(profiles[-1]["raw_width_m"], 4.0, delta=0.02)
        polygon = np.asarray(areas[0]["geometry"]["coordinates"][0])
        midpoint = len(polygon) // 2
        self.assertTrue(np.all(polygon[:midpoint, 1] > 0.0))
        self.assertTrue(np.all(polygon[midpoint:-1, 1] < 0.0))

    def test_stopline_allowlist_and_rejection(self):
        nodes = [
            {"idx": "N_KEEP", "point": [5, 0, 0], "traffic_light_id": "TL"},
            {"idx": "N_DROP", "point": [8, 0, 0], "traffic_light_id": ""},
        ]
        links = [{
            "idx": "L0", "from_node_idx": "N0", "to_node_idx": "N_KEEP",
            "points": [[0, 0, 0], [5, 0, 0]], "width_start": 3.0, "width_end": 4.0,
        }, {
            "idx": "L1", "from_node_idx": "N_KEEP", "to_node_idx": "N_DROP",
            "points": [[5, 0, 0], [8, 0, 0]], "width_start": 3.5, "width_end": 3.5,
        }]
        config = {"stoplines": {
            "h5_allowlist": ["N_KEEP", "N_DROP"],
            "driving_review_allowlist": {},
            "rejected": {"N_DROP": "not_painted"},
        }}
        features, audit = build_reviewed_stoplines(
            nodes, links, config, "candidate", "node-sha",
            np.asarray([[0, 0, 0], [10, 0, 0]], dtype=np.float64),
        )
        self.assertEqual([feature["attributes"]["node_ids"] for feature in features], [["N_KEEP"]])
        self.assertEqual({row["decision"] for row in audit}, {"kept", "rejected"})

    def test_source_hash_mismatch_fails_closed(self):
        with tempfile.TemporaryDirectory() as raw:
            root = Path(raw)
            (root / "source.txt").write_text("value", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "source hash changed"):
                _verify_file(root, {"path": "source.txt", "sha256": "0" * 64})


class OperationalMapWorkspaceTest(unittest.TestCase):
    def test_workspace_build_contract_and_determinism(self):
        repo_root = Path(__file__).resolve().parents[3]
        config = repo_root / "src/asmc_hd_map/config/kcity2025_operational_map.yaml"
        with tempfile.TemporaryDirectory() as first_raw, tempfile.TemporaryDirectory() as second_raw:
            first, second = Path(first_raw), Path(second_raw)
            summary = build_operational_map(config, repo_root, first)
            build_operational_map(config, repo_root, second)
            self.assertFalse(summary["operational"])
            self.assertEqual(summary["source_snapshot"]["verified_file_count"], 21)
            self.assertEqual(summary["counts"]["current_link_centerline"], 70)
            self.assertEqual(summary["counts"]["lane_area"], 70)
            self.assertEqual(summary["corridor_roles"]["high_speed_same_direction"], 30)
            self.assertEqual(summary["corridor_roles"]["high_speed_opposite_direction"], 2)
            self.assertEqual(summary["counts"]["stopline_kept"], 8)
            self.assertEqual(summary["counts"]["stopline_rejected"], 1)
            self.assertGreater(summary["counts"]["legacy_template_aligned_count"], 0)
            self.assertEqual(
                summary["counts"]["legacy_template_count"],
                summary["counts"]["legacy_template_aligned_count"]
                + summary["counts"]["legacy_template_driving_reviewed_count"]
                + summary["counts"]["legacy_review_only"],
            )

            document = json.loads((first / "operational_candidate_map.json").read_text())
            self.assertFalse(document["build"]["operational"])
            areas = [feature for feature in document["features"] if feature["feature_type"] == "lane_area"]
            same_direction = [
                area for area in areas
                if "high_speed_same_direction" in area["attributes"]["route_maneuver_roles"]
            ]
            opposite = [
                area for area in areas
                if "high_speed_opposite_direction" in area["attributes"]["route_maneuver_roles"]
            ]
            self.assertEqual(len(same_direction), 30)
            self.assertEqual(len(opposite), 2)
            stop_ids = {
                feature["attributes"]["node_ids"][0]
                for feature in document["features"] if feature["feature_type"] == "stopline"
            }
            self.assertEqual(stop_ids, {
                "A1256W000682", "A1256W000807", "A1256W000208", "A1256W000757",
                "A1256W000582", "A1256W000593", "A1256W000709", "A1256W000736",
            })
            self.assertNotIn("A1256W000742", stop_ids)
            stop_by_id = {
                feature["attributes"]["node_ids"][0]: feature
                for feature in document["features"] if feature["feature_type"] == "stopline"
            }
            self.assertEqual(
                stop_by_id["A1256W000736"]["validation"]["selection_source"],
                "live_driving_review",
            )
            with (first / "legacy_template_audit.csv").open(encoding="utf-8") as stream:
                legacy_audit = list(csv.DictReader(stream))
            reviewed_start_markings = [
                row for row in legacy_audit
                if "B219BS010411" in row["source_feature_id"]
                or "B219BS010474" in row["source_feature_id"]
                or "B219BS011269" in row["source_feature_id"]
            ]
            self.assertEqual(len(reviewed_start_markings), 3)
            self.assertTrue(all(
                row["decision"] == "driving_review_approved"
                for row in reviewed_start_markings
            ))
            start_features = [
                feature for feature in document["features"]
                if feature["feature_type"] == "lane_marking"
                and feature["attributes"].get("driving_review_interval")
                == "start_to_first_stopline"
            ]
            self.assertEqual(len(start_features), 3)
            right = [feature for feature in start_features if feature["attributes"]["color"] == "white"]
            self.assertEqual(len(right), 2)
            self.assertTrue(all(feature["attributes"]["pattern"] == "solid" for feature in right))
            self.assertTrue(all(feature["validation"]["driving_review_approved"] for feature in start_features))
            self.assertTrue(all(
                feature["topology"]["current_link_ids"] == ["A2256W000748"]
                for feature in start_features
            ))
            reviewed_ids = {feature["attributes"]["legacy_source_id"] for feature in start_features}
            self.assertEqual(reviewed_ids, {"B219BS010411", "B219BS010474", "B219BS011269"})
            boundary_by_id = {
                feature["id"]: feature for feature in document["features"]
                if feature["feature_type"] == "lane_boundary"
            }
            by_source = {
                feature["attributes"]["legacy_source_id"]: feature for feature in start_features
            }
            for source_id in ("B219BS010411", "B219BS010474"):
                feature = by_source[source_id]
                boundary = boundary_by_id[feature["attributes"]["logical_boundary_id"]]
                np.testing.assert_allclose(
                    np.asarray(feature["geometry"]["coordinates"])[0, :2],
                    np.asarray(boundary["geometry"]["coordinates"])[0, :2],
                    atol=1e-9,
                )
            for source_id in ("B219BS010411", "B219BS011269"):
                feature = by_source[source_id]
                boundary = boundary_by_id[feature["attributes"]["logical_boundary_id"]]
                np.testing.assert_allclose(
                    np.asarray(feature["geometry"]["coordinates"])[-1, :2],
                    np.asarray(boundary["geometry"]["coordinates"])[-1, :2],
                    atol=1e-9,
                )
            final_legacy = [
                feature for feature in document["features"]
                if feature["feature_type"] == "lane_marking"
                and feature["provenance"]["source_kind"] == "legacy_katri"
            ]
            self.assertEqual(
                len(final_legacy),
                summary["counts"]["legacy_template_aligned_count"]
                + summary["counts"]["legacy_template_driving_reviewed_count"],
            )
            self.assertEqual(
                len(document["review_layers"]["legacy_template_review_required"]),
                summary["counts"]["legacy_review_only"],
            )

            arrays = np.load(first / "static_layers.npz")
            forbidden = arrays["forbidden_surface_mask"] > 0
            self.assertGreater(int(forbidden.sum()), 0)
            self.assertEqual(int(arrays["road_surface_mask"][forbidden].sum()), 0)
            self.assertEqual(int(arrays["maneuver_surface_mask"][forbidden].sum()), 0)
            self.assertGreater(int(arrays["maneuver_surface_mask"].sum()), 0)

            for filename in (
                "summary.json", "source_snapshot.json", "operational_candidate_map.json",
                "observation_anchor_registry.csv", "legacy_template_audit.csv",
                "lane_width_profiles.csv", "static_layers.npz", "overlays/full_corridor.png",
                "overlays/high_speed.png", "overlays/shadow.png", "overlays/static_obstacle.png",
                "overlays/reviewed_start_to_first_stopline.png",
            ):
                self.assertEqual(_hash(first / filename), _hash(second / filename), filename)


if __name__ == "__main__":
    unittest.main()
