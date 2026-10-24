import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from asmc_hd_map.geometry import cumulative_distance
from asmc_hd_map.route_curation import (
    _join_h5_vectors,
    _route_normal_constant_offset,
    build_route_curation,
)
from asmc_hd_map.spatial import GridIndex

try:
    import h5py  # noqa: F401
except ImportError:
    h5py = None


def _hash(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class RouteCurationTest(unittest.TestCase):
    def setUp(self):
        self.repo_root = Path(__file__).resolve().parents[3]
        self.config = self.repo_root / "src/asmc_hd_map/config/kcity2025_route_curation.yaml"

    @unittest.skipIf(h5py is None, "python3-h5py is a declared ROS runtime dependency")
    def test_only_explicitly_reviewed_segments_are_emitted(self):
        with tempfile.TemporaryDirectory() as raw:
            output = Path(raw)
            summary = build_route_curation(self.config, self.repo_root, output)
            document = json.loads((output / "route_curated_candidate_map.json").read_text())
            self.assertFalse(summary["operational"])
            self.assertTrue(summary["partial_candidate"])
            self.assertEqual(summary["implemented_segments"], [
                "stage01_start_legacy", "stage02_new_road_219_to_213",
                "stage03_static_obstacle_intersection_222",
                "stage04_link_202_and_following_intersection",
                "stage05_dynamic_obstacle_205_to_315",
                "stage06_right_turn_318",
                "stage07_post_turn_308",
                "stage08_guide_083",
                "stage09_two_lane_expansion_304",
                "stage10_roundabout_approach_148",
                "stage11_roundabout_exit_to_high_speed",
                "stage12_high_speed_entry_and_body",
                "stage13_high_speed_tail_to_333",
                "stage14_return_332_to_finish",
            ])
            self.assertEqual(summary["planned_segment_count"], 0)
            self.assertEqual(summary["selected_marking_count"], 127)
            self.assertTrue(summary["stoplines_included"])
            self.assertEqual(summary["selected_stopline_count"], 7)
            self.assertEqual(
                document["sources"]["current_link_set"]["sha256"],
                "5ce0fd57da07b83221c0a4d117b0606b2d9c094c8de476a7d1176c78a4dc5b0f",
            )
            self.assertTrue(all(
                feature["feature_type"] in {"lane_marking", "stopline"}
                and feature["provenance"]["source_kind"] in {
                    "legacy_katri", "current_lidar_semantic",
                    "current_mgeo", "current_gps_direct",
                }
                and feature["status"] == "reviewed"
                for feature in document["features"]
            ))
            selected_links = {
                link_id for feature in document["features"]
                for link_id in feature["topology"]["current_link_ids"]
            }
            self.assertTrue(selected_links.issubset({
                "A2256W000748", "A2256W000182", "A2256W000329", "A2256W000236",
                "A2256W000219", "A2256W000215", "A2256W000213",
                "A2256W000222", "A2256W000220",
                "A2256W000202", "A2256W000009", "A2256W000728",
                "A2256W000599", "A2256W000207",
                "A2256W000205", "A2256W000315", "A2256W000310",
                "A2256W000318", "A2256W000319",
                "A2256W000308", "A2256W000309", "A2256W000083",
                "A2256W000304", "A2256W000307",
                "A2256W000148", "A2256W000146", "A2256W000151",
                "A2256W000866", "A2256W000054", "A2256W000846",
                "A2256W000411", "A2256W000420", "A2256W000408",
                "A2256W000445", "A2256W000153",
                "A2256W000029", "A2256W000175", "A2256W000406", "A2256W000409",
                "A2256W000410", "A2256W000179", "A2256W000430",
                "A2256W000434", "A2256W000431", "A2256W000433",
                "A2256W000422", "A2256W000435", "A2256W000421",
                "A2256W001105", "A2256W000423", "A2256W000425",
                "A2256W000424", "A2256W000432",
                "A2256W000418", "A2256W000429", "A2256W000451",
                "A2256W000446", "A2256W000448", "A2256W000126",
                "A2256W000128", "A2256W000333", "A2256W000154",
                "A2256W000426", "A2256W000427", "A2256W000441",
                "A2256W000131", "A2256W000332", "A2256W000751",
            }))
            self.assertIn("A2256W000866", selected_links)
            self.assertNotIn("A2256W000057", selected_links)
            overridden = [
                feature for feature in document["features"]
                if feature["provenance"]["source_id"] == "B219BS010474"
                and feature["attributes"]["curation_segment_id"]
                == "stage01_start_legacy"
            ]
            self.assertGreater(len(overridden), 0)
            self.assertTrue(all(feature["attributes"]["pattern"] == "solid" for feature in overridden))
            self.assertTrue(all(
                feature["validation"]["attribute_override_ids"] == ["stage01_a748_right_solid"]
                for feature in overridden
            ))
            yellow_end = next(
                feature for feature in document["features"]
                if feature["provenance"]["source_id"] == "B219BS011297"
            )
            white_end = next(
                feature for feature in document["features"]
                if feature["provenance"]["source_id"] == "B219BS011301"
            )
            self.assertAlmostEqual(
                yellow_end["validation"]["route_s_max_m"],
                white_end["validation"]["route_s_max_m"], places=6,
            )
            self.assertGreater(white_end["validation"]["endpoint_extension_m"], 0.5)
            self.assertLess(white_end["validation"]["endpoint_extension_m"], 1.0)

            self.assertFalse(any(
                feature["provenance"]["source_id"] == "B219BS010180"
                for feature in document["features"]
            ))
            a182_guide = next(
                feature for feature in document["features"]
                if feature["attributes"].get("curation_segment_id")
                == "stage01_start_legacy"
                and feature["provenance"]["source_id"] == "A2256W000182"
            )
            self.assertEqual(a182_guide["attributes"]["color"], "white")
            self.assertEqual(a182_guide["attributes"]["pattern"], "broken")
            self.assertEqual(
                a182_guide["provenance"]["transform"],
                "current_link_station_varying_normal_offset_with_driving_review",
            )
            self.assertAlmostEqual(
                a182_guide["validation"]["signed_normal_offset_start_m"],
                2.090181009801611,
            )
            self.assertAlmostEqual(
                a182_guide["validation"]["signed_normal_offset_end_m"],
                1.3659621265996007,
            )

            h5_features = [
                feature for feature in document["features"]
                if feature["feature_type"] == "lane_marking"
                and feature["attributes"]["curation_segment_id"] == "stage02_new_road_219_to_213"
            ]
            self.assertEqual(len(h5_features), 5)
            self.assertEqual({
                tuple(feature["validation"]["h5_vector_indices"])
                for feature in h5_features
            }, {(5,), (6,), (21,), (22,), (23,)})
            self.assertTrue(all(
                not feature["validation"]["gap_bridge_lengths_m"]
                for feature in h5_features
            ))
            h5_yellow_end = next(
                feature for feature in h5_features
                if feature["validation"]["h5_vector_indices"] == [5]
            )
            h5_white_end = next(
                feature for feature in h5_features
                if feature["validation"]["h5_vector_indices"] == [6]
            )
            self.assertAlmostEqual(
                h5_yellow_end["validation"]["route_s_max_m"],
                h5_white_end["validation"]["route_s_max_m"], places=6,
            )
            self.assertGreater(h5_white_end["validation"]["endpoint_extension_m"], 0.1)
            self.assertLess(h5_white_end["validation"]["endpoint_extension_m"], 1.0)
            # No selected marking silently crosses the unobserved A215
            # intersection gap as a continuous pre-plan solid.
            self.assertFalse(any(
                feature["validation"]["route_s_min_m"] < 212.0
                and feature["validation"]["route_s_max_m"] > 237.0
                for feature in h5_features
            ))

            stoplines = [
                feature for feature in document["features"]
                if feature["feature_type"] == "stopline"
            ]
            self.assertEqual(len(stoplines), 7)
            self.assertEqual({
                feature["provenance"]["source_id"]: feature["topology"]["current_link_ids"]
                for feature in stoplines
            }, {
                "A1256W000682": ["A2256W000219"],
                "A1256W000807": ["A2256W000213"],
                "A1256W000208": ["A2256W000728"],
                "A1256W000582": ["A2256W000310"],
                "A1256W000757": ["A2256W000315"],
                "A1256W000593+A1256W000742": [
                    "A2256W000308", "A2256W000309",
                ],
                "A1256W000709": ["A2256W000148"],
            })

            stage03 = [
                feature for feature in document["features"]
                if feature["feature_type"] == "lane_marking"
                and feature["attributes"]["curation_segment_id"]
                == "stage03_static_obstacle_intersection_222"
            ]
            self.assertEqual(len(stage03), 3)
            guide = next(
                feature for feature in stage03
                if feature["attributes"]["functional_role"] == "intersection_guide_paint"
            )
            self.assertEqual(guide["attributes"]["color"], "white")
            self.assertEqual(guide["attributes"]["pattern"], "broken")
            self.assertEqual(guide["topology"]["current_link_ids"], ["A2256W000222"])
            self.assertFalse(any(
                feature["attributes"]["pattern"] == "solid"
                and "A2256W000222" in feature["topology"]["current_link_ids"]
                for feature in stage03
            ))

            stage04 = [
                feature for feature in document["features"]
                if feature["feature_type"] == "lane_marking"
                and feature["attributes"]["curation_segment_id"]
                == "stage04_link_202_and_following_intersection"
            ]
            self.assertEqual(len(stage04), 6)
            self.assertEqual({
                tuple(feature["validation"]["h5_vector_indices"])
                for feature in stage04
                if feature["attributes"]["source_display"] == "won_tae_h5"
            }, {(7,), (8,), (10,), (17,)})
            self.assertFalse(any(
                feature["validation"]["route_s_min_m"] < 442.0
                and feature["validation"]["route_s_max_m"] > 462.0
                for feature in stage04
            ))
            stage04_white = next(
                feature for feature in stage04
                if feature["validation"].get("h5_vector_indices") == [17]
            )
            self.assertEqual(
                stage04_white["validation"]["route_normal_smoothing"]["method"],
                "constant_median_route_normal_offset",
            )
            self.assertLess(
                stage04_white["validation"]["route_normal_smoothing"]["maximum_correction_m"],
                0.20,
            )
            a202_divider = next(
                feature for feature in stage04
                if feature["validation"].get("h5_vector_indices") == [8]
                and feature["attributes"]["pattern"] == "broken"
            )
            self.assertEqual(
                a202_divider["topology"]["current_link_ids"],
                ["A2256W000009", "A2256W000202"],
            )
            self.assertLessEqual(
                a202_divider["validation"]["route_s_max_m"], 427.85184362810037,
            )
            self.assertIn(
                "stage04_a202_lane1_right_white_broken",
                a202_divider["validation"]["correction_ids"],
            )
            a009_outer = next(
                feature for feature in stage04
                if feature["provenance"]["source_id"] == "B219BS010398"
            )
            self.assertEqual(a009_outer["attributes"]["pattern"], "solid")
            self.assertEqual(a009_outer["topology"]["current_link_ids"], ["A2256W000009"])
            self.assertLess(
                a009_outer["validation"]["route_s_max_m"], 427.85184362810037,
            )
            self.assertNotIn("A2256W000728", a009_outer["topology"]["current_link_ids"])

            stage05 = [
                feature for feature in document["features"]
                if feature["feature_type"] == "lane_marking"
                and feature["attributes"]["curation_segment_id"]
                == "stage05_dynamic_obstacle_205_to_315"
            ]
            self.assertEqual(len(stage05), 11)
            stage05_h5 = [
                feature for feature in stage05
                if feature["attributes"]["source_display"] == "won_tae_h5"
            ]
            self.assertEqual({
                tuple(feature["validation"]["h5_vector_indices"])
                for feature in stage05_h5
            }, {(10,), (11,), (12,), (13,), (14,), (15,), (17,), (18,)})
            self.assertTrue(all(
                not feature["validation"]["gap_bridge_lengths_m"]
                for feature in stage05_h5
            ))
            divider = next(
                feature for feature in stage05
                if feature["provenance"]["source_id"] == "B219BS010382"
            )
            self.assertEqual(divider["attributes"]["color"], "white")
            self.assertEqual(divider["attributes"]["pattern"], "broken")
            self.assertEqual(
                divider["attributes"]["source_display"],
                "legacy_origin_aligned_supplement",
            )
            self.assertAlmostEqual(
                divider["validation"]["route_s_max_m"],
                550.6187353754743,
                places=6,
            )
            self.assertGreater(divider["validation"]["endpoint_extension_m"], 11.0)
            self.assertLess(divider["validation"]["endpoint_extension_m"], 12.5)

            uturn_dashes = [
                feature for feature in stage05_h5
                if feature["validation"]["h5_vector_indices"][0] in {11, 12, 13, 14}
            ]
            self.assertEqual(len(uturn_dashes), 4)
            self.assertTrue(all(
                feature["attributes"]["color"] == "white"
                and feature["attributes"]["pattern"] == "broken"
                and feature["validation"]["h5_source_color"] == "yellow"
                and feature["validation"]["output_color"] == "white"
                and feature["validation"]["route_normal_smoothing"]
                    ["aligned_to_h5_vector_index"] == 10
                and feature["validation"]["route_normal_smoothing"]
                    ["maximum_correction_m"] < 0.20
                for feature in uturn_dashes
            ))
            uturn_offsets = {
                round(feature["validation"]["route_normal_smoothing"]
                      ["route_normal_offset_m"], 9)
                for feature in uturn_dashes
            }
            self.assertEqual(len(uturn_offsets), 1)
            self.assertFalse(any(
                feature["provenance"]["source_kind"] == "legacy_katri"
                and feature["attributes"]["color"] == "yellow"
                for feature in stage05
            ))
            continuity_bridges = [
                feature for feature in stage05
                if feature["attributes"]["source_display"]
                == "reviewed_same_pattern_continuity"
            ]
            self.assertEqual(len(continuity_bridges), 2)
            self.assertEqual({
                (feature["attributes"]["color"], feature["attributes"]["pattern"])
                for feature in continuity_bridges
            }, {("yellow", "solid"), ("white", "solid")})
            self.assertTrue(all(
                feature["validation"]["same_color_and_pattern_required"]
                and feature["validation"]["pattern_transition_bridge_forbidden"]
                and feature["validation"]["gap_bridge_length_m"] < 0.36
                and "B219BS010382" not in feature["provenance"]["source_id"]
                for feature in continuity_bridges
            ))
            self.assertEqual(divider["attributes"]["pattern"], "broken")

            stage06 = [
                feature for feature in document["features"]
                if feature["feature_type"] == "lane_marking"
                and feature["attributes"]["curation_segment_id"]
                == "stage06_right_turn_318"
            ]
            self.assertEqual(len(stage06), 3)
            left_guide = next(
                feature for feature in stage06
                if feature["provenance"]["source_id"].endswith(
                    "post_dynamic_right_turn_broken_2025:left"
                )
            )
            shared_guide = next(
                feature for feature in stage06
                if feature["provenance"]["source_id"].endswith(
                    "post_dynamic_right_turn_broken_2025:right"
                )
            )
            a319_outer = next(
                feature for feature in stage06
                if feature["attributes"]["source_display"] == "current_link_offset"
            )
            self.assertEqual(
                left_guide["provenance"]["source_id"],
                "kcity2025_competition_hd_candidate_v3:lane_boundary:"
                "current_reconstructed:route:post_dynamic_right_turn_broken_2025:left",
            )
            self.assertEqual(left_guide["provenance"]["source_kind"], "current_mgeo")
            self.assertEqual(left_guide["provenance"]["transform"], "reviewed_route_clip")
            self.assertEqual(left_guide["attributes"]["color"], "white")
            self.assertEqual(left_guide["attributes"]["pattern"], "broken")
            self.assertEqual(left_guide["topology"]["current_link_ids"], ["A2256W000318"])
            self.assertEqual(
                left_guide["validation"]["selection_id"],
                "stage06_a318_current_curve_left_guide",
            )
            self.assertIsNone(left_guide["validation"]["pattern_override_id"])
            self.assertEqual(shared_guide["attributes"]["pattern"], "broken")
            self.assertEqual(
                shared_guide["topology"]["current_link_ids"],
                ["A2256W000318", "A2256W000319"],
            )
            self.assertEqual(a319_outer["attributes"]["pattern"], "solid")
            self.assertEqual(a319_outer["attributes"]["functional_role"], "lane_2_outer_boundary")
            self.assertEqual(a319_outer["topology"]["current_link_ids"], ["A2256W000319"])
            self.assertAlmostEqual(
                a319_outer["validation"]["signed_normal_offset_m"],
                -1.4648245680922343,
            )
            self.assertEqual(
                a319_outer["validation"]["h5_anchor"]["h5_vector_index"], 18,
            )
            self.assertLess(
                abs(a319_outer["validation"]["h5_anchor"]["longitudinal_gap_m"]),
                0.10,
            )

            link_set = json.loads(
                (self.repo_root / "src/asmc_planner/config/behavior/link_set.json").read_text()
            )
            a318 = np.asarray(next(
                row["points"] for row in link_set if row["idx"] == "A2256W000318"
            ), dtype=np.float64)
            a318_tangents = np.gradient(a318[:, :2], axis=0)
            a318_tangents /= np.maximum(
                np.linalg.norm(a318_tangents, axis=1, keepdims=True), 1e-9,
            )
            a318_index = GridIndex(
                a318, a318_tangents, ["A2256W000318"] * len(a318), cell_size_m=2.0,
            )
            for guide_feature, expected_offset in (
                (left_guide, 1.75), (shared_guide, -1.75),
            ):
                lateral_offsets = []
                for point in np.asarray(
                    guide_feature["geometry"]["coordinates"], dtype=np.float64,
                ):
                    _, _, match = a318_index.query(point, None, 5.0, 180.0)
                    self.assertIsNotNone(match)
                    normal = np.asarray(
                        [-a318_tangents[match, 1], a318_tangents[match, 0]]
                    )
                    lateral_offsets.append(float(np.dot(
                        point[:2] - a318[match, :2], normal,
                    )))
                self.assertAlmostEqual(
                    float(np.median(lateral_offsets)), expected_offset, places=2,
                )
                self.assertLess(float(np.ptp(lateral_offsets)), 0.06)

            stage07 = [
                feature for feature in document["features"]
                if feature["feature_type"] == "lane_marking"
                and feature["attributes"]["curation_segment_id"]
                == "stage07_post_turn_308"
            ]
            self.assertEqual(len(stage07), 4)
            stage07_h5 = [
                feature for feature in stage07
                if feature["attributes"]["source_display"] == "won_tae_h5"
            ]
            self.assertEqual({
                tuple(feature["validation"]["h5_vector_indices"])
                for feature in stage07_h5
            }, {(19,), (20,)})
            divider_308 = next(
                feature for feature in stage07
                if feature["provenance"]["source_id"] == "B219BS011450"
            )
            self.assertEqual(divider_308["attributes"]["color"], "white")
            self.assertEqual(divider_308["attributes"]["pattern"], "broken")
            self.assertEqual(
                divider_308["topology"]["current_link_ids"],
                ["A2256W000308", "A2256W000309"],
            )
            self.assertAlmostEqual(
                divider_308["attributes"]["dash_phase_m"],
                3.745963252936214,
            )
            self.assertEqual(
                divider_308["validation"]["explicit_legacy_selection_id"],
                "stage07_lane1_right_broken_from_legacy",
            )
            self.assertEqual(
                divider_308["validation"]["attribute_override_ids"],
                ["stage07_a308_lane1_right_solid_to_broken"],
            )
            a308_stopline = next(
                feature for feature in document["features"]
                if feature["feature_type"] == "stopline"
                and feature["provenance"]["source_id"]
                == "A1256W000593+A1256W000742"
            )
            self.assertEqual(
                a308_stopline["topology"]["current_link_ids"],
                ["A2256W000308", "A2256W000309"],
            )
            self.assertAlmostEqual(
                a308_stopline["validation"]["h5_length_m"], 7.553601264953613,
            )
            a309_outer_fill = next(
                feature for feature in stage07
                if feature["provenance"]["source_id"] == "A2256W000309"
            )
            self.assertEqual(
                a309_outer_fill["topology"]["current_link_ids"],
                ["A2256W000309"],
            )
            a308_outer = next(
                feature for feature in stage07_h5
                if feature["validation"]["h5_vector_indices"] == [20]
            )
            self.assertLess(np.linalg.norm(
                np.asarray(a309_outer_fill["geometry"]["coordinates"][-1][:2])
                - np.asarray(a308_outer["geometry"]["coordinates"][0][:2])
            ), 0.10)
            self.assertLess(np.linalg.norm(
                np.asarray(a309_outer_fill["geometry"]["coordinates"][0][:2])
                - np.asarray(a319_outer["geometry"]["coordinates"][-1][:2])
            ), 0.10)

            stage08 = [
                feature for feature in document["features"]
                if feature["feature_type"] == "lane_marking"
                and feature["attributes"]["curation_segment_id"] == "stage08_guide_083"
            ]
            self.assertEqual(len(stage08), 1)
            self.assertEqual(stage08[0]["attributes"]["color"], "white")
            self.assertEqual(stage08[0]["attributes"]["pattern"], "broken")
            self.assertEqual(stage08[0]["topology"]["current_link_ids"], ["A2256W000083"])
            self.assertTrue(stage08[0]["provenance"]["source_id"].endswith(
                "roundabout_exit_to_high_speed_2025:left"
            ))
            self.assertEqual(
                stage08[0]["validation"]["pattern_override_id"],
                "stage08_a083_yellow_solid_to_white_broken_guide",
            )
            self.assertLess(
                stage08[0]["validation"]["route_s_max_m"],
                635.1126073415768,
            )
            a083 = np.asarray(next(
                row["points"] for row in link_set if row["idx"] == "A2256W000083"
            ), dtype=np.float64)
            a083_tangents = np.gradient(a083[:, :2], axis=0)
            a083_tangents /= np.maximum(
                np.linalg.norm(a083_tangents, axis=1, keepdims=True), 1e-9,
            )
            a083_index = GridIndex(
                a083, a083_tangents, ["A2256W000083"] * len(a083), cell_size_m=2.0,
            )
            a083_offsets = []
            for point in np.asarray(stage08[0]["geometry"]["coordinates"], dtype=np.float64):
                _, _, match = a083_index.query(point, None, 5.0, 180.0)
                self.assertIsNotNone(match)
                normal = np.asarray(
                    [-a083_tangents[match, 1], a083_tangents[match, 0]]
                )
                a083_offsets.append(float(np.dot(
                    point[:2] - a083[match, :2], normal,
                )))
            self.assertAlmostEqual(float(np.median(a083_offsets)), 1.75, places=2)
            self.assertLess(float(np.ptp(a083_offsets)), 0.05)

            stage09 = [
                feature for feature in document["features"]
                if feature["feature_type"] == "lane_marking"
                and feature["attributes"]["curation_segment_id"]
                == "stage09_two_lane_expansion_304"
            ]
            self.assertEqual(len(stage09), 3)
            a304_yellow = next(
                feature for feature in stage09
                if feature["validation"].get("h5_vector_indices") == [25]
            )
            a307_outer = next(
                feature for feature in stage09
                if feature["validation"].get("h5_vector_indices") == [28]
            )
            a304_divider = next(
                feature for feature in stage09
                if feature["attributes"]["source_display"]
                == "won_tae_h5_derived_midpoint"
            )
            self.assertEqual(a304_yellow["attributes"]["color"], "yellow")
            self.assertEqual(a304_yellow["attributes"]["pattern"], "solid")
            self.assertEqual(a304_yellow["topology"]["current_link_ids"], ["A2256W000304"])
            self.assertEqual(a304_yellow["validation"]["h5_candidate_run_count"], 2)
            self.assertEqual(a304_yellow["validation"]["h5_discarded_run_count"], 1)
            self.assertGreater(a304_yellow["validation"]["start_anchor_extension_m"], 0.5)
            self.assertLess(a304_yellow["validation"]["start_anchor_extension_m"], 0.7)
            self.assertIn(
                "stage09_preserve_h5_yellow_start_without_zigzag",
                a304_yellow["validation"]["correction_ids"],
            )
            np.testing.assert_allclose(
                a304_yellow["geometry"]["coordinates"][0][:2],
                [1.044007658958435, 116.4134292602539],
            )
            self.assertEqual(a307_outer["attributes"]["color"], "white")
            self.assertEqual(a307_outer["attributes"]["pattern"], "solid")
            self.assertEqual(a307_outer["topology"]["current_link_ids"], ["A2256W000307"])
            self.assertEqual(a304_divider["attributes"]["color"], "white")
            self.assertEqual(a304_divider["attributes"]["pattern"], "broken")
            self.assertEqual(
                a304_divider["topology"]["current_link_ids"],
                ["A2256W000304", "A2256W000307"],
            )
            self.assertEqual(
                a304_divider["provenance"]["transform"],
                "route_normal_midpoint_of_won_tae_h5_boundaries",
            )
            self.assertGreater(
                a304_divider["validation"]["observed_two_lane_width_median"], 6.8,
            )
            self.assertLess(
                a304_divider["validation"]["observed_two_lane_width_median"], 7.2,
            )
            self.assertAlmostEqual(
                a304_divider["validation"]["midpoint_route_offset_median"],
                -1.7340090273399114,
            )
            self.assertFalse(any(
                feature["provenance"]["source_kind"] == "legacy_katri"
                for feature in stage09
            ))

            stage10 = [
                feature for feature in document["features"]
                if feature["feature_type"] == "lane_marking"
                and feature["attributes"]["curation_segment_id"]
                == "stage10_roundabout_approach_148"
            ]
            self.assertEqual(len(stage10), 4)
            profiles = [
                feature for feature in stage10
                if feature["attributes"]["source_display"]
                == "won_tae_h5_robust_profile"
            ]
            self.assertEqual(len(profiles), 2)
            self.assertEqual({
                tuple(feature["validation"]["h5_vector_indices"])
                for feature in profiles
            }, {(25,), (29, 31, 51, 36, 37, 43)})
            self.assertTrue(all(
                feature["validation"]["endpoint_observation_anchors_preserved"]
                and feature["validation"]["route_normal_profile"]
                    ["maximum_smoothing_correction_m"] < 0.10
                for feature in profiles
            ))
            self.assertTrue(all(
                feature["validation"]["route_s_min_m"] == 858.3073083848091
                and feature["validation"]["startpoint_alignment_route_s_m"]
                == 858.3073083848091
                and 0.0 < feature["validation"]["startpoint_extension_m"] < 0.70
                for feature in profiles
            ))
            inner_yellow = next(
                feature for feature in stage10
                if feature["validation"].get("h5_vector_indices") == [26]
            )
            self.assertEqual(inner_yellow["attributes"]["color"], "yellow")
            self.assertEqual(inner_yellow["attributes"]["pattern"], "solid")
            self.assertEqual(
                inner_yellow["topology"]["current_link_ids"],
                ["A2256W000146", "A2256W000151"],
            )
            self.assertEqual(
                inner_yellow["provenance"]["transform"],
                "reviewed_arc_length_moving_average",
            )
            self.assertLess(
                inner_yellow["validation"]["polyline_smoothing"]
                    ["maximum_correction_m"],
                0.20,
            )
            self.assertFalse(any(
                feature["provenance"]["source_kind"] == "legacy_katri"
                or feature["validation"].get("h5_vector_indices") == [27]
                for feature in stage10
            ))
            a709 = next(
                feature for feature in stoplines
                if feature["provenance"]["source_id"] == "A1256W000709"
            )
            self.assertAlmostEqual(a709["validation"]["h5_length_m"], 3.5)
            self.assertAlmostEqual(
                a709["validation"]["route_s_m"], 885.1006875594776,
            )

            a148_center = np.asarray(next(
                row["points"] for row in link_set if row["idx"] == "A2256W000148"
            ), dtype=np.float64)
            a148_tangents = np.gradient(a148_center[:, :2], axis=0)
            a148_tangents /= np.maximum(
                np.linalg.norm(a148_tangents, axis=1, keepdims=True), 1e-9,
            )
            a148_stations = cumulative_distance(a148_center)
            a148_index = GridIndex(
                a148_center, a148_tangents,
                ["A2256W000148"] * len(a148_center), cell_size_m=2.0,
            )
            stage10_left = next(
                feature for feature in profiles
                if feature["attributes"]["color"] == "yellow"
            )
            stage10_right = next(
                feature for feature in profiles
                if feature["attributes"]["color"] == "white"
            )
            post_stop_yellow = next(
                feature for feature in stage10
                if feature["attributes"]["source_display"]
                == "won_tae_h5_derived_parallel"
            )
            self.assertEqual(
                post_stop_yellow["provenance"]["transform"],
                "anchored_parallel_offset_from_won_tae_h5_boundary",
            )
            self.assertEqual(
                post_stop_yellow["topology"]["current_link_ids"],
                ["A2256W000146"],
            )
            parallel_audit = post_stop_yellow["validation"][
                "parallel_route_normal_profile"
            ]
            self.assertAlmostEqual(
                parallel_audit["route_s_min_m"], 885.1006875594776,
            )
            self.assertAlmostEqual(
                parallel_audit["route_s_max_m"], 889.3505833773406,
            )
            self.assertEqual(
                post_stop_yellow["validation"]["route_end_reference"],
                {
                    "legacy_source_id": "B219BS011472",
                    "legacy_route_end_s_m": 889.3505833773406,
                    "selection_rule": "legacy_centerline_physical_end",
                },
            )
            self.assertGreater(parallel_audit["lateral_separation_m"], 3.3)
            self.assertLess(parallel_audit["lateral_separation_m"], 3.6)
            self.assertLessEqual(parallel_audit["anchor_extrapolation_m"], 0.5)
            self.assertAlmostEqual(
                stage10_left["validation"]["endpoint_alignment_route_s_m"],
                885.1006875594776,
            )
            self.assertEqual(
                stage10_left["topology"]["current_link_ids"],
                ["A2256W000148"],
            )
            self.assertGreater(
                stage10_left["validation"]["endpoint_extension_m"], 0.0,
            )
            self.assertLess(
                stage10_left["validation"]["endpoint_extension_m"], 0.5,
            )

            def a148_offsets(feature):
                result = []
                for point in np.asarray(feature["geometry"]["coordinates"]):
                    _, _, match = a148_index.query(point, None, 5.0, 180.0)
                    if match is None:
                        continue
                    normal = np.asarray([
                        -a148_tangents[match, 1], a148_tangents[match, 0],
                    ])
                    result.append((
                        a148_stations[match],
                        float(np.dot(point[:2] - a148_center[match, :2], normal)),
                    ))
                return np.asarray(sorted(result))

            left_offsets = a148_offsets(stage10_left)
            right_offsets = a148_offsets(stage10_right)
            common_stations = np.arange(1.0, 25.0, 0.5)
            widths = (
                np.interp(common_stations, left_offsets[:, 0], left_offsets[:, 1])
                - np.interp(common_stations, right_offsets[:, 0], right_offsets[:, 1])
            )
            self.assertGreater(float(np.quantile(widths, 0.05)), 3.3)
            self.assertLess(float(np.quantile(widths, 0.95)), 3.7)
            self.assertGreater(float(np.median(widths)), 3.45)
            self.assertLess(float(np.median(widths)), 3.60)

            stage11 = [
                feature for feature in document["features"]
                if feature["feature_type"] == "lane_marking"
                and feature["attributes"]["curation_segment_id"]
                == "stage11_roundabout_exit_to_high_speed"
            ]
            self.assertEqual(len(stage11), 9)
            self.assertTrue(all(
                feature["provenance"]["source_kind"] == "legacy_katri"
                for feature in stage11
                if feature["attributes"]["source_display"]
                != "reviewed_same_pattern_continuity"
            ))
            self.assertFalse(any(
                feature["provenance"]["source_id"] == "B219BS010215"
                or "A2256W000057" in feature["topology"]["current_link_ids"]
                for feature in stage11
            ))
            self.assertEqual({
                link_id for feature in stage11
                for link_id in feature["topology"]["current_link_ids"]
            }, {
                "A2256W000151", "A2256W000866", "A2256W000054",
                "A2256W000846",
            })
            self.assertFalse(any(
                feature["provenance"]["source_id"] in {
                    "B219BS011489", "B219BS011497",
                }
                or "A2256W000150" in feature["topology"]["current_link_ids"]
                for feature in stage11
            ))
            a866_bridge = next(
                feature for feature in stage11
                if feature["validation"].get("selection_id")
                == "stage11_a866_white_gap_closed"
            )
            self.assertGreater(a866_bridge["validation"]["gap_bridge_length_m"], 3.0)
            self.assertLess(a866_bridge["validation"]["gap_bridge_length_m"], 3.5)
            a866_yellow = next(
                feature for feature in stage11
                if feature["provenance"]["source_id"] == "B219BS011499"
            )
            self.assertEqual(a866_yellow["attributes"]["color"], "yellow")
            self.assertAlmostEqual(
                a866_yellow["validation"]["route_s_max_m"],
                930.6019068794745,
            )
            self.assertGreater(
                a866_yellow["validation"]["endpoint_extension_m"], 11.0,
            )
            a846_left = next(
                feature for feature in stage11
                if feature["validation"].get("selection_id")
                == "stage11_a846_left_parallel_to_right"
            )
            self.assertEqual(a846_left["attributes"]["pattern"], "solid")
            self.assertEqual(
                a846_left["provenance"]["transform"],
                "anchored_parallel_offset_from_legacy_right_boundary",
            )
            a846_long_left = next(
                feature for feature in stage11
                if feature["provenance"]["source_id"] == "B219BS011490"
            )
            np.testing.assert_allclose(
                np.asarray(a846_left["geometry"]["coordinates"])[0],
                np.asarray(a846_long_left["geometry"]["coordinates"])[-1],
                atol=1e-9,
            )
            a846_left_profile = a846_left["validation"][
                "parallel_route_normal_profile"
            ]
            self.assertAlmostEqual(
                a846_left_profile["lateral_separation_m"],
                3.5295401883413615,
            )
            self.assertAlmostEqual(
                a846_left_profile["route_s_max_m"],
                1118.741751169101,
            )
            self.assertTrue(a846_left_profile["start_anchor_endpoint_aligned"])
            self.assertFalse(any(
                feature["provenance"]["source_id"] == "B219BS011113"
                for feature in stage11
            ))
            a846_right = next(
                feature for feature in stage11
                if feature["provenance"]["source_id"] == "B219BS011112"
            )
            self.assertEqual(a846_right["attributes"]["pattern"], "solid")
            self.assertAlmostEqual(
                a846_right["validation"]["route_s_max_m"],
                1118.741751169101,
            )
            self.assertGreater(a846_right["validation"]["endpoint_extension_m"], 1.8)
            self.assertFalse(any(
                feature["validation"].get("selection_id")
                == "stage11_a846_left_solid_gap_closed"
                for feature in stage11
            ))

            stage12 = [
                feature for feature in document["features"]
                if feature["feature_type"] == "lane_marking"
                and feature["attributes"]["curation_segment_id"]
                == "stage12_high_speed_entry_and_body"
            ]
            self.assertEqual(len(stage12), 37)
            self.assertEqual(
                {feature["attributes"]["color"] for feature in stage12},
                {"white"},
            )
            self.assertEqual(
                sum(feature["attributes"]["pattern"] == "broken" for feature in stage12),
                16,
            )
            self.assertEqual(
                sum(feature["attributes"]["pattern"] == "solid" for feature in stage12),
                21,
            )
            self.assertFalse(any(
                feature["provenance"]["source_id"] in {
                    "B219BS011102", "B219BS011105", "B219BS011110",
                    "B219BS011123", "B219BS011265", "B219BS010431",
                    "B219BS010521", "B219BS010524", "B219BS010520",
                    "B219BS010526", "LM000002",
                }
                for feature in stage12
            ))
            a411_starts = [
                feature for feature in stage12
                if feature["validation"].get("start_alignment_route_s_m")
                == 1118.741751169101
            ]
            self.assertEqual(len(a411_starts), 5)
            self.assertTrue(all(
                feature["validation"]["route_s_min_m"] == 1118.741751169101
                and 0.0 <= feature["validation"]["start_extension_m"] < 0.5
                for feature in a411_starts
            ))
            a411_left = next(
                feature for feature in stage12
                if feature["validation"].get("selection_id")
                == "stage12_a411_left_merge_broken"
            )
            self.assertEqual(a411_left["attributes"]["pattern"], "broken")
            self.assertEqual(
                a411_left["topology"]["current_link_ids"],
                ["A2256W000411", "A2256W000846"],
            )
            left_step = float(np.linalg.norm(
                np.asarray(a411_left["geometry"]["coordinates"])[0, :2]
                - np.asarray(a846_left["geometry"]["coordinates"])[-1, :2]
            ))
            self.assertGreater(left_step, 0.15)
            self.assertLess(left_step, 0.18)
            a846_a411_right = next(
                feature for feature in stage12
                if feature["validation"].get("selection_id")
                == "stage12_a846_a411_right_solid_connected"
            )
            self.assertEqual(a846_a411_right["attributes"]["pattern"], "solid")
            self.assertLess(
                a846_a411_right["validation"]["gap_bridge_length_m"], 0.01,
            )
            a420_left = next(
                feature for feature in stage12
                if feature["provenance"]["source_id"] == "B219BS011106"
            )
            self.assertEqual(a420_left["attributes"]["pattern"], "broken")
            self.assertGreaterEqual(
                a420_left["validation"]["route_s_min_m"], 1281.1572456094677,
            )
            a418_left = next(
                feature for feature in stage12
                if feature["validation"].get("selection_id")
                == "stage12_a418_left_solid_restored"
            )
            self.assertEqual(
                a418_left["topology"]["current_link_ids"],
                ["A2256W000418", "A2256W000429"],
            )
            self.assertEqual(a418_left["attributes"]["pattern"], "solid")
            self.assertEqual(a418_left["attributes"]["color"], "white")
            self.assertLess(a418_left["validation"]["start_extension_m"], 0.5)
            a445_right = next(
                feature for feature in stage12
                if feature["attributes"]["functional_role"]
                == "a445_right_boundary_joined_to_a153_vertex"
            )
            self.assertEqual(
                a445_right["provenance"]["transform"],
                "current_link_piecewise_normal_offset_from_won_tae_h5_profile",
            )
            self.assertEqual(
                a445_right["validation"]["observation_reference_ids"],
                ["hi_right.csv", "A2256W000153+A2256W000445+A2256W000451"],
            )
            self.assertFalse(any(
                feature["provenance"]["source_id"] == "hi_right.csv+hi_right3.csv"
                for feature in stage12
            ))
            self.assertFalse(any(
                feature["validation"].get("selection_id")
                == "stage12_a846_legacy_solid_gap_closed"
                for feature in stage12
            ))
            a423_left = next(
                feature for feature in stage12
                if feature["validation"].get("selection_id")
                == "stage12_a423_left_solid_restored"
            )
            self.assertEqual(a423_left["attributes"]["pattern"], "solid")
            self.assertEqual(a423_left["attributes"]["color"], "white")
            self.assertEqual(
                a423_left["topology"]["current_link_ids"],
                ["A2256W000423", "A2256W000425"],
            )
            a423_start_bridge = next(
                feature for feature in stage12
                if feature["validation"].get("selection_id")
                == "stage12_a423_left_solid_start_gap_closed"
            )
            self.assertLess(
                a423_start_bridge["validation"]["gap_bridge_length_m"], 0.5,
            )
            a179_left = next(
                feature for feature in stage12
                if feature["validation"].get("selection_id")
                == "stage12_a408_right"
            )
            self.assertEqual(
                a179_left["topology"]["current_link_ids"], ["A2256W000179"],
            )
            self.assertLessEqual(
                a179_left["validation"]["route_s_max_m"], 1518.2468855207765,
            )
            a423_right = next(
                feature for feature in stage12
                if feature["validation"].get("selection_id")
                == "stage12_a423_right_to_node"
            )
            self.assertEqual(a423_right["attributes"]["pattern"], "broken")
            self.assertEqual(
                a423_right["topology"]["current_link_ids"], ["A2256W000423"],
            )
            self.assertAlmostEqual(
                a423_right["validation"]["route_s_max_m"],
                1593.5863873852518,
            )
            self.assertFalse(any(
                feature["validation"].get("selection_id") in {
                    "stage12_a432_left_to_node", "stage12_a432_right_to_node",
                }
                for feature in stage12
            ))
            a432_preplan_handoff = next(
                feature for feature in stage12
                if feature["validation"].get("selection_id")
                == "stage12_a432_left_preplan_node_handoff"
            )
            self.assertEqual(
                a432_preplan_handoff["provenance"]["source_id"],
                "A2256W000424+A2256W000432",
            )
            self.assertLess(
                a432_preplan_handoff["validation"]["route_s_max_m"], 1600.0,
            )
            second_island_lead = next(
                feature for feature in stage12
                if feature["provenance"]["source_id"] == "B219BS010446"
            )
            self.assertEqual(
                (second_island_lead["attributes"]["color"],
                 second_island_lead["attributes"]["pattern"]),
                ("white", "solid"),
            )
            self.assertEqual(
                set(second_island_lead["validation"]["attribute_override_ids"]),
                {
                    "stage12_b010446_blue_to_white_2025",
                    "stage12_b010446_broken_to_solid_2025",
                },
            )
            self.assertGreater(
                second_island_lead["validation"]["route_s_min_m"], 1599.0,
            )
            self.assertIsNone(
                second_island_lead["validation"]["start_alignment_route_s_m"],
            )
            a432_start_bridge = next(
                feature for feature in stage12
                if feature["validation"].get("selection_id")
                == "stage12_a432_left_solid_start_gap_closed"
            )
            self.assertEqual(a432_start_bridge["attributes"]["pattern"], "solid")
            self.assertEqual(a432_start_bridge["attributes"]["color"], "white")
            self.assertLess(
                a432_start_bridge["validation"]["gap_bridge_length_m"], 0.03,
            )
            a432_smooth_handoff = next(
                feature for feature in stage12
                if feature["validation"].get("selection_id")
                == "stage12_a432_left_preplan_to_legacy_smooth"
            )
            self.assertEqual(
                a432_smooth_handoff["validation"]["handoff_smoothing"]["method"],
                "tangent_continuous_cubic_handoff",
            )
            self.assertLess(
                a432_smooth_handoff["validation"]["handoff_smoothing"]
                ["start_tangent_error_deg"],
                2.0,
            )
            self.assertLess(
                a432_smooth_handoff["validation"]["handoff_smoothing"]
                ["end_tangent_error_deg"],
                2.0,
            )
            self.assertGreater(
                second_island_lead["validation"]["route_s_max_m"], 1716.7,
            )
            second_island_split = {
                feature["provenance"]["source_id"]: feature
                for feature in stage12
                if feature["provenance"]["source_id"] in {
                    "B219BS010435", "B219BS010436",
                }
            }
            self.assertEqual(
                set(second_island_split),
                {"B219BS010435", "B219BS010436"},
            )
            self.assertTrue(all(
                feature["attributes"]["pattern"] == "solid"
                and feature["provenance"]["transform"]
                == "reviewed_full_source_geometry"
                for feature in second_island_split.values()
            ))
            lead_end = np.asarray(
                second_island_lead["geometry"]["coordinates"][-1],
                dtype=np.float64,
            )
            for feature in second_island_split.values():
                np.testing.assert_allclose(
                    lead_end,
                    np.asarray(feature["geometry"]["coordinates"][0], dtype=np.float64),
                    atol=1e-9,
                )
            for source_id in ("B219BS010447", "B219BS010518"):
                fragment_fixed = next(
                    feature for feature in stage12
                    if feature["provenance"]["source_id"] == source_id
                )
                length = cumulative_distance(np.asarray(
                    fragment_fixed["geometry"]["coordinates"], dtype=np.float64,
                ))[-1]
                self.assertGreater(length, 39.5)
                self.assertLess(length, 40.0)
            vertex = next(
                feature for feature in stage12
                if feature["attributes"]["functional_role"]
                == "a445_a153_right_boundary_vertex"
            )
            self.assertLess(vertex["validation"]["gap_bridge_length_m"], 0.5)
            a153_left = next(
                feature for feature in stage12
                if feature["validation"].get("selection_id")
                == "stage12_a153_left_parallel_to_reviewed_right"
            )
            self.assertEqual(
                a153_left["provenance"]["transform"],
                "current_link_normal_offset_parallel_to_reviewed_boundary",
            )
            self.assertEqual(
                a153_left["validation"]["parallel_reference_ids"],
                ["A2256W000153+A2256W000445+A2256W000451"],
            )
            self.assertAlmostEqual(
                a153_left["validation"]["signed_normal_offset_start_m"], 1.797,
            )
            self.assertAlmostEqual(
                a153_left["validation"]["signed_normal_offset_end_m"], 1.797,
            )
            self.assertAlmostEqual(
                a153_left["validation"]["parallel_separation_m"], 3.507,
            )
            self.assertGreaterEqual(
                a153_left["validation"]["source_station_min_m"], 36.4,
            )
            self.assertFalse(any(
                feature["provenance"]["source_id"] == "B219BS010432"
                for feature in stage12
            ))

            stage13 = [
                feature for feature in document["features"]
                if feature["feature_type"] == "lane_marking"
                and feature["attributes"]["curation_segment_id"]
                == "stage13_high_speed_tail_to_333"
            ]
            self.assertEqual(len(stage13), 28)
            self.assertEqual(
                sum(feature["attributes"]["pattern"] == "broken" for feature in stage13),
                4,
            )
            self.assertEqual(
                sum(feature["attributes"]["color"] == "yellow" for feature in stage13),
                7,
            )
            self.assertFalse(any(
                feature["provenance"]["source_id"] in {
                    "B219BS011265", "B219BS010519", "LM000002",
                }
                for feature in stage13
            ))
            self.assertFalse(any(
                feature["provenance"]["source_id"]
                == "A2256W000426+A2256W000427"
                for feature in stage13
            ))
            self.assertFalse(any(
                "A2256W000427" in feature["topology"]["current_link_ids"]
                for feature in document["features"]
            ))
            self.assertFalse(any(
                feature["provenance"]["source_id"] in {
                    "B219BS010452", "B219BS010451",
                }
                for feature in stage13
            ))
            a451_lead_in = next(
                feature for feature in stage13
                if feature["provenance"]["source_id"] == "B219BS010453"
            )
            self.assertEqual(a451_lead_in["attributes"]["pattern"], "solid")
            self.assertEqual(
                a451_lead_in["provenance"]["transform"],
                "reviewed_full_source_geometry",
            )
            tollgate_islands = {
                feature["provenance"]["source_id"]: feature
                for feature in stage13
                if feature["provenance"]["source_id"] in {
                    "B219BS010453", "B219BS010454",
                    "B219BS010455", "B219BS010456",
                }
            }
            self.assertEqual(set(tollgate_islands), {
                "B219BS010453", "B219BS010454",
                "B219BS010455", "B219BS010456",
            })
            self.assertTrue(all(
                feature["attributes"]["pattern"] == "solid"
                and feature["provenance"]["transform"]
                == "reviewed_full_source_geometry"
                for feature in tollgate_islands.values()
            ))
            np.testing.assert_allclose(
                np.asarray(second_island_split["B219BS010435"]["geometry"]["coordinates"][-1]),
                np.asarray(tollgate_islands["B219BS010455"]["geometry"]["coordinates"][0]),
                atol=1e-9,
            )
            np.testing.assert_allclose(
                np.asarray(second_island_split["B219BS010436"]["geometry"]["coordinates"][-1]),
                np.asarray(tollgate_islands["B219BS010456"]["geometry"]["coordinates"][0]),
                atol=1e-9,
            )
            a451_legacy_right = next(
                feature for feature in stage13
                if feature["provenance"]["source_id"] == "B219BS010512"
            )
            self.assertEqual(
                a451_legacy_right["topology"]["current_link_ids"],
                ["A2256W000451"],
            )
            a451_left_broken = next(
                feature for feature in stage13
                if feature["validation"].get("selection_id")
                == "stage13_a451_left_parallel_to_h5_right"
            )
            self.assertEqual(a451_left_broken["attributes"]["pattern"], "broken")
            self.assertEqual(
                a451_left_broken["provenance"]["transform"],
                "anchored_parallel_offset_from_won_tae_h5_boundary",
            )
            self.assertAlmostEqual(
                a451_left_broken["validation"]["parallel_route_normal_profile"]
                ["lateral_separation_m"],
                3.832493719697048,
            )
            self.assertAlmostEqual(
                a451_left_broken["validation"]["parallel_route_normal_profile"]
                ["route_s_max_m"],
                1781.5573293005787,
            )
            self.assertFalse(any(
                feature["provenance"]["source_id"] == "B219BS010438"
                for feature in stage13
            ))
            self.assertEqual(
                sum(
                    feature["provenance"]["source_id"]
                    == "A2256W000126+A2256W000446+A2256W000448+A2256W000451"
                    for feature in stage13
                ),
                2,
            )
            h5_transition = {
                feature["provenance"]["source_id"]: feature
                for feature in stage13
                if feature["provenance"]["source_id"] in {
                    "won_tae_h5_vectors_53", "won_tae_h5_vectors_62",
                }
            }
            self.assertEqual(set(h5_transition), {
                "won_tae_h5_vectors_53", "won_tae_h5_vectors_62",
            })
            self.assertEqual(
                h5_transition["won_tae_h5_vectors_53"]["topology"]["current_link_ids"],
                ["A2256W000446", "A2256W000451"],
            )
            self.assertLessEqual(
                h5_transition["won_tae_h5_vectors_53"]["validation"]["route_s_max_m"],
                1820.5994538178333,
            )
            self.assertEqual(
                (h5_transition["won_tae_h5_vectors_62"]["attributes"]["color"],
                 h5_transition["won_tae_h5_vectors_62"]["attributes"]["pattern"]),
                ("yellow", "solid"),
            )
            reconstructed = [
                feature for feature in stage13
                if feature["provenance"]["source_id"]
                == "A2256W000126+A2256W000446+A2256W000448+A2256W000451"
            ]
            self.assertEqual(len(reconstructed), 2)
            self.assertTrue(all(
                feature["validation"]["route_s_min_m"] >= 1820.5994538178333
                and feature["topology"]["current_link_ids"]
                == ["A2256W000446", "A2256W000448"]
                for feature in reconstructed
            ))
            a333_guides = [
                feature for feature in stage13
                if feature["provenance"]["source_id"] == "B219BS011535"
            ]
            self.assertEqual(len(a333_guides), 3)
            self.assertEqual({
                feature["validation"]["source_feature_id"].split(":")[-3]
                for feature in a333_guides
            }, {"00", "01", "02"})
            self.assertTrue(all(
                feature["attributes"]["pattern"] == "broken"
                and feature["provenance"]["transform"]
                == "reviewed_full_source_geometry"
                for feature in a333_guides
            ))
            self.assertTrue(any(
                feature["provenance"]["source_id"] == "LM000001"
                and feature["attributes"]["pattern"] == "solid"
                and feature["topology"]["current_link_ids"] == ["A2256W000131"]
                for feature in stage13
            ))
            expected_connections = {
                "stage13_a153_right_to_legacy_gap_closed": 0.50,
                "stage13_a451_legacy_right_to_h5_gap_closed": 0.50,
                "stage13_a446_h5_to_preplan_left_gap_closed": 0.50,
                "stage13_a446_h5_to_preplan_right_gap_closed": 0.50,
                "stage13_a448_shadow_left_gap_closed": 0.10,
                "stage13_a448_shadow_right_gap_closed": 0.25,
                "stage13_a128_left_gap_closed": 0.30,
                "stage13_a128_right_gap_closed": 0.30,
            }
            for selection_id, maximum in expected_connections.items():
                connection = next(
                    feature for feature in stage13
                    if feature["validation"].get("selection_id") == selection_id
                )
                self.assertLess(
                    connection["validation"]["gap_bridge_length_m"], maximum,
                )
            for selection_id in {
                "stage13_a451_legacy_right_to_h5_gap_closed",
                "stage13_a448_shadow_left_gap_closed",
                "stage13_a448_shadow_right_gap_closed",
            }:
                connection = next(
                    feature for feature in stage13
                    if feature["validation"].get("selection_id") == selection_id
                )
                self.assertEqual(
                    connection["provenance"]["transform"],
                    "reviewed_tangent_continuous_same_pattern_handoff",
                )
                self.assertLess(
                    connection["validation"]["handoff_smoothing"]
                    ["start_tangent_error_deg"],
                    1.0,
                )
                self.assertLess(
                    connection["validation"]["handoff_smoothing"]
                    ["end_tangent_error_deg"],
                    1.0,
                )

            stage14 = [
                feature for feature in document["features"]
                if feature["feature_type"] == "lane_marking"
                and feature["attributes"]["curation_segment_id"]
                == "stage14_return_332_to_finish"
            ]
            self.assertEqual(len(stage14), 4)
            self.assertEqual({
                link_id for feature in stage14
                for link_id in feature["topology"]["current_link_ids"]
            }, {"A2256W000154", "A2256W000332", "A2256W000751"})
            self.assertEqual({
                (feature["attributes"]["color"], feature["attributes"]["pattern"])
                for feature in stage14
            }, {("yellow", "solid"), ("white", "solid"), ("white", "broken")})
            finish_features = [
                feature for feature in stage14
                if feature["provenance"]["source_id"] in {
                    "B219BS010411", "B219BS010474",
                }
            ]
            self.assertEqual(len(finish_features), 2)
            self.assertTrue(all(
                feature["validation"]["route_s_max_m"] == 2184.6117233360696
                and 0.0 < feature["validation"]["endpoint_extension_m"] < 0.5
                for feature in finish_features
            ))
            return_connection = next(
                feature for feature in stage14
                if feature["validation"].get("selection_id")
                == "stage14_a154_a332_right_gap_closed"
            )
            self.assertLess(
                return_connection["validation"]["gap_bridge_length_m"], 0.30,
            )
            self.assertFalse(any(
                feature["provenance"]["source_id"] == "A1256W000616"
                for feature in document["features"]
            ))

    @unittest.skipIf(h5py is None, "python3-h5py is a declared ROS runtime dependency")
    def test_output_is_deterministic(self):
        with tempfile.TemporaryDirectory() as first_raw, tempfile.TemporaryDirectory() as second_raw:
            first, second = Path(first_raw), Path(second_raw)
            build_route_curation(self.config, self.repo_root, first)
            build_route_curation(self.config, self.repo_root, second)
            for name in (
                "route_curated_candidate_map.json", "selected_markings.csv",
                "selected_stoplines.csv", "summary.json",
                "overlays/stage01_start_legacy.png",
                "overlays/stage02_new_road_219_to_213.png",
                "overlays/stage03_static_obstacle_intersection_222.png",
                "overlays/stage04_link_202_and_following_intersection.png",
                "overlays/stage05_dynamic_obstacle_205_to_315.png",
                "overlays/stage06_right_turn_318.png",
                "overlays/stage07_post_turn_308.png",
                "overlays/stage08_guide_083.png",
                "overlays/stage09_two_lane_expansion_304.png",
                "overlays/stage10_roundabout_approach_148.png",
                "overlays/stage11_roundabout_exit_to_high_speed.png",
                "overlays/stage12_high_speed_entry_and_body.png",
                "overlays/stage13_high_speed_tail_to_333.png",
                "overlays/stage14_return_332_to_finish.png",
            ):
                self.assertEqual(_hash(first / name), _hash(second / name), name)

    def test_h5_join_orients_vectors_and_audits_gap(self):
        first = np.asarray([[0.0, 0.0], [1.0, 0.0]])
        second = np.asarray([[3.0, 0.0], [2.0, 0.0]])
        joined, gaps = _join_h5_vectors([first, second], 1.1)
        np.testing.assert_allclose(joined, [[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [3.0, 0.0]])
        self.assertEqual(gaps, [1.0])

    def test_route_normal_straightening_is_gated_and_monotonic(self):
        route = np.asarray([[float(x), 0.0, 0.0] for x in range(11)])
        tangents = np.tile([1.0, 0.0, 0.0], (len(route), 1))
        stations = np.arange(len(route), dtype=np.float64)
        index = GridIndex(route, tangents, ["route"] * len(route), cell_size_m=2.0)
        observed = np.asarray([
            [10.0, 0.9, 0.0], [5.0, 1.1, 0.0], [0.0, 1.0, 0.0],
        ])
        straightened, audit = _route_normal_constant_offset(
            observed, route, tangents, stations, index, 0.11,
        )
        np.testing.assert_allclose(straightened[:, 0], [0.0, 5.0, 10.0])
        np.testing.assert_allclose(straightened[:, 1], [1.0, 1.0, 1.0])
        self.assertAlmostEqual(audit["maximum_correction_m"], 0.1)
