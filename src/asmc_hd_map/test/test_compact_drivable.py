#!/usr/bin/env python3

import unittest

import numpy as np

from asmc_hd_map.compact_drivable import (
    CompactDrivableError,
    _same_direction_siblings,
    apply_intersection_node_pair_closures,
    corridor_bridge_polygon,
    explicit_intersection_links,
    intersection_link_widening,
    lane_boundary_polygon,
    lane_width_polygon,
    reviewed_non_drivable_islands,
    subtract_reviewed_islands,
)


def link(idx, road, points, link_type="6"):
    return {
        "idx": idx,
        "road_id": road,
        "link_type": link_type,
        "points": [[x, y, 0.0] for x, y in points],
    }


class CompactDrivableUnitTest(unittest.TestCase):
    def test_reviewed_island_requires_closed_unique_chains_and_preserves_other_geometry(self):
        from shapely.geometry import Polygon
        reviewed = {"features": [
            {"id": "left", "attributes": {"functional_role": "left"},
             "geometry": {"type": "LineString", "coordinates": [[0, 0], [-1, 2], [0, 4]]}},
            {"id": "right", "attributes": {"functional_role": "right"},
             "geometry": {"type": "LineString", "coordinates": [[0, 0], [1, 2], [0, 4]]}},
        ]}
        policy = [{"id": "fixture", "boundary_chains": [["left"], ["right"]]}]
        islands = reviewed_non_drivable_islands(reviewed, policy)
        records = [{"id": "near", "polygon": [[-2, -1], [.5, -1], [.5, 5], [-2, 5]]},
                   {"id": "far", "polygon": [[10, 0], [12, 0], [12, 5], [10, 5]]}]
        original_far = list(records[1]["polygon"])
        subtract_reviewed_islands(records, islands)
        self.assertEqual(records[1]["polygon"], original_far)
        self.assertEqual(records[0]["id"], "near")
        self.assertLess(Polygon(_p[:2] for _p in records[0]["polygon"]).intersection(
            Polygon(_p[:2] for _p in islands[0]["polygon"])).area, 1e-6)
        with self.assertRaisesRegex(CompactDrivableError, "unique"):
            reviewed_non_drivable_islands({"features": reviewed["features"] * 2}, policy)
        reviewed["features"][1]["geometry"]["coordinates"][-1] = [0, 5]
        with self.assertRaisesRegex(CompactDrivableError, "tips do not close"):
            reviewed_non_drivable_islands(reviewed, policy)

    def test_same_direction_siblings_rejects_opposite_and_explicit_exclusion(self):
        links = {
            "route": link("route", "road", [(0, 0), (10, 0)]),
            "same": link("same", "road", [(0, 3), (10, 3)]),
            "opposite": link("opposite", "road", [(10, -3), (0, -3)]),
            "excluded": link("excluded", "road", [(0, 6), (10, 6)]),
        }
        selected, rejected = _same_direction_siblings(
            links, ["route"], {"excluded"}, 0.25
        )
        self.assertEqual(selected, ["route", "same"])
        self.assertEqual(rejected, ["excluded", "opposite"])

    def test_intersection_selection_is_explicit_not_proximity_based(self):
        values, groups = explicit_intersection_links({
            "explicit_intersection_groups": {
                "static": ["A", "B"],
                "dynamic": ["C"],
            }
        })
        self.assertEqual(values, ["A", "B", "C"])
        self.assertEqual(groups, {"A": "static", "B": "static", "C": "dynamic"})

    def test_lane_width_polygon_follows_declared_width(self):
        value = link("route", "road", [(0, 0), (10, 0)])
        value["width_start"] = 4.0
        value["width_end"] = 4.0
        polygon = np.asarray(lane_width_polygon(value, 0.5))
        self.assertAlmostEqual(float(polygon[:, 1].max()), 2.0, places=4)
        self.assertAlmostEqual(float(polygon[:, 1].min()), -2.0, places=4)
        self.assertAlmostEqual(float(polygon[:, 0].min()), 0.0, places=4)
        self.assertAlmostEqual(float(polygon[:, 0].max()), 10.0, places=4)

    def test_lane_width_widening_is_one_sided_half_sine_with_flush_ends(self):
        straight = link("turn", "road", [(0, 0), (10, 0)], link_type="1")
        ring = np.asarray(lane_width_polygon(
            straight, 0.25, {"side": "right", "maximum_extra_m": 1.0}
        ))[:, :2]
        half = len(ring) // 2
        left, right = ring[:half], ring[half:][::-1]
        self.assertTrue(np.allclose(left[:, 1], 1.75))
        self.assertAlmostEqual(right[0, 1], -1.75)
        self.assertAlmostEqual(right[-1, 1], -1.75)
        self.assertAlmostEqual(right[:, 1].min(), -2.75, places=3)
        self.assertAlmostEqual(right[np.argmin(right[:, 1]), 0], 5.0, places=3)

    def test_intersection_widening_accepts_only_reviewed_intersection_links(self):
        links = {
            "turn": link("turn", "road", [(0, 0), (10, 0)], link_type="1"),
            "road": link("road", "road", [(0, 0), (10, 0)]),
        }
        item = {"link_id": "turn", "side": "right", "maximum_extra_m": 1.0,
                "profile": "half_sine", "basis": "review"}
        widening = intersection_link_widening(
            {"intersection_link_widening": [item]}, links, {"turn", "road"}, {"1"}
        )
        self.assertEqual(widening["turn"]["maximum_extra_m"], 1.0)
        for bad in (
            {"link_id": "road"},
            {"link_id": "missing"},
            {"side": "outer"},
            {"maximum_extra_m": 3.5},
            {"profile": "linear"},
            {"basis": ""},
        ):
            with self.assertRaises((CompactDrivableError, KeyError)):
                intersection_link_widening(
                    {"intersection_link_widening": [dict(item, **bad)]},
                    links, {"turn", "road"}, {"1"},
                )

    def test_lane_boundary_polygon_follows_supported_reviewed_paint(self):
        value = link("route", "road", [(0, 0), (10, 0)])
        features = {
            "left": {"geometry": {"coordinates": [[0, 2.2, 0], [10, 2.2, 0]]}},
            "right": {"geometry": {"coordinates": [[0, -2.0, 0], [10, -2.0, 0]]}},
        }
        polygon, refinement = lane_boundary_polygon(
            value,
            0.5,
            {"left": "left", "right": "right"},
            features,
            {
                "maximum_distance_m": 4.0,
                "minimum_side_coverage": 0.2,
                "smoothing_window_m": 1.0,
            },
        )
        points = np.asarray(polygon)
        self.assertTrue(refinement["left_refined"])
        self.assertTrue(refinement["right_refined"])
        self.assertAlmostEqual(float(points[:, 1].max()), 2.2, places=4)
        self.assertAlmostEqual(float(points[:, 1].min()), -2.0, places=4)
        self.assertAlmostEqual(float(points[:, 0].min()), 0.0, places=4)
        self.assertAlmostEqual(float(points[:, 0].max()), 10.0, places=4)

    def test_corridor_bridge_handles_opposite_link_direction(self):
        first = link("first", "a", [(0, 0), (10, 0)])
        second = link("second", "b", [(10, 4), (0, 4)])
        polygon = np.asarray(corridor_bridge_polygon(first, second, 0.5))
        self.assertAlmostEqual(float(polygon[:, 0].min()), 0.0, places=4)
        self.assertAlmostEqual(float(polygon[:, 0].max()), 10.0, places=4)
        self.assertAlmostEqual(float(polygon[:, 1].min()), 0.0, places=4)
        self.assertAlmostEqual(float(polygon[:, 1].max()), 4.0, places=4)

    def test_node_pair_closure_adds_only_directed_internal_links(self):
        links = {
            "A": {**link("A", "road", [(0, 0), (10, 0)], "1"),
                  "from_node_idx": "S1", "to_node_idx": "T1"},
            "B": {**link("B", "road", [(0, 1), (10, 1)], "1"),
                  "from_node_idx": "S2", "to_node_idx": "T2"},
            "cross": {**link("cross", "road", [(0, 0), (10, 1)], "1"),
                      "from_node_idx": "S1", "to_node_idx": "T2"},
            "reverse": {**link("reverse", "road", [(10, 1), (0, 0)], "1"),
                        "from_node_idx": "T2", "to_node_idx": "S1"},
            "approach": {**link("approach", "road", [(-10, 0), (0, 0)], "6"),
                         "from_node_idx": "P", "to_node_idx": "S1"},
        }
        policy = {
            "intersection_node_pair_closures": {
                "dynamic": {"anchor_link_ids": ["A", "B"]}
            }
        }
        ids, groups, resolved = apply_intersection_node_pair_closures(
            links, policy, {"1"}, {"A": "dynamic", "B": "dynamic"}
        )
        self.assertEqual(ids, ["A", "B", "cross"])
        self.assertEqual(groups["cross"], "dynamic")
        self.assertEqual(resolved["dynamic"]["derived_link_ids"], ["cross"])


if __name__ == "__main__":
    unittest.main()
