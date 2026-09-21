import unittest

import numpy as np

from asmc_hd_map.nominal import (
    build_nominal_boundaries,
    build_route_stoplines,
    variable_width_boundaries,
)


def _link(idx, y, lane, left=None, right=None, from_node="n0", to_node="n1"):
    return {
        "idx": idx,
        "points": [[0.0, y, 0.0], [10.0, y, 0.0]],
        "from_node_idx": from_node,
        "to_node_idx": to_node,
        "road_id": "road",
        "ego_lane": lane,
        "link_type": "6",
        "width_start": 3.0,
        "width_end": 4.0,
        "left_lane_change_dst_link_idx": left,
        "right_lane_change_dst_link_idx": right,
    }


class NominalGeometryTest(unittest.TestCase):
    def test_variable_width_offset_uses_start_and_end_width(self):
        boundaries = variable_width_boundaries(_link("a", 0.0, 1), 0.5)
        self.assertAlmostEqual(boundaries["left"][0, 1], 1.5, places=6)
        self.assertAlmostEqual(boundaries["left"][-1, 1], 2.0, places=6)
        self.assertAlmostEqual(boundaries["right"][0, 1], -1.5, places=6)
        self.assertAlmostEqual(boundaries["right"][-1, 1], -2.0, places=6)

    def test_adjacent_links_share_one_boundary_candidate(self):
        first = _link("a", 0.0, 1, left="b")
        second = _link("b", 3.0, 2, right="a")
        links = {"a": first, "b": second}
        relations = {
            "a": {
                "left_adjacent_link_id": "b", "right_adjacent_link_id": None,
                "is_merge_target": False, "is_split_source": False,
            },
            "b": {
                "left_adjacent_link_id": None, "right_adjacent_link_id": "a",
                "is_merge_target": False, "is_split_source": False,
            },
        }
        boundaries, stats = build_nominal_boundaries(
            {"a", "b"}, links, relations, step_m=0.5,
            minimum_link_length_m=1.0, excluded_link_types={"1"},
            shared_boundary_max_p95_m=0.75,
        )
        self.assertEqual(stats["raw_side_count"], 4)
        self.assertEqual(stats["deduplicated_boundary_count"], 3)
        self.assertEqual(stats["shared_boundary_count"], 1)
        shared = next(item for item in boundaries if item["boundary_role"] == "shared")
        self.assertEqual(shared["current_link_ids"], ["a", "b"])

    def test_stopline_group_spans_adjacent_lane_nodes(self):
        links = [
            _link("a", 0.0, 1, from_node="a0", to_node="s0"),
            _link("b", 3.5, 2, from_node="b0", to_node="s1"),
            _link("back", 0.0, 1, from_node="s0", to_node="a0"),
        ]
        nodes = [
            {"idx": "a0", "point": [0.0, 0.0, 0.0], "on_stop_line": False},
            {"idx": "b0", "point": [0.0, 3.5, 0.0], "on_stop_line": False},
            {"idx": "s0", "point": [10.0, 0.0, 0.0], "on_stop_line": True,
             "traffic_light_id": "signal"},
            {"idx": "s1", "point": [10.0, 3.5, 0.0], "on_stop_line": True,
             "traffic_light_id": "signal"},
        ]
        route_records = [{"link_id": "a", "start_s_m": 0.0, "end_s_m": 10.0}]
        features, stats = build_route_stoplines(
            route_records, ["a"], links, nodes,
            group_radius_m=12.0, max_heading_diff_deg=30.0,
            max_longitudinal_m=1.5,
        )
        self.assertEqual(stats["route_stop_node_count"], 1)
        self.assertEqual(len(features), 1)
        self.assertEqual(features[0]["node_ids"], ["s0", "s1"])
        # The incoming links are 4.0m wide at their end, so two centers 3.5m
        # apart span 2.0 + 3.5 + 2.0 = 7.5m.
        self.assertAlmostEqual(features[0]["length_m"], 7.5, places=5)
        self.assertTrue(features[0]["signal_controlled"])


if __name__ == "__main__":
    unittest.main()
