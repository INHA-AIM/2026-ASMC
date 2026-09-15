import unittest

import numpy as np

from hd_map_builder.corridor import (
    clip_parallel_link_points,
    expand_context_links,
    route_link_records,
    select_parallel_links,
    topology_indices,
    validate_route_chain,
)


def _link(idx, start, end, from_node, to_node):
    return {
        "idx": idx,
        "points": [[*start, 0.0], [*end, 0.0]],
        "from_node_idx": from_node,
        "to_node_idx": to_node,
    }


class CorridorTest(unittest.TestCase):
    def test_route_chain_is_closed_and_stationed(self):
        links = {
            "a": _link("a", (0, 0), (10, 0), "n0", "n1"),
            "b": _link("b", (10, 0), (0, 0), "n1", "n0"),
        }
        validate_route_chain(["a", "b"], links)
        records = route_link_records(["a", "b"], links)
        self.assertAlmostEqual(records[0]["start_s_m"], 0.0)
        self.assertAlmostEqual(records[1]["end_s_m"], 20.0)

    def test_route_chain_rejects_topology_gap(self):
        links = {
            "a": _link("a", (0, 0), (10, 0), "n0", "n1"),
            "b": _link("b", (10, 0), (0, 0), "other", "n0"),
        }
        with self.assertRaises(ValueError):
            validate_route_chain(["a", "b"], links)

    def test_parallel_selection_excludes_perpendicular_crossing(self):
        route_points = np.column_stack((np.arange(21.0), np.zeros(21), np.zeros(21)))
        route_tangents = np.tile(np.array([[1.0, 0.0]]), (21, 1))
        links = [
            _link("parallel", (0, 3), (20, 3), "p0", "p1"),
            _link("crossing", (10, -5), (10, 5), "c0", "c1"),
        ]
        selected = select_parallel_links(
            links, route_points, route_tangents,
            half_width_m=5.0, max_heading_deg=30.0, minimum_support=3,
        )
        self.assertEqual(selected, {"parallel"})

    def test_parallel_selection_distinguishes_opposite_direction(self):
        route_points = np.column_stack((np.arange(21.0), np.zeros(21), np.zeros(21)))
        route_tangents = np.tile(np.array([[1.0, 0.0]]), (21, 1))
        links = [_link("opposite", (20, 3), (0, 3), "p1", "p0")]
        directed = select_parallel_links(
            links, route_points, route_tangents,
            half_width_m=5.0, max_heading_deg=30.0, minimum_support=3,
        )
        undirected = select_parallel_links(
            links, route_points, route_tangents,
            half_width_m=5.0, max_heading_deg=30.0, minimum_support=3,
            undirected_heading=True,
        )
        self.assertEqual(directed, set())
        self.assertEqual(undirected, {"opposite"})

    def test_parallel_geometry_is_clipped_to_route_only_segment(self):
        route_points = np.column_stack((np.arange(21.0), np.zeros(21), np.zeros(21)))
        route_tangents = np.tile(np.array([[1.0, 0.0]]), (21, 1))
        link = {
            "idx": "long_parallel",
            "points": [[-20.0, 3.0, 0.0], [40.0, 3.0, 0.0]],
        }
        clipped, stats = clip_parallel_link_points(
            link, route_points, route_tangents,
            half_width_m=5.0, max_heading_deg=30.0,
            sample_step_m=0.5, minimum_segment_length_m=2.0,
        )
        self.assertGreaterEqual(clipped[0, 0], -5.0)
        self.assertLessEqual(clipped[-1, 0], 25.0)
        self.assertLess(stats["corridor_length_m"], stats["original_length_m"])
        self.assertEqual(stats["valid_run_count"], 1)

    def test_context_expansion_keeps_direction_continuous_links(self):
        links = [
            _link("before", (-10, 0), (0, 0), "n0", "n1"),
            _link("seed", (0, 0), (10, 0), "n1", "n2"),
            _link("after", (10, 0), (20, 0), "n2", "n3"),
            _link("turn", (10, 0), (10, 10), "n2", "n4"),
        ]
        by_id = {link["idx"]: link for link in links}
        incoming, outgoing = topology_indices(links)
        selected = expand_context_links(
            {"seed"}, by_id, incoming, outgoing, hops=1, max_heading_deg=30.0,
        )
        self.assertEqual(selected, {"before", "seed", "after"})


if __name__ == "__main__":
    unittest.main()
