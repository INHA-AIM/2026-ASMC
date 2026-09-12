#!/usr/bin/env python3

import unittest

import numpy as np

from drivable_bev.lane_evidence import LaneNode
from drivable_bev.lane_tracking import associate_lane_nodes


def node(x, y, x_bin, class_id=1):
    return LaneNode(np.asarray([x, y]), class_id, 0.9, 1.0, 1, 8, x_bin)


class LaneTrackingTest(unittest.TestCase):
    def test_short_gap_is_connected_and_long_gap_is_not(self):
        nodes = [
            node(0.0, 1.0, 0),
            node(0.5, 1.05, 1),
            node(1.0, 1.10, 2),
            node(2.5, 1.20, 5),
            node(3.0, 1.25, 6),
            node(6.0, 1.40, 12),
        ]
        tracks = associate_lane_nodes(
            nodes, min_track_nodes=4, min_observed_span_m=2.0
        )
        self.assertEqual(1, len(tracks))
        self.assertEqual(5, len(tracks[0].nodes))
        self.assertLess(float(tracks[0].xy[-1, 0]), 4.0)

    def test_classes_are_never_merged(self):
        nodes = []
        for index in range(5):
            nodes.append(node(index * 0.5, 1.0, index, 1))
            nodes.append(node(index * 0.5, 1.05, index, 2))
        tracks = associate_lane_nodes(nodes)
        self.assertEqual({1, 2}, {track.class_id for track in tracks})


if __name__ == "__main__":
    unittest.main()
