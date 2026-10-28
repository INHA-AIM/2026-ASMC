"""Native association must preserve memberships, ordering and audit decisions."""
import math
import unittest
from unittest.mock import patch

import numpy as np

from asmc_camera.bev import lane_tracking as tracking
from asmc_camera.bev.lane_evidence import LaneNode


def node(x, y, x_bin, class_id=1):
    return LaneNode(np.array([x, y], dtype=np.float64), class_id, .9, 1., 1, 8, x_bin)


class NativeAssociationTest(unittest.TestCase):
    def check_parity(self, nodes, **parameters):
        expected, audit = tracking.associate_lane_nodes_with_audit(nodes, backend='python', **parameters)
        actual, native_audit = tracking.associate_lane_nodes_with_audit(nodes, backend='native', **parameters)
        identities = {id(n): i for i, n in enumerate(nodes)}
        pack = lambda tracks: [[identities[id(n)] for n in t.nodes] for t in tracks]
        self.assertEqual(pack(expected), pack(actual))
        self.assertEqual(audit, native_audit)

    @unittest.skipIf(tracking._lane_association is None, 'native extension not built')
    def test_random_curves_gaps_noise_crossings_and_input_order(self):
        rng = np.random.RandomState(20261028)
        for scene in range(300):
            nodes = []
            for lane in range(1 + scene % 5):
                offset = rng.uniform(-4, 4)
                slope = rng.uniform(-.2, .2)
                curvature = rng.uniform(-.01, .01)
                for b in range(25):
                    if rng.random_sample() < .22:
                        continue
                    x = b * .25 + rng.uniform(-.08, .08)
                    y = offset + slope * x + curvature * x * x + rng.normal(0, .035)
                    nodes.append(node(x, y, b, 1 + lane % 2))
            rng.shuffle(nodes)
            self.check_parity(nodes, min_track_nodes=2 + scene % 4,
                              min_observed_span_m=.5 + (scene % 4) * .3)

    @unittest.skipIf(tracking._lane_association is None, 'native extension not built')
    def test_gate_edges_ties_repeated_x_and_degenerate_prediction(self):
        for residual in [np.nextafter(.45, -np.inf), .45, np.nextafter(.45, np.inf)]:
            self.check_parity([node(i * .5, 0., i) for i in range(6)] + [node(3., residual, 6)])
        for gap in [1e-6, np.nextafter(1e-6, np.inf), 2., np.nextafter(2., np.inf)]:
            self.check_parity([node(0., 0., 0), node(gap, 0., 1), node(gap + .5, 0., 2)],
                              min_track_nodes=2, min_observed_span_m=.1)
        for heading in [np.nextafter(20., 0.), 20., np.nextafter(20., 90.)]:
            self.check_parity([node(0., 0., 0), node(.5, math.tan(math.radians(heading)) * .5, 1)],
                              min_track_nodes=2, min_observed_span_m=.1)
        self.check_parity([node(i * .5, y, i) for i in range(12) for y in (-.1, 0., 0., .1)])
        self.check_parity([node(1., i * .01, i) for i in range(8)])
        self.check_parity([])

    def test_missing_extension_falls_back_and_explicit_native_reports_missing_build(self):
        nodes = [node(i * .5, 1., i) for i in range(6)]
        expected = tracking.associate_lane_nodes_with_audit(nodes, backend='python')
        with patch.object(tracking, '_lane_association', None):
            actual = tracking.associate_lane_nodes_with_audit(nodes)
            self.assertEqual(expected[1], actual[1])
            self.assertEqual([list(t.nodes) for t in expected[0]], [list(t.nodes) for t in actual[0]])
            with self.assertRaises(RuntimeError):
                tracking.associate_lane_nodes_with_audit(nodes, backend='native')

    def test_invalid_backend(self):
        with self.assertRaises(ValueError):
            tracking.associate_lane_nodes_with_audit([], backend='typo')

    @unittest.skipIf(tracking._lane_association is None, 'native extension not built')
    def test_non_float64_nodes_preserve_original_subtraction_semantics(self):
        from dataclasses import replace
        nodes = [replace(node(i * .5, .1 * i, i), xy=np.array([i * .5, .1 * i], np.float32)) for i in range(8)]
        self.check_parity(nodes)

    @unittest.skipIf(tracking._lane_association is None, 'native extension not built')
    def test_native_rejects_implicit_dtype_and_shape_conversion(self):
        native = tracking._lane_association
        labels = np.ones(3, np.int64)
        with self.assertRaises(TypeError):
            native.associate(np.zeros((3, 2), np.float32), labels, labels, 2., .45, 20., 4, 2.)
        with self.assertRaises(ValueError):
            native.associate(np.zeros((3, 3), np.float64), labels, labels, 2., .45, 20., 4, 2.)


if __name__ == '__main__':
    unittest.main()
