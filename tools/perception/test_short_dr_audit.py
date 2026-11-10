"""Offline pilot must not anchor on states that arrived in the future."""

import unittest
from tools.perception.audit_short_dr import causal_point, interpolate, stats


class ShortDrTest(unittest.TestCase):
    def test_future_arrival_cannot_anchor(self):
        self.assertIsNone(
            causal_point(
                [(1.0, 0.0, 0.0), (1.02, 2.0, 0.0)], {1.0: 1.0, 1.02: 1.03}, 1.01, 1.01
            )
        )
        rows = [(1.0, 0.0, 0.0), (1.02, 2.0, 0.0), (1.04, 999.0, 0.0)]
        self.assertEqual(
            causal_point(rows, {1.0: 1.0, 1.02: 1.02, 1.04: 1.2}, 1.03, 1.03),
            (3.0, 0.0),
        )
        self.assertIsNone(
            causal_point(rows, {1.0: 1.0, 1.02: 1.02, 1.04: 1.2}, 1.2, 1.2)
        )

    def test_score_does_not_bridge_unavailable_intervals(self):
        self.assertIsNone(interpolate([(1.0, 0.0, 0.0), (2.0, 1.0, 1.0)], 1.5))
        self.assertEqual(
            interpolate([(1.0, 0.0, 0.0), (1.1, 1.0, 1.0)], 1.05), (0.5, 0.5)
        )
        self.assertIsNone(stats([])["p95"])


if __name__ == "__main__":
    unittest.main()
