import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from analyze_speed_trial import measure


def row(t, speed, complete=True):
    return dict(monotonic=t, speed_kmh=speed, station=t,
                observation_complete=complete)


class SpeedMeasurementTest(unittest.TestCase):
    def test_time_weighting_includes_stop_and_handles_irregular_samples(self):
        result = measure([row(0, 0), row(.2, 0), row(.3, 36), row(.6, 36)], True)
        self.assertAlmostEqual(result['mean_including_stops_kmh'], 21)
        self.assertAlmostEqual(result['stopped_below_036_kmh_s'], .2)
        self.assertAlmostEqual(result['distance_m'], 3.5)

    def test_partial_drive_and_observation_gap_cannot_claim_target(self):
        for rows, completed in (([row(0, 55), row(.2, 55)], False),
                                ([row(0, 55), row(.4, 55)], True),
                                ([row(0, 55), row(.2, 55, False)], True)):
            result = measure(rows, completed)
            self.assertIsNone(result['full_lap_mean_kmh'])
            self.assertFalse(result['original_50_kmh_goal_met'])

    def test_overspeed_and_bad_timestamps_are_not_accepted(self):
        result = measure([row(0, 59.1), row(.2, 59.1)], True)
        self.assertFalse(result['observed_within_59_kmh_ceiling'])
        self.assertFalse(result['original_50_kmh_goal_met'])
        with self.assertRaises(ValueError):
            measure([row(0, 50), row(0, 50)])
        with self.assertRaises(ValueError):
            measure([row(0, 50), row(.2, float('nan'))])


if __name__ == '__main__':
    unittest.main()
