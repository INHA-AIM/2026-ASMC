from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from audit_stationary_scenes import measurement_windows


class SceneWindowTest(unittest.TestCase):
    def test_transition_settling_is_outside_measurement(self):
        scenes = [{"name": "start", "transition_wall_s": 1., "measurement_start_wall_s": 4., "measurement_end_wall_s": 9.},
                  {"name": "next", "transition_wall_s": 9., "measurement_start_wall_s": 12., "measurement_end_wall_s": 17.}]
        self.assertEqual(measurement_windows({"scenes": scenes}),
                         [("start", 4_000_000_000, 9_000_000_000), ("next", 12_000_000_000, 17_000_000_000)])

    def test_incomplete_or_overlapping_scenes_fail(self):
        for scenes in [[], [{"name": "incomplete"}],
                       [{"name": "a", "measurement_start_wall_s": 1, "measurement_end_wall_s": 4},
                        {"name": "b", "measurement_start_wall_s": 3, "measurement_end_wall_s": 5}]]:
            with self.assertRaises(ValueError):
                measurement_windows({"scenes": scenes})


if __name__ == "__main__":
    unittest.main()
