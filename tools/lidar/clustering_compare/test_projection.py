import unittest
import numpy as np

if __package__:
    from .projection import project, audit_cloud
else:
    from projection import project, audit_cloud

AOS = {"min_vertical_angle_deg": -15, "max_vertical_angle_deg": 15,
       "vertical_channels": 16, "horizontal_columns": 1200, "min_range_m": .1,
       "max_range_m": 100, "angle_uncertainty_deg": 1}


def point(elevation, azimuth=0, distance=10):
    el, az = np.radians([elevation, azimuth])
    return [distance * np.cos(el) * np.cos(az), distance * np.cos(el) * np.sin(az),
            distance * np.sin(el), 101]


class ProjectionTest(unittest.TestCase):
    def test_top_channel_boundary_and_azimuth_wrap(self):
        r = project(np.array([point(15.2, 179.999), point(-15.2, -179.999)]), AOS)
        self.assertEqual(r["rows"].tolist(), [15, 0])
        self.assertTrue(all(r["reasons"] == "eligible"))
        self.assertTrue(np.all((r["columns"] >= 0) & (r["columns"] < 1200)))

    def test_invalid_range_and_uncertainty_remain_distinct(self):
        r = project(np.array([point(17), [0, 0, 0, 0], [np.nan, 1, 2, 3]]), AOS)
        self.assertEqual(r["reasons"].tolist(), ["ambiguous_channel", "range", "nonfinite"])

    def test_cell_collision_is_visible_and_does_not_change_intensity(self):
        points = np.array([point(1), point(1, distance=11)], dtype=np.float32)
        before = points.copy()
        result = project(points, AOS)
        self.assertEqual(len(np.unique(result["cells"])), 1)
        np.testing.assert_array_equal(points, before)

    def test_column_resolution_belongs_to_each_instance(self):
        p = np.array([point(1, azimuth=90)])
        self.assertEqual(project(p, AOS)["columns"][0], 900)
        self.assertEqual(project(p, dict(AOS, horizontal_columns=1440))["columns"][0], 1080)

    def test_point_count_does_not_prove_channel_layout(self):
        bad = np.array([point(1, azimuth=i * .3) for i in range(32)])
        config = {"expected_interleaved_elevations_deg": [-15, 1, -13, 3, -11, 5, -9, 7,
                                                            -7, 9, -5, 11, -3, 13, -1, 15],
                  "expected_capture_slots": 32, "slot_channel_mismatch_fraction_max": .005,
                  "elevation_residual_p99_deg_max": .8,
                  "adjacent_azimuth_step_residual_p95_deg_max": .01}
        result = audit_cloud(bad, AOS, config)
        self.assertTrue(result["checks"]["slot_count"])
        self.assertFalse(result["checks"]["channel_layout"])
        self.assertFalse(result["admitted"])


if __name__ == "__main__":
    unittest.main()
