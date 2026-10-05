"""Check that L1-A's frozen synthetic inputs have internally consistent geometry.

These are reference values for the next runtime change, NOT tests of Son's code.
"""

import math
from pathlib import Path
import unittest

import yaml


FIXTURE = Path(__file__).with_name("fixtures") / "l1a_contract_cases.yaml"


def rotate_rpy(vector, degrees):
    roll, pitch, yaw = (math.radians(value) for value in degrees)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    x, y, z = vector
    x1, y1, z1 = x, cr * y - sr * z, sr * y + cr * z
    x2, y2, z2 = cp * x1 + sp * z1, y1, -sp * x1 + cp * z1
    return (cy * x2 - sy * y2, sy * x2 + cy * y2, z2)


class ContractFixtureTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fixture = yaml.safe_load(FIXTURE.read_text())

    def assert_vector_close(self, actual, expected):
        for index, (value, target) in enumerate(zip(actual, expected)):
            self.assertAlmostEqual(value, target, places=9, msg=f"axis {index}")

    def test_roll_pitch_mount_and_observation_stamp(self):
        case = self.fixture["coordinate_case"]
        mount = case["sensor_mount_base_m"]
        center_sensor = case["object_center_sensor_m"]
        center_base = tuple(a + b for a, b in zip(center_sensor, mount))
        self.assert_vector_close(center_base, case["expected_center_base_m"])
        pose = case["ego_pose_map_m"]
        degrees = case["ego_pose_rpy_deg"]
        center_map = tuple(a + b for a, b in zip(rotate_rpy(center_base, degrees), pose))
        self.assert_vector_close(center_map, case["expected_center_map_m"])

        length, width, height = case["object_obb_dimensions_m"]
        corner_base = (center_base[0] - length / 2,
                       center_base[1] - width / 2,
                       center_base[2] - height / 2)
        corner_map = tuple(a + b for a, b in zip(rotate_rpy(corner_base, degrees), pose))
        self.assert_vector_close(corner_map, case["expected_negative_local_axes_corner_map_m"])
        self.assertEqual(case["expected_observation_stamp_s"], case["observation_time_s"])
        self.assertNotEqual(case["observation_time_s"], case["processing_time_s"])
        self.assertNotEqual(case["observation_time_s"], case["receipt_time_s"])

    def test_velocity_includes_ego_translation_and_rotation(self):
        case = self.fixture["coordinate_case"]
        x, y, _ = case["expected_center_base_m"]
        yaw_rate = case["ego_yaw_rate_rps"]
        relative = case["object_velocity_base_relative_mps"]
        ego = case["ego_linear_velocity_base_mps"]
        body_absolute = (relative[0] + ego[0] - yaw_rate * y,
                         relative[1] + ego[1] + yaw_rate * x,
                         relative[2] + ego[2])
        self.assert_vector_close(rotate_rpy(body_absolute, case["ego_pose_rpy_deg"]),
                                 case["expected_absolute_velocity_map_mps"])

    def test_invalid_pose_and_camera_boundary_are_distinct(self):
        gps = {item["id"]: item for item in self.fixture["gps_cases"]["cases"]}
        self.assertEqual(set(gps), {"valid", "explicit_invalid", "stopped_receiving",
                                    "nan", "recovery_jump"})
        self.assertTrue(gps["valid"]["expected_map_available"])
        self.assertTrue(all(not row["expected_map_available"] for key, row in gps.items()
                            if key != "valid"))
        camera = {item["id"]: item for item in self.fixture["camera_cases"]["cases"]}
        limit = self.fixture["camera_cases"]["match_limit_ns_for_fixture"]
        self.assertLess(camera["matched"]["delta_ns"], limit)
        self.assertEqual(camera["boundary"]["delta_ns"], limit)
        self.assertGreater(camera["late"]["delta_ns"], limit)
        self.assertEqual(camera["valid_empty"]["expected_state"], "valid_empty")
        self.assertEqual(camera["missing"]["expected_state"], "missing")
        self.assertEqual(camera["disabled"]["expected_state"], "disabled")
        self.assertEqual(len(set(self.fixture["status_cases"].values())), 4)


if __name__ == "__main__":
    unittest.main()
