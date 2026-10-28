#!/usr/bin/env python3
import copy
import importlib.util
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
TOOL = PACKAGE / "tools/configure_point_refinement.py"
spec = importlib.util.spec_from_file_location("refinement_profiles", TOOL)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class RefinementProfileTest(unittest.TestCase):
    def setUp(self):
        self.base = yaml.safe_load((PACKAGE / "config/lidar_pipeline.yaml").read_text())

    def test_preserves_main_model_views_tolerances_and_motion_without_mutating_input(self):
        original = copy.deepcopy(self.base)
        output = module.configured(self.base)
        self.assertEqual(self.base, original)
        for key in ("fusion", "ground", "motion", "vehicle", "map"):
            self.assertEqual(output[key], original[key])
        self.assertEqual(output["lidar"], original["lidar"])
        self.assertEqual(output["fusion"]["views"], "all")
        self.assertEqual(output["fusion"]["camera_class_profile"], "mission15_no_cone_v1")

    def test_low_obstacle_tuning_keeps_front_and_rear_tolerances_and_voxel_absent(self):
        output = module.configured(self.base, tuning="low-obstacle")
        self.assertEqual(output["lidar"]["Clustering"]["MinPoints"], 3)
        self.assertEqual(output["lidar"]["Clustering"]["Adaptive"], self.base["lidar"]["Clustering"]["Adaptive"])
        self.assertEqual(output["ground"]["ground_distance_threshold_near_m"], 0.23)
        self.assertEqual(output["ground"]["ground_threshold_far_range_m"], 32.0)
        self.assertEqual(output["experimental"]["tunnel_filter"]["original_parameters"]["cluster_min_points"], 3)
        self.assertNotIn("voxel_leaf_m", output["lidar"])

    def test_profiles_control_both_features_and_disabled_path(self):
        for name in ("disabled", "steel", "wall", "integrated"):
            c = module.configured(self.base, name)["camera_point_refinement"]
            self.assertEqual(c["steel_fragment_merge"], name in ("steel", "integrated"))
            self.assertEqual(c["enabled"], name in ("wall", "integrated"))
            self.assertEqual(c["detached_wall_cleanup"], name in ("wall", "integrated"))
        with self.assertRaises(ValueError):
            module.configured(self.base, "unknown")

    def test_cli_never_overwrites_input_or_existing_output(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "input.yaml"
            target = Path(directory) / "output.yaml"
            source.write_text(yaml.safe_dump(self.base))
            before = source.read_bytes()
            prefix = [sys.executable, str(TOOL), "--input", str(source), "--output"]
            self.assertNotEqual(subprocess.run(prefix + [str(source)], capture_output=True).returncode, 0)
            self.assertEqual(subprocess.run(prefix + [str(target)], capture_output=True).returncode, 0)
            self.assertNotEqual(subprocess.run(prefix + [str(target)], capture_output=True).returncode, 0)
            self.assertEqual(source.read_bytes(), before)


if __name__ == "__main__":
    unittest.main(verbosity=2)
