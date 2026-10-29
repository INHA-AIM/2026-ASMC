"""Prevent runtime launch overrides from shrinking the collision corridor."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml


class StaticAvoidLaunchMarginTest(unittest.TestCase):
    def test_effective_default_margin_matches_collision_checks(self):
        package = Path(__file__).resolve().parents[1]
        with (package / "config/frenet/params.yaml").open() as stream:
            expected = yaml.safe_load(stream)["planner"]["collision_check"]["safety_margin"]
        with (package / "config/behavior/behavior_params.yaml").open() as stream:
            self.assertEqual(yaml.safe_load(stream)["avoid"]["path_margin_m"], expected)
        roots = {}
        for name in ("autonomy_no_bridges_v2", "planner_v2"):
            root = ET.parse(package / "launch" / (name + ".launch")).getroot()
            roots[name] = root
            argument = root.find("arg[@name='avoid_path_margin_m']")
            self.assertIsNotNone(argument, name)
            self.assertEqual(float(argument.attrib["default"]), expected, name)
        forwarded = roots["autonomy_no_bridges_v2"].find(
            "include[@file='$(find asmc_planner)/launch/planner_v2.launch']/arg[@name='avoid_path_margin_m']"
        )
        self.assertIsNotNone(forwarded)
        self.assertEqual(forwarded.attrib["value"], "$(arg avoid_path_margin_m)")
        override = roots["planner_v2"].find(
            "param[@name='/behavior_planner/avoid/path_margin_m']"
        )
        self.assertIsNotNone(override)
        self.assertEqual(override.attrib["value"], "$(arg avoid_path_margin_m)")


if __name__ == "__main__":
    unittest.main()
