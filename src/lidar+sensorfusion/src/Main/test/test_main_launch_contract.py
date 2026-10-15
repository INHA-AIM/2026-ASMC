#!/usr/bin/env python3
"""Launch contract: production main.launch defaults and the B0 main_integration.launch profile.

No ROS master is needed: only `roslaunch --dump-params` is used.
"""

import os
import re
import subprocess
import unittest
import xml.etree.ElementTree as ET

import yaml

PACKAGE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAIN_LAUNCH = os.path.join(PACKAGE, "main.launch")
INTEGRATION_LAUNCH = os.path.join(PACKAGE, "main_integration.launch")
PARAM_LINE = re.compile(r"^(/\S+): (.*)$")


def dump(launch, *args):
    result = subprocess.run(
        ["roslaunch", "--dump-params", launch] + list(args),
        stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        universal_newlines=True, timeout=180, check=False)
    if result.returncode != 0:
        raise AssertionError("roslaunch --dump-params failed: %s" % result.stderr[-2000:])
    params = {}
    for line in result.stdout.splitlines():
        match = PARAM_LINE.match(line)
        if match:
            params[match.group(1)] = yaml.safe_load(match.group(2))
    if not params:
        raise AssertionError("no parameters dumped: %s" % result.stdout[-2000:])
    return params


def arg_defaults(path):
    root = ET.parse(path).getroot()
    return {arg.get("name"): arg.get("default") for arg in root.findall("arg")}


def main_params(params):
    return {key: value for key, value in params.items() if key.startswith("/Main/")}


class MainLaunchContractTest(unittest.TestCase):
    def test_production_main_launch_defaults_unchanged(self):
        defaults = arg_defaults(MAIN_LAUNCH)
        self.assertEqual(defaults["enable_common_geometry"], "false")
        self.assertEqual(defaults["lidar_level_roll"], "false")
        self.assertEqual(defaults["planner_udp_enabled"], "true")
        params = dump(MAIN_LAUNCH)
        self.assertIs(params["/Main/enable_common_geometry"], False)
        self.assertIs(params["/Main/lidar_level_roll"], False)
        self.assertIs(params["/Main/planner_udp_enabled"], True)

    def test_production_main_launch_planner_udp_override(self):
        params = dump(MAIN_LAUNCH, "planner_udp_enabled:=false")
        self.assertIs(params["/Main/planner_udp_enabled"], False)
        self.assertIs(params["/Main/enable_common_geometry"], False)

    def test_integration_launch_changes_only_common_geometry_and_roll(self):
        production = main_params(dump(MAIN_LAUNCH))
        integration = main_params(dump(INTEGRATION_LAUNCH))
        self.assertEqual(set(production), set(integration))
        changed = {key: (production[key], integration[key])
                   for key in production if production[key] != integration[key]}
        self.assertEqual(changed, {
            "/Main/enable_common_geometry": (False, True),
            "/Main/lidar_level_roll": (False, True),
        })
        self.assertIs(integration["/Main/planner_udp_enabled"], True)   # production default

    def test_integration_launch_planner_udp_override(self):
        params = dump(INTEGRATION_LAUNCH, "planner_udp_enabled:=false")
        self.assertIs(params["/Main/planner_udp_enabled"], False)
        self.assertIs(params["/Main/enable_common_geometry"], True)

    def test_integration_launch_pins_roll_and_pitch_leveling(self):
        defaults = arg_defaults(INTEGRATION_LAUNCH)
        self.assertNotIn("lidar_level_roll", defaults)
        self.assertNotIn("enable_common_geometry", defaults)
        params = dump(INTEGRATION_LAUNCH, "lidar_level_roll:=false", "enable_common_geometry:=false")
        self.assertIs(params["/Main/lidar_level_roll"], True)
        self.assertIs(params["/Main/enable_common_geometry"], True)


if __name__ == "__main__":
    unittest.main()
