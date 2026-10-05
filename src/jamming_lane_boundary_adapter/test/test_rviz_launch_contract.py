#!/usr/bin/env python3

import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


class ShadowBoundaryFamilyRvizContractTest(unittest.TestCase):
    def test_launch_starts_base_link_preset_by_default(self):
        root = ET.parse(
            PACKAGE_ROOT
            / "launch"
            / "shadow_boundary_family_candidate.launch"
        ).getroot()
        arguments = {item.get("name"): item for item in root.findall("arg")}
        self.assertEqual("true", arguments["rviz"].get("default"))
        self.assertIn(
            "rviz/shadow_boundary_family_candidate.rviz",
            arguments["rviz_config"].get("default"),
        )

        rviz_node = next(
            node
            for node in root.findall("node")
            if node.get("pkg") == "rviz"
        )
        self.assertEqual("$(arg rviz)", rviz_node.get("if"))
        self.assertEqual("rviz", rviz_node.get("type"))
        self.assertEqual("-d $(arg rviz_config)", rviz_node.get("args"))

    def test_preset_uses_base_link_and_both_family_views(self):
        config = yaml.safe_load(
            (
                PACKAGE_ROOT
                / "rviz"
                / "shadow_boundary_family_candidate.rviz"
            ).read_text(encoding="utf-8")
        )
        manager = config["Visualization Manager"]
        self.assertEqual("base_link", manager["Global Options"]["Fixed Frame"])
        topics = {
            display.get("Marker Topic"): display.get("Enabled")
            for display in manager["Displays"]
            if display.get("Class") == "rviz/MarkerArray"
        }
        self.assertTrue(
            topics["/planning/debug/lane_boundary_family/strips"]
        )
        self.assertTrue(
            topics["/perception/lane/debug/shared_family/markers"]
        )


if __name__ == "__main__":
    unittest.main()
