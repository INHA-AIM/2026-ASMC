#!/usr/bin/env python3
"""Keep the privileged LBC teacher tool outside production package paths."""
from __future__ import annotations

import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[4]
TOOL_ROOT = REPO_ROOT / "tools" / "research" / "lbc_teacher_bev"
CONTROLLER = (
    REPO_ROOT
    / "tools"
    / "aim_scenario_runner"
    / "utils"
    / "route_bev_visualizer_controller.py"
)
SCENARIO = (
    REPO_ROOT
    / "tools"
    / "aim_scenario_runner"
    / "zones"
    / "urban_drive_cases.py"
)


class LBCTeacherBEVLayoutTest(unittest.TestCase):
    def test_old_source_tree_is_absent(self) -> None:
        self.assertFalse((REPO_ROOT / "src" / "learning_by_cheating").exists())

    def test_scenario_runner_uses_research_tool(self) -> None:
        text = CONTROLLER.read_text(encoding="utf-8")
        self.assertIn('"lbc_teacher_bev"', text)
        self.assertNotIn('"learning_by_cheating"', text)

        scenario = SCENARIO.read_text(encoding="utf-8")
        self.assertNotIn(
            'get("route_bev_visualizer_enabled", True)', scenario
        )
        self.assertEqual(
            2,
            scenario.count('get("route_bev_visualizer_enabled", False)'),
        )

    def test_seven_channel_contract_is_retained(self) -> None:
        spec = (TOOL_ROOT / "lbc_bev" / "spec.py").read_text(encoding="utf-8")
        self.assertIn("NUM_CHANNELS = 7", spec)
        self.assertIn("MAP_SIZE = 320", spec)
        self.assertIn("CROP_SIZE = 192", spec)

    def test_runtime_outputs_are_external(self) -> None:
        collector = (TOOL_ROOT / "scripts" / "lbc_bev_collector.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("lbc_data_root", collector)
        self.assertNotIn("src/learning_by_cheating/data", collector)


if __name__ == "__main__":
    unittest.main()
