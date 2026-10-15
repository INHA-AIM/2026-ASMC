#!/usr/bin/env python3

import io
import sys
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "tools" / "audit"))

from check_legacy_lidar_publishers import check, main  # noqa: E402


NODE = "src/asmc_hd_map/development/won_tae_link_node/drivable_filter_node.py"


def write(root: Path, relative: str, text: str) -> Path:
    path = root / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")
    return path


class LegacyLidarPublisherCheckTest(unittest.TestCase):
    def setUp(self):
        self._directory = tempfile.TemporaryDirectory()
        self.root = Path(self._directory.name)
        write(self.root, "pkg/launch/base.launch", '<launch>\n  <node pkg="Main" type="main" name="Main"/>\n</launch>\n')

    def tearDown(self):
        self._directory.cleanup()

    def scan(self):
        return check(self.root, use_git=False)

    def test_repository_launches_never_start_drivable_filter_node(self):
        scanned, errors = check(REPO_ROOT)
        self.assertEqual([], errors)
        names = {path.relative_to(REPO_ROOT).as_posix() for path in scanned}
        for expected in (
            "src/lidar+sensorfusion/src/Main/main.launch",
            "src/lidar+sensorfusion/src/Main/main_integration.launch",
            "src/lidar+sensorfusion/src/Main/test/main_integration_gating.test",
            "src/planning_inputs/launch/objects.launch",
            "src/asmc_planner/launch/autonomy_no_bridges_v2.launch",
            "src/lidar+sensorfusion/start_main.sh",
        ):
            self.assertIn(expected, names)
        # The node itself exists; the check is about who starts it, not about deleting it.
        self.assertTrue((REPO_ROOT / NODE).is_file())

    def test_node_element_in_a_launch_file_fails(self):
        write(
            self.root,
            "pkg/launch/stack.launch",
            "<launch>\n"
            '  <include file="$(find Main)/main.launch"/>\n'
            '  <node pkg="asmc_hd_map" type="drivable_filter_node.py" name="drivable_filter"/>\n'
            "</launch>\n",
        )
        _, errors = self.scan()
        self.assertEqual(1, len(errors), errors)
        self.assertIn("pkg/launch/stack.launch:3:", errors[0])
        self.assertIn("second /last_lidar publisher", errors[0])

    def test_rostest_and_launch_xml_files_are_scanned(self):
        write(self.root, "pkg/test/smoke.test", '<launch>\n<node pkg="x" type="drivable_filter_node.py" name="f"/>\n</launch>\n')
        write(self.root, "pkg/launch/stack.launch.xml", '<launch>\n<node pkg="x" type="Drivable_Filter_Node" name="f"/>\n</launch>\n')
        _, errors = self.scan()
        self.assertEqual(2, len(errors), errors)

    def test_command_param_or_args_reference_fails(self):
        write(
            self.root,
            "pkg/launch/manual.launch",
            "<launch>\n"
            '  <node pkg="rosbash" type="rosrun" name="filter"\n'
            f'        args="python3 $(find asmc_hd_map)/../../{NODE}"/>\n'
            "</launch>\n",
        )
        _, errors = self.scan()
        self.assertEqual(1, len(errors), errors)
        self.assertIn("manual.launch:3:", errors[0])

    def test_xml_comment_mention_is_allowed(self):
        write(
            self.root,
            "pkg/launch/doc.launch",
            "<launch>\n"
            "  <!-- drivable_filter_node.py is a manual cross-check;\n"
            "       never start it next to Main. -->\n"
            '  <node pkg="Main" type="main" name="Main"/>\n'
            "</launch>\n",
        )
        scanned, errors = self.scan()
        self.assertEqual([], errors)
        self.assertEqual(2, len(scanned))

    def test_shell_launcher_fails_but_shell_comment_is_allowed(self):
        write(self.root, "scripts/start.sh", f"#!/bin/bash\nroslaunch Main main.launch &\npython3 {NODE} _margin_m:=0.5\n")
        write(self.root, "scripts/notes.sh", "#!/bin/bash\n# do not run drivable_filter_node.py here\necho ok  # drivable_filter_node\n")
        _, errors = self.scan()
        self.assertEqual(1, len(errors), errors)
        self.assertIn("scripts/start.sh:3: shell launcher", errors[0])

    def test_consumers_of_last_lidar_are_not_flagged(self):
        write(self.root, "pkg/launch/objects.launch", '<launch>\n  <arg name="input_topic" default="/last_lidar"/>\n</launch>\n')
        _, errors = self.scan()
        self.assertEqual([], errors)

    def test_scan_without_launch_files_fails_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            write(Path(directory), "README.md", "no launch here\n")
            _, errors = check(Path(directory), use_git=False)
            self.assertEqual(1, len(errors))
            self.assertIn("refusing to pass an empty scan", errors[0])

    def test_cli_exit_codes(self):
        out, err = io.StringIO(), io.StringIO()
        with redirect_stdout(out), redirect_stderr(err):
            self.assertEqual(0, main(["--root", str(self.root), "--no-git"]))
        self.assertIn("check passed", out.getvalue())
        write(self.root, "pkg/launch/bad.launch", '<launch><node pkg="x" type="drivable_filter_node.py" name="f"/></launch>\n')
        out, err = io.StringIO(), io.StringIO()
        with redirect_stdout(out), redirect_stderr(err):
            self.assertEqual(1, main(["--root", str(self.root), "--no-git"]))
        self.assertIn("pkg/launch/bad.launch:1:", err.getvalue())


if __name__ == "__main__":
    unittest.main()
