#!/usr/bin/env python3
"""Package isolation contract of the asmc_lidar shadow pipeline (no ROS needed).

- package.xml names no Main/Msg/asmc_camera/tensorrt/torch dependency;
- catkin_package() exports are exactly the pre-shadow ones (the shadow
  libraries stay package-private, so Main never links them);
- no asmc_lidar source includes Main/Msg headers or hard-codes /home paths;
- launch files start only asmc_lidar_shadow_node (outputs under
  /asmc_lidar/shadow) and, in monitor.launch, the read-only web monitor node
  bound to loopback on a port other than the planner dashboard's 8088;
  no other launch file in the repository starts or includes the shadow node;
- the monitor scripts are executable, import no ROS message publisher and
  the web assets reference no external URL;
- config/shadow_pipeline.yaml uses the mount of config/autonav_v3.yaml and the
  HD-map origin of config/contracts/hd_map_topic_contract_v1.yaml;
- the rostest node script is executable (and tracked as 100755 once staged) and never skips
  a case (rosunit would report the skip as a pass).
"""
import os
from pathlib import Path
import re
import subprocess
import unittest
import xml.etree.ElementTree as ET

import yaml

PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parents[1]  # repository root when the package sits in <root>/src/asmc_lidar
SELF = Path(__file__).resolve()
SHADOW_NODE = 'asmc_lidar_shadow_node'
MONITOR_NODE = 'asmc_lidar_monitor_node.py'
MONITOR_SCRIPTS = ('scripts/asmc_lidar_monitor_node.py', 'scripts/asmc_lidar_monitor_core.py')
WEB_ASSETS = ('web/index.html', 'web/app.js', 'web/monitor_view.js', 'web/app.css')
PLANNER_DASHBOARD_PORT = '8088'
OUTPUT_ARGS = ('objects_local_topic', 'objects_map_topic', 'diagnostics_topic', 'trace_topic')
FORBIDDEN_DEPENDENCIES = {'Main', 'Msg', 'asmc_camera', 'tensorrt', 'torch'}
SOURCE_SUFFIXES = {'.c', '.cc', '.cpp', '.h', '.hpp', '.py', '.launch', '.test', '.yaml', '.yml', '.xml',
                   '.txt', '.cmake', '.sh'}
PRUNED_DIRECTORIES = {'.git', 'build', 'devel', 'install', 'logs', '__pycache__', 'node_modules'}


def repository_available():
    return (ROOT / 'src' / 'asmc_lidar').resolve() == PACKAGE and (ROOT / 'config' / 'contracts').is_dir()


def walk(base, suffixes):
    for directory, subdirectories, files in os.walk(base):
        subdirectories[:] = [d for d in subdirectories if d not in PRUNED_DIRECTORIES]
        for name in files:
            path = Path(directory) / name
            if path.suffix in suffixes or name == 'CMakeLists.txt':
                yield path


def catkin_package_block(text):
    match = re.search(r'catkin_package\((.*?)\)', text, re.S)
    if not match:
        raise AssertionError('catkin_package() not found')
    sections, current = {}, None
    for token in match.group(1).split():
        if token in ('INCLUDE_DIRS', 'LIBRARIES', 'CATKIN_DEPENDS', 'DEPENDS', 'CFG_EXTRAS'):
            current = sections.setdefault(token, [])
        elif current is not None:
            current.append(token)
    return sections


def without_comments(text, marker):
    return '\n'.join(line.split(marker, 1)[0] for line in text.splitlines())


class ShadowContractTest(unittest.TestCase):
    def test_package_xml_has_no_forbidden_dependency(self):
        package = ET.parse(PACKAGE / 'package.xml').getroot()
        dependencies = {node.text.strip() for node in package if node.tag.endswith('depend') and node.text}
        self.assertFalse(FORBIDDEN_DEPENDENCIES & dependencies)

    def test_catkin_exports_are_unchanged(self):
        sections = catkin_package_block(without_comments((PACKAGE / 'CMakeLists.txt').read_text(), '#'))
        self.assertEqual(sections.get('LIBRARIES'),
                         ['asmc_lidar_frame_time', 'asmc_lidar_camera_core', 'asmc_lidar_camera_ros'])
        self.assertEqual(sections.get('CATKIN_DEPENDS'), ['asmc_msgs', 'roscpp', 'diagnostic_msgs'])
        self.assertEqual(sections.get('DEPENDS'), ['Eigen3', 'yaml-cpp', 'OpenSSL'])
        self.assertEqual(sections.get('INCLUDE_DIRS'), ['include', '${EIGEN3_INCLUDE_DIRS}'])

    def test_sources_do_not_include_main_msg_or_absolute_paths(self):
        include = re.compile(r'^\s*#\s*include\s*[<"]([^>"]+)[>"]', re.M)
        offenders = []
        for path in walk(PACKAGE, SOURCE_SUFFIXES):
            if path.resolve() == SELF:
                continue
            text = path.read_text(errors='replace')
            for header in include.findall(text):
                if header.endswith('Global/Global.hpp') or header.startswith('Msg/') or 'lidar+sensorfusion' in header:
                    offenders.append('%s includes %s' % (path, header))
            if path.name == 'CMakeLists.txt' and 'lidar+sensorfusion' in without_comments(text, '#'):
                offenders.append('%s references lidar+sensorfusion' % path)
            if path.suffix in ('.launch', '.test') and re.search(r'\$\(find (Main|Msg)\)|lidar\+sensorfusion', text):
                offenders.append('%s references Main/Msg' % path)
            if re.search(r'(?<![\w.])/home/', text):
                offenders.append('%s hard-codes a /home path' % path)
        self.assertEqual(offenders, [])

    def test_launch_files_start_only_the_shadow_node(self):
        launches = sorted((PACKAGE / 'launch').glob('*.launch'))
        self.assertTrue(launches, 'shadow launch files are expected')
        for path in launches:
            root = ET.parse(path).getroot()
            self.assertEqual(root.findall('.//include'), [], path)
            all_nodes = root.findall('.//node')
            monitors = [n for n in all_nodes if n.get('type') == MONITOR_NODE]
            nodes = [n for n in all_nodes if n.get('type') != MONITOR_NODE]
            self.assertEqual([(n.get('pkg'), n.get('type')) for n in nodes], [('asmc_lidar', SHADOW_NODE)], path)
            arguments = {a.get('name'): a.get('default', a.get('value')) for a in root.findall('.//arg')}
            if path.name == 'monitor.launch':
                self.assertEqual([(n.get('pkg'), n.get('required')) for n in monitors], [('asmc_lidar', 'false')], path)
                self.assertEqual(arguments.get('start_pipeline'), 'true', path)
                self.assertEqual(nodes[0].get('if'), '$(arg start_pipeline)', path)
                self.assertEqual(arguments.get('bind_address'), '127.0.0.1', path)
                self.assertEqual(arguments.get('open_browser'), 'false', path)
                self.assertNotEqual(arguments.get('dashboard_port'), PLANNER_DASHBOARD_PORT, path)
                self.assertTrue(arguments.get('debug_topic_prefix', '').startswith('/asmc_lidar/shadow/'), path)
                monitor_params = {p.get('name'): p.get('value') for p in monitors[0].findall('param')}
                self.assertEqual(monitor_params.get('bind_address'), '$(arg bind_address)', path)
                self.assertEqual(monitor_params.get('port'), '$(arg dashboard_port)', path)
                self.assertEqual(monitor_params.get('web_dir'), '$(find asmc_lidar)/web', path)
            else:
                self.assertEqual(monitors, [], path)
                self.assertNotIn('debug_enabled', {p.get('name') for p in nodes[0].findall('param')}, path)
            for name in OUTPUT_ARGS:
                self.assertIn(name, arguments, '%s: %s' % (path, name))
                self.assertTrue(arguments[name].startswith('/asmc_lidar/shadow/'), '%s: %s' % (path, name))
            params = {p.get('name'): p.get('value') for p in nodes[0].findall('param')}
            for name in OUTPUT_ARGS:
                self.assertEqual(params.get(name), '$(arg %s)' % name, '%s: %s' % (path, name))
            self.assertNotIn('allow_operational_output', params, path)
            self.assertEqual(root.findall('.//remap'), [], path)
            self.assertEqual(arguments.get('publish_tf'), 'false', path)

    def test_no_other_launch_file_starts_the_shadow_node(self):
        if not repository_available():
            self.skipTest('repository root not reachable from the package directory')
        candidates = {p for p in walk(ROOT / 'src', {'.launch', '.test', '.xml'})
                      if p.suffix != '.xml' or p.name.endswith('.launch.xml')}
        candidates |= set(walk(ROOT / 'tools', {'.launch', '.test'}))
        candidates |= set((ROOT / 'src/lidar+sensorfusion/src/Main').glob('*.launch'))
        self.assertTrue(candidates)
        offenders = []
        for path in sorted(candidates):
            if PACKAGE in path.resolve().parents:
                continue
            text = path.read_text(errors='replace')
            if SHADOW_NODE in text or re.search(r'asmc_lidar\)?/launch/shadow_pipeline', text):
                offenders.append(str(path))
        self.assertEqual(offenders, [])

    def test_rostest_node_is_executable(self):
        # rostest finds <test type="shadow_node_test.py"> only if it is executable.
        # The repository uses core.filemode=false, so a plain `git add` would record
        # 100644: stage it with `git add --chmod=+x`; this check catches a 100644 entry.
        for name in ('shadow_node_test.py', 'monitor_node_test.py'):
            script = PACKAGE / 'test' / 'ros' / name
            self.assertTrue(os.access(str(script), os.X_OK), '%s is not executable' % script)
        if not (ROOT / '.git').exists():
            self.skipTest('not a git checkout')
        # Explicit git-dir/work-tree: CI and docker run as root on a checkout owned by
        # another user, where git's repository discovery stops at "dubious ownership".
        command = ['git', '--git-dir=%s' % (ROOT / '.git'), '--work-tree=%s' % ROOT,
                   'ls-files', '-s', '--', str(script)]
        try:
            out = subprocess.run(command, cwd=str(ROOT), stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                 universal_newlines=True, timeout=30).stdout.split()
        except (OSError, subprocess.SubprocessError):
            self.skipTest('git not available')
        if not out:
            self.skipTest('not tracked yet (stage it with git add --chmod=+x)')
        self.assertEqual(out[0], '100755', 'index mode of %s' % script)

    def test_monitor_scripts_are_executable_and_read_only(self):
        for relative in MONITOR_SCRIPTS:
            script = PACKAGE / relative
            self.assertTrue(script.is_file(), script)
            self.assertTrue(os.access(str(script), os.X_OK), '%s is not executable' % script)
            text = script.read_text()
            self.assertTrue(text.startswith('#!/usr/bin/env python3'), script)
            # Read-only: no ROS publisher, service call/proxy, parameter write or shell execution.
            for forbidden in ('rospy.Publisher', 'ServiceProxy', 'rospy.Service(', 'set_param', 'os.system(',
                              'shell=True'):
                self.assertNotIn(forbidden, text, '%s uses %s' % (script, forbidden))
        core = (PACKAGE / 'scripts/asmc_lidar_monitor_core.py').read_text()
        self.assertNotIn('import rospy', core)
        for relative in WEB_ASSETS:
            asset = PACKAGE / relative
            self.assertTrue(asset.is_file(), asset)
            text = asset.read_text()
            self.assertIsNone(re.search(r'https?://|//cdn\.|<script[^>]+src="(?!(?:app|monitor_view)\.js")', text), asset)
        if (ROOT / '.git').exists():
            command = ['git', '--git-dir=%s' % (ROOT / '.git'), '--work-tree=%s' % ROOT, 'ls-files', '-s', '--']
            command += [str(PACKAGE / relative) for relative in MONITOR_SCRIPTS]
            try:
                out = subprocess.run(command, cwd=str(ROOT), stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                     universal_newlines=True, timeout=30).stdout.splitlines()
            except (OSError, subprocess.SubprocessError):
                out = []
            for line in out:  # tracked entries must be 100755 (core.filemode=false: git add --chmod=+x)
                self.assertEqual(line.split()[0], '100755', line)

    def test_rostest_node_cases_cannot_skip(self):
        # rosunit's XML runner records a skipped case as passed, so CI could not see it.
        for name in ('shadow_node_test.py', 'monitor_node_test.py'):
            text = (PACKAGE / 'test' / 'ros' / name).read_text()
            self.assertIsNone(re.search(r'skipTest|unittest\.skip|skipIf|skipUnless|SkipTest', text), name)

    def test_pipeline_config_matches_mount_and_map_contract(self):
        config = yaml.safe_load((PACKAGE / 'config/shadow_pipeline.yaml').read_text())
        self.assertEqual(config['schema_version'], 1)
        self.assertEqual((PACKAGE / config['mount_config']).resolve(), (PACKAGE / 'config/autonav_v3.yaml').resolve())
        mount = yaml.safe_load((PACKAGE / 'config/autonav_v3.yaml').read_text())['sensor_pose_base_link']
        lidar = config['vehicle']['LiDAR']
        for key, reference in (('x', 'x_m'), ('y', 'y_m'), ('z', 'z_m'), ('roll', 'roll_deg'),
                               ('pitch', 'pitch_deg'), ('yaw', 'yaw_deg')):
            self.assertAlmostEqual(float(lidar[key]), float(mount[reference]), delta=1e-6, msg=key)
        origin = [float(v) for v in config['map']['origin']]
        self.assertEqual(len(origin), 3)
        self.assertAlmostEqual(float(config['map']['map_altitude_origin_m']), origin[2], delta=1e-9)
        if not repository_available():
            self.skipTest('repository root not reachable: map contract not checked')
        contract = yaml.safe_load((ROOT / 'config/contracts/hd_map_topic_contract_v1.yaml').read_text())
        expected = contract['local_crop_policy']['gps_imu_policy']['local_origin_in_global']
        self.assertEqual(origin, [float(v) for v in expected])


if __name__ == '__main__':
    unittest.main()
