#!/usr/bin/env python3
"""Package isolation contract of the asmc_lidar shadow pipeline (no ROS needed).

- package.xml names no Main/Msg/asmc_camera/tensorrt/torch dependency;
- catkin_package() exports are exactly the pre-shadow ones (the shadow
  libraries stay package-private, so Main never links them);
- no asmc_lidar source includes Main/Msg headers or hard-codes /home paths;
- lidar.launch owns the single pipeline node; only the explicit migration
  autonomy wrapper and backend-selected scenario runner may include it;
- config/lidar_pipeline.yaml uses the mount of config/autonav_v3.yaml and the
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
PIPELINE_NODE = 'lidar_pipeline_node'
MONITOR_NODE = 'asmc_lidar_monitor_node.py'
MONITOR_SCRIPTS = ('scripts/asmc_lidar_monitor_node.py', 'scripts/asmc_lidar_monitor_core.py')
WEB_ASSETS = ('web/index.html', 'web/app.js', 'web/monitor_view.js', 'web/app.css')
PLANNER_DASHBOARD_PORT = '8088'
OUTPUT_ARGS = ('objects_local_topic', 'objects_map_topic', 'diagnostics_topic', 'trace_topic', 'observations_local_topic', 'observations_map_topic', 'tracks_local_topic', 'tracks_map_topic')
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
            if path.is_file() and (path.suffix in suffixes or name == 'CMakeLists.txt'):
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


class PipelineContractTest(unittest.TestCase):
    def test_map_bundle_refresh_preserves_filter_audit_and_off_modes(self):
        import importlib.util
        import tempfile
        import types
        from unittest import mock
        def prepare(_config, output, compressed=False):
            output.mkdir()
            (output/'manifest.json').write_text('{}')
            return {'manifest_sha256': 'b'*64}
        with tempfile.TemporaryDirectory() as temporary:
            package=Path(temporary);(package/'config').mkdir()
            spec=importlib.util.spec_from_file_location('sync_default_hd_map', PACKAGE/'tools/sync_default_hd_map.py')
            module=importlib.util.module_from_spec(spec)
            with mock.patch.dict('sys.modules', {'prepare_hd_map_audit':types.SimpleNamespace(prepare=prepare)}):
                spec.loader.exec_module(module)
            module.PKG=package
            for mode in ('filter','audit','off'):
                config=package/'config/lidar_pipeline.yaml'
                config.write_text('hd_map_audit:\n  mode: "'+mode+'"\n  manifest: "old.json"\n  manifest_sha256: "'+('a'*64)+'"\n')
                module.sync()
                updated=yaml.safe_load(config.read_text())['hd_map_audit']
                self.assertEqual(updated['mode'],mode)
                self.assertEqual(updated['manifest_sha256'],'b'*64)
                module.sync(check=True)

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

    def test_lidar_launch_is_the_only_pipeline_entry_point(self):
        entry_point = PACKAGE / 'launch/lidar.launch'
        self.assertTrue(entry_point.is_file())
        path = entry_point
        root = ET.parse(path).getroot()
        self.assertEqual(root.findall('.//include'), [], path)
        nodes = [n for n in root.findall('.//node') if n.get('type') == PIPELINE_NODE]
        self.assertEqual([(n.get('pkg'), n.get('type')) for n in nodes], [('asmc_lidar', PIPELINE_NODE)], path)
        arguments = {a.get('name'): a.get('default', a.get('value')) for a in root.findall('.//arg')}
        for name in OUTPUT_ARGS:
            self.assertIn(name, arguments, '%s: %s' % (path, name))
            self.assertTrue(not arguments[name].startswith('/'), '%s: %s' % (path, name))
        params = {p.get('name'): p.get('value') for p in nodes[0].findall('param')}
        for name in OUTPUT_ARGS:
            self.assertEqual(params.get(name), '$(arg %s)' % name, '%s: %s' % (path, name))
        self.assertEqual(params['allow_operational_output'], "$(eval arg('namespace') == '/perception/lidar')")
        self.assertEqual(nodes[0].findall('remap'), [], path)
        self.assertEqual(arguments.get('publish_tf'), 'false', path)
        self.assertEqual(arguments.get('namespace'), '/perception/lidar')
        self.assertEqual(nodes[0].get('ns'), '$(arg namespace)')
        self.assertEqual(arguments.get('fusion'), 'true')
        self.assertEqual(params.get('enable_camera_fusion'), '$(arg fusion)')
        self.assertEqual(arguments.get('rviz'), 'false')
        displays = root.find("group[@if='$(arg rviz)']")
        self.assertIsNotNone(displays)
        self.assertEqual([(n.get('pkg'), n.get('type')) for n in displays.findall('node')],
                         [('asmc_lidar', 'lidar_markers.py'), ('rviz', 'rviz')])
        markers, rviz = displays.findall('node')
        self.assertEqual(markers.get('ns'), '$(arg namespace)')
        self.assertEqual(rviz.get('ns'), '$(arg namespace)')
        marker_params = {p.get('name'): p.get('value') for p in markers.findall('param')}
        self.assertEqual(marker_params, {'input': '$(arg observations_local_topic)', 'output': 'markers'})
        self.assertEqual({p.get('from'): p.get('to') for p in rviz.findall('remap')},
                         {'/perception/lidar/markers': 'markers', '/velodyne_points': '$(arg points_topic)'})
        # The same launch must resolve on a LiDAR-only checkout. Camera
        # calibration discovery belongs to the enabled runtime branch.
        self.assertNotIn('enable_camera_fusion', arguments)
        self.assertNotIn('$(find asmc_camera)', ET.tostring(root, encoding='unicode'))
        # A future read-only view.launch is allowed; it must not start a second
        # pipeline, either directly or by including the inference entry point.
        for path in (PACKAGE / 'launch').glob('*.launch'):
            if path == entry_point:
                continue
            root = ET.parse(path).getroot()
            self.assertFalse(any(n.get('pkg') == 'asmc_lidar' and n.get('type') == PIPELINE_NODE
                                 for n in root.findall('.//node')), path)
            self.assertFalse(any(re.search(r'asmc_lidar\)?/launch/lidar\.launch', n.get('file', ''))
                                 for n in root.findall('.//include')), path)

    def test_view_launch_is_read_only_and_namespaced(self):
        root = ET.parse(PACKAGE / 'launch/view.launch').getroot()
        self.assertEqual(root.findall('.//include'), [])
        nodes = root.findall('.//node')
        self.assertEqual([(n.get('pkg'), n.get('type')) for n in nodes], [('asmc_lidar', MONITOR_NODE)])
        self.assertEqual(nodes[0].get('ns'), '$(arg namespace)')
        args = {a.get('name'): a.get('default') for a in root.findall('arg')}
        self.assertEqual(args['namespace'], '/perception/lidar')
        self.assertEqual(args['bind_address'], '127.0.0.1')
        self.assertEqual(args['open_browser'], 'false')
        self.assertNotEqual(args['dashboard_port'], PLANNER_DASHBOARD_PORT)
        self.assertNotIn('start_pipeline', args)
        params = {a.get('name'): a.get('value') for a in nodes[0].findall('param')}
        self.assertEqual(params['debug_topic_prefix'], 'debug')
        self.assertEqual(params['diagnostics_topic'], 'diagnostics')
        self.assertEqual(params['points_topic'], '/velodyne_points')

    def test_no_other_launch_file_starts_the_pipeline_node(self):
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
            relative = str(path.relative_to(ROOT))
            if relative in ('src/asmc_planner/launch/autonomy_asmc_lidar.launch',
                            'tools/scen_runner/launch/perception.launch'):
                root = ET.parse(path).getroot()
                includes = [n for n in root.findall('.//include')
                            if n.get('file') == '$(find asmc_lidar)/launch/lidar.launch']
                self.assertEqual(len(includes), 1, relative)
                self.assertEqual(includes[0].find("arg[@name='planning_output']").get('value'), 'true')
                self.assertFalse(any(n.get('type') == PIPELINE_NODE for n in root.findall('.//node')))
                continue
            text = path.read_text(errors='replace')
            if PIPELINE_NODE in text or re.search(r'asmc_lidar\)?/launch/lidar\.launch', text):
                offenders.append(str(path))
        self.assertEqual(offenders, [])

    def test_rostest_node_is_executable(self):
        # rostest finds <test type="lidar_pipeline_node_test.py"> only if it is executable.
        # The repository uses core.filemode=false, so a plain `git add` would record
        # 100644: stage it with `git add --chmod=+x`; this check catches a 100644 entry.
        script = PACKAGE / 'test' / 'ros' / 'lidar_pipeline_node_test.py'
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
        text = (PACKAGE / 'test' / 'ros' / 'lidar_pipeline_node_test.py').read_text()
        self.assertIsNone(re.search(r'skipTest|unittest\.skip|skipIf|skipUnless|SkipTest', text))

    def test_pipeline_config_matches_mount_and_map_contract(self):
        config = yaml.safe_load((PACKAGE / 'config/lidar_pipeline.yaml').read_text())
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
