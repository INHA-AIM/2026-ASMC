#!/usr/bin/env python3
"""CI gate for the asmc_lidar shadow pipeline: fail unless it was really built and tested.

A configure-time option alone proves nothing, so this inspects the artifacts:
- the shadow executables exist in devel, are ELF and resolve all shared libraries;
- catkin exports still list only the pre-shadow libraries (dependents such as Main
  cannot link the package-private shadow libraries);
- every required test is registered in ctest, passed in the last `ctest -T Test`
  run and wrote fresh results with at least the expected number of cases;
- no case was skipped or disabled (unittest "skipped=", gtest skipped/disabled cases;
  rosunit reports a skip as a pass, so shadow_contract_test.py forbids skips in the
  rostest node script);
- optionally, no link line of a dependent build tree names a shadow library.

Usage (from the outer workspace, after `ctest -T Test` in build/asmc_lidar):
  shadow_ci_check.py --devel devel --build build/asmc_lidar --results build/test_results/asmc_lidar
  shadow_ci_check.py --devel devel --dependent-build src/lidar+sensorfusion/build
"""
import argparse
import base64
import os
from pathlib import Path
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
import zlib

EXECUTABLES = ('asmc_lidar_shadow_node', 'asmc_lidar_shadow_replay')
EXPORTED_LIBRARIES = ['asmc_lidar_frame_time', 'asmc_lidar_camera_core', 'asmc_lidar_camera_ros']
PRIVATE_LIBRARIES = ('asmc_lidar_pipeline', 'asmc_lidar_shadow_runtime')
# Plain executables (assert-based) of the pre-shadow package.
LEGACY_PROGRAMS = ('asmc_lidar_camera_buffer_test', 'asmc_lidar_camera_calibration_test',
                   'asmc_lidar_geometry_boundary_test', 'asmc_lidar_frame_time_core_test',
                   'asmc_lidar_imu_selection_test')
# Python unittest scripts registered with add_test: minimum number of cases.
UNITTESTS = {'asmc_lidar_camera_calibration_hash_test': 2, 'asmc_lidar_camera_contract_test': 5,
             'asmc_lidar_shadow_contract_test': 8}
# catkin_add_gtest targets: minimum number of cases.
GTESTS = {'asmc_lidar_test_tunnel_filter_core': 27, 'asmc_lidar_test_preprocessing': 18,
          'asmc_lidar_test_ground': 17, 'asmc_lidar_test_clustering': 15, 'asmc_lidar_test_obb': 24,
          'asmc_lidar_test_scan_frame': 11, 'asmc_lidar_test_tracker': 52,
          'asmc_lidar_test_pose_history': 32, 'asmc_lidar_test_gps_ingest': 15,
          'asmc_lidar_test_fusion_policy': 30, 'asmc_lidar_test_front_camera_fusion': 27,
          'asmc_lidar_test_tunnel_filter_stage': 7, 'asmc_lidar_test_pipeline': 15,
          'asmc_lidar_test_typed_output': 11, 'asmc_lidar_test_runtime': 16}
# rostest file stem -> (rosunit result name, minimum number of cases).
ROSTESTS = {'test_camera_receiver_ros': ('camera_receiver_ros', 1),
            'test_ros_shadow_node': ('shadow_node', 14)}


def required_tests():
    """ctest names: catkin prefixes gtest/rostest entries with _ctest_<package>_<kind>_."""
    names = list(LEGACY_PROGRAMS) + list(UNITTESTS)
    names += ['_ctest_asmc_lidar_gtest_%s' % name for name in GTESTS]
    names += ['_ctest_asmc_lidar_rostest_%s.test' % stem for stem in ROSTESTS]
    return names


class Gate:
    def __init__(self):
        self.errors = []

    def check(self, condition, message):
        if not condition:
            self.errors.append(message)
        return condition


def check_executables(gate, devel):
    for name in EXECUTABLES:
        path = devel / 'lib' / 'asmc_lidar' / name
        if not gate.check(path.is_file() and os.access(str(path), os.X_OK), 'missing executable %s' % path):
            continue
        gate.check(path.read_bytes()[:4] == b'\x7fELF', '%s is not an ELF executable' % path)
        ldd = subprocess.run(['ldd', str(path)], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                             universal_newlines=True, timeout=60)
        gate.check(ldd.returncode == 0 and 'not found' not in ldd.stdout, 'unresolved libraries in %s' % path)
    replay = devel / 'lib' / 'asmc_lidar' / 'asmc_lidar_shadow_replay'
    if replay.is_file():
        run = subprocess.run([str(replay), '--help'], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                             universal_newlines=True, timeout=60)
        gate.check(run.returncode == 2 and 'usage: asmc_lidar_shadow_replay' in run.stdout,
                   'asmc_lidar_shadow_replay --help did not run (rc %d)' % run.returncode)


def check_exports(gate, devel):
    config = devel / 'share' / 'asmc_lidar' / 'cmake' / 'asmc_lidarConfig.cmake'
    match = re.search(r'set\(libraries "([^"]*)"\)', config.read_text()) if config.is_file() else None
    if gate.check(match, 'libraries not found in %s' % config):
        own = [entry for entry in match.group(1).split(';') if 'asmc_lidar' in entry]
        gate.check(own == EXPORTED_LIBRARIES, 'catkin exports changed: %s' % own)
    pkgconfig = devel / 'lib' / 'pkgconfig' / 'asmc_lidar.pc'
    if gate.check(pkgconfig.is_file(), 'missing %s' % pkgconfig):
        own = re.findall(r'-l(asmc_lidar\w*)', pkgconfig.read_text())
        gate.check(own == EXPORTED_LIBRARIES, 'pkg-config exports changed: %s' % own)


def check_dependent_build(gate, build):
    links = sorted(build.rglob('link.txt')) if build.is_dir() else []
    gate.check(links, 'no link.txt under dependent build %s' % build)
    for path in links:
        text = path.read_text(errors='replace')
        for library in PRIVATE_LIBRARIES:
            gate.check(library not in text, '%s links %s' % (path, library))
    return len(links)


def registered_tests(build):
    out = subprocess.run(['ctest', '-N'], cwd=str(build), stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                         universal_newlines=True, timeout=120).stdout
    return re.findall(r'Test\s+#\d+: (\S+)', out)


def measurement_text(test):
    value = test.find('./Results/Measurement/Value')
    if value is None or value.text is None:
        return ''
    if value.get('encoding') == 'base64':
        data = base64.b64decode(value.text)
        return (zlib.decompress(data) if value.get('compression') else data).decode('utf-8', 'replace')
    return value.text


def last_ctest_run(build):
    tag = build / 'Testing' / 'TAG'
    if not tag.is_file():
        return None, {}
    test_xml = build / 'Testing' / tag.read_text().split()[0] / 'Test.xml'
    if not test_xml.is_file():
        return tag.stat().st_mtime, {}
    tests = {}
    for test in ET.parse(str(test_xml)).getroot().iter('Test'):
        name = test.findtext('Name')
        if name and test.get('Status'):
            tests[name] = (test.get('Status'), measurement_text(test))
    return tag.stat().st_mtime, tests


def check_ctest(gate, build):
    registered = set(registered_tests(build))
    for name in required_tests():
        gate.check(name in registered, 'required test not registered: %s' % name)
    started, tests = last_ctest_run(build)
    if not gate.check(tests, 'no `ctest -T Test` results in %s/Testing' % build):
        return started, tests
    for name in required_tests():
        status, output = tests.get(name, ('not run', ''))
        gate.check(status == 'passed', '%s: %s' % (name, status))
        gate.check(not re.search(r'\[  SKIPPED \]|DISABLED TEST|skipped=', output), '%s skipped cases' % name)
    for name, minimum in UNITTESTS.items():
        output = tests.get(name, ('', ''))[1]
        ran = re.search(r'Ran (\d+) tests?', output)
        gate.check(ran and int(ran.group(1)) >= minimum and re.search(r'^OK$', output, re.M),
                   '%s: expected OK with >= %d cases' % (name, minimum))
    return started, tests


def junit_cases(gate, path, minimum, started):
    if not gate.check(path.is_file(), 'missing result %s' % path):
        return 0
    if started is not None:
        gate.check(path.stat().st_mtime >= started - 2, 'stale result %s' % path)
    root = ET.parse(str(path)).getroot()
    cases = root.findall('.//testcase')
    for key in ('failures', 'errors', 'disabled', 'skipped'):
        gate.check(int(root.get(key) or 0) == 0, '%s: %s=%s' % (path.name, key, root.get(key)))
    gate.check(int(root.get('tests') or 0) == len(cases), '%s: tests attribute != cases' % path.name)
    for case in cases:
        bad = [child.tag for child in case if child.tag in ('skipped', 'failure', 'error')]
        gate.check(not bad and case.get('status', 'run') == 'run' and case.get('result') != 'skipped',
                   '%s: %s %s' % (path.name, case.get('name'), bad or case.get('result')))
    gate.check(len(cases) >= minimum, '%s: %d cases < %d' % (path.name, len(cases), minimum))
    return len(cases)


def check_results(gate, results, started):
    counts = {}
    for name, minimum in GTESTS.items():
        counts[name] = junit_cases(gate, results / ('gtest-%s.xml' % name), minimum, started)
    for stem, (unit, minimum) in ROSTESTS.items():
        junit_cases(gate, results / ('rostest-%s.xml' % stem), 1, started)
        counts[unit] = junit_cases(gate, results / ('rosunit-%s.xml' % unit), minimum, started)
    return counts


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--devel', type=Path, required=True)
    parser.add_argument('--build', type=Path, help='asmc_lidar build directory with `ctest -T Test` results')
    parser.add_argument('--results', type=Path, help='catkin test_results/asmc_lidar directory')
    parser.add_argument('--dependent-build', type=Path, action='append', default=[],
                        help='build tree of a package that depends on asmc_lidar (e.g. nested Main)')
    args = parser.parse_args(argv)
    if bool(args.build) != bool(args.results):
        parser.error('--build and --results go together')
    gate = Gate()
    check_executables(gate, args.devel)
    check_exports(gate, args.devel)
    for build in args.dependent_build:
        print('dependent link lines checked: %d in %s' % (check_dependent_build(gate, build), build))
    if args.build:
        started, tests = check_ctest(gate, args.build)
        counts = check_results(gate, args.results, started)
        print('required ctest entries: %d, passed: %d' % (
            len(required_tests()), sum(tests.get(n, ('',))[0] == 'passed' for n in required_tests())))
        print('cases: ' + ', '.join('%s=%d' % item for item in sorted(counts.items())))
    for error in gate.errors:
        print('FAIL: ' + error, file=sys.stderr)
    print('asmc_lidar shadow CI gate: %s' % ('FAILED' if gate.errors else 'PASSED'))
    return 1 if gate.errors else 0


if __name__ == '__main__':
    sys.exit(main())
