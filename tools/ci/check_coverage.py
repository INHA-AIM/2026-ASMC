#!/usr/bin/env python3
"""Guard the pre-refactor CI checks, including each ROS fixture/option variant."""
import json
from pathlib import Path
import re
import shlex
import sys

import yaml

ROOT = Path(__file__).resolve().parents[2]


def commands(text):
    """Ignore environment/setup and artifact paths, never test arguments."""
    result = set()
    fixtures = re.search(r'for fixture_case in ([^;]+); do', text)
    text = text.replace('${ASMC_CI_DEVEL:-devel}', 'devel').replace('../../src/', 'src/')
    text = re.sub(r'"(devel/lib/[^"\n]+)"', r'\1', text).replace('./devel/', 'devel/')
    for line in text.splitlines():
        line = line.strip()
        if line.startswith('#'):
            continue
        match = re.search(r'(?:^|\s)(python3 |node |roslaunch --nodes |rostest |devel/lib/)', line)
        if not match:
            continue
        line = line[match.start(1):]
        tokens = shlex.split(line)
        if '--output' in tokens:
            tokens[tokens.index('--output')+1] = '<artifact>'
        if len(tokens) > 1 and tokens[1].endswith('/pipeline_ci_check.py'):
            # The gate now runs from the repository root, not build/asmc_lidar.
            for flag in ('--devel', '--build', '--results'):
                tokens[tokens.index(flag)+1] = '<'+flag[2:]+'>'
        if tokens[0] == 'rostest' and fixtures and '$fixture_case.test' in tokens:
            for fixture in fixtures.group(1).split():
                result.add(tuple(fixture+'.test' if t == '$fixture_case.test' else t for t in tokens))
        else:
            result.add(tuple(tokens))
    return result


def audit(root=ROOT):
    baseline = json.loads((root/'.github/ci/coverage-baseline.json').read_text())
    expected = set().union(*(commands(script) for script in baseline['scripts'].values()))
    groups = json.loads((root/'.github/ci/groups.json').read_text())['groups']
    actual = set()
    workflows = [yaml.safe_load(p.read_text()) for p in (root/'.github/workflows').glob('*.yml')]
    errors = []
    for workflow in workflows:
        for job in workflow['jobs'].values():
            for step in job.get('steps', []):
                if step.get('continue-on-error'):
                    errors.append('CI step hides failures: '+step.get('name', '?'))
                actual.update(commands(step.get('run', '')))
    for group in groups:
        script = root/group['script']
        if script.is_file():
            actual.update(commands(script.read_text()))
    for command in sorted(expected-actual):
        errors.append('missing former check: '+shlex.join(command))
    if {g['name'] for g in groups} != {'lane','lidar','camera-unit','camera-ros','bridge','planning'}:
        errors.append('required regression group missing')
    if not any(g['name']=='lidar' and g.get('exclusive') is True for g in groups):
        errors.append('wall-time-sensitive HTTP/LiDAR tests must run without parallel test load')
    for group in groups:
        if not (root/group['script']).is_file():
            errors.append('missing group script: '+group['script'])
    lidar = (root/'.github/ci/lidar.sh').read_text()
    if 'pipeline_ci_check.py' not in lidar or 'ctest --output-on-failure -T Test' not in lidar:
        errors.append('full mandatory LiDAR CTest gate missing')
    bridge = (root/'.github/ci/bridge.sh').read_text()
    if 'asmc_bridge' not in bridge or 'ctest --output-on-failure' not in bridge:
        errors.append('full bridge CTest replacement missing')
    # The old Planning integration workflow must not regain a duplicate build
    # or lose unique checks when its tests move into the shared build job.
    ci = yaml.safe_load((root/'.github/workflows/ci.yml').read_text())
    gate = ci['jobs'].get('planning-catkin-build', {})
    if gate.get('needs') != 'catkin-build' or gate.get('if') != 'always()':
        errors.append('planning check must propagate the shared job result, including failure')
    if not any(step.get('env', {}).get('BUILD_RESULT') == '${{ needs.catkin-build.result }}'
               and 'test "$BUILD_RESULT" = success' in step.get('run', '')
               for step in gate.get('steps', [])):
        errors.append('planning check must fail unless the shared job succeeded')
    all_runs = [step.get('run', '') for workflow in workflows
                for job in workflow['jobs'].values()
                if job.get('if') != "github.event_name == 'workflow_dispatch'"
                for step in job.get('steps', [])]
    if sum(run.count('./scripts/build_ws.sh') for run in all_runs) != 1:
        errors.append('ROS workspace must be built exactly once per CI revision')
    planning = (root/'.github/ci/planning.sh').read_text()
    for required in ('runtime_launch_contract_test.py', 'object_filter_contract_test.py',
                     'dashboard_motion_test.js', 'dashboard_shadow_test.js',
                     'dashboard_observation_test.py', 'dashboard_startup_test.py',
                     'prepared_static_inputs_test', 'rostest asmc_planner highway_runtime.test',
                     'command_avoid_target_test', 'roundabout_execution_test',
                     'avoid_road_constraints_test', 'polygon_collision_test',
                     '_ctest_asmc_planner_gtest_detected_msg_converter_test',
                     '_ctest_asmc_planner_gtest_feature_builder_avoid_test',
                     '_ctest_asmc_planner_gtest_dynamic_object_policy_test'):
        if required not in planning:
            errors.append('missing former planning check: '+required)
    standalone = yaml.safe_load((root/'.github/workflows/planning-integration.yml').read_text())['jobs'].get('planning-standalone-build', {})
    if standalone.get('if') != "github.event_name == 'workflow_dispatch'":
        errors.append('standalone dependency build must remain manual')
    if sum(script.count('rostest planning_inputs lidar_observation.test') for script in
           all_runs + [(root/g['script']).read_text() for g in groups]) != 1:
        errors.append('planning observation ROS fixture must run exactly once')
    # These input regressions are owned by lidar.sh and run once, not in each workflow.
    for test in ('planning_lidar_observation_test', 'planning_group_conversion_test',
                 'planning_lidar_companion_test', 'planning_gps_recovery_test'):
        if sum(script.count('/lib/planning_inputs/'+test) for script in
               all_runs + [(root/g['script']).read_text() for g in groups]) != 1:
            errors.append('planning input executable must run exactly once: '+test)
    return errors, len(expected)


def main():
    errors, count = audit()
    for error in errors:
        print('FAIL '+error)
    if not errors:
        print('PASS %d unique pre-refactor checks retained; mandatory LiDAR/bridge groups present' % count)
    return bool(errors)


if __name__ == '__main__':
    sys.exit(main())
