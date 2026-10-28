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
    # The user chose a small parallel PR gate; retain the old matrix only as
    # an explicit manual run. No successful test result is reused as a cache.
    ci = yaml.safe_load((root/'.github/workflows/ci.yml').read_text())
    required_jobs = {'validated-lidar-build', 'planning-catkin-build', 'catkin-build'}
    if set(ci['jobs']) != required_jobs:
        errors.append('three essential check identities required')
    for name, job in ci['jobs'].items():
        if job.get('needs') or job.get('if'):
            errors.append('essential checks must run in parallel without conditional skipping: '+name)
        if not 0 < job.get('timeout-minutes', 0) <= 6:
            errors.append('essential job needs a budget of at most six minutes: '+name)
    for name in ('full-regression.yml', 'planning-integration.yml'):
        workflow = yaml.safe_load((root/'.github/workflows'/name).read_text())
        events = workflow.get('on', workflow.get(True, {}))
        if set(events or {}) != {'workflow_dispatch'}:
            errors.append('heavy regression must remain manual-only: '+name)
    full = (root/'.github/workflows/full-regression.yml').read_text()
    if ('./scripts/build_ws.sh full' not in full or
            'tools/ci/run_checks.py --output .ci-results --workers 2' not in full):
        errors.append('manual full build and every regression group must remain available')
    auto = (root/'.github/workflows/ci.yml').read_text()
    for forbidden in ('./scripts/build_ws.sh all', 'cpu-requirements.txt',
                      'tools/ci/run_checks.py --output .ci-results'):
        if forbidden in auto:
            errors.append('slow full suite reintroduced into required CI: '+forbidden)
    spec = json.loads((root/'.github/ci/essential.json').read_text())
    if spec.get('wall_target_minutes') != 7:
        errors.append('essential wall target must remain seven minutes')
    required_targets = {
        'planning': {'asmc_planner_node', 'mpc_controller_node', 'asmc_control_selector_node',
                     'planning_objects_adapter', 'highway_merge_preflight', 'planning_gps_recovery_test',
                     'stop_brake_response_test', 'reference_validator_test'},
        'perception': {'lidar_pipeline_node', 'vehicle_state_node', 'camera_bbox_lane_node',
                       'camera_bbox_lane_change_node', '_lane_association',
                       'asmc_lidar_frame_time_core_test', 'asmc_lidar_geometry_boundary_test'}}
    for scope, expected_targets in required_targets.items():
        job = spec['jobs'][scope]
        if not expected_targets.issubset(job['targets']) or 'all' in job['targets']:
            errors.append('essential production/safety targets missing or full build requested: '+scope)
        if not (root/job['script']).is_file():
            errors.append('essential test script missing: '+scope)
    for script in ('essential-contracts.sh', 'essential-planning.sh', 'essential-perception.sh'):
        if not (root/'.github/ci'/script).is_file():
            errors.append('essential script missing: '+script)
    planning = (root/'.github/ci/planning.sh').read_text()
    for required in ('rostest asmc_planner highway_runtime.test', 'roundabout_execution_test',
                     'command_avoid_target_test', 'avoid_road_constraints_test', 'polygon_collision_test'):
        if required not in planning:
            errors.append('missing manual planning regression: '+required)
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
