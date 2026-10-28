#!/usr/bin/env python3
"""Check highway runtime assets and the real native geometry before ROS driving.

This does not generate/edit maps, change parameters or start vehicle control.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[2]


def required_file(path):
    path = Path(path).resolve()
    if not path.is_file() or path.stat().st_size == 0:
        raise ValueError('missing or empty runtime file: %s' % path)
    return path


def resolve(value, base, repo):
    value = str(value).replace('$(find asmc_planner)', str(repo / 'src/asmc_planner'))
    if '$(' in value:
        raise ValueError('unresolved package expression: %s' % value)
    path = Path(value).expanduser()
    return required_file(path if path.is_absolute() else base / path)


def segment_distance(point, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    length2 = dx * dx + dy * dy
    w = max(0.0, min(1.0, ((point[0] - a[0]) * dx + (point[1] - a[1]) * dy) / length2)) if length2 else 0.0
    return math.hypot(point[0] - a[0] - w * dx, point[1] - a[1] - w * dy)


def matching_centerlines(first, second):
    if len(first) < 2 or len(second) < 2:
        return False
    if any(len(p) < 2 or not all(math.isfinite(float(v)) for v in p[:2])
           for line in (first, second) for p in line):
        return False
    if any(math.hypot(a[0] - b[0], a[1] - b[1]) > 0.05
           for a, b in ((first[0], second[0]), (first[-1], second[-1]))):
        return False
    return all(min(segment_distance(p, a, b) for a, b in zip(line, line[1:])) <= 0.05
               for points, line in ((first, second), (second, first)) for p in points)


def check_assets(repo, config):
    repo = Path(repo).resolve()
    config = required_file(config)
    settings = yaml.safe_load(config.read_text(encoding='utf-8'))
    waypoint = resolve(settings['waypoint_file'], repo / 'src/asmc_planner/src', repo)
    policy = settings['planner']['highway_merge']
    if not policy['enabled'] or not policy['zones']:
        raise ValueError('planner/highway_merge geometry must be enabled and contain zones')
    map_settings = settings['avoid_hd_map']
    if not map_settings['enabled']:
        raise ValueError('highway requires avoid_hd_map/enabled=true')
    if 'directory' in map_settings:
        directory = Path(map_settings['directory'])
        if not directory.is_absolute():
            directory = config.parent / directory
    else:
        local_map = config.parent / 'hd_map'
        directory = local_map if (local_map / 'drivable.json').is_file() else repo / 'src/asmc_planner/config/hd_map'
    areas, lines, grid = [required_file(directory / name) for name in
                          ('drivable.json', 'lines.json', 'drivable_grid.rle')]
    product = json.loads(areas.read_text(encoding='utf-8'))
    if product['frame_id'] != 'map' or not product['lanes'] or not product['drivable_areas']:
        raise ValueError('incomplete HD-map frame/layers')
    behavior = yaml.safe_load(required_file(repo / 'src/asmc_planner/config/behavior/behavior_params.yaml').read_text())
    links = resolve(behavior['map']['link_set_path'], repo, repo)
    nodes = resolve(behavior['map']['node_set_path'], repo, repo)
    route = resolve(behavior['route']['route_links_path'], repo, repo)
    link_data = json.loads(links.read_text(encoding='utf-8'))
    node_data = json.loads(nodes.read_text(encoding='utf-8'))
    link_data = link_data['links'] if isinstance(link_data, dict) else link_data
    node_data = node_data['nodes'] if isinstance(node_data, dict) else node_data
    if not isinstance(link_data, list) or not link_data or not isinstance(node_data, list) or not node_data:
        raise ValueError('Behavior link_set/node_set must contain non-empty arrays')
    known = {str(row.get('idx', row.get('id', ''))) for row in link_data}
    route_data = yaml.safe_load(route.read_text())
    if isinstance(route_data, dict) and 'route' in route_data:
        route_data = route_data['route']
    if isinstance(route_data, dict):
        route_data = route_data.get('route_links', route_data.get('links', []))
    if not isinstance(route_data, list) or not route_data or any(str(x) not in known for x in route_data):
        raise ValueError('Behavior route links are missing from its map')
    for zone in policy['zones']:
        if zone['source_link'] not in known:
            raise ValueError('highway source link missing from Behavior map: %s' % zone['source_link'])
        for lane in zone['target_lanes']:
            if (lane[5:] if lane.startswith('lane:') else lane) not in known:
                raise ValueError('highway target link missing from Behavior map: %s' % lane)
    behavior_links = {str(row.get('idx', row.get('id', ''))): row for row in link_data}
    hd_links = {str(row['link_id']): row for row in product['lanes']}
    used = {zone['source_link'] for zone in policy['zones']}
    used.update(lane[5:] if lane.startswith('lane:') else lane
                for zone in policy['zones'] for lane in zone['target_lanes'])
    lane_change = policy.get('lane_change', {})
    if lane_change.get('enabled', False):
        sequences = lane_change.get('lanes', [])
        if not isinstance(sequences, list) or not 2 <= len(sequences) <= 8:
            raise ValueError('lane-change requires a bounded explicit lane catalog')
        common = policy['zones'][-1]['target_lanes'][-1]
        for sequence in sequences:
            if not isinstance(sequence, list) or len(sequence) < 2 or sequence[-1] != common:
                raise ValueError('lane-change sequence must reach the explicit common lane')
            for lane in sequence:
                link = lane[5:] if lane.startswith('lane:') else lane
                if link not in known:
                    raise ValueError('lane-change link missing from Behavior map: %s' % link)
                used.add(link)
    for link in used:
        if link not in hd_links or not matching_centerlines(
                hd_links[link]['centerline'], behavior_links[link].get('points', [])):
            raise ValueError('Behavior/HD-map centerline mismatch: %s' % link)
    # Catch wrapper argument omissions independently of a running ROS master.
    for name in ('planner_v2.launch', 'autonomy_no_bridges_v2.launch'):
        launch = ET.parse(required_file(repo / 'src/asmc_planner/launch' / name)).getroot()
        args = {arg.attrib['name']: arg.attrib for arg in launch.findall('arg')}
        expected = 'true' if name == 'autonomy_no_bridges_v2.launch' else 'false'
        if args.get('highway_merge', {}).get('default') != expected:
            raise ValueError('%s must declare highway_merge with default=%s' % (name, expected))
        if name == 'planner_v2.launch':
            wired = any(p.attrib.get('name') == '/frenet_planner/highway_merge_enabled'
                        and p.attrib.get('value') == '$(arg highway_merge)'
                        for p in launch.findall('param'))
        else:
            wired = any(a.attrib.get('name') == 'highway_merge'
                        and a.attrib.get('value') == '$(arg highway_merge)'
                        for include in launch.findall('include')
                        if include.attrib.get('file', '').endswith('/planner_v2.launch')
                        for a in include.findall('arg'))
        if not wired:
            raise ValueError('%s does not forward highway_merge' % name)
    files = [config, waypoint, areas, lines, grid, links, nodes, route]
    return {'map_build': product['build_id'], 'geometry_sha256': product['reviewed_geometry_sha256'],
            'zone_count': len(policy['zones']),
            'files': [{'path': str(p), 'sha256': hashlib.sha256(p.read_bytes()).hexdigest(),
                       'bytes': p.stat().st_size} for p in files]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo-root', type=Path, default=ROOT)
    parser.add_argument('--config', type=Path)
    parser.add_argument('--native', type=Path, help='Built highway_merge_preflight executable')
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    config = args.config or args.repo_root / 'src/asmc_planner/config/frenet/params.yaml'
    try:
        result = check_assets(args.repo_root, config)
        native = args.native or args.repo_root / 'devel/lib/asmc_planner/highway_merge_preflight'
        if native.is_file():
            command = [str(native.resolve()), str(config.resolve())]
        elif shutil.which('rosrun'):
            command = ['rosrun', 'asmc_planner', 'highway_merge_preflight', str(config.resolve())]
        else:
            raise ValueError('build the planner first or provide --native; geometry check was not run')
        completed = subprocess.run(command, text=True, capture_output=True, timeout=120)
        if completed.returncode:
            raise ValueError('native preflight failed:\n%s\n%s' % (completed.stdout, completed.stderr))
        result['native_output'] = completed.stdout
        if args.report:
            args.report.write_text(json.dumps(result, indent=2), encoding='utf-8')
        print(completed.stdout.rstrip())
        print('Runtime files, Behavior route, launch and native geometry: OK')
        return 0
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError, ET.ParseError,
            subprocess.TimeoutExpired) as error:
        print('PREFLIGHT FAILED: %s' % error)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
