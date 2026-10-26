#!/usr/bin/env python3
"""Run the C++ lane-reference core offline; never publish vehicle commands."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
import tempfile

import yaml

ROOT = Path(__file__).resolve().parents[2]


def xy(points):
    return [[float(p[0]), float(p[1])] for p in points]


def snapshot(product, markings, markings_bytes):
    expected = product['sources']['reviewed_lane_markings']['sha256']
    if hashlib.sha256(markings_bytes).hexdigest() != expected:
        raise ValueError('reviewed markings do not match the compact product source SHA')
    result = {
        'identity': {'frame_id': product['frame_id'], 'build_id': product['build_id'],
                     'map_id': product['map_id'], 'geometry_sha256': product['reviewed_geometry_sha256']},
        'complete': True, 'lanes': [], 'areas': [], 'markings': []}
    for lane in product['lanes']:
        result['lanes'].append({
            'id': lane['id'], 'link_id': lane['link_id'],
            'left_adjacent': lane['left_adjacent_lane_id'], 'right_adjacent': lane['right_adjacent_lane_id'],
            'predecessors': lane['predecessor_lane_ids'], 'successors': lane['successor_lane_ids'],
            'centerline': xy(lane['centerline']), 'polygon': xy(lane['polygon']),
            'traversable': lane['lane_type'] not in ('unknown', 'shoulder') and lane['direction'] != 'unknown'})
    for area in product['drivable_areas']:
        result['areas'].append({'id': area['id'], 'polygon': xy(area['polygon']),
                               'validated': area['validity'] == 'validated'})
    for feature in markings['features']:
        if feature['geometry']['type'] != 'LineString':
            continue
        a = feature['attributes']
        result['markings'].append({
            'id': feature['id'], 'points': xy(feature['geometry']['coordinates']),
            'pattern': a.get('pattern', 'unknown'), 'width_m': float(a.get('line_width_m', 0)),
            'stop_line': feature['feature_type'] == 'stop_line' or a.get('functional_role') == 'stop_line',
            'virtual_line': a.get('physical_type') == 'virtual'})
    return result


def build_options(params):
    p = params['planner']
    vehicle = p['vehicle_shape']
    return {
        'reference': p['reference'],
        'front_m': vehicle['length']-vehicle['rear_overhang'], 'rear_m': vehicle['rear_overhang'],
        'half_width_m': vehicle['width']/2,
        'max_curvature': p['kinematic_limits']['max_curvature'],
        'max_lateral_accel': p['kinematic_limits']['max_lateral_accel'],
        # Offline static-inspection resolution and work budget, not runtime tuning.
        'max_join_gap_m': .05, 'sample_distance_m': .2, 'sample_yaw_rad': .02,
        'footprint_margin_m': 0., 'max_samples': 30000}


def bag_observations(path, source_link):
    observations = []
    try:
        import rosbag
    except ImportError:
        rosbag = None

    def visit(available, messages):
        object_topic = '/planning/objects'
        required = {'/vehicle/state', '/behavior/feature_debug', object_topic}
        if not required <= available:
            raise ValueError(f'{path.name}: missing topics {sorted(required-available)}')
        link, object_stamp, last_sample = '', 0., -math.inf

        def seconds(stamp):
            return stamp.to_sec() if hasattr(stamp, 'to_sec') else stamp.sec + stamp.nanosec/1e9

        for topic, recorded, msg in messages(required):
            if topic == '/behavior/feature_debug':
                link = msg.current_link_id
            elif topic == object_topic:
                object_stamp = seconds(msg.header.stamp)
            elif link == source_link:
                stamp = seconds(msg.header.stamp)
                if 0 <= stamp-last_sample < .1:
                    continue
                last_sample = stamp
                observations.append({
                    'now_s': recorded, 'ego_stamp_s': stamp, 'objects_stamp_s': object_stamp,
                    'ego_valid': bool(msg.valid and msg.header.frame_id == 'map'), 'objects_valid': object_stamp > 0,
                    'x': float(msg.x), 'y': float(msg.y), 'yaw': float(msg.yaw), 'speed': float(msg.speed),
                    'kappa': float(msg.yaw_rate/msg.speed) if abs(msg.speed) > .01 else 0.,
                    'acceleration': float(msg.acceleration)})
        return object_topic

    if rosbag is not None:
        with rosbag.Bag(str(path), 'r') as reader:
            available = set(reader.get_type_and_topic_info().topics)
            topic = visit(available, lambda required: ((name, stamp.to_sec(), msg)
                for name, msg, stamp in reader.read_messages(topics=list(required))))
    else:
        # Optional host-side reader. ROS Noetic uses its installed rosbag above.
        from rosbags.highlevel import AnyReader
        with AnyReader([path]) as reader:
            available = {c.topic for c in reader.connections}
            topic = visit(available, lambda required: ((c.topic, stamp/1e9, reader.deserialize(raw, c.msgtype))
                for c, stamp, raw in reader.messages(connections=[c for c in reader.connections if c.topic in required])))
    if not observations:
        raise ValueError(f'{path.name}: no recorded ego poses on source link {source_link}')
    return observations, topic


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--map', type=Path, default=ROOT/'src/asmc_hd_map/config/kcity2025_compact_drivable_map.json')
    parser.add_argument('--markings', type=Path, default=ROOT/'src/asmc_hd_map/reports/kcity2025_route_curation_20260917/route_curated_candidate_map.json')
    parser.add_argument('--params', type=Path, default=ROOT/'src/asmc_planner/config/frenet/params.yaml')
    parser.add_argument('--target-lanes', nargs='+', required=True, help='Explicit mission-compatible lane IDs, in order')
    parser.add_argument('--source-link', help='Required with --bag; restricts recorded poses to this source link')
    parser.add_argument('--synthetic-target', action='store_true', help='Also test simulated target-aligned poses; this is not closed-loop driving')
    parser.add_argument('--bag', type=Path, action='append', default=[])
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    if args.bag and not args.source_link:
        parser.error('--source-link is required with --bag')
    params = yaml.safe_load(args.params.read_text())
    product = json.loads(args.map.read_bytes())
    raw_markings = args.markings.read_bytes()
    scene = {'map': snapshot(product, json.loads(raw_markings), raw_markings), 'options': build_options(params),
             'requests': []}
    lc = params['planner']['lane_change']
    scene['handover'] = {
        'lateral_tolerance_m': lc['lateral_position_tolerance'], 'heading_tolerance_rad': lc['heading_tolerance'],
        'lateral_speed_tolerance_mps': lc['lateral_speed_tolerance'],
        # Test thresholds only. No automatic runtime handover is enabled.
        'settle_duration_s': .5, 'ego_max_age_s': .3, 'objects_max_age_s': .3, 'max_pose_gap_s': .21}
    request = {'name': 'static_target_reference', 'lane_ids': args.target_lanes, 'command_id': 1, 'observations': []}
    scene['requests'].append(request)
    if args.synthetic_target:
        scene['requests'].append({'name': 'synthetic_target_tracking', 'lane_ids': args.target_lanes,
                                  'command_id': 100, 'synthetic_target': True, 'observations': []})
    provenance = []
    for i, bag in enumerate(args.bag, start=2):
        observations, topic = bag_observations(bag, args.source_link)
        scene['requests'].append({'name': bag.name, 'lane_ids': args.target_lanes,
                                  'command_id': i, 'observations': observations})
        provenance.append({'bag': bag.name, 'object_stamp_topic': topic, 'ego_poses': len(observations)})
    with tempfile.TemporaryDirectory(prefix='asmc_lane_reference_') as temp:
        fixture = Path(temp)/'snapshot.json'
        fixture.write_text(json.dumps(scene, allow_nan=False))
        run = subprocess.run([str(args.binary.resolve()), str(fixture)], text=True, capture_output=True)
    if run.returncode:
        raise RuntimeError(run.stderr.strip() or f'audit exit {run.returncode}')
    result = json.loads(run.stdout)
    result['input_mode'] = 'artifact_snapshot_and_recorded_poses_not_live_ros_or_closed_loop'
    result['source_sha256'] = {'compact_file': hashlib.sha256(args.map.read_bytes()).hexdigest(),
                               'markings_file': hashlib.sha256(raw_markings).hexdigest()}
    result['bag_provenance'] = provenance
    text = json.dumps(result, ensure_ascii=False, indent=2)
    print(text)
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(text+'\n')
    passed = all(r['static_reference_valid'] == 'true' for r in result['reports'])
    if args.synthetic_target:
        passed = passed and any(r.get('fixture') and int(r.get('geometry_eligible', 0)) > 0 for r in result['reports'])
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
