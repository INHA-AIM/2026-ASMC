#!/usr/bin/env python3
"""Pin existing bag signals/RGB for an offline lane-motion review (no ROS node).

The Ego pose is an offline reference, never a proposed competition input.
Run with the ROS Python environment; model inference is a separate CPU step.
"""
import argparse
from collections import defaultdict
import hashlib
import json
from pathlib import Path


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def select_windows(frames, count=12):
    """Contiguous original-fixture windows, not spaced representative frames."""
    groups = defaultdict(list)
    for frame in frames:
        groups[frame['phase']].append(frame)
    selected = []
    for phase, values in sorted(groups.items()):
        # Interior ends and exit beginnings include the known dropout region.
        start = (max(0, len(values) - count) if phase == 'interior' else
                 0 if phase == 'exit' else max(0, (len(values) - count) // 2))
        for index, frame in enumerate(values[start:start + count]):
            selected.append(dict(frame, sequence=phase, sequence_index=index))
    return selected


def main():
    import rosbag
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fixture', type=Path, required=True)
    parser.add_argument('--representatives', type=Path, required=True)
    parser.add_argument('--data-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    pin = json.loads((args.fixture.parent / 'summary.json').read_text())
    if sha(args.fixture) != pin['manifest_sha256']:
        raise ValueError('fixture SHA mismatch')
    frames = [json.loads(line) for line in args.fixture.read_text().splitlines()]
    representatives = [json.loads(line) for line in args.representatives.read_text().splitlines()]
    by_run, rep_by_run = defaultdict(list), defaultdict(list)
    for row in frames:
        by_run[row['run_id']].append(row)
    for row in representatives:
        rep_by_run[row['run_id']].append(row)
    signals, windows, rgb_index, summaries = [], [], [], []
    for run, values in sorted(by_run.items()):
        bag_path = args.data_root / values[0]['bag']
        expected = next(r['bag_sha256'] for r in pin['runs'] if r['run_id'] == run)
        if sha(bag_path) != expected:
            raise ValueError('bag SHA mismatch: ' + run)
        chosen = select_windows(values)
        windows.extend(chosen)
        targets = {}
        for row in rep_by_run[run] + chosen:
            key = row['views']['front']['source_ns']
            for view, source in row['views'].items():
                targets[(source['topic'], source['bag_ns'])] = (key, view, source)
        rgb_dir = args.output / 'rgb' / run
        rgb_dir.mkdir(parents=True)
        run_signals = []
        with rosbag.Bag(str(bag_path)) as bag:
            metadata = bag.get_type_and_topic_info().topics
            sensor_topics = [t for t in metadata if any(word in t.lower() for word in
                             ('imu', 'gps', 'ego_topic', 'odometry', '/odom', '/vehicle/state'))]
            for topic, message, recorded in bag.read_messages(
                    topics=sensor_topics + sorted({t for t, _ in targets})):
                if topic in sensor_topics:
                    row = dict(run_id=run, topic=topic, type=message._type,
                               source_ns=message.header.stamp.to_nsec(), bag_ns=recorded.to_nsec())
                    if topic == '/imu':
                        row.update(q_xyzw=[getattr(message.orientation, k) for k in 'xyzw'],
                                   angular_xyz=[getattr(message.angular_velocity, k) for k in 'xyz'],
                                   acceleration_xyz=[getattr(message.linear_acceleration, k) for k in 'xyz'],
                                   frame_id=message.header.frame_id)
                    elif topic == '/Ego_topic':
                        row.update(position_xyz=[getattr(message.position, k) for k in 'xyz'],
                                   velocity_xyz=[getattr(message.velocity, k) for k in 'xyz'],
                                   heading_deg=message.heading)
                    elif topic in ('/gps', '/gps/fix'):
                        status = message.status if topic == '/gps' else message.status.status
                        row.update(latitude=message.latitude, longitude=message.longitude,
                                   status=int(status))
                    else:
                        row['message'] = str(message)
                    run_signals.append(row)
                target = targets.get((topic, recorded.to_nsec()))
                if target:
                    stamp, view, source = target
                    blob = bytes(message.data)
                    if hashlib.sha256(blob).hexdigest() != source['sha256']:
                        raise ValueError('RGB SHA mismatch')
                    path = rgb_dir / ('%d-%s.jpg' % (stamp, view))
                    path.write_bytes(blob)
                    rgb_index.append(dict(run_id=run, front_ns=stamp, view=view,
                                          source_ns=source['source_ns'], sha256=source['sha256'],
                                          path=str(path.relative_to(args.output))))
        signals.extend(run_signals)
        summaries.append(dict(run_id=run, bag=values[0]['bag'], bag_sha256=expected,
                              topics={t:dict(type=m.msg_type, count=m.message_count)
                                      for t,m in metadata.items()},
                              signals=len(run_signals), continuous_frames=len(chosen)))
        print('pinned', run, len(run_signals), flush=True)
    for name, rows in (('signals', signals), ('continuous', windows), ('rgb-index', rgb_index)):
        (args.output / (name + '.jsonl')).write_text(''.join(json.dumps(r, sort_keys=True) + '\n' for r in rows))
    (args.output / 'summary.json').write_text(json.dumps(dict(
        schema='lane_motion_inputs_v1', runs=summaries,
        fixture_sha256=sha(args.fixture), representatives_sha256=sha(args.representatives),
        signals_sha256=sha(args.output / 'signals.jsonl'),
        continuous_sha256=sha(args.output / 'continuous.jsonl'),
        note='Header/source clocks preserved; Ego pose is development-only. No sensor timing ground truth.'), indent=2) + '\n')


if __name__ == '__main__':
    main()
