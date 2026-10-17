#!/usr/bin/env python3
"""Offline boundary-evidence removal through the production family fitter.

CPU inference and an owned, isolated ROS master keep this review separate from
live camera execution. Removing assigned track evidence is a fitter test, not
an image-occlusion test, a geometric GT measurement, or live readiness evidence.
"""
import argparse
from collections import Counter, defaultdict
from contextlib import contextmanager
import hashlib
import html
import json
import os
from pathlib import Path
import signal
import subprocess
import time
from xmlrpc.client import ServerProxy

import numpy as np

from asmc_camera.bev.lane_tracking import LaneTrack
from asmc_camera.bev.shared_lane_family import (
    ROLE_ORDER, SAMPLE_OBSERVED, SAMPLE_STRUCTURE_INFERRED,
    _assign_observed_roles, _config, fit_shared_lane_family_with_audit,
)


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def cases(assigned):
    """Remove whole selected roles or their 5–10m support after association.

    Every selected role is tested, including a naturally inferred role: that
    removal is a no-op and explicitly reported instead of counted as a loss.
    Keeping only one role is the negative control against invented families.
    """
    for role in ROLE_ORDER:
        yield 'drop:' + role, [t for r, t in assigned.items() if r != role], role
        gapped = []
        for current, track in assigned.items():
            nodes = tuple(n for n in track.nodes
                          if current != role or not 5. <= float(n.xy[0]) <= 10.)
            if len(nodes) >= 2:
                gapped.append(LaneTrack(nodes))
        yield 'gap5to10:' + role, gapped, role
        yield 'only:' + role, ([assigned[role]] if role in assigned else []), role


def describe(family, audit):
    result = dict(valid=family is not None, reason=audit.get('reason'),
                  observed_roles=audit.get('observed_roles', []),
                  acceptance_mode=audit.get('acceptance_mode'),
                  boundaries={})
    if family is None:
        return result
    result['valid_x_m'] = [family.valid_x_min_m, family.valid_x_max_m]
    for boundary in family.boundaries:
        points, states = boundary.sample_points, boundary.sample_state
        # Project segment extents onto forward x, clipped to the consumer zone.
        # Unsupported or absent extents remain uncovered; endpoint extrapolation
        # must not make a short valid family appear to cover the whole 0–15m.
        low = np.maximum(np.minimum(points[:-1, 0], points[1:, 0]), 0.)
        high = np.minimum(np.maximum(points[:-1, 0], points[1:, 0]), 15.)
        lengths = np.maximum(high - low, 0.)
        usable = (states[:-1] != 0) & (states[1:] != 0)
        observed = ((states[:-1] == SAMPLE_OBSERVED) &
                    (states[1:] == SAMPLE_OBSERVED))
        covered = min(float(np.sum(lengths[usable])), 15.)
        result['boundaries'][boundary.role] = dict(
            overall_state=boundary.state, observed_length_m=boundary.observed_length_m,
            inferred_length_m=boundary.inferred_length_m,
            invalid_length_m=boundary.invalid_length_m,
            observed_samples=int(np.sum(states == SAMPLE_OBSERVED)),
            inferred_samples=int(np.sum(states == SAMPLE_STRUCTURE_INFERRED)),
            invalid_samples=int(np.sum(states == 0)),
            consumer_covered_x_m=covered,
            consumer_uncovered_x_m=max(15. - covered, 0.),
            consumer_observed_x_m=float(np.sum(lengths[observed])))
    return result


def lateral_delta(reference, candidate):
    """Fitted-curve change on shared usable samples; not a GT error."""
    if reference is None or candidate is None:
        return None
    deltas = []
    a = {b.role: b for b in reference.boundaries}
    for boundary in candidate.boundaries:
        curves = []
        for value in (a[boundary.role], boundary):
            points = value.sample_points[value.sample_state != 0]
            points = points[np.argsort(points[:, 0], kind='stable')]
            _, indices = np.unique(points[:, 0], return_index=True)
            curves.append(points[indices])
        lo = max(0., *(c[0, 0] for c in curves))
        hi = min(15., *(c[-1, 0] for c in curves))
        if hi > lo:
            x = np.linspace(lo, hi, 101)
            deltas.extend(np.abs(np.interp(x, curves[0][:, 0], curves[0][:, 1]) -
                                 np.interp(x, curves[1][:, 0], curves[1][:, 1])).tolist())
    return dict(p95_m=float(np.percentile(deltas, 95)), max_m=float(max(deltas))) if deltas else None


@contextmanager
def isolated_master(port, output):
    if not 1024 <= port <= 65535 or port == 11311:
        raise ValueError('a non-default ROS master port is required')
    uri = 'http://127.0.0.1:' + str(port)
    try:
        ServerProxy(uri).getPid('/family_dropout_port_check')
    except OSError:
        pass
    else:
        raise RuntimeError('isolated master port is occupied')
    os.environ.update(ROS_MASTER_URI=uri, ROS_HOSTNAME='127.0.0.1')
    with (output / 'roscore.log').open('w') as log:
        process = subprocess.Popen(['roscore', '-p', str(port)], stdout=log,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        try:
            for _ in range(100):
                try:
                    ServerProxy(uri).getPid('/family_dropout_port_check')
                    break
                except OSError:
                    time.sleep(.1)
            else:
                raise RuntimeError('isolated master did not start')
            yield
        finally:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()


def summarize(rows):
    grouped = defaultdict(list)
    for row in rows:
        grouped[(row['run_id'], row['phase'], row['case'])].append(row)
    return [dict(run_id=run, phase=phase, case=case, frames=len(values),
                 valid=sum(v['result']['valid'] for v in values),
                 reasons=dict(Counter(v['result']['reason'] for v in values)),
                 affected_frames=sum(v['affected'] for v in values))
            for (run, phase, case), values in sorted(grouped.items())]


def run(args):
    import cv2
    import rosbag
    import rospy
    import torch
    import yaml
    from std_msgs.msg import Header
    from asmc_camera.camera.backends.twinlite import TwinLiteBackend
    from asmc_camera.camera.inference_engine import InferenceEngine
    from asmc_camera.nodes.lane_vectorizer_node import LaneVectorizerNode, _ViewFrame
    from asmc_camera.bev.grid import BevGridSpec
    from asmc_camera.bev.shared_lane_family import contiguous_sample_state_segments
    from asmc_camera.bev.visualization import draw_metric_grid, draw_stateful_lane_family

    torch.set_num_threads(args.cpu_threads)
    cv2.setNumThreads(1)
    rospy.init_node('family_dropout_review', anonymous=True, disable_signals=True)
    config_files = ['cameras_v3.yaml', 'bev/lane_fitting.yaml', 'camera/lane_input.yaml']
    for name in config_files:
        for key, value in yaml.safe_load((args.config_root / name).read_text()).items():
            rospy.set_param('~' + key, value)
    rospy.set_param('~managed', False)
    rospy.set_param('~enable_boundary_family_candidate', True)
    rospy.set_param('~enable_independent_boundary_pipeline', False)
    node = LaneVectorizerNode()
    node.demand_inputs.close()
    node.diagnostics_timer.shutdown()
    backend = TwinLiteBackend(checkpoint=str(args.checkpoint), model_config='large',
                              device='cpu', amp=False, input_hw=(480, 640),
                              input_hw_by_view={'front': (544, 960)})
    engine = InferenceEngine(backend)
    pin = json.loads((args.fixture.parent / 'summary.json').read_text())
    if sha(args.fixture) != pin['manifest_sha256']:
        raise ValueError('fixture manifest SHA mismatch')
    frames = [json.loads(line) for line in args.fixture.read_text().splitlines()]
    groups = defaultdict(list)
    for frame in frames:
        groups[frame['run_id']].append(frame)
    (args.output / 'grids').mkdir()
    grid = BevGridSpec.from_mapping(yaml.safe_load((args.config_root / 'bev/bev_grid.yaml').read_text()))
    rows, gallery = [], []
    try:
        with (args.output / 'cases.jsonl').open('w') as stream:
            for run_id, selected in sorted(groups.items()):
                bag_path = args.data_root / selected[0]['bag']
                expected = next(r['bag_sha256'] for r in pin['runs'] if r['run_id'] == run_id)
                if sha(bag_path) != expected:
                    raise ValueError('pinned bag SHA mismatch')
                wanted = {(v['topic'], v['bag_ns']) for f in selected for v in f['views'].values()}
                blobs = {}
                with rosbag.Bag(str(bag_path)) as bag:
                    for topic, message, stamp in bag.read_messages(topics=sorted({t for t, _ in wanted})):
                        key = (topic, stamp.to_nsec())
                        if key in wanted:
                            blobs[key] = bytes(message.data)
                for index, frame in enumerate(selected):
                    images = {}
                    for view, source in frame['views'].items():
                        blob = blobs[(source['topic'], source['bag_ns'])]
                        if hashlib.sha256(blob).hexdigest() != source['sha256']:
                            raise ValueError('fixture JPEG SHA mismatch')
                        images[view] = cv2.imdecode(np.frombuffer(blob, np.uint8), cv2.IMREAD_COLOR)
                        if images[view] is None:
                            raise ValueError('fixture JPEG did not decode')
                    prediction = engine.infer(images, native_outputs={v: set() for v in images})
                    inputs, cache = {}, {}
                    for view, value in prediction.views.items():
                        stamp = frame['views'][view]['source_ns']
                        classes = value.model_grid_probabilities['road_marking'].astype(np.uint8)
                        confidence = np.rint(np.clip(value.model_grid_categorical_confidences['road_marking'], 0, 1) * 255).astype(np.uint8)
                        cache[view + '_class'] = classes
                        cache[view + '_confidence'] = confidence
                        header = Header(stamp=rospy.Time(stamp // 10**9, stamp % 10**9),
                                        frame_id=frame['views'][view]['frame_id'])
                        inputs[view] = _ViewFrame(view, stamp, header, classes, confidence, 0., node.epoch_gate.context())
                    cache_path = args.output / 'grids' / (run_id + '-%03d.npz' % index)
                    np.savez_compressed(cache_path, **cache)
                    _, _, _, tracks, baseline, baseline_audit = node._compute_family_frame(inputs, {}, dict(projection={}, association={}, prefit={}, shared_family={}))
                    assigned, _ = _assign_observed_roles(tracks, _config(node.shared_family_parameters))
                    selected_family, _ = fit_shared_lane_family_with_audit(list(assigned.values()), node.shared_family_parameters)
                    parity = lateral_delta(baseline, selected_family)
                    if (baseline is None) != (selected_family is None) or (parity and parity['max_m'] > 1e-6):
                        raise RuntimeError('selected support tracks changed the baseline family')
                    scenarios = [('baseline', baseline, baseline_audit, None, False)]
                    for case, remaining, role in cases(assigned):
                        family, audit = fit_shared_lane_family_with_audit(remaining, node.shared_family_parameters)
                        affected = role in assigned
                        if case.startswith('gap'):
                            affected = affected and any(5. <= float(n.xy[0]) <= 10. for n in assigned[role].nodes)
                        scenarios.append((case, family, audit, role, affected))
                    panels = []
                    for case, family, audit, role, affected in scenarios:
                        result = dict(run_id=run_id, phase=frame['phase'], index=index,
                                      source_stamp_ns=frame['views']['front']['source_ns'],
                                      route_station_m=frame['route_station_m'], case=case,
                                      affected=affected, baseline_roles=list(assigned),
                                      grid_sha256=sha(cache_path), result=describe(family, audit),
                                      lateral_delta=lateral_delta(baseline, family))
                        stream.write(json.dumps(result) + '\n')
                        rows.append(result)
                        if case == 'baseline' or case.startswith('drop:'):
                            canvas = np.zeros((*grid.shape, 3), np.uint8)
                            draw_metric_grid(canvas, grid)
                            segments = [] if family is None else [
                                (i + 1, state, b.class_id, points)
                                for i, b in enumerate(family.boundaries)
                                for state, points in contiguous_sample_state_segments(b.sample_points, b.sample_state)]
                            draw_stateful_lane_family(canvas, segments, grid, case + ': ' + audit.get('reason', '?'))
                            panels.append(cv2.resize(canvas, (256, 400)))
                    stream.flush()
                    filename = run_id + '-%03d.jpg' % index
                    front = cv2.resize(images['front'], (512, 288))
                    picture = np.vstack((np.hstack((front, np.zeros_like(front))), np.hstack(panels)))
                    cv2.imwrite(str(args.output / filename), picture)
                    gallery.append((filename, run_id, frame['phase'], index))
                    if index % 4 == 0:
                        print(run_id, frame['phase'], index + 1, '/', len(selected), flush=True)
    finally:
        node._shutdown()
    summary = dict(schema='family_evidence_dropout_review_v1', frames=len(frames),
                   cases=len(rows), fixture_sha256=sha(args.fixture), checkpoint_sha256=sha(args.checkpoint),
                   backend='pytorch_cpu_fp32', model_config='large',
                   input_hw=dict(front=[544, 960], sides=[480, 640]),
                   config_sha256={name: sha(args.config_root / name) for name in config_files},
                   selected_track_baseline_equivalence_max_m=1e-6,
                   grouped=summarize(rows), live_readiness_verified=False,
                   protocol='Selected support tracks only; removing roles/gaps after association, then rerunning unmodified production fitter. Selection is checked against full-track baseline. This does not simulate upstream pixel occlusion, measure GT accuracy, or verify TensorRT/live readiness.')
    (args.output / 'report.json').write_text(json.dumps(summary, indent=2) + '\n')
    (args.output / 'index.html').write_text(
        '<meta charset="utf-8"><title>Large family dropout review</title><h1>Large family: one-boundary evidence removal</h1>'
        '<p>CPU FP32; uniformly sampled conditions/phases. Baseline, left removed, yellow removed, right removed. '
        'Solid = observed, dashed = structure inferred. Evidence removal is not image occlusion or geometric GT.</p>'
        '<p><a href="report.json">Report</a> · <a href="cases.jsonl">Frame results</a></p>' + ''.join(
            '<p>{} {} #{}</p><img loading="lazy" width="1024" src="{}">'.format(html.escape(run), phase, index, file)
            for file, run, phase, index in gallery))
    print('Completed', len(frames), 'triplets /', len(rows), 'fitter cases', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('checkpoint', 'fixture', 'config-root', 'output'):
        parser.add_argument('--' + name, type=Path, required=True)
    parser.add_argument('--data-root', type=Path, default=Path('/data'))
    parser.add_argument('--master-port', type=int, default=11362)
    parser.add_argument('--cpu-threads', type=int, choices=[1, 2], default=2)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    with isolated_master(args.master_port, args.output):
        run(args)


if __name__ == '__main__':
    main()
