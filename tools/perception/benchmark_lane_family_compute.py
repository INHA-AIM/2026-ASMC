#!/usr/bin/env python3
"""Paired CPU-only production family calculation on immutable saved grids.

No ROS master, subscriptions, inference, control, or live deployment. Supply
registered output and a frozen reference bev directory. Disk IO, source-clock
interpolation, parity serialization and cProfile are outside timed samples.
"""
import argparse
from collections import defaultdict
from dataclasses import fields, is_dataclass
import hashlib
import importlib.util
import json
from pathlib import Path
import platform
import sys
import threading
import time
from types import SimpleNamespace

import cv2
import numpy as np
import yaml

from asmc_camera.nodes import lane_vectorizer_node as runtime
from asmc_camera.bev import lane_tracking, model_grid_projection, planning_usability, shared_lane_family
from asmc_camera.bev.calibration import calibrations_from_dict
from asmc_camera.bev.family_fit_policy import FamilyFitPolicy
from asmc_camera.bev.imu_projection import attitude_camera
from asmc_camera.bev.invalid_support import invalid_support_config_from_dict
from review_family_motion import interpolate, sha, stats
from review_source_tunnel_motion import source_signals

VIEWS = ('front', 'left', 'right')
MODULES = (lane_tracking, model_grid_projection, planning_usability, shared_lane_family)


def load_reference(directory):
    result = {}
    for module in MODULES:
        short = module.__name__.rsplit('.', 1)[-1]
        name = 'asmc_camera.bev._benchmark_reference_' + short
        spec = importlib.util.spec_from_file_location(name, directory / (short + '.py'))
        loaded = importlib.util.module_from_spec(spec)
        sys.modules[name] = loaded
        spec.loader.exec_module(loaded)
        result[short] = loaded
    return result


def make_node(modules, config):
    cfg = yaml.safe_load((config / 'bev/lane_fitting.yaml').read_text())
    motion = yaml.safe_load((config / 'bev/imu_motion.yaml').read_text())
    camera = calibrations_from_dict(yaml.safe_load((config / 'cameras_v3.yaml').read_text()))
    inputs = yaml.safe_load((config / 'camera/lane_input.yaml').read_text())
    node = runtime.LaneVectorizerNode.__new__(runtime.LaneVectorizerNode)
    node.views = VIEWS
    node.projectors = {v: modules['model_grid_projection'].ModelGridGroundProjector(
        camera[v], model_hw=tuple(inputs.get('input_hw_by_view', {}).get(v, inputs['input_hw'])),
        source_mask=cfg['source_masks'][v],
        x_bounds_m=(cfg['evidence_x_min_m'], cfg['evidence_x_max_m']),
        y_bounds_m=(cfg['evidence_y_min_m'], cfg['evidence_y_max_m']),
        **{k: cfg[k] for k in ('max_ground_range_m', 'min_camera_depth_m',
                               'edge_taper_fraction', 'forward_fade_start_m')}) for v in VIEWS}
    node.confidence_by_class = {1: cfg['confidence_by_class']['white'], 2: cfg['confidence_by_class']['yellow']}
    node.node_parameters = dict(x_bin_m=cfg['x_bin_m'], lateral_cluster_gap_m=cfg['lateral_cluster_gap_m'],
                                x_origin_m=cfg['evidence_x_min_m'])
    node.track_parameters = {k: cfg[k] for k in ('max_track_gap_m', 'max_lateral_residual_m',
        'max_heading_change_deg', 'min_track_nodes', 'min_observed_span_m')}
    node.shadow_boundary_parameters = dict(cfg['shadow_boundary'])
    node.shadow_boundary_enabled = node.shadow_boundary_parameters.pop('enabled', True)
    for key in ('temporal_max_lateral_residual_m', 'temporal_max_missed_frames',
                'temporal_reset_gap_sec', 'temporal_velocity_smoothing'):
        node.shadow_boundary_parameters.pop(key, None)
    node.family_fit_policy = FamilyFitPolicy(cfg['shared_lane_family'],
        corrected_width_prior_strength=motion['corrected_width_prior_strength'],
        corrected_joint_width_fit=motion['corrected_joint_width_fit'])
    node.imu_lock = threading.Lock()
    node.imu_enabled = True
    node.imu_last = {}
    node.publish_pipeline_audit = False
    node.shared_family_runtime_enabled = True
    node.shared_family_debug_enabled = False
    node.boundary_family_candidate_enabled = True
    node.shared_family_errors = 0
    node.planning_zones = planning_usability.planning_zones_from_config(cfg.get('planning_usability_zones'))
    node.invalid_support_config = invalid_support_config_from_dict(cfg.get('invalid_support_classification'))
    node._trace_supply = lambda *a, **kw: None
    node._publish_frame_projection = lambda *a, **kw: None
    return node


def freeze(value):
    if isinstance(value, np.ndarray):
        return dict(dtype=str(value.dtype), shape=list(value.shape), values=value.tolist())
    if isinstance(value, np.generic):
        return value.item()
    if is_dataclass(value):
        return {f.name: freeze(getattr(value, f.name)) for f in fields(value)}
    if isinstance(value, dict):
        return {str(k): freeze(v) for k, v in value.items() if k != 'fit_ms'}
    if isinstance(value, (list, tuple)):
        return [freeze(v) for v in value]
    return value


def signature(output, audit):
    # Include every evidence/node field, identity membership/order, complete
    # fitted dataclass and diagnostics; do not compare wall-clock fit_ms.
    evidence, nodes, tracks, fitted, family, family_audit = output
    node_ids = {id(node): i for i, node in enumerate(nodes)}
    content = freeze(dict(evidence=evidence, nodes=nodes,
        tracks=[[node_ids[id(n)] for n in t.nodes] for t in tracks],
        family=family, audit=audit, family_audit=family_audit))
    encoded = json.dumps(content, sort_keys=True, separators=(',', ':'), allow_nan=True).encode()
    return hashlib.sha256(encoded).hexdigest(), content


def calculate(node, modules, frames, corrected, mode):
    # Swap functions only outside the timed call. This runner is single-threaded.
    runtime.associate_lane_nodes_with_audit = modules['lane_tracking'].associate_lane_nodes_with_audit
    runtime.fit_shared_lane_family_with_audit = modules['shared_lane_family'].fit_shared_lane_family_with_audit
    measured = {}
    measure = modules['planning_usability'].measure_planning_usability
    def measure_usage(*a, **kw):
        start = time.perf_counter()
        result = measure(*a, **kw)
        measured['planning_usability'] = (time.perf_counter() - start) * 1000
        return result
    runtime.measure_planning_usability = measure_usage
    node.imu_last = dict(reason='corrected' if mode == 'M1' else 'missing_source_timing')
    node._attitude_cameras = lambda _frames: corrected if mode == 'M1' else {}
    audit = dict(projection={}, association={}, prefit={}, shared_family={})
    timings = {}
    start = time.perf_counter()
    output = node._compute_family_frame(frames, timings, audit)
    elapsed = (time.perf_counter() - start) * 1000
    timings.update(measured)
    timings['total'] = elapsed
    return output, audit, timings


def main(args):
    if args.output.name != args.run_id:
        raise ValueError('output basename must equal the registered run ID')
    args.output.mkdir(parents=True, exist_ok=False)
    cv2.setNumThreads(1)
    current = {m.__name__.rsplit('.', 1)[-1]: m for m in MODULES}
    variants = {'reference': load_reference(args.reference), 'candidate': current}
    nodes = {name: make_node(modules, args.config) for name, modules in variants.items()}
    capture = args.trial / 'capture'
    read = lambda p: [json.loads(line) for line in p.read_text().splitlines()]
    imu, _, signals = source_signals(capture, read(args.trial / 'poses.jsonl'))
    samples = read(capture / 'frames.jsonl')
    if args.limit:
        samples = samples[:args.limit]
    inputs = []
    for index, sample in enumerate(samples):
        corrected, frames = {}, {}
        with np.load(capture / sample['grid'], allow_pickle=False) as saved:
            arrays = dict(saved)
        times = []
        for view in VIEWS:
            lineage = sample['views'][view]['lineage']
            raw = lineage['sensor_timing']
            if not lineage['source_timing_valid'] or raw['bridge_session_id'] != signals['bridge_session_id']:
                raise ValueError('invalid source lineage')
            source = raw['source_sec'] * 10**9 + raw['source_nsec']
            times.append(source)
            q = interpolate(imu, source, 'imu')
            if q is None:
                raise ValueError('missing source IMU bracket')
            corrected[view] = attitude_camera(nodes['reference'].projectors[view].camera, q['q_xyzw'])
            frames[view] = SimpleNamespace(stamp_ns=source, class_id=arrays[view + '_class'],
                                          confidence=arrays[view + '_confidence'])
        if max(times) - min(times) > 35_000_000:
            raise ValueError('source skew')
        inputs.append((index, frames, corrected, sha(capture / sample['grid'])))
    for i in range(args.warmup):
        _, frames, corrected, _ = inputs[i % len(inputs)]
        for name in variants:
            calculate(nodes[name], variants[name], frames, corrected, 'M1')
    results, mismatch = [], []
    for repeat in range(args.rounds):
        for index, frames, corrected, grid_sha in inputs:
            for mode in ('M1', 'M0'):
                order = ('reference', 'candidate') if (index + repeat) % 2 == 0 else ('candidate', 'reference')
                outputs = {}
                for name in order:
                    output, audit, timings = calculate(nodes[name], variants[name], frames, corrected, mode)
                    digest, content = signature(output, audit)
                    outputs[name] = dict(signature=digest, content=content, timings=timings,
                                         valid=output[4] is not None)
                equal = outputs['reference']['signature'] == outputs['candidate']['signature']
                if not equal:
                    mismatch.append(dict(index=index, mode=mode, repeat=repeat))
                    if len(mismatch) <= 3:
                        (args.output / ('mismatch-%d-%s-%d.json' % (repeat, mode, index))).write_text(
                            json.dumps(outputs, indent=2))
                results.append(dict(index=index, mode=mode, repeat=repeat, grid_sha256=grid_sha,
                    exact=equal, variants={k: {f: v[f] for f in ('signature', 'timings', 'valid')}
                                          for k, v in outputs.items()}))
            if (index + 1) % 40 == 0:
                print('paired compute', repeat + 1, index + 1, '/', len(inputs), flush=True)
    stages = defaultdict(lambda: defaultdict(list))
    for row in results:
        for name, variant in row['variants'].items():
            for stage, value in variant['timings'].items():
                stages[name + '/' + row['mode']][stage].append(value)
    report = dict(schema='lane_family_paired_compute_v1', run_id=args.run_id,
        samples=len(inputs), rounds=args.rounds, comparisons=len(results), exact_matches=sum(r['exact'] for r in results),
        mismatches=mismatch, passed=not mismatch,
        stages_ms={k: {s: stats(v) for s, v in d.items()} for k, d in stages.items()},
        validity={name: sum(r['variants'][name]['valid'] for r in results) for name in variants},
        environment=dict(python=sys.version, numpy=np.__version__, platform=platform.platform()),
        config_sha256={str(p.relative_to(args.config)): sha(p) for p in args.config.rglob('*.yaml')
                       if p.name in ('lane_fitting.yaml', 'cameras_v3.yaml', 'lane_input.yaml', 'imu_motion.yaml')},
        module_sha256={name: {short: sha(m.__file__) for short, m in modules.items()} for name, modules in variants.items()},
        tool_sha256=sha(__file__), gpu_used=False, live_master_used=False, live_acceptance=False,
        limits='Cached post-inference grids; fixed source IMU supplied; no lookup wait, ROS publishing, '
               'model inference, downstream queue or control. Exact equality excludes fit_ms only.')
    (args.output / 'frames.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in results))
    (args.output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    if args.profile:
        import cProfile
        for name in variants:
            profiler = cProfile.Profile()
            for _, frames, corrected, _ in inputs:
                for mode in ('M1', 'M0'):
                    profiler.runcall(calculate, nodes[name], variants[name], frames, corrected, mode)
            profiler.dump_stats(str(args.output / (name + '.pstats')))
    print(json.dumps({k: report[k] for k in ('samples', 'comparisons', 'exact_matches', 'passed', 'stages_ms')}), flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('trial', 'config', 'reference', 'output'):
        parser.add_argument('--' + name, type=Path, required=True)
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--rounds', type=int, default=3)
    parser.add_argument('--warmup', type=int, default=8)
    parser.add_argument('--limit', type=int, default=0)
    parser.add_argument('--profile', action='store_true')
    raise SystemExit(main(parser.parse_args()))
