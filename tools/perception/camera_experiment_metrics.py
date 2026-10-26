"""Reusable camera experiment metrics and source provenance, no ROS launches."""
import copy
import hashlib
import json
import os
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]

def debug_masks_enabled(job):
    return bool(job.get('debug',False) and job.get('debug_masks',True))

def debug_topics(job):
    if not job.get('debug'): return []
    topics=['/perception/camera/objects/debug/image/compressed',
        '/perception/camera/objects/debug/mosaic/compressed',
        '/perception/camera/objects/debug/left/image/compressed',
        '/perception/camera/objects/debug/right/image/compressed',
        '/perception/camera/debug/mosaic/compressed',
        '/perception/bev/debug/comparison/compressed',
        '/perception/traffic_light/debug/image/compressed']
    if 'seg' in job['model'] and debug_masks_enabled(job):
        topics += ['/perception/camera/objects/debug/instances/compressed',
            '/perception/camera/objects/debug/left/instances/compressed',
            '/perception/camera/objects/debug/right/instances/compressed']
    return topics

def measurement_gate(row, minimum=9.5):
    return bool(row.get('complete') and row.get('five_second_hz') and
        all(hz >= minimum for hz in row['five_second_hz']) and
        row['valid_messages']==row['messages'] and not row['duplicate_stamps'] and not row['reversed_stamps'] and
        row['max_gap_sec'] <= .25 and row['input_anchor_missing']==0 and
        row['input_to_output_wall_ms']['p95'] is not None and row['input_to_output_wall_ms']['p95'] <= 150 and
        row['input_to_output_wall_ms']['p99'] <= 200)

def deadline_report(rows, begin, end, maximum_gap, minimum_hz):
    kept = [r for r in rows if begin <= r['arrival'] < end]
    times = [begin]+[r['arrival'] for r in kept]+[end]
    gap = max(b-a for a,b in zip(times,times[1:]))
    return dict(messages=len(kept), hz=len(kept)/(end-begin), max_gap_sec=gap,
                valid_messages=sum(r.get('valid', True) for r in kept),
                passed=bool(kept) and gap<=maximum_gap and len(kept)/(end-begin)>=minimum_hz)

def provenance(models, engine, revision):
    # Include dirty source digests: HEAD alone does not identify an experiment.
    files = sorted(set(ROOT.glob('tools/perception/*.py')) | set(ROOT.glob('src/asmc_camera/src/asmc_camera/**/*.py')) |
                   set(ROOT.glob('src/asmc_camera/config/camera/**/*.yaml')) |
                   set(ROOT.glob('src/asmc_lidar/src/**/*.*')) |
                   set(ROOT.glob('src/asmc_lidar/include/**/*.hpp')) |
                   set(ROOT.glob('src/asmc_lidar/launch/*.launch')) |
                   set(ROOT.glob('src/asmc_camera/launch/*.launch')))
    hashes = {str(f.relative_to(ROOT)): hashlib.sha256(f.read_bytes()).hexdigest() for f in files if f.is_file()}
    if len(revision)!=40 or any(c not in '0123456789abcdef' for c in revision):
        raise ValueError('full host checkout revision required')
    binaries={}
    for relative in ('asmc_lidar/lidar_pipeline_node','asmc_planner/asmc_planner_node','asmc_controller/mpc_controller_node'):
        for prefix in os.environ.get('CMAKE_PREFIX_PATH','').split(':'):
            path=Path(prefix)/'lib'/relative
            if path.is_file():
                binaries[relative]=dict(path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest())
                break
    return dict(head=revision,
                source_sha256=hashes, engine_sha256=hashlib.sha256(engine.read_bytes()).hexdigest(),
                built_binaries=binaries,
                model_provenance={str(f):json.loads(f.read_text()) for f in
                    sorted(set(models.glob(engine.stem+'*.json')) | set(engine.parent.glob(engine.stem+'*.json')))},
                renderer='MORAI left running with user sensor/render settings; preset not independently verified',
                windows_background_cpu_ram='unavailable; global NVIDIA GPU includes simulator and desktop')

def pin_uniform_rate(runtime, target_hz):
    if not 0 < target_hz <= 20:
        raise ValueError('target Hz must be in (0,20]')
    result=copy.deepcopy(runtime)
    result['target_hz']=target_hz
    result['target_hz_by_view']={v:target_hz for v in ('front','left','right')}
    return result

def reentry_gate(owner, row):
    # A successfully processed image without a visible traffic light is an
    # invalid planning verdict, not an inference supply failure. Keep verdict
    # validity in the report; assess new processing independently.
    supply=dict(row)
    if owner=='resnet18':
        supply['valid_messages']=supply.get('messages',0)
    return bool(not row.get('stale_messages',0) and measurement_gate(supply) and
                (owner=='yolo' or supply.get('first_second',0)>=9))

def candidate_order(candidates, repeat, policy):
    if policy == 'cyclic':
        offset = (repeat-1) % len(candidates)
        return candidates[offset:]+candidates[:offset]
    return candidates if repeat % 2 else list(reversed(candidates))

