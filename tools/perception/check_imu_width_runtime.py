#!/usr/bin/env python3
"""Production family compute parity, isolated ROS master, cached source grids.

Attitude lookup is supplied by the separately checked source-clock interpolator;
exact wire/session/freshness behavior is covered by the ROS adapter tests. No
model, live sensor subscription, public master or vehicle control is used here.
"""
import argparse
import json
from pathlib import Path
from types import MethodType

import numpy as np
import yaml

from asmc_camera.bev.imu_projection import attitude_camera
from review_family_motion import family_dict, interpolate, sha
from review_family_dropout import isolated_master
from review_source_tunnel_motion import source_signals
from review_tunnel_imu_motion import read, VIEWS


def compare(reference, candidate):
    if reference['valid'] != candidate['valid']:
        raise ValueError('validity mismatch')
    if len(reference['boundaries']) != len(candidate['boundaries']):
        raise ValueError('boundary count mismatch')
    maximum = 0.
    for a, b in zip(reference['boundaries'], candidate['boundaries']):
        if (a['role'], a['class_id'], a['support']) != (b['role'], b['class_id'], b['support']):
            raise ValueError('role/class/support mismatch')
        maximum = max(maximum, float(np.max(np.abs(np.asarray(a['xy']) - b['xy']))))
    if maximum > 1e-6:
        raise ValueError('production geometry mismatch: ' + str(maximum))
    return maximum


def main(args):
    args.output.mkdir(parents=True, exist_ok=False)
    capture = args.trial / 'capture'
    imu, _, _ = source_signals(capture, read(args.trial / 'poses.jsonl'))
    expected = {r['index']: r for r in read(args.reference / 'frames.jsonl')}
    results = []
    with isolated_master(args.port, args.output):
        import rospy
        from std_msgs.msg import Header
        from asmc_camera.nodes.lane_vectorizer_node import LaneVectorizerNode, _ViewFrame
        rospy.init_node('imu_width_compute_parity', anonymous=True, disable_signals=True)
        for name in ('cameras_v3.yaml', 'bev/lane_fitting.yaml', 'camera/lane_input.yaml'):
            for key, value in yaml.safe_load((args.config / name).read_text()).items():
                rospy.set_param('~' + key, value)
        rospy.set_param('~managed', False)
        rospy.set_param('~enable_boundary_family_candidate', True)
        rospy.set_param('~enable_independent_boundary_pipeline', False)
        rospy.set_param('~imu_motion', dict(enabled=True, corrected_width_prior_strength=0.0,
                                           corrected_joint_width_fit=args.joint_fit))
        node = LaneVectorizerNode()
        node.demand_inputs.close()
        node.diagnostics_timer.shutdown()
        try:
            for index, sample in enumerate(read(capture / 'frames.jsonl')):
                if sha(capture / sample['grid']) != expected[index]['grid_sha256']:
                    raise ValueError('cached input grid SHA mismatch')
                arrays = dict(np.load(capture / sample['grid']))
                corrected, frames = {}, {}
                for view in VIEWS:
                    raw = sample['views'][view]['lineage']['sensor_timing']
                    source = raw['source_sec'] * 10**9 + raw['source_nsec']
                    q = interpolate(imu, source, 'imu')
                    if q is None:
                        raise ValueError('missing source bracket in parity fixture')
                    corrected[view] = attitude_camera(node.projectors[view].camera, q['q_xyzw'])
                    stamp = sample['views'][view]['lineage']['image_header']['stamp']
                    header = Header(stamp=rospy.Time(stamp // 10**9, stamp % 10**9), frame_id='camera_' + view)
                    frames[view] = _ViewFrame(view, stamp, header, arrays[view + '_class'],
                        arrays[view + '_confidence'], 0., node.epoch_gate.context())
                corrected_name = 'M1_source_joint' if args.joint_fit else 'M1_source_width0'
                for corrected_mode, reason, name in ((True, 'corrected', corrected_name),
                                                      (False, 'missing_source_timing', 'M0')):
                    def supplied_attitude(self, frames, used=corrected_mode, why=reason):
                        self.imu_last = dict(reason=why, used=used)
                        return corrected if used else {}
                    node._attitude_cameras = MethodType(supplied_attitude, node)
                    *_, family, audit = node._compute_family_frame(frames, {},
                        dict(projection={}, association={}, prefit={}, shared_family={}))
                    actual = family_dict(family, audit)
                    delta = compare(expected[index]['variants'][name], actual)
                    policy = audit['fit_policy']
                    if policy['width_prior_strength'] != (0. if corrected_mode else .35):
                        raise ValueError('incorrect effective width prior')
                    if policy['joint_width_fit'] != (args.joint_fit and corrected_mode):
                        raise ValueError('incorrect effective joint-fit policy')
                    results.append(dict(index=index, variant=name, delta_m=delta, policy=policy,
                                        valid=actual['valid'], grid_sha256=sha(capture / sample['grid'])))
                if (index + 1) % 40 == 0:
                    print('production source/fallback parity', index + 1, flush=True)
        finally:
            node._shutdown()
            rospy.signal_shutdown('isolated parity complete')
    report = dict(schema='imu_width_runtime_compute_parity_v1', frames=len(expected), joint_fit=args.joint_fit,
        comparisons=len(results), maximum_delta_m=max(r['delta_m'] for r in results),
        passed=True, public_master_used=False, gpu_used=False, live_acceptance=False,
        lookup='source interpolator injection; exact wire adapter tested separately',
        reference_sha256=sha(args.reference / 'frames.jsonl'), tool_sha256=sha(__file__))
    (args.output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    (args.output / 'frames.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in results))
    print(json.dumps(report), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('trial', 'reference', 'config', 'output'):
        parser.add_argument('--' + name, type=Path, required=True)
    parser.add_argument('--port', type=int, default=11366)
    parser.add_argument('--joint-fit', action='store_true', help='Compare corrected production solver with M1_source_joint; fallback remains M0')
    main(parser.parse_args())
