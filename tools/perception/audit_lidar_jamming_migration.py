#!/usr/bin/env python3
"""Read-only bag evidence for the LiDAR cutover; no ROS master or control output.

Use ROS Python with the workspace sourced. --planning-bag is the optional output
of lidar_replay --planning-output. This checks availability, not ground-truth
velocity accuracy or closed-loop driving. Never treats missing exit as a pass.
"""
import argparse
from collections import Counter
import importlib.util
import json
from pathlib import Path

import rosbag
import rospy


def camera_converter():
    path = Path(__file__).resolve().parents[2] / 'src/jamming_lane_boundary_adapter/scripts/camera_bbox_frame_bridge.py'
    spec = importlib.util.spec_from_file_location('migration_camera_frame', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.camera_frame


def audit(source, planning):
    convert = camera_converter()
    counts = Counter()
    with rosbag.Bag(str(source)) as bag:
        begin, end = bag.get_start_time(), bag.get_end_time()
        images = set()
        camera = []
        gps_loss = gps_recovery = None
        # Include the bridge's image-cache warmup so a detection near the window
        # boundary is not falsely counted as having no matching original image.
        for topic, msg, t in bag.read_messages(topics=['/gps', '/image_jpeg/compressed', '/perception/camera/objects'],
                                               start_time=rospy.Time.from_sec(max(begin, end - 23.))):
            if topic != '/image_jpeg/compressed' and t.to_sec() < end - 20.:
                continue
            if topic == '/gps':
                invalid = msg.latitude == msg.longitude == msg.altitude == 0.
                if invalid and gps_loss is None: gps_loss = t.to_sec()
                if not invalid and gps_loss is not None and gps_recovery is None: gps_recovery = t.to_sec()
            elif topic == '/image_jpeg/compressed':
                images.add((msg.header.stamp.to_nsec(), msg.header.frame_id))
            elif msg.header.frame_id == 'camera_front':
                frame = convert(msg)
                camera.append((frame, t.to_sec() - msg.header.stamp.to_sec()))
        for frame, age in camera:
            counts['front_frames'] += 1
            counts['converted_valid'] += int(frame.valid)
            counts['exact_image_match'] += int((frame.header.stamp.to_nsec(), frame.header.frame_id) in images)
            counts['age_over_500ms'] += int(age > .5)
            if not frame.valid: counts['invalid:' + frame.reason] += 1
        result = dict(source=str(source), duration_s=end-begin,
                      tail_s=20, gps_loss_s=None if gps_loss is None else gps_loss-begin,
                      gps_recovery_s=None if gps_recovery is None else gps_recovery-begin,
                      camera=dict(counts), max_camera_age_s=max((a for _, a in camera), default=None))
    if planning:
        counts = Counter(); last_usable = None; max_gap = 0.; stamps = set(); states = Counter()
        gap_had_pose_loss = False; max_velocity_gap = 0.
        with rosbag.Bag(str(planning)) as bag:
            for _, packet, arrival in bag.read_messages():
                a = packet.observation.objects
                counts['frames'] += 1; counts['status:' + str(a.status)] += 1
                counts['duplicate_stamp'] += int(a.header.stamp.to_nsec() in stamps)
                stamps.add(a.header.stamp.to_nsec())
                for state in packet.motion_states: states[str(state)] += 1
                valid = (a.status in (a.OBSERVED, a.OBSERVED_EMPTY) and
                         all(b.absolute_velocity_valid for b in a.objects))
                counts['velocity_withheld_frames'] += int(a.status in (a.OBSERVED, a.OBSERVED_EMPTY) and not valid)
                counts['velocity_qualified_frames'] += int(valid)
                counts['objects'] += len(a.objects)
                counts['objects_without_velocity'] += sum(not b.absolute_velocity_valid for b in a.objects)
                if gps_loss is not None and arrival.to_sec() >= gps_loss + .3:
                    counts['map_observed_after_gps_timeout'] += int(a.status in (a.OBSERVED, a.OBSERVED_EMPTY))
                elif valid:
                    if last_usable is not None:
                        gap = a.header.stamp.to_sec()-last_usable
                        max_gap = max(max_gap, gap)
                        if not gap_had_pose_loss:
                            max_velocity_gap = max(max_velocity_gap, gap)
                            counts['map_valid_gaps_over_300ms'] += int(gap > .3)
                    last_usable = a.header.stamp.to_sec()
                    gap_had_pose_loss = False
                else:
                    gap_had_pose_loss |= a.status not in (a.OBSERVED, a.OBSERVED_EMPTY)
        # Counterfactual strict velocity-only boundary, retained to reproduce the
        # gap fixed by the explicit private velocity_unavailable flag. The actual
        # C++ gate summary below is authoritative for the current adapter.
        result['strict_velocity_gate_before_fix'] = dict(counts=counts, motion_states=states, max_usable_frame_gap_before_jamming_s=max_gap,
                                  max_map_valid_usable_frame_gap_s=max_velocity_gap)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-bag', required=True, type=Path)
    parser.add_argument('--planning-bag', type=Path)
    parser.add_argument('--gate-summary', type=Path, help='JSON from planning_lidar_bag_check')
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    result = audit(args.source_bag, args.planning_bag)
    if args.gate_summary:
        gate = json.loads(args.gate_summary.read_text())
        if args.planning_bag and gate['accepted'] + gate['rejected'] != result['strict_velocity_gate_before_fix']['counts']['frames']:
            raise ValueError('gate summary frame count does not match planning bag')
        result['planning_gate'] = gate
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2, ensure_ascii=False)
        stream.write('\n')
    print(json.dumps(result, indent=2, ensure_ascii=False))
