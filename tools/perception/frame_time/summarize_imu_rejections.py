#!/usr/bin/env python3
"""Correlate native causal IMU selection with original raw sensor/trace evidence."""
import argparse
import bisect
from collections import Counter
import json
import math
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from audit_frame_time import read, sha, stats, write_json


def summarize(bag_path, probe_path, trial_path):
    import rosbag
    samples = []
    with rosbag.Bag(str(bag_path)) as bag:
        for _, m, receipt in bag.read_messages(topics=['/imu']):
            q = m.orientation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            samples.append((m.header.stamp.to_nsec(), receipt.to_nsec(), yaw))
    # Only select at/before scan. Receipt is kept separately; this reconstructs
    # source chronology, not the exact original ROS worker thread schedule.
    samples.sort()
    stamps = [s[0] for s in samples]
    rows = list(read(probe_path))
    stale = [r for r in rows if r['recorded_reason'] == 'IMU is stale for LiDAR scan']
    details = []
    for row in stale:
        index = bisect.bisect_right(stamps, row['stamp_ns']) - 1
        if index < 0:
            raise ValueError('no raw IMU before affected scan')
        details.append(dict(row, raw_imu_age_ms=(row['stamp_ns']-stamps[index])/1e6,
            previous_raw_gap_ms=(stamps[index]-stamps[index-1])/1e6 if index else None))
    discontinuities = []
    for a, b in zip(samples, samples[1:]):
        jump = abs(math.atan2(math.sin(b[2]-a[2]), math.cos(b[2]-a[2])))
        if jump > math.radians(30):
            discontinuities.append(dict(stamp_ns=b[0], jump_deg=math.degrees(jump), interval_ms=(b[0]-a[0])/1e6))
    trial = json.loads(trial_path.read_text())
    return dict(input_sha=sha(bag_path), probe_sha=sha(probe_path), trial_sha=sha(trial_path),
        scans=len(rows), recorded_stale=len(stale),
        reclassified=dict(Counter(r['reason'] for r in stale)),
        rejected_reason=dict(Counter(r['rejection_reason'] for r in stale)),
        native_all_reasons=dict(Counter(r['reason'] for r in rows)),
        imu_rejections=max(r['rejected'] for r in rows),
        raw_age_ms=stats(r['raw_imu_age_ms'] for r in details),
        raw_gap_ms=stats(r['previous_raw_gap_ms'] for r in details),
        yaw_discontinuities=discontinuities,
        teleport_heading_deg=dict(before=trial['ego_before']['transform']['rotation']['z'],
            start=trial['start_transform']['rotation']['z'], after=trial['ego_after']['transform']['rotation']['z']),
        details=details,
        passed=bool(stale) and all(r['reason']=='imu_input_invalid' and
            r['rejection_reason']=='implausible IMU yaw jump' for r in stale),
        limits=['The original scan remains invalid; no invalid orientation is promoted to usable data.',
                'Probe queries run at trace receipt; callback thread scheduling is not reconstructed.',
                'No collision topic was recorded; collision or packet-loss root causes are not inferred.'])


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('bag', 'probe', 'trial', 'output'):
        p.add_argument('--'+name, required=True, type=Path)
    args = p.parse_args()
    result = summarize(args.bag, args.probe, args.trial)
    write_json(args.output, result)
    print(json.dumps({k: v for k, v in result.items() if k != 'details'}, indent=2))
    sys.exit(0 if result['passed'] else 1)
