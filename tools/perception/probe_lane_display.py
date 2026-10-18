#!/usr/bin/env python3
"""Read-only live display probe: fresh source stamps, not repeated JPEG rate.

Does not start inference/rqt, publish commands, or change model/bridge modes.
Loopback reception is measured; rqt screen presentation is not observable here.
"""
import argparse
from collections import defaultdict, deque
import json
from pathlib import Path
import threading
import time

import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=60.)
    parser.add_argument('--family-view-topic', default='/perception/lane/debug/shared_family_bev/compressed')
    args = parser.parse_args(); args.output.mkdir(parents=True, exist_ok=False)
    import rospy
    from sensor_msgs.msg import CompressedImage
    from asmc_msgs.msg import LaneBoundaryFamily, InferenceStatus
    rospy.init_node('lane_display_probe', anonymous=True, disable_signals=True)
    lock = threading.Lock(); start = time.monotonic()
    state = defaultdict(lambda: dict(last_stamp=None, last_wall=None, count=0, duplicate=0,
                                    regressions=0, windows=defaultdict(int),
                                    age_ms=deque(maxlen=20000), gaps_ms=deque(maxlen=20000),
                                    upstream_ms=deque(maxlen=20000)))
    latest_family = {}; statuses = defaultdict(int)
    stream = (args.output/'events.jsonl').open('w')
    def consume(message, name):
        now = time.monotonic(); stamp = message.header.stamp.to_nsec()
        with lock:
            r = state[name]; fresh = stamp > 0 and stamp != r['last_stamp']
            age = (rospy.Time.now().to_nsec()-stamp)/1e6 if stamp else None
            row = dict(topic=name, elapsed=now-start, stamp_ns=stamp, fresh=fresh, age_ms=age)
            if stamp == r['last_stamp']: r['duplicate'] += 1
            if r['last_stamp'] is not None and stamp < r['last_stamp']: r['regressions'] += 1
            if fresh:
                if r['last_wall'] is not None: r['gaps_ms'].append(1000*(now-r['last_wall']))
                r['last_wall'] = now; r['count'] += 1
                r['windows'][int((now-start)//5)] += 1
                if age is not None: r['age_ms'].append(age)
            r['last_stamp'] = stamp
            if name == 'family':
                latest_family[stamp] = now
                if len(latest_family)>256: latest_family.pop(next(iter(latest_family)))
                statuses['family_valid' if message.valid else 'family_invalid'] += 1
            if name == 'family_view' and fresh and stamp in latest_family:
                row['family_to_display_ms'] = 1000*(now-latest_family[stamp])
                r['upstream_ms'].append(row['family_to_display_ms'])
            stream.write(json.dumps(row)+'\n')
    def status(message):
        with lock:
            statuses['ready' if message.pipeline_ready else 'not_ready'] += 1
    subs = [rospy.Subscriber(topic, kind, lambda msg,n=name: consume(msg,n), queue_size=1,
                             buff_size=16<<20) for name,topic,kind in (
        ('family','/perception/lane/boundary_family',LaneBoundaryFamily),
        ('family_view',args.family_view_topic,CompressedImage),
        ('semantic_view','/perception/camera/debug/mosaic/compressed',CompressedImage),
        ('objects_view','/perception/camera/objects/debug/mosaic/compressed',CompressedImage))]
    subs.append(rospy.Subscriber('/perception/camera/lane/status',InferenceStatus,status,queue_size=1))
    try:
        while time.monotonic()-start<args.duration and not rospy.is_shutdown(): time.sleep(.1)
    finally:
        for sub in subs: sub.unregister()
        stream.close(); rospy.signal_shutdown('read-only display probe complete')
    def stats(values):
        a=np.asarray(values,dtype=float)
        return dict(count=len(a), **({'max':float(a.max()),**{'p'+str(q):float(np.percentile(a,q))
                    for q in (50,95,99)}} if len(a) else {}))
    report=dict(duration_s=time.monotonic()-start,status=dict(statuses),topics={},
        limitations=['Existing live inference and exact source headers; no synthetic stamps.',
                     'ROS JPEG reception, not physical exposure or rqt presentation.',
                     'First/last 5s windows excluded from steady-rate gate.'])
    for name,r in state.items():
        windows={str(k):v/5 for k,v in r['windows'].items() if k>=1 and (k+1)*5<=args.duration}
        report['topics'][name]=dict(fresh_count=r['count'],repeated_stamps=r['duplicate'],
            regressions=r['regressions'],complete_window_hz=windows,source_age_ms=stats(r['age_ms']),
            gaps_ms=stats(r['gaps_ms']),family_to_display_ms=stats(r['upstream_ms']))
    (args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2),flush=True)


if __name__=='__main__': main()
