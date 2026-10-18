#!/usr/bin/env python3
"""Read-only live display probe: fresh source stamps, not repeated JPEG rate.

Does not start inference/rqt, publish commands, or change model/bridge modes.
Loopback reception is measured; rqt screen presentation is not observable here.
"""
import argparse
import gc
from collections import defaultdict, deque
import json
import os
from pathlib import Path
import threading
import time

import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=60.)
    parser.add_argument('--all-visuals', action='store_true', help='Also subscribe to traffic-light and RGB/semantic BEV comparison')
    parser.add_argument('--extra-views', default='', help='Optional comma-separated traffic_light,bev')
    parser.add_argument('--family-view-topic', default='/perception/lane/debug/shared_family_bev/compressed')
    args = parser.parse_args(); args.output.mkdir(parents=True, exist_ok=False)
    import rospy
    from sensor_msgs.msg import CompressedImage
    from asmc_msgs.msg import LaneBoundaryFamily, InferenceStatus
    rospy.init_node('lane_display_probe', anonymous=True, disable_signals=True)
    lock = threading.Lock(); finished = threading.Event(); start = time.monotonic()
    state = defaultdict(lambda: dict(last_stamp=None, last_wall=None, count=0, duplicate=0,
                                    regressions=0, windows=defaultdict(int),
                                    age_ms=deque(maxlen=20000), gaps_ms=deque(maxlen=20000),
                                    upstream_ms=deque(maxlen=20000)))
    latest_family = {}; statuses = defaultdict(int)
    from asmc_camera.runtime.jsonl_journal import JsonlJournal
    stream = JsonlJournal(args.output/'events.jsonl')
    gc_started={};gc_durations=deque(maxlen=512)
    def gc_observe(phase, info):
        generation=info['generation']
        if phase=='start':gc_started[generation]=time.monotonic()
        elif generation in gc_started:
            gc_durations.append(dict(generation=generation,elapsed=time.monotonic()-start,
                duration_ms=1000*(time.monotonic()-gc_started.pop(generation))))
    gc.callbacks.append(gc_observe)
    def resources():
        result={}
        for proc in Path('/proc').glob('[0-9]*'):
            try:
                command=(proc/'cmdline').read_bytes().replace(b'\0',b' ').decode()
                label=next((v for v in ('family_view_node.py','object_view_node.py',
                    'semantic_visualizer_node.py','camera_bev_visualizer_node.py',
                    'traffic_light_visualizer_node.py','lane_vectorizer_node.py') if v in command),None)
                if label is None:continue
                raw=(proc/'stat').read_text().rpartition(') ')[2].split()
                ticks=int(raw[11])+int(raw[12]);rss=int(raw[21])*os.sysconf('SC_PAGE_SIZE')
                result[proc.name]=dict(label=label,cpu_ticks=ticks,rss_bytes=rss)
            except (OSError,ValueError):pass
        return result
    resource_before=resources()
    def consume(message, name):
        now = time.monotonic(); stamp = message.header.stamp.to_nsec()
        with lock:
            if finished.is_set():return
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
            stream.append(row)
    def status(message):
        with lock:
            if finished.is_set():return
            if not message.pipeline_ready:
                stream.append(dict(topic='status',elapsed=time.monotonic()-start,
                    pipeline_ready=False,reason=message.reason,mode=message.mode))
            statuses['ready' if message.pipeline_ready else 'not_ready'] += 1
    subs = [rospy.Subscriber(topic, kind, lambda msg,n=name: consume(msg,n), queue_size=1,
                             buff_size=16<<20) for name,topic,kind in (
        ('family','/perception/lane/boundary_family',LaneBoundaryFamily),
        ('family_view',args.family_view_topic,CompressedImage),
        ('semantic_view','/perception/camera/debug/mosaic/compressed',CompressedImage),
        ('objects_view','/perception/camera/objects/debug/mosaic/compressed',CompressedImage))]
    extras=set(args.extra_views.split(',')) - {''}
    if args.all_visuals:extras.update(('traffic_light','bev'))
    if extras:
        for name,topic in (('traffic_light_view','/perception/traffic_light/debug/image/compressed'),
                           ('bev_view','/perception/bev/debug/comparison/compressed')):
            if name[:-5] not in extras:continue
            subs.append(rospy.Subscriber(topic,CompressedImage,lambda msg,n=name:consume(msg,n),
                                         queue_size=1,buff_size=16<<20))
    subs.append(rospy.Subscriber('/perception/camera/lane/status',InferenceStatus,status,queue_size=1))
    try:
        while time.monotonic()-start<args.duration and not rospy.is_shutdown(): time.sleep(.1)
    finally:
        for sub in subs: sub.unregister()
        finished.set()
        with lock:
            snapshot={name:{k:list(v) if isinstance(v,deque) else dict(v) if k=='windows' else v
                           for k,v in row.items()} for name,row in state.items()}
        stream.close()
        gc.callbacks.remove(gc_observe)
        rospy.signal_shutdown('read-only display probe complete')
    def stats(values):
        a=np.asarray(values,dtype=float)
        return dict(count=len(a), **({'max':float(a.max()),**{'p'+str(q):float(np.percentile(a,q))
                    for q in (50,95,99)}} if len(a) else {}))
    duration=time.monotonic()-start;after=resources()
    resource_usage={v['label']:dict(cpu_percent_one_core=100*(v['cpu_ticks']-resource_before[pid]['cpu_ticks'])/
        os.sysconf('SC_CLK_TCK')/duration,rss_mib=v['rss_bytes']/2**20)
        for pid,v in after.items() if pid in resource_before}
    report=dict(duration_s=duration,status=dict(statuses),topics={},resources=resource_usage,
        observer_gc=list(gc_durations),journal_dropped=stream.dropped,
        limitations=['Existing live inference and exact source headers; no synthetic stamps.',
                     'ROS JPEG reception, not physical exposure or rqt presentation.',
                     'First/last 5s windows excluded from steady-rate gate.',
                     'Mosaic header alone cannot describe every per-view revision; repeated header is not necessarily repeated content.'])
    for name,r in snapshot.items():
        windows={str(k):v/5 for k,v in r['windows'].items() if k>=1 and (k+1)*5<=args.duration}
        report['topics'][name]=dict(fresh_count=r['count'],repeated_stamps=r['duplicate'],
            regressions=r['regressions'],complete_window_hz=windows,source_age_ms=stats(r['age_ms']),
            gaps_ms=stats(r['gaps_ms']),family_to_display_ms=stats(r['upstream_ms']))
    (args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2),flush=True)


if __name__=='__main__': main()
