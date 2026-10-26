#!/usr/bin/env python3
"""Read-only C0 live metadata bag summary. Never subscribes or controls the vehicle."""
import argparse
from collections import Counter,defaultdict
import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from audit_frame_time import read,stats,sha,write_json


def summarize(path):
    import rosbag
    topics=Counter();times=defaultdict(list);timings=defaultdict(Counter);status=Counter();states=Counter()
    zero=False;changed=None; examples=[];trace_reasons=Counter();imu_ages=[];span=[];zero_local=Counter()
    with rosbag.Bag(str(path)) as bag:
        for topic,m,t in bag.read_messages():
            now=t.to_nsec();topics[topic]+=1;times[topic].append(now)
            if topic=='/gps':
                current=m.latitude==0 and m.longitude==0
                if current!=zero or changed is None:changed=now
                zero=current
            elif topic in ('/vehicle/state','/planning/vehicle/state'):
                # Exclude transition delivery races, not the sustained invalid-GPS behavior.
                if changed is not None and now-changed>100000000:
                    states[(topic,'gps_zero' if zero else 'gps_nonzero',str(m.valid))]+=1
                    if zero and m.valid and len(examples)<3:examples.append(dict(topic=topic,x=m.x,y=m.y,stamp_ns=m.header.stamp.to_nsec()))
            elif topic.endswith('/observations_local') or topic.endswith('/observations_map'):
                status[(topic,str(m.objects.status),m.objects.reason)]+=1
                if zero and topic.endswith('/observations_local'):zero_local[(str(m.objects.status),m.objects.reason)]+=1
            elif topic.endswith('/frame_timing'):
                timings[m.stream_id][(m.source_clock_domain,str(m.source_valid),m.header_timestamp_policy)]+=1
                if m.stream_id=='lidar' and m.source_valid:
                    # Both are device-hour microseconds; never subtract from camera Unix source.
                    delta=(m.source_device_last_us-m.source_device_first_us)%(3600*1000000)
                    span.append(delta/1000.)
            elif topic.endswith('/trace'):
                r=json.loads(m.data);trace_reasons[r.get('reason','')]+=1
                if r.get('imu_ns'):imu_ages.append((r['stamp_ns']-r['imu_ns'])/1e6)
    return dict(schema='asmc.frame_time_live.v1',bag_sha256=sha(path),topics=dict(topics),
        supply={k:dict(mean_hz=(len(v)-1)*1e9/(v[-1]-v[0]) if len(v)>1 else None,
                        receive_gap_ms=stats((b-a)/1e6 for a,b in zip(v,v[1:]))) for k,v in times.items()},
        states={' | '.join(k):v for k,v in states.items()},
        lidar_status={' | '.join(k):v for k,v in status.items()},
        gps_zero_local={' | '.join(k):v for k,v in zero_local.items()},
        bridge_zero_gps_valid_examples=examples,trace_reasons=dict(trace_reasons),imu_selected_age_ms=stats(imu_ages),
        timing={k:{' | '.join(a):b for a,b in v.items()} for k,v in timings.items()},
        lidar_device_scan_span_ms=stats(span),
        limits=['Receive cadence is metadata-bag subscriber timing, not pure model latency.',
                'velodyne_device_hour cannot be subtracted from morai_sensor without an established clock mapping.',
                'No camera inference performance acceptance: duplicate pre-existing GPU allocations.',
                'State comparison excludes 100ms after GPS-zero transitions.'])

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--bag',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();write_json(a.output,summarize(a.bag))
