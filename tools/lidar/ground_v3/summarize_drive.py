#!/usr/bin/env python3
"""Summarize measured route/attitude/health; no detector accuracy claim."""
import argparse
from collections import Counter,defaultdict
import json
import math
from pathlib import Path

import numpy as np
from extract_tunnel_bag import rpy


def main():
    p=argparse.ArgumentParser()
    p.add_argument('capture',type=Path)
    a=p.parse_args()
    import rosbag
    meta=json.loads((a.capture/'manifest.json').read_text())
    if not meta.get('complete'):raise ValueError('incomplete capture cannot pass drive gate')
    out=a.capture/'drive_summary.json'
    if out.exists():raise FileExistsError(out)
    start=meta['measurement_start_wall_s'];end=meta['measurement_end_wall_s']
    samples=meta['samples'];links=defaultdict(list);stages=defaultdict(list)
    def stage(link):
        if link=='A2256W000126':return 'shadow_link_126'
        if link in ['A2256W000451','A2256W000446','A2256W000448']:return 'approach'
        if link in ['A2256W000128','A2256W000333','A2256W000154','A2256W000332','A2256W000751']:return 'recovery'
        return 'other'
    for i,row in enumerate(samples):
        ident=row.get('map_link',row['state']['vehicle_state']['current_link_info']['id']['value'])
        links[ident].append(row['elapsed_s']);stages[stage(ident)].append(row['elapsed_s'])
    imu=[];stamps=[];states=Counter();diagnostics=[];counts=Counter();images={}
    image_times=sorted(set([min(10.,(end-start)/2)]+[float(v) for v in range(30,int(end-start),30)]))
    image_index=0
    for item in meta['bags']:
        with rosbag.Bag(str(a.capture/item['path'])) as bag:
            for topic,msg,arrival in bag.read_messages(topics=['/imu','/velodyne_points',
                '/lidar_candidate/ground_live/status','/image_jpeg/compressed']):
                t=arrival.to_sec()
                if not start<=t<=end:continue
                counts[topic]+=1
                if topic=='/imu':
                    roll,pitch=rpy(msg)
                    if math.isfinite(roll) and math.isfinite(pitch):imu.append([t-start,math.degrees(roll),math.degrees(pitch)])
                elif topic=='/velodyne_points':stamps.append(t)
                elif topic=='/lidar_candidate/ground_live/status':
                    for d in msg.status:
                        if d.name=='asmc_lidar/ground_live':
                            states[d.message]+=1
                            if d.message=='valid':diagnostics.append({v.key:v.value for v in d.values})
                elif image_index<len(image_times) and t-start>=image_times[image_index]:
                    name='front_'+str(int(image_times[image_index])).zfill(3)+'.jpg'
                    (a.capture/name).write_bytes(bytes(msg.data));images[name]={'arrival_elapsed_s':t-start,'stamp_ns':msg.header.stamp.to_nsec()}
                    image_index+=1
    if len(imu)<2:raise ValueError('insufficient IMU observations')
    attitude=np.asarray(imu);cte=[r['cross_track_point_distance_m'] for r in samples if 'cross_track_point_distance_m' in r]
    report={'capture_complete':meta['complete'],'restored':meta['restored'],'duration_wall_s':end-start,
        'controller':meta.get('controller','native_cruise'),
        'links':{k:{'first_s':v[0],'last_s':v[-1],'span_s':v[-1]-v[0]} for k,v in links.items()},
        'stages':{k:{'first_s':v[0],'last_s':v[-1],'span_s':v[-1]-v[0]} for k,v in stages.items()},
        'cross_track_sample_distance_p99_max_m':[float(np.percentile(cte,99)),max(cte)] if cte else None,
        'scan_hz':(len(stamps)-1)/(stamps[-1]-stamps[0]) if len(stamps)>1 else None,
        'topic_counts':dict(counts),'states':dict(states),'images':images,
        'roll_deg_min_max':[float(attitude[:,1].min()),float(attitude[:,1].max())],
        'pitch_deg_min_max':[float(attitude[:,2].min()),float(attitude[:,2].max())],
        'attitude_exposure_wall_s':{str(deg):float(np.sum(np.diff(attitude[:,0])[
            (np.abs(attitude[:-1,1])>=deg)&(np.abs(attitude[1:,1])>=deg)&(np.diff(attitude[:,0])<=.2)])) for deg in [1,2,3,5]},
        'latency_ms':{key:{'p'+str(q):float(np.percentile([float(d[key]) for d in diagnostics],q))
            for q in [50,95,99]} for key in ['queue_wait_ms','prepare_ms','compute_ms','receipt_to_publish_ms'] if diagnostics},
        'queue_dropped_change':int(diagnostics[-1]['queue_dropped'])-int(diagnostics[0]['queue_dropped']) if diagnostics else None,
        'limits':['Link 126 is a team-reviewed shadow interval, not per-point wall/roof GT.',
            'Roll is vehicle attitude including suspension; map/ground corroboration needed for road cross-slope.',
            'Arrival rate and wall-time exposure include simulator speed and scheduling effects.']}
    out.write_text(json.dumps(report,indent=2,sort_keys=True)+'\n')
    np.save(a.capture/'imu_attitude.npy',attitude)
    print(json.dumps({k:report[k] for k in ['duration_wall_s','stages','cross_track_sample_distance_p99_max_m',
        'roll_deg_min_max','pitch_deg_min_max','attitude_exposure_wall_s','scan_hz','states','queue_dropped_change']},indent=2))


if __name__=='__main__':main()
