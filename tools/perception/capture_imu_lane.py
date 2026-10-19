#!/usr/bin/env python3
"""Read-only exact RGB/grid/lineage capture with source-time attitude and supply.

No control, simulator pose, model execution or parameter changes. Disk work runs
in the main thread; transport callbacks keep bounded latest frame references.
"""
import argparse
from collections import deque
import hashlib
import json
from pathlib import Path
import threading
import time

import message_filters
import numpy as np
import rospy
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import CompressedImage, Image, Imu
from asmc_msgs.msg import CameraModelGridTiming, SensorFrameTiming, LaneBoundaryFamily
from asmc_camera.runtime.sensor_timing import ImuTimeline, sensor_key


def plain(value):
    if isinstance(value,(bytes,bytearray)): return list(value)
    if hasattr(value, 'to_nsec'): return value.to_nsec()
    if hasattr(value, '__slots__'): return {k:plain(getattr(value,k)) for k in value.__slots__}
    if isinstance(value,(tuple,list)): return [plain(x) for x in value]
    return value


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--duration',type=float,default=20.)
    parser.add_argument('--sample-hz',type=float,default=2.)
    args=parser.parse_args()
    if not 0 < args.duration <= 600:parser.error('duration must be in (0,600]')
    if not .1 <= args.sample_hz <= 15:parser.error('sample-hz must be in [.1,15]')
    args.output.mkdir(parents=True,exist_ok=False)
    rospy.init_node('imu_lane_capture',anonymous=True,disable_signals=True)
    timeline=ImuTimeline();lock=threading.Lock();latest={};journal=deque(maxlen=2048)
    dropped=[0];counts={};subscribers=[];synchronizers=[];closing=threading.Event()
    def record(kind,message):
        with lock:
            if closing.is_set():return
            counts[kind]=counts.get(kind,0)+1
            if len(journal)==journal.maxlen:dropped[0]+=1
            journal.append(dict(kind=kind,received_monotonic=time.monotonic(),message=plain(message)))
    def imu(message):
        if message.orientation_covariance[0] >= 0:
            timeline.add_imu(sensor_key('/imu',message.header),tuple(getattr(message.orientation,k) for k in 'xyzw'),message.header.frame_id)
        record('imu',message)
    def timing(message):
        if message.stream_id=='imu':
            timeline.add_timing(message)
            record('imu_timing',message)
    subscribers.extend([rospy.Subscriber('/imu',Imu,imu,queue_size=64),
        rospy.Subscriber('/morai_sensor_bridge/frame_timing',SensorFrameTiming,timing,queue_size=256),
        rospy.Subscriber('/perception/lane/boundary_family',LaneBoundaryFamily,lambda m:record('family',m),queue_size=4),
        rospy.Subscriber('/perception/lane/diagnostics',DiagnosticArray,lambda m:record('diagnostics',m),queue_size=2)])
    views={'front':'/image_jpeg/compressed','left':'/cam_left/image_jpeg/compressed','right':'/cam_right/image_jpeg/compressed'}
    def matched(view,classes,confidence,metadata,rgb):
        if (classes.header!=metadata.class_header or confidence.header!=metadata.confidence_header or
                rgb.header!=metadata.image_header or view!=metadata.view):return
        with lock:
            if not closing.is_set():latest[view]=(classes,confidence,metadata,rgb)
    for view,topic in views.items():
        base='/perception/camera/'+view+'/road_marking/model_grid/'
        group=[message_filters.Subscriber(base+'class_id',Image,queue_size=2,buff_size=4<<20),
               message_filters.Subscriber(base+'confidence',Image,queue_size=2,buff_size=4<<20),
               message_filters.Subscriber(base+'timing',CameraModelGridTiming,queue_size=4),
               message_filters.Subscriber(topic,CompressedImage,queue_size=4,buff_size=4<<20)]
        sync=message_filters.TimeSynchronizer(group,8)
        sync.registerCallback(lambda *m,v=view:matched(v,*m))
        subscribers.extend(group);synchronizers.append(sync)
    start=time.monotonic();last_saved=0.;last_key=None;index=0
    with (args.output/'events.jsonl').open('w') as events,(args.output/'frames.jsonl').open('w') as frames:
        try:
            while not rospy.is_shutdown() and time.monotonic()-start<args.duration:
                with lock:
                    pending=list(journal);journal.clear();sample=dict(latest)
                for row in pending:events.write(json.dumps(row)+'\n')
                if set(sample)==set(views) and time.monotonic()-last_saved>=1./args.sample_hz:
                    stamps=[row[0].header.stamp.to_nsec() for row in sample.values()]
                    key=tuple((sample[v][2].producer_session_id,sample[v][2].generation,
                               sample[v][2].image_header.seq, sample[v][2].image_header.stamp.to_nsec()) for v in views)
                    if max(stamps)-min(stamps)<=35e6 and key!=last_key:
                        arrays={};metadata={}
                        for view,(classes,confidence,timing,rgb) in sample.items():
                            for name,message in [('class',classes),('confidence',confidence)]:
                                if message.encoding!='mono8' or message.step!=message.width:raise ValueError('unexpected grid layout')
                                arrays[view+'_'+name]=np.frombuffer(message.data,np.uint8).reshape(message.height,message.width)
                            file='%03d-%s.jpg'%(index,view);(args.output/file).write_bytes(bytes(rgb.data))
                            pose,reason=timeline.attitude(timing.sensor_timing)
                            metadata[view]=dict(lineage=plain(timing),rgb=file,
                                rgb_sha256=hashlib.sha256(rgb.data).hexdigest(),attitude_reason=reason,
                                quaternion=list(pose.quaternion) if pose else None,
                                bracket_ms=pose.bracket_ms if pose else None)
                        grid='%03d.npz'%index;np.savez_compressed(str(args.output/grid),**arrays)
                        frames.write(json.dumps(dict(index=index,grid=grid,views=metadata))+'\n');frames.flush()
                        index+=1;last_saved=time.monotonic();last_key=key
                time.sleep(.02)
        finally:
            closing.set()
            for sub in subscribers:sub.unregister()
            with lock:
                for row in journal:events.write(json.dumps(row)+'\n')
    report=dict(duration_sec=time.monotonic()-start,frames=index,counts=counts,journal_drops=dropped[0],
        sample_hz=args.sample_hz,
        timeline=timeline.snapshot(),imu_enabled=rospy.get_param('/camera_lane_vectorizer/imu_motion/enabled',None))
    (args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2),flush=True)


if __name__=='__main__':main()
