#!/usr/bin/env python3
"""Held-ego camera/map association check. Simulator state is an oracle only.

Run in the ROS environment with an explicit endpoint and reviewed geometry.
Each observation is recorded, including UNKNOWN and disagreements; an RPC
acknowledgement never substitutes for actual camera recognition.
"""
import argparse
import collections
import json
import math
import threading
import time
from pathlib import Path

import rospy
import yaml
from asmc_msgs.msg import TrafficLightDetection
from sensor_msgs.msg import CompressedImage
from scen_runner.morai.clock_contract import require_nominal_rate
from scen_runner.runtime.signal_control import SignalControl
from scen_runner.runtime.validation_episode import EpisodeLink, EpisodeRoute


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host',required=True)
    parser.add_argument('--port',type=int,default=7789)
    parser.add_argument('--geometry',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--distances',type=float,nargs='+',default=[30.,15.,6.])
    args=parser.parse_args()
    rospy.init_node('yellow_camera_qualification',anonymous=True,disable_signals=True)
    require_nominal_rate()
    args.output.mkdir(parents=True,exist_ok=True)
    stream=(args.output/'observations.jsonl').open('a',buffering=1)
    emit=lambda event:stream.write(json.dumps(dict(monotonic=time.monotonic(),**event))+'\n')
    cfg={'morai':dict(host=args.host,port=args.port,client_key='asmc_yellow_validation',
                     ego_id='Ego',grpc_src='tools/grpc_inha_univ/src')}
    link=EpisodeLink(cfg);link.connect();link.hold()
    route=EpisodeRoute();lights=yaml.safe_load(args.geometry.read_text())['reviewed_bindings']
    owner=SignalControl(link,emit)
    received=[];image=[None];lock=threading.Lock()
    def detection(msg):
        with lock:
            received.append(dict(received=time.monotonic(),stamp=msg.header.stamp.to_sec(),
                frame=msg.header.frame_id,state=msg.state,label=msg.label,valid=msg.valid,
                stale=msg.stale,confidence=msg.confidence,candidates=list(msg.candidate_labels)))
    def jpeg(msg):
        with lock:image[0]=msg
    subs=[rospy.Subscriber('/perception/traffic_light/detection',TrafficLightDetection,detection,queue_size=128),
          rospy.Subscriber('/cam_extra/image_jpeg/compressed',CompressedImage,jpeg,queue_size=1)]
    results=[]
    try:
        for light in lights:
            aid=light['traffic_light_id']
            owner.begin(aid,light['intersection_id'],light['green_phase'],light['green_color'],light['phase_duration_s'])
            station,error=route.curve.project(*light['line_point'])
            if error>3.:raise RuntimeError('stopline is not on the official route')
            for distance in args.distances:
                x,y,z,h=route.curve.pose(station-distance)
                pose=dict(x=x,y=y,z=z+.35,roll=0.,pitch=0.,yaw=math.degrees(h))
                link.reset(pose)
                if not link.held_at(pose):raise RuntimeError('ego reset/hold verification failed')
                time.sleep(.5)
                for color in (1,4,light['green_color']):
                    require_nominal_rate()
                    owner.set_aspect(aid,color)
                    begun=time.monotonic();end=begun+1.5
                    while time.monotonic()<end:
                        owner.tick();time.sleep(.05)
                    actual=link.traffic_light_color(aid)
                    with lock:
                        samples=[r for r in received if r['received']>=begun+.35]
                        frame=image[0]
                    filename=f'{aid}_{distance:g}m_{color}'
                    if frame is not None:(args.output/(filename+'.jpg')).write_bytes(frame.data)
                    # Detailed direction-specific qualification remains explicit.
                    expected={1:{2},4:{3},16:{4},32:{8},33:{6},48:{7}}[color]
                    good=[r for r in samples if r['valid'] and not r['stale'] and r['state'] in expected]
                    wrong=[r for r in samples if r['valid'] and r['state'] not in expected]
                    result=dict(event='camera_aspect',light=aid,distance=distance,requested=color,actual=actual,
                        sample_count=len(samples),matching=len(good),different=len(wrong),samples=samples,
                        observed_states=dict(collections.Counter(str(r['state']) for r in samples)),
                        qualified=actual==color and len(good)>=3 and not wrong,pose=pose)
                    results.append(result);emit(result)
                    print(json.dumps({k:v for k,v in result.items() if k not in ('samples','pose')}),flush=True)
    finally:
        link.hold()
        try:owner.restore()
        finally:
            (args.output/'summary.json').write_text(json.dumps(results,indent=2))
            stream.close();link.disconnect()
            for sub in subs:sub.unregister()
    return 0 if results and all(r['qualified'] for r in results) else 1


if __name__=='__main__':raise SystemExit(main())
