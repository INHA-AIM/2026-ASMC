#!/usr/bin/env python3
"""Isolated ROS renderer timing, using saved family geometry, without rqt/GPU.

The optional render prototype exists only in this probe. No installed node is
patched. Synthetic arrival timing is not a MORAI full-stack acceptance test.
"""
import argparse
from collections import defaultdict
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
from xmlrpc.client import ServerProxy


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--samples',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--port',type=int,default=11364)
    args=parser.parse_args()
    if args.port==11311 or not 1024<=args.port<=65535:raise ValueError('isolated master required')
    args.output.mkdir(parents=True,exist_ok=False)
    uri='http://127.0.0.1:'+str(args.port)
    try:ServerProxy(uri).getPid('/family_view_probe')
    except OSError:pass
    else:raise RuntimeError('master port occupied')
    os.environ.update(ROS_MASTER_URI=uri,ROS_HOSTNAME='127.0.0.1')
    with (args.output/'roscore.log').open('w') as log:
        master=subprocess.Popen(['roscore','-p',str(args.port)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            for _ in range(100):
                try:ServerProxy(uri).getPid('/family_view_probe');break
                except OSError:time.sleep(.1)
            else:raise RuntimeError('master unavailable')
            run(args)
        finally:
            os.killpg(master.pid,signal.SIGTERM)
            try:master.wait(timeout=10)
            except subprocess.TimeoutExpired:os.killpg(master.pid,signal.SIGKILL);master.wait()


def run(args):
    import cv2
    import numpy as np
    import rospy
    from geometry_msgs.msg import Point32
    from sensor_msgs.msg import CompressedImage
    from asmc_msgs.msg import LaneBoundaryFamily,LaneBoundaryFamilyMember
    from asmc_camera.nodes import family_view_node as mod
    cv2.setNumThreads(1)
    rospy.init_node('family_view_probe',anonymous=True,disable_signals=True)
    for k,v in dict(frame_id='base_link',x_min_m=-5.,x_max_m=20.,y_min_m=-8.,y_max_m=8.,resolution_m=.1,managed=False).items():
        rospy.set_param('~'+k,v)
    samples=json.loads(args.samples.read_text())
    stage={};source={};received={};output=[];lock=threading.Lock()
    active=[False];trace=defaultdict(list);variant=['baseline']
    original_grid=mod.draw_metric_grid;original_family=mod.draw_stateful_lane_family;original_encode=cv2.imencode
    def instrument(name,func):
        def wrapper(*a,**kw):
            start=time.monotonic();result=func(*a,**kw)
            if active[0]:stage[name]=stage.get(name,0)+1000*(time.monotonic()-start)
            return result
        return wrapper
    mod.draw_metric_grid=instrument('grid_ms',original_grid)
    mod.draw_stateful_lane_family=instrument('family_draw_ms',original_family)
    cv2.imencode=instrument('jpeg_ms',original_encode)

    class Measured(mod.FamilyViewNode):
        def __init__(self):
            # Preserve this tool's timer/prototype comparison after the normal
            # renderer moved to arrival-driven scheduling.
            super().__init__(arrival_driven=False)
        def _receive(self,message):
            begin=time.monotonic();stamp=message.header.stamp.to_nsec()
            with lock:received[stamp]=begin
            super()._receive(message)
            trace[variant[0]+'_receive_callback_ms'].append(1000*(time.monotonic()-begin))
        def _publish(self,event):
            if not active[0]:return
            stage.clear();begin=time.monotonic()
            if variant[0]=='baseline':super()._publish(event)
            else:self.prototype()
            trace[variant[0]+'_render_timer_ms'].append(1000*(time.monotonic()-begin))
            for k,v in stage.items():trace[variant[0]+'_'+k].append(v)
        def prototype(self):
            if not self._requested():self._clear();return
            begun=time.monotonic()
            with self.gate.lock,self.lock:
                sample=self.sample;context=self.gate.context()
                current=bool(sample and sample[3]==context and self.gate.allows(sample[0].stamp)
                    and time.monotonic()-sample[4]<=.35 and -.05<=(rospy.Time.now()-sample[0].stamp).to_sec()<=.35)
            trace['prototype_snapshot_lock_ms'].append(1000*(time.monotonic()-begun))
            canvas=self.background.copy()
            mod.draw_stateful_lane_family(canvas,sample[1] if current else [],self.grid,sample[2] if current else 'WAITING / STALE')
            ok,jpeg=cv2.imencode('.jpg',canvas,[cv2.IMWRITE_JPEG_QUALITY,80])
            if not ok:return
            with self.gate.lock:
                if context!=self.gate.context():return
                if current and (not self.gate.allows(sample[0].stamp) or
                                not -.05<=(rospy.Time.now()-sample[0].stamp).to_sec()<=.35):return
                msg=CompressedImage(format='jpeg',data=jpeg.tobytes())
                if sample:msg.header=sample[0]
                self.publisher.publish(msg)

    node=Measured()
    node.background=np.zeros((*node.grid.shape,3),np.uint8);original_grid(node.background,node.grid)
    pub=rospy.Publisher('/perception/lane/boundary_family',LaneBoundaryFamily,queue_size=1)
    def consume(msg):
        now=time.monotonic();stamp=msg.header.stamp.to_nsec()
        with lock:
            if stamp in source:
                output.append(dict(variant=variant[0],stamp=stamp,received=now,
                    source_to_output_ms=1000*(now-source[stamp]),
                    source_to_receive_ms=1000*(received.get(stamp,now)-source[stamp]),
                    receive_to_output_ms=1000*(now-received.get(stamp,now))))
    sub=rospy.Subscriber(node.publisher.resolved_name,CompressedImage,consume,queue_size=1)
    try:
        time.sleep(1.)
        for candidate in ('baseline','prototype'):
            variant[0]=candidate;active[0]=True
            for i in range(180):
                raw=samples[i%len(samples)];msg=LaneBoundaryFamily(valid=raw['valid'],status_reason=raw['reason'] or '')
                msg.header.stamp=rospy.Time.now();msg.header.frame_id='base_link'
                for j,b in enumerate(raw['boundaries']):
                    member=LaneBoundaryFamilyMember(role=j+1,color=b['class_id'],support_state=b['support'])
                    member.polyline=[Point32(x=x,y=y,z=0.) for x,y in b['xy']]
                    msg.boundaries.append(member)
                stamp=msg.header.stamp.to_nsec()
                with lock:source[stamp]=time.monotonic()
                pub.publish(msg);time.sleep(1/15)
            active[0]=False;time.sleep(.2)
        # Demand gating is measured, not assumed from an absent GUI.
        sub.unregister();time.sleep(.6)
        demand_unsubscribed=node.input.sub is None
    finally:
        active[0]=False;node.close();node.gate.subscriber.unregister()
        pub.unregister();sub.unregister();node.publisher.unregister()
        mod.draw_metric_grid=original_grid;mod.draw_stateful_lane_family=original_family;cv2.imencode=original_encode
        rospy.signal_shutdown('offline renderer probe complete')
    def stats(values):
        return dict(count=len(values),**{'p'+str(q):float(np.percentile(values,q)) for q in (50,95,99)}) if values else dict(count=0)
    report=dict(schema='family_view_probe_v1',duration_per_variant_s=12,input_hz=15,
        demand_unsubscribed=demand_unsubscribed,
        stages={k:stats(v) for k,v in trace.items()},variants={},
        limitations=['Saved geometry with synthetic fresh stamps; not a simulator exposure timestamp test.',
                     'Loopback ROS consumer, not rqt screen presentation.',
                     'Prototype is local to this probe. Operational code/defaults unchanged.'])
    for candidate in ('baseline','prototype'):
        values=[r for r in output if r['variant']==candidate]
        report['variants'][candidate]={k:stats([r[k] for r in values]) for k in
            ('source_to_output_ms','source_to_receive_ms','receive_to_output_ms')}
    (args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    (args.output/'events.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in output))
    print(json.dumps(report,indent=2),flush=True)


if __name__=='__main__':main()
