#!/usr/bin/env python3
"""Read-only live visualization probe. No control, weather or mode requests."""
import argparse,json,time,threading
from pathlib import Path
from collections import deque
import numpy as np
import psutil
import rospy,rosgraph,rosnode
from sensor_msgs.msg import CompressedImage
from asmc_msgs.msg import LaneBoundaryFamily,InferenceStatus,CameraGroundProjection
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--seconds',type=float,default=60);a=p.parse_args()
a.output.mkdir(parents=True,exist_ok=False)
rospy.init_node('shared_bev_measure',anonymous=True,disable_signals=True)
lock=threading.Lock(); started=time.monotonic(); records={}; statuses={}; contexts={}; saved=set(); streams={
 'comparison':('/perception/bev/debug/comparison/compressed',CompressedImage),
 'family_view':('/perception/lane/debug/shared_family_bev/compressed',CompressedImage),
 'semantic_view':('/perception/camera/debug/mosaic/compressed',CompressedImage),
 'objects_view':('/perception/camera/objects/debug/mosaic/compressed',CompressedImage),
 'family':('/perception/lane/boundary_family',LaneBoundaryFamily)}
def receive(name,message):
 now=time.monotonic(); age=(rospy.Time.now()-message.header.stamp).to_sec()*1000
 with lock:
  records.setdefault(name,deque(maxlen=5000)).append((now,message.header.stamp.to_nsec(),age,bool(getattr(message,'valid',True))))
  if name in ('comparison','family_view') and name not in saved and now-started>5:
   (a.output/(name+'.jpg')).write_bytes(message.data);saved.add(name)
def status(name,message):
 with lock: statuses.setdefault(name,deque(maxlen=1000)).append((time.monotonic(),bool(message.pipeline_ready),message.errors,message.generation,message.producer_session_id))
def context(message):
 with lock:
  contexts.setdefault('events',deque(maxlen=5000)).append((time.monotonic(),message.header.stamp.to_nsec(),message.imu_corrected,message.reason))
subs=[rospy.Subscriber(topic,kind,lambda m,n=name:receive(n,m),queue_size=1,buff_size=8*1024*1024) for name,(topic,kind) in streams.items()]
subs += [rospy.Subscriber('/perception/camera/'+name+'/status',InferenceStatus,lambda m,n=name:status(n,m),queue_size=1) for name in ('objects','lane','traffic_light')]
subs += [rospy.Subscriber('/perception/lane/ground_projection',CameraGroundProjection,context,queue_size=1)]
master=rosgraph.Master(rospy.get_name()); processes=[]
for proc in psutil.process_iter(['pid','cmdline']):
 command=proc.info['cmdline'] or []
 names=[s.split('=',1)[1] for s in command if s.startswith('__name:=')]
 if names and any(k in names[0] for k in ('camera_', 'traffic_light')):
  proc.cpu_percent();processes.append(('/'+names[0],proc))
cpu=[]
while time.monotonic()-started<a.seconds+5 and not rospy.is_shutdown():
 time.sleep(1.)
 if time.monotonic()-started>=5:
  cpu.append({name:proc.cpu_percent() for name,proc in processes if proc.is_running()})
end=time.monotonic();begin=started+5
summary=dict(duration_sec=end-begin,read_only=True,gui_window=False,streams={},status={},cpu_mean={},graph=master.getSystemState())
for name,values in records.items():
 rows=[r for r in values if begin<=r[0]<=end];times=np.array([r[0] for r in rows]);stamps=[r[1] for r in rows]
 windows=[sum(begin+i*5<=t<begin+(i+1)*5 for t in times)/5 for i in range(int((end-begin)//5))]
 summary['streams'][name]=dict(count=len(rows),hz=len(rows)/(end-begin),minimum_5s_hz=min(windows) if windows else None,
  gap_max_ms=float(np.max(np.diff(times))*1000) if len(times)>1 else None,
  source_age_ms=dict(zip(('p50','p95','p99'),np.percentile([r[2] for r in rows],[50,95,99]).tolist())) if rows else {},
  duplicate_or_reverse=sum(b<=a for a,b in zip(stamps,stamps[1:])),invalid=sum(not r[3] for r in rows))
for name,values in statuses.items():
 rows=[r for r in values if r[0]>=begin]
 summary['status'][name]=dict(samples=len(rows),not_ready=sum(not r[1] for r in rows),max_errors=max((r[2] for r in rows),default=None))
for name,_ in processes: summary['cpu_mean'][name]=float(np.mean([r[name] for r in cpu if name in r])) if cpu else None
family_times={r[1]:r[0] for r in records.get('family',()) if r[0]>=begin}
leads=[1000*(family_times[r[1]]-r[0]) for r in contexts.get('events',()) if r[1] in family_times]
summary['context_lead_before_family_ms']=dict(zip(('p50','p95'),np.percentile(leads,[50,95]).tolist())) if leads else {}
summary['shared_context']=dict(count=len(contexts.get('events',())),imu=sum(r[2] for r in contexts.get('events',())),m0=sum(not r[2] for r in contexts.get('events',())))
(a.output/'report.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps({k:v for k,v in summary.items() if k not in ('graph',)},indent=2))
for s in subs:s.unregister()
rospy.signal_shutdown('complete')
