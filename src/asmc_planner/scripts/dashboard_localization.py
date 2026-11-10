"""Bounded, clock-aware map geometry from the selected research worktree."""
import json
import math
import time
import threading


def map_trajectory_fresh(message,ros_now,receive_age):
    header=message.get('header') or {};stamp=header.get('stamp',0)
    if not isinstance(stamp,(int,float)) or not math.isfinite(stamp):return False
    return bool(message.get('valid') and header.get('frame_id')=='map' and
                0<=ros_now-stamp<=.3 and 0<=receive_age<=.3)


class LocalizationView:
    def __init__(self):
        self.payload=None;self.received=None;self.trial=None;self.terminal=False;self.lock=threading.Lock()
        self.retired_frames=set()

    def experiment(self,identity,terminal=False):
        with self.lock:
            if identity!=self.trial or terminal:
                self.payload=None;self.received=None
            if identity!=self.trial:self.retired_frames.clear()
            self.trial=identity;self.terminal=terminal

    def put(self,raw):
        if len(raw)>256*1024:return
        try:p=json.loads(raw)
        except (ValueError,TypeError):return
        if not isinstance(p,dict) or p.get('frame')!='map':return
        stamp=p.get('stamp')
        if not isinstance(stamp,(int,float)) or not math.isfinite(stamp):return
        if p.get('motion_frame') is not None and not isinstance(p['motion_frame'],str):return
        def points(values):
            return isinstance(values,list) and len(values)<=2048 and all(
                isinstance(v,list) and len(v)==2 and all(isinstance(x,(int,float)) and math.isfinite(x) for x in v) for v in values)
        if not points(p.get('selected',[])):return
        candidates=p.get('candidates',[]);obstacles=p.get('obstacles',[])
        if not isinstance(candidates,list) or len(candidates)>24 or not all(isinstance(c,dict) and points(c.get('points')) for c in candidates):return
        if not isinstance(obstacles,list) or len(obstacles)>512 or not all(isinstance(c,dict) and points(c.get('points')) for c in obstacles):return
        ego=p.get('ego')
        if ego is not None and (not isinstance(ego,dict) or not all(
            isinstance(ego.get(k),(int,float)) and math.isfinite(ego[k]) for k in ('x','y','yaw'))):return
        with self.lock:
            if self.terminal:return
            if self.trial and (p.get('run_id'),p.get('trial_id'))!=self.trial:return
            frame=p.get('motion_frame')
            if frame in self.retired_frames:return
            if self.payload and frame!=self.payload.get('motion_frame'):
                if len(self.retired_frames)>=128:return
                self.retired_frames.add(self.payload.get('motion_frame'))
            if self.payload and stamp<self.payload['stamp'] and p.get('motion_frame')==self.payload.get('motion_frame'):return
            self.payload=p;self.received=time.monotonic()

    def snapshot(self,ros_now):
        with self.lock:
            if self.payload is None:return None
            p=dict(self.payload);received=self.received
        p['receive_age_s']=time.monotonic()-received;p['source_age_s']=ros_now-p['stamp']
        p['fresh']=0<=p['source_age_s']<=.6 and p['receive_age_s']<=.6
        if not p['fresh']:
            p.update(valid=False,selected=[],candidates=[],obstacles=[],ego=None,reason='stale_localization_geometry')
        elif not p.get('valid'):p['selected']=[]
        return p


def mask_inactive_experiment(snapshot):
    """Ended/reconnecting runs cannot leave any current vehicle/path overlay."""
    experiment=snapshot.get('experiment')
    if not experiment:return
    if (experiment.get('heartbeat_fresh') and not experiment.get('terminal') and
            experiment.get('phase') not in ('restoring','stopped','failed','cancelled','completed')):
        return
    for key in ('ego','trajectory','lidar_objects','planning_markers','local_experiment','local_control'):
        snapshot[key]=None
    snapshot['shadow']={}
    snapshot['normal_path_fresh']=False
    snapshot['jamming_transition_fresh']=False
    if snapshot.get('localization'):
        snapshot['localization']=dict(snapshot['localization'],fresh=False,valid=False,
            ego=None,candidates=[],selected=[],obstacles=[],reason='experiment_inactive')


class ExperimentView:
    TERMINAL={'completed','failed','cancelled','stopped'}
    def __init__(self,geometry):self.geometry=geometry;self.payload=None;self.received=None;self.lock=threading.Lock()
    def put(self,raw):
        if len(raw)>32768:return
        try:p=json.loads(raw)
        except (ValueError,TypeError):return
        if not isinstance(p,dict) or p.get('schema')!='asmc.localization.experiment.v1':return
        if not all(isinstance(p.get(k),str) and p[k] for k in ('run_id','trial_id','phase','revision','branch','master_uri')):return
        with self.lock:
            if self.payload and p.get('started_at',0)<self.payload.get('started_at',0):return
            self.geometry.experiment((p['run_id'],p['trial_id']),p['phase'] in self.TERMINAL|{'restoring'})
            self.payload=p;self.received=time.monotonic()
    def snapshot(self):
        with self.lock:
            if self.payload is None:return None
            p=dict(self.payload);age=time.monotonic()-self.received
        p['receive_age_s']=age;p['heartbeat_fresh']=age<=3.;p['terminal']=p['phase'] in self.TERMINAL
        if not p['terminal'] and age>3:p['display_phase']='monitor_connection_lost'
        else:p['display_phase']=p['phase']
        return p
