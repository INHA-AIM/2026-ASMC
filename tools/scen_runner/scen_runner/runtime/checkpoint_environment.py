"""Checkpoint weather/hour changes, independent of scene spawning randomness."""
import math
import random
import time
import copy
from collections import deque

from scen_runner.config import TOOL_ROOT, load_config, load_yaml, repo_path


def segment_distance(point, start, end):
    dx,dy=end[0]-start[0],end[1]-start[1]
    length2=dx*dx+dy*dy
    t=max(0.,min(1.,((point[0]-start[0])*dx+(point[1]-start[1])*dy)/length2)) if length2 else 0.
    return math.dist(point,(start[0]+t*dx,start[1]+t*dy))


class CheckpointEnvironment:
    def __init__(self,link,config=None,checkpoints=None):
        self.link=link
        self.cfg=config if config is not None else load_yaml(TOOL_ROOT/'config/checkpoint_environment.yaml')
        self.combinations=[(w,int(h)) for w in self.cfg['weathers'] for h in self.cfg['hours']]
        if len(set(self.combinations))!=6 or set(self.combinations)!={(w,h) for w in ('Sunny','Foggy') for h in (11,13,15)}:
            raise ValueError('환경 후보는 Sunny/Foggy × 11/13/15시의 6가지 조합이어야 합니다.')
        if checkpoints is None:
            path=repo_path(load_config()['map']['competition_metadata'])
            checkpoints=load_yaml(path)['checkpoints']
        self.checkpoints=[]
        self.start_end=None
        for item in sorted(checkpoints,key=lambda c:c['sequence']):
            position=dict(item['pose']['position'])
            position.update(self.cfg.get('checkpoint_pose_overrides',{}).get(str(item['id']),{}))
            point=dict(id=str(item['id']),xy=(float(position['x']),float(position['y'])),
                       radius=float(item['pass_radius_m']))
            if point['radius']<=0:raise ValueError('체크포인트 반경은 양수여야 합니다.')
            if str(item['id']).isdigit():self.checkpoints.append(point)
            elif item.get('is_start') and item.get('is_finish'):self.start_end=point
        self.current=None
        self.rng=random.Random()
        self.mode=None
        self.visited=set()
        self.last_sample=None
        self.switch_count=0
        self.last_checkpoint=None
        self.reason='환경 적용 전'
        self.error=None
        self.clock=time.monotonic;self.timeline=deque(maxlen=256)

    def change(self,reason,checkpoint=None):
        choices=[pair for pair in self.combinations if pair!=self.current]
        weather,hour=self.rng.choice(choices)
        # Commit only after successful RPCs and readback. A partial/failed
        # update must not be labelled as the confirmed current environment.
        try:
            self.link.set_environment(weather,hour,instantly=self.cfg['instantly'])
        except Exception as error:
            self.error=str(error)
            raise
        self.current=(weather,hour)
        self.error=None
        self.switch_count+=1
        self.last_checkpoint=checkpoint
        self.reason=reason
        self.timeline.append((self.clock(),self.snapshot()))
        print(f'[환경 변경] {reason} · {weather} · {hour}:00',flush=True)

    def begin(self,seed,mode):
        self.rng=random.Random(int(seed)^0x454E5652)
        self.mode=mode
        self.visited.clear()
        self.last_sample=None
        self.switch_count=0
        self.last_checkpoint=None
        self.change('씬 시작' if mode=='scene' else '구간 반복 시작')

    def update(self,state,now):
        if self.mode!='scene':return []
        p,v=state.transform.location,state.velocity
        xy=(p.x,p.y)
        speed=math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)/3.6
        previous=self.last_sample
        self.last_sample=(xy,now,speed)
        if previous is None:
            start=xy
        else:
            old_xy,old_time,old_speed=previous
            # Do not interpret a teleport/reset as driving through every point
            # on the line between the old and new positions.
            radius=max([c['radius'] for c in self.checkpoints] or [3.])
            possible=max(2*radius,2*max(speed,old_speed)*max(0.,now-old_time)+2*radius)
            if math.dist(old_xy,xy)>possible:return []
            start=old_xy
        # One pass per checkpoint per lap. On return to START/END after CP14,
        # re-arm CP1..14 without an additional weather change at START/END.
        if '14' in self.visited and self.start_end and segment_distance(self.start_end['xy'],start,xy)<=self.start_end['radius']:
            self.visited.clear()
            start=xy  # Do not replay the finishing lap's segment into the new lap.
        changed=[]
        for checkpoint in self.checkpoints:
            aid=checkpoint['id']
            if aid in self.visited:continue
            if segment_distance(checkpoint['xy'],start,xy)<=checkpoint['radius']:
                self.change(f'체크포인트 {aid}',checkpoint=aid)
                self.visited.add(aid)
                changed.append(aid)
        return changed

    def snapshot(self):
        weather,hour=self.current if self.current is not None else (None,None)
        return dict(weather=weather,hour=hour,reason=self.reason,last_checkpoint=self.last_checkpoint,
                    switch_count=self.switch_count,visited=sorted(self.visited,key=int),error=self.error)

    def snapshot_at(self,now):
        for confirmed,snapshot in reversed(list(self.timeline)):
            if confirmed<=now:return dict(copy.deepcopy(snapshot),confirmed_at=confirmed)
        return dict(weather=None,hour=None,error='조회 시점 이전에 확인된 환경 없음')
