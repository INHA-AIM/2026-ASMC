"""Observe requested targets and native controls without changing vehicle motion."""
import json
import math
import time
import uuid
import threading
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from scen_runner.config import TOOL_ROOT


def vehicle_records(states, details, now):
    records = []
    vehicles = [(s.actor_info.id.value,s.transform.location.x,s.transform.location.y)
                for s in states if s.actor_info.object_type == 1]
    selected=[s for s in states if s.actor_info.id.value in details and details[s.actor_info.id.value]['kind']==1]
    directions=[(math.cos(math.radians(s.transform.rotation.z)),math.sin(math.radians(s.transform.rotation.z)))
                for s in selected]
    front_centers=None
    if len(selected)*len(vehicles)>=4096:
        # Keep the same heading, identity, ahead-of-car and 2m lateral tests.
        # Broadcast only the diagnostic arithmetic; no TTC/control rules here.
        own=np.asarray([(s.transform.location.x,s.transform.location.y) for s in selected])
        other=np.asarray([(x,y) for _,x,y in vehicles]);direction=np.asarray(directions)
        own_ids=np.asarray([s.actor_info.id.value for s in selected])
        other_ids=np.asarray([aid for aid,_,_ in vehicles])
        with np.errstate(invalid='ignore'):
            delta=other[None,:,:]-own[:,None,:]
            along=delta[:,:,0]*direction[:,0,None]+delta[:,:,1]*direction[:,1,None]
            across=-delta[:,:,0]*direction[:,1,None]+delta[:,:,1]*direction[:,0,None]
            mask=(along>0) & (np.abs(across)<2.) & (own_ids[:,None]!=other_ids[None,:])
            minimum=np.where(mask,along,np.inf).min(axis=1)
        front_centers=[float(v) if present else None for v,present in zip(minimum,mask.any(axis=1))]
    for index,state in enumerate(selected):
        aid = state.actor_info.id.value;info = details[aid]
        p, v, vs = state.transform.location, state.velocity, state.vehicle_state
        if front_centers is None:
            cosine,sine=directions[index];front=None
            for other_id,x,y in vehicles:
                if other_id==aid:continue
                dx,dy=x-p.x,y-p.y
                along,across=dx*cosine+dy*sine,-dx*sine+dy*cosine
                if along>0 and abs(across)<2. and (front is None or along<front):front=along
        else:front=front_centers[index]
        records.append(dict(info, id=aid, x=p.x, y=p.y, z=p.z,
                            speed=math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z),
                            age_s=max(0.,now-info['born_at']), throttle=vs.throttle, brake=vs.brake,
                            gear=vs.gear_mode, link=vs.current_link_info.id.value,
                            remaining_distance=vs.remaining_distance,
                            traffic_light=vs.tl_id.value, traffic_color=vs.tl_color,
                            front_center_m=front))
    return records


def crossing_records(states, details, ego_id='Ego'):
    live = {s.actor_info.id.value:s for s in states}
    ego = live.get(ego_id)
    records = []
    for aid, info in details.items():
        record = dict(info, state_received=aid in live, ego_received=ego is not None)
        actor = live.get(aid)
        if actor is not None:
            p, v = actor.transform.location, actor.velocity
            record.update(position=[p.x,p.y,p.z], speed=math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z),
                          moved_from_start_m=math.dist((p.x,p.y),info['start'][:2]),
                          remaining_to_end_m=math.dist((p.x,p.y),info['end'][:2]))
        if ego is not None:
            p = ego.transform.location
            distance = math.dist((p.x,p.y),info['start'][:2])
            record.update(ego_position=[p.x,p.y,p.z],ego_to_start_m=distance,
                          inside_activation=distance <= info['activation_distance_m'])
        if actor is None or ego is None:
            record['status']='상태 미수신'
        elif not info.get('route_registered',True):
            record['status']='생성 후 경로 등록 대기'
        elif info['preparation_paused']:
            record['status']='배치 준비 정지'
        elif record['remaining_to_end_m'] <= .4:
            record['status']='끝점 도착'
        elif record['speed'] > .2 and record['moved_from_start_m'] > .1:
            record['status']='이동 관찰'
        elif record['moved_from_start_m'] > .4:
            record['status']='경로 중 정지 관찰'
        elif record['inside_activation']:
            record['status']='거리 조건 충족 · 출발점 대기'
        else:
            record['status']='거리 조건 밖 · 출발점 대기'
        records.append(record)
    return records


class CrossingConsole:
    def __init__(self):
        self.previous={}
        self.inside_since={}
        self.stalled_reported=set()

    def sample(self, records, episode, phase, now):
        active_keys={(episode,r['section'],r['id']) for r in records}
        self.previous={k:v for k,v in self.previous.items() if k in active_keys}
        self.inside_since={k:v for k,v in self.inside_since.items() if k in active_keys}
        self.stalled_reported.intersection_update(active_keys)
        for r in records:
            key=(episode,r['section'],r['id'])
            status=r['status']
            # Only native observations, never a distance-triggered control RPC.
            stalled=(phase=='running' and status=='거리 조건 충족 · 출발점 대기')
            if stalled:
                start=self.inside_since.setdefault(key,now)
                if now-start >= 3. and key not in self.stalled_reported:
                    self.stalled_reported.add(key)
                    status+=' · 3초 이상 이동 미관찰'
            else:
                self.inside_since.pop(key,None)
                self.stalled_reported.discard(key)
            signature=r['status']
            newly_stalled=status!=signature
            if self.previous.get(key)==signature and not newly_stalled:
                continue
            self.previous[key]=signature
            distance=r.get('ego_to_start_m')
            distance_text=f'{distance:.2f}m' if distance is not None else '미수신'
            print(f'[동적 활성화 진단] {r["section"]} · {r["model"]} ID={r["id"]} · {status} · '
                  f'ego→생성점 {distance_text} / 설정 {r["activation_distance_m"]:.2f}m · '
                  f'시작점 이동 {r.get("moved_from_start_m",0):.2f}m · '
                  f'끝점 잔여 {r.get("remaining_to_end_m",0):.2f}m',flush=True)


class SpeedTrace:
    def __init__(self):
        self.lock=threading.RLock()
        name = f'validation_speed_{uuid.uuid4().hex[:12]}.jsonl'
        self.path = TOOL_ROOT/'.logs'/name
        try:
            self.path.parent.mkdir(parents=True,exist_ok=True)
            self.stream = self.path.open('x',encoding='utf-8')
        except PermissionError:
            self.path = Path.home()/'.local/state/asmc-scen-runner'/name
            self.path.parent.mkdir(parents=True,exist_ok=True)
            self.stream = self.path.open('x',encoding='utf-8')
        self.last_record = self.last_console = float('-inf')
        print(f'[통합 속도 진단 파일] {self.path}',flush=True)

    def sample(self, records, episode, phase, crossings=None, seed=None, actors=None,contact=None,environment=None,
               *, actor_supplier=None):
        if self.stream is None:
            return
        now = time.monotonic()
        if now-self.last_record < 1.:
            return
        self.last_record = now
        # Resolve synchronously only for a sample that will actually be written.
        # Contact events use record_contact and are never throttled here.
        if actor_supplier is not None:
            actors = actor_supplier()
        self.write(dict(event='sample',episode=episode,seed=seed,phase=phase,vehicles=records,
                        crossings=crossings or [],actors=actors or [],contact=contact,environment=environment))
        low = [r for r in records if r['section']=='high_speed' and r['age_s'] >= 10 and r['target']-r['speed'] >= 15]
        if low and now-self.last_console >= 5.:
            self.last_console = now
            text = ' / '.join(f'ID={r["id"]} {r["model"]} 목표={r["target"]} 실제={r["speed"]:.1f} '
                f'가속={r["throttle"]:.2f} 브레이크={r["brake"]:.2f} 생성후={r["age_s"]:.0f}초' for r in low[:4])
            print('[고주로 속도 진단] '+text,flush=True)

    def write(self,record):
        with self.lock:
            if self.stream is None:return
            try:
                self.stream.write(json.dumps(dict(time_utc=datetime.now(timezone.utc).isoformat(),**record),
                                             ensure_ascii=False)+'\n')
                self.stream.flush()
            except OSError as error:
                print(f'[통합 속도 진단 저장 오류] {error}',flush=True)
                self.close()

    def record_contact(self,evidence,actors):
        self.write(dict(event='contact',episode=evidence['episode'],seed=evidence['seed'],
                        phase=evidence['phase'],contact=evidence,actors=actors))

    def close(self):
        with self.lock:
            if self.stream is not None:
                self.stream.close()
                self.stream = None
