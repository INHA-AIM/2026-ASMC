"""Replay chosen settings and section-entry traffic; never draw new random choices."""
import copy
import math
from types import SimpleNamespace as S

import numpy as np

from scen_runner.runtime.episode_actors import Section
from scen_runner.runtime.episode_sections import crossing_definition
from scen_runner.morai.native_vehicle_source import create_ticket,enable_ticket,destroy_ticket,ticket_candidates,SourcePlacementDeferred
from scen_runner.planning.traffic_geometry import load_links,route_between
from scen_runner.config import repo_path


class ReplayEnvironment:
    def __init__(self,link,definition,clock):
        self.link=link;self.definition=definition;self.clock=clock;self.started=None;self.index=0;self.current=None;self.timeline=[]

    def apply(self,environment):
        self.link.set_environment(environment['weather'],environment['hour'])
        self.current=copy.deepcopy(environment)
        self.timeline.append((self.clock(),copy.deepcopy(self.current)))
        print(f'[재현 환경] {environment["weather"]} · {environment["hour"]}:00',flush=True)

    def begin(self,*_):
        self.started=None;self.index=0;self.timeline=[];self.apply(self.definition['environment'])

    def update(self,state,now):
        if self.started is None:self.started=now
        events=self.definition.get('weather_events',[])
        while self.index<len(events) and events[self.index]['time_s']<=now-self.started:
            self.apply(events[self.index]['environment']);self.index+=1

    def snapshot(self):return copy.deepcopy(self.current or {})

    def snapshot_at(self,now):
        for confirmed,snapshot in reversed(self.timeline):
            if confirmed<=now:return dict(copy.deepcopy(snapshot),confirmed_at=confirmed)
        return dict(weather=None,hour=None,error='조회 시점 이전에 확인된 환경 없음')


def crossing_plan(definition,state=None):
    d=copy.deepcopy(definition);start=d['start']
    finished=math.dist(start,d['end'])<=.05 or d['cfg']['walk_speed_kmh']<=0
    started=False
    if state:
        pose=state['pose'];point=[pose['x'],pose['y'],pose['z']]
        moved=math.dist(point[:2],start[:2])
        if moved>.1:
            start=point;finished=finished or math.dist(point[:2],d['end'][:2])<=.4;started=not finished
    return S(cfg=d['cfg'],models=[(d['group'],d['model'])],start=start,end=d['end'],
             heading_deg=d['heading_deg'],move_distance=math.dist(start,d['end']),
             direction_label=d['direction'],timing=d.get('timing',{}),
             replay_started=started,replay_finished=finished)


class ReplaySection(Section):
    def __init__(self,*args,definition,**kwargs):
        super().__init__(*args,**kwargs)
        self.definition=copy.deepcopy(definition);self.baseline=list(self.definition['baseline'])
        self.events=sorted(self.definition.get('events',[]),key=lambda e:(e['time_s'],e['action']=='spawn'))
        self.next_event=0;self.by_key={};self.plans={};self.tickets=[];self.pending=None;self.started=None

    def ready_live(self, states):
        if (self.definition.get('allow_empty_scene') is True and
                self.definition.get('baseline')==[] and not self.events):
            return not self.actors
        return super().ready_live(states)

    def vehicle_cfg(self,definition,state):
        cfg=copy.deepcopy(definition['cfg'])
        if state:
            pose=state['pose'];cfg['spawn_pose']=dict(x=pose['x'],y=pose['y'],z=pose['z'],yaw_deg=pose['yaw'])
            cfg['initial_speed_kmh']=state['speed_kmh']
            current=state['link']
            links=load_links(repo_path(cfg['link_map']))
            if current not in links:raise RuntimeError('재현 NPC 현재 링크를 맵에서 찾을 수 없습니다: '+current)
            cfg['npc_links']=route_between(links,current,cfg['npc_links'][-1])
            points=np.asarray(links[current]['points'])
            cfg['start_point_index']=int(np.argmin(np.linalg.norm(points[:,:2]-[pose['x'],pose['y']],axis=1)))
        return cfg

    def register_replay(self,record,aid,state=None):
        reused=[a for a in self.actors if a.native_id==aid]
        for key,old in list(self.by_key.items()):
            if old in reused:self.by_key.pop(key)
        self.actors=[a for a in self.actors if a not in reused]
        target=record['definition'].get('target_kmh',record['target_kmh'])
        actor=self.register(aid,record['kind'],record['model'],target,source=record['source'],
                            replay=record['definition'])
        self.by_key[record['key']]=actor
        if state and actor.kind==1:
            self.link.pause_actor(aid,1,True);self.link.restore_actor(aid,1,state)
            if self.started is not None:self.link.pause_actor(aid,1,False)
        return actor

    def create(self,item):
        record=item['record'];d=record['definition'];state=item.get('state')
        if d['type']=='static':
            pose=state['pose'] if state else d['pose']
            aid=self.link.spawn_obstacle(d['model'],pose['x'],pose['y'],pose['z'],pose['yaw'],'collision_replay')
            self.register_replay(record,aid)
        elif d['type']=='crossing':
            plan=crossing_plan(d,state)
            aid=self.link.spawn_crossing(plan,with_route=False);actor=self.register_replay(record,aid)
            # Perception warmup can last longer than a short native crossing.
            # Keep the owned actor at the saved start until its route is ready;
            # never let it finish/despawn before control is released.
            self.link.pause_actor(aid,2,True)
            self.plans[actor.request_id]=plan
        elif d['type']=='vehicle':
            cfg=self.vehicle_cfg(d,state);plan=S(cfg=cfg,period=d.get('period_s') or 1.)
            if d['method']=='direct':
                plan.destination=d['destination']
                self.link.spawn(d['model'],d['target_kmh'],plan,
                    on_created=lambda aid:self.register_replay(record,aid,state),paused=self.started is None)
            elif d['method']=='native':
                lane=S(plan=plan,source_id=d['source'])
                ticket=create_ticket(self.link,lane,d['model'],d['target_kmh'])
                ticket['baseline']={s.actor_info.id.value for s in self.link.states()}
                self.tickets.append(ticket);self.pending=dict(ticket=ticket,item=item)
                enable_ticket(self.link,ticket,True)
            else:raise RuntimeError('지원하지 않는 재현 차량 생성 방식')
        else:raise RuntimeError('지원하지 않는 재현 객체 유형')

    def bind_pending(self,states):
        if self.pending is None:return True
        pending=self.pending;ticket=pending['ticket']
        ticket['baseline'].intersection_update(s.actor_info.id.value for s in states)
        matches=ticket_candidates(ticket,states,set(self.owned()))
        if len(matches)>1:raise RuntimeError('재현 신규 차량 ID 대응 모호: 생성 중단')
        if not matches:
            self.status='재현 · MORAI 안전거리/차량 상태 수신 대기';return False
        aid=matches[0].actor_info.id.value
        enable_ticket(self.link,ticket,False);ticket['actor']=aid
        self.link.actor_scopes[aid]=ticket['info'].client_key
        actor=self.register_replay(pending['item']['record'],aid,pending['item'].get('state'))
        self.link.set_speed_limit(aid,actor.target_kmh)
        self.pending=None
        return True

    def poll_prepare(self):
        if self.cancelled():return False
        states=self.link.states()
        if not self.bind_pending(states):return False
        if self.baseline:
            try:self.create(self.baseline[0])
            except SourcePlacementDeferred:
                self.status='재현 · MORAI 생성점 배치 거부 · 같은 설정으로 대기';return False
            self.baseline.pop(0);return False
        self.map_statics()
        self.prepared=self.ready_live(states)
        self.status='저장된 구간 진입 상태 준비 완료' if self.prepared else '재현 객체 상태 수신 대기'
        return self.prepared

    def resume_schedule(self,now):
        self.started=now
        for actor in self.actors:
            if actor.kind==1:self.link.pause_actor(actor.request_id,1,False)
            elif actor.kind==2:
                plan=self.plans[actor.request_id]
                if not plan.replay_finished:self.link.register_crossing(actor.native_id,plan)
                self.link.pause_actor(actor.native_id,2,plan.replay_finished)
                plan.registered=True

    def register_live_crossings(self,states):
        live={s.actor_info.id.value for s in states}
        for actor in self.actors:
            if actor.kind==2 and actor.request_id in self.plans:
                plan=self.plans[actor.request_id]
                if not getattr(plan,'registered',False) and actor.native_id in live:
                    if not plan.replay_finished:self.link.register_crossing(actor.native_id,plan)
                    self.link.pause_actor(actor.native_id,2,plan.replay_finished)
                    plan.registered=True

    def tick(self):
        if self.cancelled():return
        states=self.link.states()
        self.map_statics();self.register_live_crossings(states)
        if not self.bind_pending(states):return
        if self.next_event>=len(self.events):return
        event=self.events[self.next_event]
        if self.clock()-self.started<event['time_s']:return
        if event['action']=='spawn':
            try:self.create(dict(record=event['record']))
            except SourcePlacementDeferred:
                self.status='재현 · MORAI 생성점 배치 거부 · 같은 설정으로 대기';return
            # A future crossing route is registered only after its native state arrives.
        elif event['action']=='remove':
            actor=self.by_key.get(event['key'])
            if actor is not None:
                live={s.actor_info.id.value for s in states}
                if actor.native_id in live and not self.link.destroy(actor.request_id,object_type=actor.kind):
                    raise RuntimeError('재현 객체 정리 실패')
                self.actors=[a for a in self.actors if a is not actor]
        self.next_event+=1

    def crossing_details(self):
        return {a.native_id:crossing_definition(a,self.plans[a.request_id],self.name,
                    self.started is not None) for a in self.actors if a.kind==2 and a.native_id}

    def cleanup(self):
        for ticket in self.tickets:enable_ticket(self.link,ticket,False)
        if self.pending:
            self.bind_pending(self.link.states())
            # No match after disabling the one-shot point: clean the point,
            # without guessing another actor's numeric ID.
            self.pending=None
        self.delete_actors()
        for ticket in list(self.tickets):destroy_ticket(self.link,ticket);self.tickets.remove(ticket)
        self.link.disconnect()


def configure_replay(engine,definition):
    section=definition['section']
    engine.mode='section';engine.section=section;engine.next_seed=definition['seed']
    engine.run_budget=None  # Saved-case replay keeps its original repeat/navigation behavior.
    engine.stagnation=None
    # Saved ego speed is diagnostic data. Driving remains with the user's
    # existing input; normal release starts at zero velocity on every repeat.
    engine.replay_seed=definition['seed']
    engine.signal_replay=copy.deepcopy(definition.get('traffic_lights',dict(policy='unchanged',colors={})))
    engine.route.cfg=copy.deepcopy(definition['route_config']);engine.route._sections.clear()
    engine.route._section_completion.clear()
    engine.route.section(section)['pose']=copy.deepcopy(definition['start_pose'])
    engine.environment=ReplayEnvironment(engine.link,definition,engine.clock)
    engine.factory=lambda name,seed,scope,base,cancelled,progress:ReplaySection(
        name,definition['seed'],scope,base,cancelled,progress,clock=engine.clock,definition=definition)
