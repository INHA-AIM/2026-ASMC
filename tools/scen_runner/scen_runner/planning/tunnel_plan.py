"""One fixed tunnel scene, using the existing static and native crossing tools."""
import copy
import json
import math
import random
import time
from types import SimpleNamespace

import numpy as np

from scen_runner.config import TOOL_ROOT,load_config,load_catalog,load_yaml,repo_path
from scen_runner.planning.map_scene import reusable_scene as load_scene
from scen_runner.planning.motion_curve import Curve
from scen_runner.planning.vehicle import VehicleSpec,Obstacle
from scen_runner.planning.feasibility import AvoidanceChecker
from scen_runner.planning.static_plan import candidate_models,blocks_straight
from scen_runner.planning.placement import basic_check,obstacle_at
from scen_runner.planning.random_crossing import RandomCrossingPlan,approach_pose
from scen_runner.planning.traffic_geometry import load_links


SCENE_LABELS={'static':'정적 1개','dynamic':'동적 1개','both':'정적 1개 + 동적 1개'}


def static_lateral_bounds(vehicle_width,object_width,jitter):
    center=(vehicle_width+object_width)/2
    if jitter<0 or center-jitter<0:raise ValueError('정적 좌우 후보 범위 설정 오류')
    return center-jitter,center+jitter


def ego_box(curve,station,vehicle):
    x,y,z,h=approach_pose(curve,station)
    center=(vehicle.front_extent-vehicle.rear_overhang)/2
    return Obstacle(x+center*math.cos(h),y+center*math.sin(h),h,vehicle.length,vehicle.width,1.)


def stopped_path_gap(curve,station,vehicle,start,end):
    box=ego_box(curve,station,vehicle)
    n=max(2,int(math.ceil(np.linalg.norm(end-start)/.025))+1)
    pts=np.linspace(start,end,n)
    # Signed distance is 1-Lipschitz; subtract the sampling error bound.
    return float(box.signed_distance(pts[:,0],pts[:,1]).min()-.0125)


def braking_path_gap(curve,activation,vehicle,start,end,speed,c):
    ve=c['ego_speed_kmh']/3.6;vn=speed/3.6;reaction=c['ego_reaction_s'];decel=c['ego_deceleration_mps2']
    length=float(np.linalg.norm(end-start));direction=(end-start)/length
    stop_time=reaction+ve/decel;stop_distance=ve*reaction+ve*ve/(2*decel)
    end_time=max(stop_time,c['npc_start_delay_s']+length/vn)
    n=int(math.ceil(end_time/.02))+1;times=np.linspace(0,end_time,n);gap=float('inf')
    for time in times:
        if time<=reaction:distance=ve*time
        elif time<=stop_time:
            bt=time-reaction;distance=ve*reaction+ve*bt-decel*bt*bt/2
        else:distance=stop_distance
        point=start+direction*np.clip((time-c['npc_start_delay_s'])*vn,0,length)
        box=ego_box(curve,activation+distance,vehicle)
        gap=min(gap,float(box.signed_distance(point[0],point[1])))
    yaw_rate=ve*float(np.max(np.abs(np.gradient(curve.heading,curve.s))))
    corner_radius=math.hypot(vehicle.front_extent,vehicle.width/2)
    return gap-(ve+vn+yaw_rate*corner_radius)*(end_time/(n-1))/2


def stopping_crossing(generator,prefix,station_range,cancelled):
    """Cross ahead of ego; validate both its braking path and stopped footprint."""
    curve=generator.ego;c=generator.settings;vehicle=generator.vehicle
    target_length=Curve(generator.target_points).length
    pattern=generator.rng.choice(c.get('cross_patterns',[0,1]))
    if pattern not in (0,1):raise ValueError('터널은 가로/전방 대각선만 지원합니다.')
    # Fix the selected pattern before search; failed timing candidates must not
    # silently turn a horizontal scene into a different diagonal scene.
    candidate_generator=copy.copy(generator)
    candidate_generator.settings=dict(c,cross_patterns=[pattern])
    for _ in range(3000):
        if cancelled():raise RuntimeError('터널 씬 준비 취소')
        p=RandomCrossingPlan(candidate_generator);center=p.timing['cross_station_m']
        x,y,z,h=curve.pose(center);t=np.array([math.cos(h),math.sin(h)])
        n=np.array([-t[1],t[0]])
        side=1 if np.dot(np.array(p.start[:2])-[x,y],n)>0 else -1
        # Put endpoints relative to the road at their own stations, avoiding
        # the large tangent-extension error on A126's bend.
        anchors=[];stations=[]
        for point,lateral in [(p.start,side*p.timing['start_lateral_m']),(p.end,-side*c['end_lateral_m'])]:
            station=center+float(np.dot(np.array(point[:2])-[x,y],t))
            if not prefix<=station<=prefix+target_length:break
            a,b,z0,heading=curve.pose(station)
            anchors.append(np.array([a-lateral*math.sin(heading),b+lateral*math.cos(heading)]));stations.append(station)
        if len(anchors)!=2:continue
        start,end=anchors;length=np.linalg.norm(end-start);direction=(end-start)/length
        # Exact segment intersection with the sampled ego route.
        a=curve.xyz[:-1,:2];v=np.diff(curve.xyz[:,:2],axis=0);w=start-a
        cross=lambda u,b:u[...,0]*b[...,1]-u[...,1]*b[...,0]
        denominator=cross(v,direction);valid=np.abs(denominator)>1e-9
        fraction=np.zeros(len(v));q=np.zeros(len(v))
        fraction[valid]=cross(w,direction)[valid]/denominator[valid]
        q[valid]=cross(w,v)[valid]/denominator[valid]
        hits=np.flatnonzero(valid&(fraction>=0)&(fraction<=1)&(q>=0)&(q<=length))
        if len(hits)!=1:continue
        i=hits[0];s=curve.s[i]+fraction[i]*(curve.s[i+1]-curve.s[i])
        if not prefix+station_range[0]<=s<=prefix+station_range[1]:continue
        x,y,z,h=curve.pose(s);t=np.array([math.cos(h),math.sin(h)]);n=np.array([-t[1],t[0]])
        cosine=abs(float(np.dot(direction,n)))
        if cosine<.05:continue
        forward=float(np.dot(direction,t))
        if forward < -1e-6:continue
        if pattern==0 and abs(forward)>1e-5:continue
        if math.degrees(math.atan2(abs(forward),cosine))>c['diagonal_angle_deg'][1]+.1:continue
        vn=p.cfg['walk_speed_kmh']/3.6;ve=c['ego_speed_kmh']/3.6
        radius=p.timing['npc_radius_m'];band=vehicle.width/2+radius+c['clearance_m']
        center_time=c['npc_start_delay_s']+q[i]/vn
        entry=center_time-band/(vn*cosine);exit=center_time+band/(vn*cosine)
        stop_time=c['ego_reaction_s']+ve/c['ego_deceleration_mps2']
        stop_distance=ve*c['ego_reaction_s']+ve*ve/(2*c['ego_deceleration_mps2'])
        required=radius+c['clearance_m']+abs(band*np.dot(direction,t)/cosine)+c['minimum_stop_gap_m']
        # Short horizontal crossings cannot occupy the lane until full braking
        # ends at these speeds. Use the required braking distance independently
        # of NPC centre arrival instead of discarding every horizontal crossing.
        lead=max(ve*center_time,stop_distance+required)
        stationary_gap=braking_gap=float('-inf')
        for _ in range(12):
            if cancelled():raise RuntimeError('터널 씬 준비 취소')
            activation_station=s-vehicle.front_extent-lead
            stationary_gap=stopped_path_gap(curve,activation_station+stop_distance,vehicle,start,end)
            braking_gap=braking_path_gap(curve,activation_station,vehicle,start,end,p.cfg['walk_speed_kmh'],c)
            if stationary_gap>=radius+c['clearance_m']+c['minimum_stop_gap_m'] and braking_gap>=radius+c['clearance_m']:break
            lead+=.5
        else:continue
        activation=approach_pose(curve,activation_station);activation_distance=math.dist(activation[:2],start)
        if any(math.dist(approach_pose(curve,s0)[:2],start)<=activation_distance for s0 in
               np.linspace(activation_station-40,activation_station-.05,160)):continue
        p.start=(*map(float,start),curve.pose(stations[0])[2]);p.end=(*map(float,end),curve.pose(stations[1])[2])
        p.heading_deg=math.degrees(math.atan2(direction[1],direction[0]));p.move_distance=float(length)
        p.direction_label=('왼쪽→오른쪽' if side==1 else '오른쪽→왼쪽')+' · '+('가로' if pattern==0 else '대각선 전방')
        p.cfg['activation_distance_m']=activation_distance;p.cross_xy=(x,y);p.activation_xy=activation[:2]
        p.stop_xy=approach_pose(curve,activation_station+stop_distance)[:2]
        p.timing.update(cross_station_m=float(s),entry_s=entry,center_s=center_time,exit_s=exit,
                        front_gap_m=lead-stop_distance,required_gap_m=required,
                        stop_timing_feasible=entry<stop_time<exit,
                        stopped_path_gap_m=stationary_gap,braking_path_gap_m=braking_gap,
                        stop_path_safe=True,cross_pattern=pattern,
                        nominal_conflict=entry<=lead/ve<=exit,
                        activation_station_m=activation_station)
        return p
    raise RuntimeError('정지 차체·제동 궤적과 겹치지 않는 터널 횡단을 찾지 못했습니다.')


def tunnel_static_models(catalog=None):
    policy=load_yaml(TOOL_ROOT/'config/one_scene.yaml')
    catalog=load_catalog() if catalog is None else catalog
    excluded=set(policy['excluded_models'])
    excluded.update(name for name in catalog['models'] if name.startswith('OBJ_Hyundai_Xcient'))
    return candidate_models(catalog,policy['intersection'],excluded)


class TunnelPlan:
    def __init__(self,seed=None,scene_type=None,cross_pattern=None,catalog=None):
        self.catalog=catalog
        self.cfg=load_yaml(TOOL_ROOT/'config/tunnel.yaml')
        if self.cfg['static_max_tries']<=0 or self.cfg.get('static_tick_candidates',12)<=0:
            raise ValueError('정적 탐색 배치 크기는 양수여야 합니다.')
        if cross_pattern is not None:
            choices={'horizontal':0,'forward':1}
            if cross_pattern not in choices:raise ValueError('지원하지 않는 터널 횡단 방향')
            self.cfg['cross_patterns']=[choices[cross_pattern]]
        self.seed=seed if seed is not None else random.SystemRandom().randrange(2**32)
        self.rng=random.Random(self.seed)
        self.scene_type=scene_type or self.rng.choice(self.cfg['scene_types'])
        if self.scene_type not in SCENE_LABELS:raise ValueError('지원하지 않는 터널 씬 유형')
        self.links=load_links(repo_path(self.cfg['link_map']))
        for a,b in zip(self.cfg['ego_links'],self.cfg['ego_links'][1:]):
            if self.links[a]['to_node_idx']!=self.links[b]['from_node_idx']:raise ValueError('터널 진로 링크 연결 오류')
        self.curve=Curve([p for lid in self.cfg['ego_links'] for p in self.links[lid]['points']])
        self.prefix=Curve(self.links[self.cfg['ego_links'][0]]['points']).length
        length=Curve(self.links[self.cfg['target_link']]['points']).length
        low,high=self.cfg['static_station_m']
        high=length if high is None else min(high,length)
        if not 0<=low<=high:raise ValueError('정적 범위가 터널 링크 범위를 벗어납니다.')
        self.cfg['static_station_m']=[float(low),float(high)]
        self.static=None;self.static_size=None;self.dynamic=None;self.scene=None;self.prepared=False
        self._static_context=None;self.search_attempts=0;self.search_rounds=0;self._round_attempts=0
        self.search_status='터널 배치 준비 중';self._last_search_notice=float('-inf')
        self.cfg['runner_label']='터널'

    def prepare(self,cancelled=lambda:False,incremental=False):
        if self.prepared:return True
        settings=load_yaml(TOOL_ROOT/'config/dynamic_crossing.yaml')
        settings.update({k:copy.deepcopy(self.cfg[k]) for k in
                         ('start_lateral_m','end_lateral_m','diagonal_angle_deg','cross_patterns','ego_speed_kmh','speeds_kmh')})
        settings['cross_station_m']=[self.prefix+x for x in self.cfg['cross_station_m']]
        base=load_yaml(TOOL_ROOT/'config/dynamic_validation.yaml')
        base.update(ego_links=self.cfg['ego_links'])
        generator=SimpleNamespace(settings=settings,base=base,rng=self.rng,seed=self.seed,
                                  ego=self.curve,vehicle=VehicleSpec.from_config(load_config()),
                                  target_points=self.links[self.cfg['target_link']]['points'])
        while True:
            if cancelled():return False
            try:
                if self.scene_type in ('dynamic','both') and self.dynamic is None:
                    self.dynamic=stopping_crossing(generator,self.prefix,self.cfg['cross_station_m'],cancelled)
                if cancelled():return False
                limit=self.cfg.get('static_tick_candidates',12) if incremental else self.cfg['static_max_tries']
                found=self.scene_type=='dynamic' or self.prepare_static(cancelled,limit)
            except Exception:
                if cancelled():return False
                raise
            if found:
                self.prepared=True;return True
            if cancelled():return False
            if self._round_attempts>=self.cfg['static_max_tries']:
                self.search_rounds+=1;self._round_attempts=0
                if self.scene_type=='both':self.dynamic=None
            self.search_status=(f'정적 배치 탐색 중 · 후보 {self.search_attempts}개 검사 · '
                                '다른 위치·모델 재선택'+(' · 필요 시 횡단 위치도 재선택' if self.scene_type=='both' else ''))
            if time.monotonic()-self._last_search_notice>=5:
                print('[터널 배치 탐색] '+self.search_status,flush=True)
                self._last_search_notice=time.monotonic()
            if incremental:return False

    def prepare_static(self,cancelled,max_candidates=None):
        if self.static is not None:return True
        if self._static_context is None:
            cfg=copy.deepcopy(load_config())
            if self.cfg.get('allow_lane_marking_crossing',False):
                cfg['forbidden_lines']=[]
            doc=json.loads(repo_path(cfg['map']['drivable_map']).read_text())
            lane=next(l for l in doc['lanes'] if l.get('link_id')==self.cfg['target_link'])
            s0,s1=lane['route_station_start_m'],lane['route_station_end_m']
            cfg['zone'].update(evaluation_s_range=[s0,s1],view_s_range=[s0-45,s1+10],grid_resolution_m=.1)
            cfg['search']['goal_after_checkpoint_m']=0.
            self.scene=load_scene(cfg)
            self._static_context=(s0,AvoidanceChecker(self.scene,cfg),tunnel_static_models(self.catalog))
        s0,checker,models=self._static_context
        low,high=self.cfg['static_station_m']
        count=max_candidates if max_candidates is not None else self.cfg['static_max_tries']
        count=min(count,self.cfg['static_max_tries']-self._round_attempts)
        deadline=time.monotonic()+.25
        for _ in range(count):
            if cancelled():return False
            self.search_attempts+=1;self._round_attempts+=1
            size=self.rng.choice(models)
            station=self.rng.uniform(low,high)
            # Intrude into the straight-driving safety envelope from either side.
            side=self.rng.choice((-1,1))
            d0,d1=static_lateral_bounds(checker.vehicle.width,size.width,self.cfg['static_lateral_jitter_m'])
            d=side*self.rng.uniform(d0,d1)
            ob=obstacle_at(checker,size,s0+station,d,self.rng.uniform(*self.cfg['static_yaw_deg']))
            if basic_check(checker,ob,[],1.) or not blocks_straight(checker,ob):
                if time.monotonic()>=deadline:return False
                continue
            if self.dynamic:
                path=np.linspace(self.dynamic.start[:2],self.dynamic.end[:2],100)
                radius=self.dynamic.timing['npc_radius_m']
                if np.min(ob.signed_distance(path[:,0],path[:,1]))<radius+self.cfg['static_dynamic_gap_m']:
                    if time.monotonic()>=deadline:return False
                    continue
            results=checker.evaluate([ob])
            if not all(r.blocked and r.feasible and not r.start_blocked for r in results):
                if time.monotonic()>=deadline:return False
                continue
            self.static=ob;self.static_size=size
            self.static_avoidance=[s.path for r in results for s in r.feasible_sides()]
            return True
        return False
