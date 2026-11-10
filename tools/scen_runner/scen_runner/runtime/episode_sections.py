"""Fresh scene sections. Share placement math, never invoke old runner sessions."""
import math
import random

from scen_runner.config import load_catalog, load_yaml
from scen_runner.runtime.episode_actors import Section
from scen_runner.runtime.episode_traffic import HighSpeedSection, RoundaboutSection
from scen_runner.planning.placement import ModelSize, spawn_pose
from scen_runner.planning.random_crossing import CrossingGenerator
from scen_runner.planning.static_plan import (POLICY_PATH, runtime_config, make_checkers, generate_scene,
                          validate_scene, SceneCancelled)
from scen_runner.planning.tunnel_plan import TunnelPlan
from scen_runner.records.replay_spec import crossing_spec, static_spec


def crossing_definition(actor,plan,section,route_registered):
    return dict(section=section,model=actor.model,id=actor.native_id,start=list(plan.start),end=list(plan.end),
                target=actor.target_kmh,activation_distance_m=plan.cfg['activation_distance_m'],
                preparation_paused=False,direction=plan.direction_label,route_registered=route_registered)


def live_static_catalog(link):
    """Keep measured geometry, but select only models this simulator exposes."""
    catalog = load_catalog()
    available = set(link.available_obstacle_models())
    recorded = catalog.get('available_models', [])
    supported = [name for name in recorded
                 if catalog['models'].get(name, {}).get('morai_model', name) in available]
    missing = [name for name in recorded if name not in supported]
    if missing:
        print('[모델 호환] 현재 MORAI 미지원 정적 모델 제외: '+', '.join(missing), flush=True)
    if not supported:
        raise RuntimeError('현재 MORAI에서 사용할 수 있는 실측 정적 모델이 없습니다.')
    catalog['available_models'] = supported
    return catalog


class StaticSection(Section):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.policy = load_yaml(POLICY_PATH)
        self.cfg = runtime_config(self.base, self.policy)
        self.catalog = live_static_catalog(self.link)
        if not self.catalog.get('measurement_complete'):
            raise RuntimeError('정적 객체 실측 카탈로그가 필요합니다. 정적구간 모델 측정을 먼저 실행하세요.')
        self.checkers = None
        self.objects = None
        self.attempt = 0
        self.target_count = random.Random(self.seed).choice(self.policy['intersection'].get('count_choices', [1,2]))

    def poll_prepare(self):
        if self.cancelled():
            raise SceneCancelled('정적 배치 취소')
        if self.checkers is None:
            self.checkers = make_checkers(self.cfg)
        if self.objects is None:
            fixed = [self.link._obstacle_from_state(s) for s in self.link.states() if s.actor_info.object_type == 3]
            objects, _, _ = generate_scene(self.cfg, self.catalog, self.policy,
                (self.seed+self.attempt*2654435761) % 2**32, fixed, self.progress, self.cancelled,
                intersection_count=self.target_count, checkers=self.checkers)
            self.objects = objects
            for ob in objects:
                if self.cancelled():
                    raise SceneCancelled('정적 생성 취소')
                size = ModelSize.from_catalog(ob.morai_model, self.catalog['models'][ob.morai_model])
                x, y = spawn_pose(ob, size)
                aid = self.link.spawn_obstacle(ob.morai_model, x, y, ob.z, math.degrees(ob.yaw), 'episode_static')
                ob.actor_id = aid
                self.register(aid, 3, ob.model, replay=static_spec(ob,x,y))
        self.map_statics()
        scoped = {s.actor_info.id.value:s for s in self.link.scoped_statics()}
        if any(a.native_id is None or a.request_id not in scoped for a in self.actors):
            return False
        actual = []
        for ob in self.objects:
            measured = self.link._obstacle_from_state(scoped[ob.actor_id])
            if min(measured.length, measured.width, measured.height) <= 0:
                return False
            measured.morai_model, measured.model = ob.morai_model, ob.model
            measured.extra['section'] = ob.extra['section']
            actual.append(measured)
        fixed = [self.link._obstacle_from_state(s) for s in self.link.states()
                 if s.actor_info.object_type == 3 and s.actor_info.id.value not in self.owned()]
        try:
            validate_scene(self.checkers, actual, fixed, self.policy)
        except ValueError as error:
            self.delete_actors()
            self.objects = None
            self.attempt += 1
            self.progress(f'정적 실제 외형 검증 재시도: {error}')
            return False
        self.prepared = True
        self.status = '정적 생성·실제 외형 검증 완료'
        return True


class DynamicSection(Section):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.plan = CrossingGenerator(self.seed).next_plan()
        self.route_registered=False

    def poll_prepare(self):
        if self.cancelled():
            return False
        if not self.actors:
            aid = self.link.spawn_crossing(self.plan,with_route=False)
            self.register(aid, 2, self.plan.models[0][1], self.plan.cfg['walk_speed_kmh'], replay=crossing_spec(self.plan))
            self.status='생성 완료 · 동적 상태 수신 후 경로 등록 대기'
            return False
        if not self.route_registered:
            if not self.ready_live(self.link.states()):return False
            aid=self.actors[0].native_id
            self.link.register_crossing(aid,self.plan)
            self.route_registered=True
            print(f'[동적 경로 등록] {self.name} · {self.plan.models[0][1]} · ID={aid} · '
                  f'Activation {self.plan.cfg["activation_distance_m"]:.2f}m · {self.plan.cfg["walk_speed_kmh"]}km/h',flush=True)
        self.prepared = True
        self.status = '횡단 경로 등록 완료'
        return True

    def crossing_details(self):
        return {a.native_id:crossing_definition(a,self.plan,self.name,self.route_registered) for a in self.actors if a.native_id}


class TunnelSection(Section):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.plan = TunnelPlan(self.seed, catalog=live_static_catalog(self.link),
                               **self.base.get('tunnel_profile',{}))
        self.spawned = False
        self.route_registered=False

    def poll_prepare(self):
        if self.cancelled():
            return False
        if not self.plan.prepare(self.cancelled, incremental=True):
            self.status = self.plan.search_status
            self.progress(self.status)
            return False
        if not self.spawned:
            if self.plan.static:
                ob = self.plan.static
                x, y = spawn_pose(ob, self.plan.static_size)
                aid = self.link.spawn_obstacle(ob.morai_model,x,y,ob.z,math.degrees(ob.yaw),'episode_tunnel')
                self.register(aid, 3, ob.model, replay=static_spec(ob,x,y))
            if self.cancelled():
                return False
            if self.plan.dynamic:
                aid = self.link.spawn_crossing(self.plan.dynamic,with_route=False)
                self.register(aid, 2, self.plan.dynamic.models[0][1], self.plan.dynamic.cfg['walk_speed_kmh'],
                              replay=crossing_spec(self.plan.dynamic))
            self.spawned = True
            if self.plan.dynamic:return False
        self.map_statics()
        if self.plan.dynamic and not self.route_registered:
            if not self.ready_live(self.link.states()):return False
            actor=next(a for a in self.actors if a.kind==2)
            self.link.register_crossing(actor.native_id,self.plan.dynamic)
            self.route_registered=True
            print(f'[동적 경로 등록] tunnel · {actor.model} · ID={actor.native_id}',flush=True)
        self.prepared = all(a.native_id for a in self.actors)
        self.status = f'터널 {self.plan.scene_type} 배치 완료'
        return self.prepared

    def crossing_details(self):
        return {a.native_id:crossing_definition(a,self.plan.dynamic,self.name,self.route_registered)
                for a in self.actors if a.kind==2 and a.native_id}


SECTION_TYPES = dict(static=StaticSection, dynamic=DynamicSection, tunnel=TunnelSection,
                     roundabout=RoundaboutSection, high_speed=HighSpeedSection)


def build_section(name, seed, scope, base, cancelled, progress):
    return SECTION_TYPES[name](name, seed, scope, base, cancelled, progress)
