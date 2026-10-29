"""Integrated scene/section lifecycle. No ego driving controller or DLL changes."""
import json
import math
import random
import threading
import time
import uuid
from pathlib import Path

import numpy as np

from scen_runner.config import TOOL_ROOT,load_config,load_yaml,repo_path
from scen_runner.planning.motion_curve import Curve
from scen_runner.morai.episode_link import SceneLink
from scen_runner.runtime.episode_sections import build_section
from scen_runner.runtime.contact_evidence import ContactTransitions
from scen_runner.runtime.checkpoint_environment import CheckpointEnvironment
from scen_runner.runtime.scene_completion import SceneCompletion
from scen_runner.runtime.section_completion import SectionCompletion
from scen_runner.runtime.section_signals import SectionSignals
from scen_runner.morai.rpc_observation import observe_link
from scen_runner.runtime.validation_lease import ValidationLease


LABELS={'static':'정적구간','dynamic':'동적구간','roundabout':'회전교차로','high_speed':'고주로','tunnel':'터널'}
PREPARE_ORDER=['static','dynamic','tunnel','roundabout','high_speed']


class EpisodeRoute:
    def __init__(self,sample_json=None):
        self.cfg=load_yaml(TOOL_ROOT/'config/validation.yaml');base=load_config()
        self.planning_recovery_tail=float(self.cfg.get('planning_recovery_tail_m',0.))
        if not math.isfinite(self.planning_recovery_tail) or not 0<=self.planning_recovery_tail<=100:
            raise ValueError('판제 복귀 검증 거리는 0~100m이어야 합니다.')
        self.curve=Curve(np.loadtxt(repo_path(base['map']['global_path']))[:,:3])
        doc=json.loads(repo_path(base['map']['drivable_map']).read_text())
        self.lanes={l['link_id']:l for l in doc['lanes'] if l.get('route_station_start_m') is not None}
        self.scene_start=dict(self.cfg['scene_start'])
        self._sections={}
        self._section_completion={}
        if sample_json:
            ego=json.loads(Path(sample_json).read_text())['egoVehicle']['initPosition']
            self.scene_start={k:float(ego['pos'][k]) for k in ('x','y','z')}
            self.scene_start.update({k:float(v) for k,v in ego['rot'].items()})

    def section(self,name):
        if name in self._sections:return self._sections[name]
        c=self.cfg['sections'][name]
        anchor=c.get('start_anchor','start')
        if anchor not in ('start','end'):raise ValueError('start_anchor는 start/end여야 합니다.')
        start=max(0.,self.lanes[c['start_link']][f'route_station_{anchor}_m']-c['lead_in_m'])
        end=self.lanes[c['end_link']]['route_station_end_m']
        x,y,z,h=self.curve.pose(start)
        pose=dict(x=x,y=y,z=z+self.cfg['section_height_offset_m'],roll=0.,pitch=0.,yaw=math.degrees(h))
        stations=np.r_[np.arange(start,end,.25),end]
        local=Curve([self.curve.pose(s)[:3] for s in stations])
        end_pose=self.curve.pose(end)
        result=dict(pose=pose,start=start,end=end,curve=local,end_xy=end_pose[:2],end_heading=end_pose[3])
        self._sections[name]=result;return result

    def start(self,mode,section):
        if mode not in ('scene','section'):raise ValueError('검증 모드는 scene/section입니다.')
        if section not in LABELS:raise ValueError('지원하지 않는 구간')
        return dict(self.scene_start) if mode=='scene' else self.section(section)['pose']

    def prepare_section_end(self,section,tail_m=0.):
        definition=self.section(section)
        if tail_m:
            start=definition['start'];end=min(self.curve.length,definition['end']+tail_m)
            stations=np.r_[np.arange(start,end,.25),end]
            end_pose=self.curve.pose(end)
            definition=dict(definition,curve=Curve([self.curve.pose(s)[:3] for s in stations]),
                            end_xy=end_pose[:2],end_heading=end_pose[3])
        tracker=SectionCompletion(definition,
            half_width_m=float(self.cfg.get('section_end_half_width_m',10.)),
            approach_m=float(self.cfg.get('section_end_approach_m',25.)),
            arrival_radius_m=float(self.cfg['end_radius_m']))
        self._section_completion[section]=tracker
        return tracker

    def finished(self,section,state,now=None):
        tracker=self._section_completion.get(section)
        if tracker is not None and now is not None:return tracker.update(state,now)
        # Stateless callers and saved-case replay retain their close-arrival check.
        definition=self.section(section);p=state.transform.location
        station,error=definition['curve'].project(p.x,p.y)
        return (error<=self.cfg['end_radius_m'] and
                station>=definition['curve'].length-self.cfg['end_radius_m'] and
                math.dist((p.x,p.y),definition['end_xy'])<=self.cfg['end_radius_m'])


class EpisodeLink(SceneLink):
    def states(self):
        from proto.morai.actor.actor_get_pb2 import GetAllActorsFilter
        return list(self._require()._actor_stub.GetAllActorsState(
            GetAllActorsFilter(vehicle=True,pedestrian=True,obstacle=True),timeout=3).states)

    def ego_info(self):
        from proto.morai.common.object_info_pb2 import ObjectInfo
        info=ObjectInfo(object_type=1,client_key=self.client_key);info.id.value=self.ego_id
        return info

    def ego(self):
        state=self._require()._actor_stub.GetActorState(self.ego_info(),timeout=3)
        if not state.actor_info.id.value:raise RuntimeError('ego 상태 미수신')
        return state

    def pause_ego(self,paused):
        from proto.morai.actor.actor_set_pb2 import EnableActor
        r=EnableActor(enable=paused);r.actor_info.CopyFrom(self.ego_info())
        self.check(self._require()._actor_stub.SetPause(r,timeout=3),'ego 정지/재개 실패')

    def zero_ego(self):
        from proto.morai.actor.actor_set_pb2 import SetVelocityParam
        r=SetVelocityParam(velocity=0);r.actor_info.CopyFrom(self.ego_info())
        self.check(self._require()._actor_stub.SetVelocity(r,timeout=3),'ego 속도 초기화 실패')

    def gear_ego(self,drive):
        from proto.morai.actor.actor_set_pb2 import VehicleGear
        r=VehicleGear(gear=4 if drive else 1);r.actor_info.CopyFrom(self.ego_info())
        self.check(self._require()._actor_stub.SetVehicleGear(r,timeout=3),'ego 기어 설정 실패')

    def hold(self):
        self.pause_ego(True);self.zero_ego();self.gear_ego(False)

    def reset(self,pose):
        from proto.morai.actor.actor_set_pb2 import SetTransformParam
        self.hold();r=SetTransformParam();r.actor_info.CopyFrom(self.ego_info())
        r.transform.location.x=pose['x'];r.transform.location.y=pose['y'];r.transform.location.z=pose['z']
        r.transform.rotation.x=pose['roll'];r.transform.rotation.y=pose['pitch'];r.transform.rotation.z=pose['yaw']
        self.check(self._require()._actor_stub.SetTransform(r,timeout=3),'ego 시작 위치 초기화 실패')
        self.hold()  # Transform reset must not leave ego unpaused.
        # The controller transform and physical chassis can have a persistent
        # local rotation offset. Correct from the reported chassis orientation,
        # rather than repeatedly accumulating the offset across episode resets.
        for attempt in range(4):
            actual=self.ego().transform.rotation.z
            error=(actual-pose['yaw']+180)%360-180
            if abs(error)<=.1:break
            if attempt==3:raise RuntimeError('ego 실제 차체 방향을 시작 방향으로 맞추지 못했습니다.')
            # Roll/pitch are aligned to the road by MORAI; only driving
            # direction is corrected from the physical chassis readback.
            r.transform.rotation.z-=error
            self.check(self._require()._actor_stub.SetTransform(r,timeout=3),'ego 차체 방향 보정 실패')
            self.hold()

    @staticmethod
    def body_heading_consistent(state):
        """IONIQ5 reset check: rear-axle -> box center must point forward.

        A 2026-10-30 native reset returned a plausible yaw while the body
        center and subsequent travel were approximately 180 degrees opposite.
        Do not repair IMU yaw from this diagnostic or alter planner inputs.
        This check is specific to this runner's rear-axle ego reference.
        """
        try:
            p=state.transform.location;b=state.bounding_box.location
            dx,dy=b.x-p.x,b.y-p.y
            yaw=math.radians(state.transform.rotation.z)
            length=math.hypot(dx,dy)
            return (all(math.isfinite(v) for v in (dx,dy,yaw)) and length>.25 and
                    (dx*math.cos(yaw)+dy*math.sin(yaw))/length>.7)
        except AttributeError:
            return False

    def held_at(self,pose):
        s=self.ego();p=s.transform.location;v=s.velocity
        yaw_error=abs((s.transform.rotation.z-pose['yaw']+180)%360-180)
        return (self.body_heading_consistent(s) and math.dist((p.x,p.y),(pose['x'],pose['y']))<=.25 and abs(p.z-pose['z'])<=.75 and
                yaw_error<=1. and math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)<=.1)

    def release(self):
        if not self.body_heading_consistent(self.ego()):
            raise RuntimeError('ego reset 차체/보고 방향 불일치: 출발 차단, 시뮬 자세 재설정 필요')
        self.zero_ego();self.gear_ego(True);self.pause_ego(False)




class EpisodeController:
    def __init__(self,link,route,mode='scene',section='static',seed=None,factory=build_section,clock=time.monotonic,lease=None):
        self.link=link;self.route=route;self.mode=mode;self.section=section;self.clock=clock;self.factory=factory
        self.rng=random.Random(seed);self.next_seed=seed;self.seed=None;self.episode=0
        self.phase='idle';self.adapters=[];self.pending=[];self.owned={};self.pose=None
        self.status='준비 중';self.cancelled=threading.Event();self.contact_lock=threading.Lock()
        self.collision_requested=False;self.contact_epoch=0;self.run_started=0.;self.error=None
        self.continue_on_static_contact=False
        self.base=load_config();self.names=[];self.last_states=[]
        self.id_owners={}
        self.on_progress=None
        self.last_contact_evidence=None
        self.contact_transitions=ContactTransitions()
        self.environment=CheckpointEnvironment(link)
        self.environment.clock=self.clock
        self.signals=SectionSignals(link)
        self.signal_replay=None
        self.completion=SceneCompletion(self.environment.checkpoints,self.environment.start_end)
        self.on_contact_observation=None
        self.on_lifecycle_observation=None
        self.ownership_signature=None
        self.contact_signature=None
        self.current_operation={}
        self.on_rpc_observation=None
        self.on_episode_ready=None;self.on_state_observation=None;self.on_collision=None
        self.replay_seed=None
        self.state_revision=0
        self.run_budget=None  # The normal GUI supplies YAML limits; diagnostic fixtures/replay do not.
        self.stagnation=None
        self.on_stagnation=None
        self.lease=lease if lease is not None else ValidationLease(self.base['morai']['host'],self.base['morai']['port'])
        self.planning=None  # Optional external supervisor; keyboard validation stays independent.
        self.section_finish_pending=False
        self.ego_start_delay_s=0.;self.ego_release_at=None

    def claim_simulator(self):
        self.lease.acquire()

    def read_states(self):
        states=self.link.states()
        self.last_states=states
        self.state_revision+=1
        if self.on_state_observation:self.on_state_observation(self,states)
        return states

    def record_rpc(self,section,record):
        if self.on_rpc_observation:
            self.on_rpc_observation(dict(event='rpc',episode=self.episode,seed=self.seed,phase=self.phase,
                section=section,operation=dict(self.current_operation),**record))

    def observe_ego_link(self):
        if self.on_rpc_observation:
            observe_link(self.link,lambda record:self.record_rpc('ego',record))

    def record_lifecycle(self,event,**details):
        if self.on_lifecycle_observation:
            self.on_lifecycle_observation(dict(event=event,episode=self.episode,seed=self.seed,
                mode=self.mode,section=self.section,phase=self.phase,epoch=self.contact_epoch,**details))

    @staticmethod
    def actor_registrations(adapter):
        return [dict(request_id=a.request_id,native_id=a.native_id,kind=a.kind,model=a.model,
                     scope=adapter.link.actor_scopes.get(a.request_id,adapter.scope))
                for a in getattr(adapter,'actors',())]

    def cleanup(self):
        signal_error=None
        try:self.signals.restore()
        except Exception as error:signal_error=error
        for adapter in list(reversed(self.adapters)):
            self.current_operation=dict(section=adapter.name,action='cleanup')
            self.record_lifecycle('cleanup_requested',actor_section=adapter.name,
                                  actors=self.actor_registrations(adapter))
            try:adapter.cleanup()
            except Exception as error:
                self.record_lifecycle('cleanup_failed',actor_section=adapter.name,error=str(error))
                raise
            self.record_lifecycle('cleanup_confirmed',actor_section=adapter.name)
            self.adapters.remove(adapter)
        self.owned={}
        self.id_owners={}
        if signal_error:raise signal_error

    def begin(self,epoch,selection=None,reset_budget=False):
        if selection and (selection[0] not in ('scene','section') or selection[1] not in LABELS):
            raise ValueError('지원하지 않는 검증 모드/구간입니다.')
        if self.run_budget is not None and not reset_budget and not self.run_budget.can_start:
            self.finish_runs('횟수 소진');return
        self.claim_simulator()
        with self.contact_lock:
            self.phase='resetting';self.contact_epoch=epoch;self.collision_requested=False
            self.last_contact_evidence=None
            self.contact_transitions=ContactTransitions()
            self.contact_signature=None
        self.current_operation=dict(section=None,action='reset')
        self.link.hold()
        if self.planning:
            self.current_operation=dict(section=None,action='planning_stop')
            self.planning.stop()
        self.cleanup()
        # GUI selection changes are committed only after the previous planner
        # and objects have been stopped/cleaned. All restarts share this path.
        if selection:self.mode,self.section=selection
        self.ego_start_delay_s=(float(self.route.cfg.get('sections',{}).get(self.section,{}).get('ego_start_delay_s',0.))
                               if self.mode=='section' and self.replay_seed is None else 0.)
        if not math.isfinite(self.ego_start_delay_s) or self.ego_start_delay_s<0:
            raise ValueError('ego 출발 대기 시간은 0 이상의 유한한 초 단위 값이어야 합니다.')
        self.ego_release_at=None
        if reset_budget and self.run_budget:self.run_budget.reset(self.mode,self.section)
        self.current_operation=dict(section=None,action='reset')
        self.pose=self.route.start(self.mode,self.section);self.link.reset(self.pose)
        self.episode+=1;self.seed=self.replay_seed if self.replay_seed is not None else (
            self.next_seed if self.next_seed is not None else self.rng.randrange(2**32))
        self.next_seed=None;self.names=list(PREPARE_ORDER) if self.mode=='scene' else [self.section]
        self.environment.begin(self.seed,self.mode)
        self.current_operation=dict(section=self.section,action='section_signals')
        self.signals.begin(self.mode,self.section,self.route.cfg.get('section_traffic_lights',{}),
                           replay=self.signal_replay)
        self.completion.reset()
        if self.stagnation:self.stagnation.reset()
        self.pending=list(self.names);self.cancelled.clear();self.error=None
        self.phase='preparing';self.status='ego 정지 · 객체 배치 준비 중';self.section_finish_pending=False
        self.record_lifecycle('episode_begin',pose=dict(self.pose),traffic_lights=self.signals.snapshot())
        print(f'[검증 {self.episode}] {self.mode} · {LABELS[self.section] if self.mode=="section" else "전체 씬"} · seed={self.seed}',flush=True)

    def progress(self,text):
        self.status='ego 정지 · '+text if self.phase=='preparing' else text
        if self.on_progress:self.on_progress(self.status)

    def actor_details(self):
        return {aid: info for adapter in self.adapters for aid, info in adapter.actor_details().items()
                if self.id_owners.get(aid) is adapter}

    def crossing_details(self):
        return {aid: info for adapter in self.adapters for aid, info in adapter.crossing_details().items()
                if self.id_owners.get(aid) is adapter}

    def refresh_owned(self):
        for adapter in self.adapters:
            ids=set(adapter.owned())
            for aid in ids-adapter.known_ids:
                previous=self.id_owners.get(aid)
                if previous is not None and previous is not adapter:previous.forget_reused_id(aid)
                self.id_owners[aid]=adapter
            adapter.known_ids=ids
        merged={}
        for adapter in self.adapters:
            for aid,value in adapter.owned().items():
                if self.id_owners.get(aid) is adapter:merged[aid]=value
        self.owned=merged
        if self.on_lifecycle_observation:
            registrations=[dict(actor_section=a.name,actors=self.actor_registrations(a)) for a in self.adapters]
            signature=tuple((a['actor_section'],tuple((r['request_id'],r['native_id']) for r in a['actors']))
                            for a in registrations)
            if signature!=self.ownership_signature:
                self.ownership_signature=signature
                self.record_lifecycle('ownership_changed',registrations=registrations,owned_ids=sorted(self.owned))

    def contact(self,sample):
        if sample.get('error') or sample.get('epoch')!=self.contact_epoch:return False
        with self.contact_lock:
            if sample.get('epoch')!=self.contact_epoch:return False
            started=sample.get('query_started_at')
            eligible=(self.phase=='running' and not self.cancelled.is_set() and
                      started is not None and started>=self.run_started)
            evidence=self.contact_transitions.observe(sample,self.owned,self.link.ego_id,eligible)
            self.last_contact_evidence=dict(epoch=sample['epoch'],query_started_at=started,
                                           received_at=sample.get('received_at'),phase=self.phase,
                                           eligible=eligible,episode=self.episode,seed=self.seed,
                                           raw_contacts=list(sample.get('contacts',())),records=evidence)
            signature=(self.phase,eligible,tuple(sample.get('contacts',())),
                       tuple((r['id'],r['verdict'],r['reason']) for r in evidence))
            if signature!=self.contact_signature:
                self.contact_signature=signature
                # Save the transition before requesting a reset: the next
                # episode may begin before the one-second GUI snapshot runs.
                if self.on_contact_observation:self.on_contact_observation(self.last_contact_evidence,sample)
            if not eligible or self.collision_requested:return False
            hits={r['id'] for r in evidence if r['verdict']=='accepted'}
            if not hits:return False
            if self.on_collision:
                try:self.on_collision(self,self.last_contact_evidence,sample,hits)
                except Exception as exc:print('[충돌 저장 실패] '+str(exc),flush=True)
            live_fixed={s.actor_info.id.value for s in sample.get('actor_states',())
                        if getattr(s.actor_info,'object_type',None)==3}
            fixed={aid for aid in hits if aid in live_fixed and aid in self.id_owners and
                   self.id_owners[aid].name=='static' and
                   any(a.native_id==aid and a.kind==3 for a in getattr(self.id_owners[aid],'actors',()))}
            if self.continue_on_static_contact and fixed==hits:
                self.record_lifecycle('static_contact_continued',contact_ids=sorted(hits))
                print('[정적 접촉 기록 · 주행 계속] '+','.join(sorted(hits)),flush=True)
                return False
            self.collision_requested=True
        action='다음 반복 요청' if self.run_budget is None or self.run_budget.can_start else '설정 주행 종료 요청'
        print('[검증 충돌] ego ↔ 생성 객체 '+','.join(sorted(hits))+' · '+action,flush=True)
        return True

    def finish_runs(self,reason,already_held=False):
        self.claim_simulator()
        self.phase='finishing_runs';self.cancelled.set()
        if not already_held:self.link.hold()
        if self.planning:self.planning.stop()
        self.cleanup();self.pending=[]
        self.current_operation=dict(section=None,action='final_reset')
        self.pose=self.route.start(self.mode,self.section)
        self.link.reset(self.pose)
        # Confirm the returned chassis pose/zero speed before closing. Allow a
        # few cache updates; a failed reset must keep the error GUI open.
        for attempt in range(6):
            if self.link.held_at(self.pose):break
            if attempt==5:raise RuntimeError('최종 시작점 복귀 또는 ego 정지 상태를 확인하지 못했습니다.')
            time.sleep(.1)
        self.phase='runs_done'
        self.record_lifecycle('runs_finished',reason=reason,ego_held=True)
        budget=self.run_budget
        label='전체 씬' if self.mode=='scene' else LABELS[self.section]
        self.status=f'총 주행 {budget.started}/{budget.limit}회 완료 · 시작점 복귀 · ego 정지 · 자동 종료'
        print(f'[검증 횟수 완료] {label} · {budget.started}/{budget.limit}회 · {reason} · '
              '객체 정리 완료 · 시작점 복귀 확인 · ego 정지 유지 · 자동 종료',flush=True)

    def end_run(self,reason):
        self.link.hold()
        with self.contact_lock:
            if self.cancelled.is_set():return None
            if self.collision_requested:reason='collision'
            self.phase='transition'
        self.record_lifecycle('episode_end',reason=reason,
                              elapsed_s=max(0.,self.clock()-self.run_started),owned_ids=sorted(self.owned))
        if self.run_budget is not None and not self.run_budget.can_start:
            label={'collision':'충돌','scene_end':'완주','section_end':'구간 종료','stagnation':'정체'}[reason]
            self.finish_runs(label,already_held=True);return None
        return reason

    def pause_prepared_traffic(self):
        errors=[]
        for adapter in self.adapters:
            pause=getattr(adapter,'pause_prepared',None)
            if pause is None:continue
            try:pause()
            except Exception as error:errors.append(f'{adapter.name}: {error}')
        if errors:raise RuntimeError('NPC 준비 정지 복구 실패: '+'; '.join(errors))

    def release_prepared_ego(self,states):
        recovery=bool(self.planning) and self.section=='tunnel'
        section_end=(self.route.prepare_section_end(self.section,
            self.route.planning_recovery_tail if recovery else 0.)
            if self.mode=='section' and (self.replay_seed is None or recovery) else None)
        self.link.release()
        if self.phase=='ego_wait':
            now=self.clock()
            for adapter in self.adapters:
                set_wait=getattr(adapter,'set_ego_wait',None)
                if set_wait is not None:set_wait(False,now)
        with self.contact_lock:
            if self.run_budget is not None:self.run_budget.start_run()
            self.run_started=self.clock();self.phase='running';self.ego_release_at=None
            self.completion.reset((self.pose['x'],self.pose['y']),self.run_started)
            if section_end is not None:section_end.reset((self.pose['x'],self.pose['y']),self.run_started)
            if self.stagnation:self.stagnation.reset((self.pose['x'],self.pose['y']),self.run_started)
            if self.on_episode_ready:self.on_episode_ready(self,states)
        self.status='주행 허용 · '+('판제·MPC 준비 확인' if self.planning else '기존 주행 코드 입력 사용')
        count='' if self.run_budget is None else f' · 주행 {self.run_budget.started}/{self.run_budget.limit}'
        print('[검증 준비 완료] ego 주행 허용'+count,flush=True)
        configuration=self.planning.configuration() if self.planning else {}
        self.record_lifecycle('driving_ready',owned_ids=sorted(self.owned),
                              planning_configuration=configuration if isinstance(configuration,dict) else {})

    def wait_for_ego_departure(self):
        """Keep the GUI responsive and NPC traffic live during the ego hold."""
        try:
            self.current_operation=dict(section=self.section,action='ego_start_delay')
            self.signals.check()
            if self.planning:self.planning.check()
            if not self.link.held_at(self.pose):
                raise RuntimeError('출발 대기 중 ego 위치 또는 속도가 정지 조건을 벗어났습니다.')
            for adapter in self.adapters:
                if self.cancelled.is_set():break
                adapter.tick();self.refresh_owned()
            states=self.read_states()
            if self.cancelled.is_set():
                self.pause_prepared_traffic();return None
            remaining=max(0.,self.ego_release_at-self.clock())
            self.status=f'NPC 선출발 · ego 정지 · 출발까지 {remaining:.1f}초'
            if remaining>0:return None
            self.signals.verify()
            if self.planning:self.planning.check()
            if self.cancelled.is_set():
                self.pause_prepared_traffic();return None
            self.release_prepared_ego(states)
        except Exception:
            try:self.pause_prepared_traffic()
            finally:self.link.hold()
            raise
        return None

    def step(self):
        if self.cancelled.is_set():
            if self.phase=='ego_wait':self.pause_prepared_traffic()
            return None
        if self.phase=='ego_wait':return self.wait_for_ego_departure()
        if self.phase in ('preparing','running'):
            try:self.signals.check()
            except Exception:
                self.current_operation=dict(section=self.section,action='section_signals')
                self.link.hold()
                raise
        if self.phase=='preparing':
            if self.cancelled.is_set():return None
            if self.pending:
                name=self.pending[0]
                self.current_operation=dict(section=name,action='prepare')
                adapter=next((a for a in self.adapters if a.name==name),None)
                if adapter is None:
                    adapter=self.factory(name,self.replay_seed if self.replay_seed is not None else self.rng.randrange(2**32),f'validation_{uuid.uuid4().hex}',
                                         self.base,self.cancelled.is_set,self.progress)
                    self.adapters.append(adapter)
                    if self.on_rpc_observation:
                        observe_link(adapter.link,lambda record,section=name:self.record_rpc(section,record))
                self.status='ego 정지 · '+LABELS[name]+' 준비 중'
                if adapter.poll_prepare():self.pending.pop(0)
                self.refresh_owned();return None
            self.current_operation=dict(section=None,action='ready_check')
            states=self.read_states()
            # Wait for gRPC cache to contain every prepared actor before releasing ego.
            if not all(a.ready_live(states) for a in self.adapters):
                self.status='ego 정지 · 생성 객체 상태 수신 대기';return None
            if not self.link.held_at(self.pose):
                raise RuntimeError('준비 중 ego 위치 또는 속도가 정지 조건을 벗어났습니다.')
            if self.cancelled.is_set():return None
            if self.run_budget is not None and not self.run_budget.can_start:
                self.finish_runs('횟수 소진');return None
            if self.planning and not self.planning.ready(self.pose,self.episode):
                self.status='ego 정지 · '+self.planning.status
                return None
            self.signals.verify()
            now=self.clock()
            try:
                for adapter in self.adapters:
                    if self.cancelled.is_set():break
                    adapter.resume_schedule(now)
                if self.cancelled.is_set():
                    self.pause_prepared_traffic();return None
                if self.ego_start_delay_s>0:
                    wait_started=self.clock()
                    for adapter in self.adapters:
                        set_wait=getattr(adapter,'set_ego_wait',None)
                        if set_wait is not None:set_wait(True,wait_started)
                    self.ego_release_at=wait_started+self.ego_start_delay_s
                    self.phase='ego_wait'
                    self.status=f'NPC 선출발 · ego 정지 · 출발까지 {self.ego_start_delay_s:.1f}초'
                    self.record_lifecycle('npc_head_start',ego_delay_s=self.ego_start_delay_s)
                    return None
                self.release_prepared_ego(states)
            except Exception:
                try:self.pause_prepared_traffic()
                finally:self.link.hold()
                raise
        if self.phase=='running':
            if self.planning:
                try:self.planning.check()
                except Exception:
                    self.link.hold()
                    raise
            if self.collision_requested:
                return self.end_run('collision')
            self.current_operation=dict(section=None,action='ego_state')
            ego=self.link.ego()
            now=self.clock()
            self.environment.update(ego,now)
            if self.mode=='scene':
                passed=self.completion.checkpoint14_passed
                complete=self.completion.update(ego,now)
                if not passed and self.completion.checkpoint14_passed:
                    print('[완주 조건] 체크포인트 14 통과 · START/END 복귀 대기',flush=True)
                if complete and not self.cancelled.is_set():
                    reason=self.end_run('scene_end')
                    if not self.collision_requested and (reason=='scene_end' or self.phase=='runs_done'):
                        action='총 주행 횟수 완료' if self.phase=='runs_done' else '다음 반복 요청'
                        print('[검증 완주] 체크포인트 14 통과 후 START/END 복귀 · '+action,flush=True)
                    return reason
            if self.mode=='section' and now-self.run_started>.5 and (
                    self.section_finish_pending or self.route.finished(self.section,ego,now)):
                self.section_finish_pending=True
                if self.planning and self.section=='tunnel' and not self.planning.recovery_ready():
                    self.link.hold();self.status='터널 종료선 통과 · 일반 모드 복귀 확인 대기'
                else:
                    reason=self.end_run('section_end')
                    if not self.collision_requested and (reason=='section_end' or self.phase=='runs_done'):
                        action='총 주행 횟수 완료' if self.phase=='runs_done' else '다음 반복 요청'
                        print(f'[구간 종료] {LABELS[self.section]} · 도착/통과 확인 · {action}',flush=True)
                    return reason
            evidence=self.stagnation.update(ego,now) if self.stagnation else None
            if evidence:
                self.current_operation=dict(section=None,action='save_stagnation')
                states=self.read_states()
                with self.contact_lock:
                    if self.cancelled.is_set():return None
                    if not self.collision_requested:
                        evidence['window_started_elapsed_s']=evidence['window_started_at']-self.run_started
                        if self.on_stagnation:self.on_stagnation(self,evidence,states)
                        self.phase='transition'
                        print(f'[정체 감지] {evidence["duration_s"]:.1f}초 · '
                              f'최대 위치 변화 {evidence["maximum_displacement_m"]:.2f}m · 저장 후 주행 종료/반복',flush=True)
                return self.end_run('stagnation')
            for adapter in self.adapters:
                if self.cancelled.is_set():break
                guard=getattr(self,'adapter_tick_guard',None)
                if guard is not None and not guard(adapter,ego,now):
                    continue
                self.current_operation=dict(section=adapter.name,action='tick')
                adapter.tick()
                self.refresh_owned()
            self.current_operation=dict(section=None,action='actor_states')
            self.read_states()
        return None

    def stop(self):
        if not self.lease.held:
            self.cancelled.set();self.phase='stopped'
            return
        self.phase='stopping'
        self.record_lifecycle('stop_requested')
        self.cancelled.set()
        self.link.hold()
        planning_error=None
        if self.planning:
            from scen_runner.orchestration.control import PlanningUnavailable
            try:self.planning.stop()
            except PlanningUnavailable as error:
                planning_error=error
                self.record_lifecycle('planning_stop_unconfirmed',error=str(error))
                print('[판제 종료 미확인] '+str(error)+' · 생성 객체 정리는 계속합니다.',flush=True)
        self.cleanup()
        # The runner pauses ego only while it owns episode preparation/cleanup.
        # Release that pause after cleanup so existing keyboard/network input
        # works without starting a new validation episode.
        if planning_error is None:self.link.release()
        self.lease.release()
        self.phase='stopped'
        self.record_lifecycle('stop_completed',ego_released=planning_error is None)
        if planning_error:
            print('[검증 종료] 생성 객체 정리 완료 · 판제 종료 미확인으로 ego 정지 유지',flush=True)
        else:
            print('[검증 종료] 생성 객체 정리 완료 · ego 일시 정지 해제 · D 기어 · 기존 조종 입력 사용 가능',flush=True)
