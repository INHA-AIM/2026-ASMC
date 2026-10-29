"""GUI and worker for integrated repeatable MORAI validation episodes."""
import argparse
import atexit
import json
import hashlib
import math
import os
from pathlib import Path
import queue
import shutil
import signal
import sys
import tempfile
import threading
import time
import uuid

import numpy as np
import grpc
from PyQt5 import QtCore,QtGui,QtWidgets
from matplotlib.figure import Figure
from matplotlib.collections import PolyCollection
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg,NavigationToolbar2QT

from scen_runner.config import load_config,repo_path,require_morai_endpoint
from scen_runner.morai.grpc_collision import CollisionMonitor
from scen_runner.runtime.validation_episode import EpisodeController,EpisodeLink,EpisodeRoute,LABELS
from scen_runner.planning.static_plan import SceneCancelled
from scen_runner.records.episode_telemetry import SpeedTrace, vehicle_records, crossing_records, CrossingConsole
from scen_runner.runtime.contact_evidence import state_evidence
from scen_runner.records.collision_archive import CollisionArchive,load_collision,SAVE_ROOT
from scen_runner.records.collision_replay import configure_replay
from scen_runner.runtime.validation_lease import ValidationBusy
from scen_runner.runtime.run_counts import RunBudget
from scen_runner.runtime.stagnation import StagnationMonitor


def transport_unavailable(error):
    if isinstance(error,(ConnectionError,TimeoutError)):
        return True
    return isinstance(error,grpc.RpcError) and error.code() in (
        grpc.StatusCode.UNAVAILABLE,grpc.StatusCode.DEADLINE_EXCEEDED,grpc.StatusCode.CANCELLED)


def use_korean_font() -> None:
    """설치된 한글 글꼴을 matplotlib 에 등록한다. 없으면 기본 글꼴을 쓴다."""
    import subprocess

    import matplotlib
    from matplotlib import font_manager

    for family in ("NanumGothic", "NanumBarunGothic", "Noto Sans CJK KR", "Malgun Gothic"):
        try:
            path = subprocess.run(["fc-match", "-f", "%{file}", family], capture_output=True,
                                  text=True, timeout=3).stdout.strip()
        except (OSError, subprocess.SubprocessError):
            return
        if path and family.replace(" ", "").lower() in path.replace(" ", "").lower():
            font_manager.fontManager.addfont(path)
            matplotlib.rcParams["font.family"] = font_manager.FontProperties(fname=path).get_name()
            matplotlib.rcParams["axes.unicode_minus"] = False
            return


def show_on_current_screen(window):
    """Keep the initial window, including its title bar, on one monitor."""
    app=QtWidgets.QApplication.instance()
    screen=app.screenAt(QtGui.QCursor.pos()) or app.primaryScreen()
    if screen is None:
        window.resize(1350,950);window.show();return
    # Screen origins need not be (0, 0), especially with WSLg/mixed monitors.
    # WSLg can add native borders and adjust placement asynchronously; keep
    # clearance on every edge as well as accounting for Qt's frame below.
    bounds=screen.availableGeometry().adjusted(48,48,-48,-48)
    window.resize(min(1350,bounds.width()),min(950,bounds.height()))

    def place():
        frame=window.frameGeometry()
        border_width=max(0,frame.width()-window.width())
        border_height=max(0,frame.height()-window.height())
        window.resize(min(window.width(),bounds.width()-border_width),
                      min(window.height(),bounds.height()-border_height))
        frame=window.frameGeometry();frame.moveCenter(bounds.center())
        window.move(frame.topLeft())

    place();window.show()
    # X11/WSLg may add the title bar or override placement after show().
    def settle():
        if window.isVisible():place()
    QtCore.QTimer.singleShot(0,settle)
    QtCore.QTimer.singleShot(200,settle)



class ValidationRunner(QtCore.QThread):
    frame=QtCore.pyqtSignal(object)
    message=QtCore.pyqtSignal(str)
    finished_clean=QtCore.pyqtSignal()

    def __init__(self,mode,section,seed,sample_json=None,collision_debug=False,speed_debug=False,
                 replay=None,save_dir=None,save_collisions=True,replay_runs=(),run_counts=None,
                 with_planning=False,stack_config=None,recording_spec=None,tunnel_profile=None,
                 continue_on_static_contact=False):
        super().__init__();base=load_config();self.route=EpisodeRoute(sample_json)
        self.link=EpisodeLink(base);self.link.client_key='validation_ego_'+uuid.uuid4().hex
        self.engine=EpisodeController(self.link,self.route,mode,section,seed)
        self.engine.continue_on_static_contact=continue_on_static_contact
        self.tunnel_profile=None
        if tunnel_profile:
            from scen_runner.config import load_yaml
            from scen_runner.planning.tunnel_plan import TunnelPlan
            raw=Path(tunnel_profile).read_bytes();profile=load_yaml(tunnel_profile)
            if not isinstance(profile,dict) or set(profile)-{'scene_type','cross_pattern','static_models'}:
                raise ValueError('터널 프로필은 scene_type/cross_pattern/static_models만 지정합니다.')
            if profile.get('static_models') and profile.get('scene_type') not in ('static','both'):
                raise ValueError('모델 지정 시 scene_type은 static 또는 both여야 합니다.')
            TunnelPlan(0,**profile)  # Validate before simulator mutation, no placement/spawn.
            self.engine.base['tunnel_profile']=profile
            self.tunnel_profile=dict(path=str(tunnel_profile),sha256=hashlib.sha256(raw).hexdigest(),values=profile)
            if replay or replay_runs:raise ValueError('재현 파일과 새 터널 프로필을 동시에 적용할 수 없습니다.')
        if with_planning:
            from scen_runner.orchestration.control import PlanningClient,settings
            self.engine.planning=PlanningClient(settings(stack_config))
        self.replay_runs=tuple(replay_runs);self.replay_run_index=0;self.replay_case_index=0;self.replay_done_reason=None
        if self.replay_runs and replay is None:replay=self.replay_runs[0].cases[0].load()
        self.replay=replay
        if replay:configure_replay(self.engine,replay)
        else:
            self.engine.run_budget=RunBudget.load(mode,section,run_counts)
            self.engine.stagnation=StagnationMonitor.load()
        self.archive=None;self.save_dir=save_dir;self.save_collisions=save_collisions and not self.replay_runs
        self.engine.on_episode_ready=self.archive_ready
        self.engine.on_state_observation=self.archive_observe
        self.engine.on_collision=self.archive_collision
        self.engine.on_stagnation=self.archive_stagnation
        self.engine.on_progress=self.message.emit
        self.commands=queue.Queue();self.closing=False;self.status='연결 준비 중'
        self.close_requested=threading.Event()
        self.monitor=CollisionMonitor(base,self.link.client_key,link_factory=EpisodeLink)
        self.monitor.owned_provider=lambda:dict(self.engine.owned)
        self.monitor.publish_actors=True
        self.monitor.publish_contact_evidence=True
        self.monitor.sample.connect(self.receive_contact)
        self.collision_debug=collision_debug;self.last_contacts=None
        self.speed_debug=speed_debug;self.speed_trace=None
        self.crossing_console=CrossingConsole()
        self.last_contact_verdict={}
        self.verdict_epoch=None
        self.last_native_contact=None
        self.engine.on_contact_observation=self.record_contact
        self.engine.on_lifecycle_observation=self.record_lifecycle
        self.recording=None
        if recording_spec:
            from scen_runner.records.sensor_recording import SensorRecording
            self.recording=SensorRecording(recording_spec,notify=self.archive_message)
            if not self.save_collisions:raise ValueError('센서 기록에는 장면 저장도 필요합니다.')
            self.save_dir=self.recording.directory.parent/'scenes'

    def archive_ready(self,engine,states):
        if self.archive:
            self.archive.begin(engine,states)
            if self.recording:self.archive.checkpoint(engine,'ready')

    def archive_observe(self,engine,states):
        if self.archive:self.archive.observe(engine,states)

    def archive_collision(self,engine,evidence,sample,hits):
        if self.archive:self.archive.collision(engine,evidence,sample,hits)

    def archive_stagnation(self,engine,evidence,states):
        if self.archive:return self.archive.stagnation(engine,evidence,states)
        if self.save_collisions:
            raise RuntimeError('정체 사례 저장 준비가 안 됐습니다. 현재 배치를 유지하고 오류를 표시합니다.')

    def archive_message(self,text):
        print(text,flush=True);self.message.emit(text)

    def record_contact(self,evidence,sample):
        if self.archive:
            ids=set(sample.get('contacts',()))|{self.link.ego_id}
            self.archive.event(dict(event='contact_changed',**evidence,owned_ids=sorted(self.engine.owned),
                actor_states=[state_evidence(s) for s in sample.get('actor_states',()) if s.actor_info.id.value in ids]))
        if self.speed_trace:
            self.speed_trace.record_contact(evidence,[state_evidence(s) for s in sample.get('actor_states',())])

    def record_lifecycle(self,record):
        if self.archive:
            self.archive.event(record)
            if self.recording and record.get('event')=='cleanup_requested':
                self.archive.checkpoint(self.engine,'before_cleanup')

    def receive_contact(self,sample):
        # Reuse the existing native query. Record background contacts as well as
        # owned actors; the scenario collision oracle deliberately owns only its
        # actors, so its silence alone is not an all-scene collision proof.
        if self.recording and self.archive and sample['epoch']==self.engine.contact_epoch:
            self.archive.event(dict(event='native_contact_sample',epoch=sample['epoch'],
                query_started_utc=sample.get('query_started_utc'),
                received_at_utc=sample.get('received_at_utc'),query_s=sample.get('query_s'),
                native_timestamp=sample.get('native_timestamp'),native_tick_count=sample.get('native_tick_count'),
                contacts=list(sample.get('contacts',())),error=sample.get('error'),
                actor_states=[state_evidence(s) for s in sample.get('actor_states',())]))
        self.engine.contact(sample)
        current=self.engine.last_contact_evidence
        if (sample['epoch']==self.engine.contact_epoch and current is not None and
                current['query_started_at']==sample.get('query_started_at')):
            self.last_native_contact=current
        key=(sample['epoch'],tuple(sample.get('contacts',())),sample.get('error'))
        if self.collision_debug and sample['epoch']==self.engine.contact_epoch and key!=self.last_contacts:
            print('[검증 접촉 원본] '+str(sample.get('error') or ','.join(sample.get('contacts',())) or '없음'),flush=True)
            self.last_contacts=key
        if self.collision_debug and sample['epoch']==self.engine.contact_epoch:
            if self.verdict_epoch!=sample['epoch']:
                self.last_contact_verdict.clear()
                self.verdict_epoch=sample['epoch']
            for row in (self.last_native_contact or {}).get('records',[]):
                signature=(sample['epoch'],row['id'],row['verdict'],row['reason'])
                slot=(sample['epoch'],row['id'])
                if row['verdict'] in ('accepted','ongoing') or signature==self.last_contact_verdict.get(slot):continue
                self.last_contact_verdict[slot]=signature
                distance=row.get('separation_m')
                where=f' · 차체 중심거리 {distance:.1f}m' if distance is not None else ''
                print(f'[접촉 ID 검증] ID={row["id"]} · {row["reason"]}{where}',flush=True)

    def request(self,action,mode=None,section=None):
        self.engine.cancelled.set()
        if action=='close':self.close_requested.set()
        self.commands.put((action,mode,section))

    def close_after_disconnect(self,error):
        # The simulator cannot acknowledge cleanup. Finish local resources
        # without reconnecting, retrying remote writes, or claiming deletion.
        self.engine.cancelled.set();self.engine.phase='stopped';self.closing=True
        source=getattr(error,'connection_name','MORAI')
        self.archive_message(f'[검증 종료] {source} 통신 불가 · 원격 객체 정리 미확인 · '
                             '로컬 연결·저장 기록 정리 후 종료: '+str(error))

    def shutdown(self):
        self.close_requested.clear()
        try:self.engine.stop()
        except Exception as error:
            if not transport_unavailable(error):raise
            self.close_after_disconnect(error)
        self.closing=True

    def close_resources(self):
        self.monitor.stopping.set()
        links=[self.monitor.link,self.link]+[getattr(a,'link',None) for a in self.engine.adapters]
        seen=set()
        for link in links:
            if link is None or id(link) in seen:continue
            seen.add(id(link))
            try:link.disconnect()
            except Exception as error:self.archive_message('[연결 종료 오류] '+str(error))
        # Closing the channel also cancels any pending contact RPC.
        self.monitor.wait(2000)
        for name,close in (('판제 실행 정리',self.engine.planning.close if self.engine.planning else None),
                           ('실행 잠금',self.engine.lease.release),
                           ('속도 기록',self.speed_trace.close if self.speed_trace else None),
                           ('재현 기록',self.archive.close if self.archive else None),
                           ('센서 기록',self.recording.close if self.recording else None)):
            if close is None:continue
            try:close()
            except Exception as error:self.archive_message(f'[{name} 종료 오류] '+str(error))

    def select_replay(self,run_index,case_index):
        case=self.replay_runs[run_index].cases[case_index]
        definition=case.load()  # Reject invalid files before changing the active scene.
        self.engine.claim_simulator();self.link.hold()
        if self.engine.planning:self.engine.planning.stop()
        self.engine.cleanup()
        configure_replay(self.engine,definition);self.replay=definition
        self.replay_run_index=run_index;self.replay_case_index=case_index;self.replay_done_reason=None
        kind={'trial':'장면','stagnation':'정체'}.get(case.event_type,'충돌')
        print(f'[{kind} 재현] {case_index+1}/{len(self.replay_runs[run_index].cases)} · '
              f'{LABELS[case.section]} · {case.path.name}',flush=True)
        self.engine.begin(self.monitor.reset_context())

    def replay_case_status(self):
        if not self.replay_runs:return None
        run=self.replay_runs[self.replay_run_index];case=run.cases[self.replay_case_index]
        return dict(run_index=self.replay_run_index,case_index=self.replay_case_index,count=len(run.cases),
                    path=str(case.path),time_kst=case.time_kst,models=case.models,
                    event_type=case.event_type,
                    weather=case.weather,hour=case.hour,done_reason=self.replay_done_reason)

    def snapshot(self):
        states=[]
        for s in self.engine.last_states:
            p=s.transform.location;v=s.velocity
            states.append(dict(id=s.actor_info.id.value,type=s.actor_info.object_type,x=p.x,y=p.y,
                               speed=math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)))
        sections=[]
        for name in LABELS:
            a=next((a for a in self.engine.adapters if a.name==name),None)
            selected=name in self.engine.names
            sections.append(dict(name=name,status='미선택' if not selected else '대기' if a is None else
                                  a.status if a.prepared else '준비 중',count=len(a.owned()) if a else 0,
                                  spawn_period_s=getattr(a,'period',None) if a else None))
        details=self.engine.actor_details()
        vehicles=vehicle_records(self.engine.last_states,details,time.monotonic())
        crossings=crossing_records(self.engine.last_states,self.engine.crossing_details(),self.link.ego_id)
        if self.collision_debug or self.speed_debug:
            self.crossing_console.sample(crossings,self.engine.episode,self.engine.phase,time.monotonic())
        if self.speed_trace:self.speed_trace.sample(vehicles,self.engine.episode,self.engine.phase,crossings,self.engine.seed,
                                                  None,
                                                  self.last_native_contact if self.last_native_contact and
                                                  self.last_native_contact['epoch']==self.engine.contact_epoch else None,
                                                  self.engine.environment.snapshot(),
                                                  actor_supplier=lambda:[state_evidence(s) for s in self.engine.last_states])
        self.frame.emit(dict(mode=self.engine.mode,section=self.engine.section,episode=self.engine.episode,
                            seed=self.engine.seed,phase=self.engine.phase,status=self.engine.status,
                            states=states,owned=dict(self.engine.owned),sections=sections,
                            start=self.engine.pose,error=self.engine.error,vehicles=vehicles,details=details,crossings=crossings,
                            environment=self.engine.environment.snapshot(),
                            traffic_lights=self.engine.signals.snapshot(),
                            completion=self.engine.completion.snapshot() if self.engine.mode=='scene' else None,
                            run_counts=self.engine.run_budget.snapshot() if self.engine.run_budget else None,
                            stagnation=self.engine.stagnation.snapshot() if self.engine.stagnation else None,
                            planning=self.engine.planning.snapshot() if self.engine.planning else dict(enabled=False),
                            archive=dict(folder=str(self.archive.folder),saved=self.archive.saved,
                                         last_file=self.archive.last_file,error=self.archive.error) if self.archive else None,
                            replay=bool(self.replay),replay_case=self.replay_case_status()))

    def run(self):
        if self.engine.run_budget:
            budget=self.engine.run_budget
            label='전체 씬' if self.engine.mode=='scene' else LABELS[self.engine.section]
            print(f'[주행 횟수 설정] {label} · 총 {budget.limit}회 (최초 포함) · {budget.path}',flush=True)
        if self.speed_debug:
            self.speed_trace=SpeedTrace()
            self.engine.on_rpc_observation=self.speed_trace.write
        initial=True
        try:
            if self.recording:self.recording.start()
            if self.save_collisions:
                try:
                    self.archive=CollisionArchive(self.save_dir,notify=self.archive_message)
                    if self.tunnel_profile:self.archive.metadata['tunnel_profile']=self.tunnel_profile
                    if self.recording:self.archive.metadata['sensor_run_id']=self.recording.spec['run_id']
                    self.archive.start()
                except Exception as exc:
                    self.archive=None;self.archive_message('[충돌 저장 준비 실패] '+str(exc))
                    if self.recording:raise
            # Collect Git provenance before the monitor opens a gRPC channel;
            # subprocess creation during active gRPC polling triggers fork warnings.
            self.monitor.start()
            while True:
                try:command=self.commands.get_nowait()
                except queue.Empty:command=None
                try:
                    # A close request has priority over reconnection and queued resets.
                    if self.close_requested.is_set() or command and command[0]=='close':
                        self.shutdown();return
                    if self.recording:self.recording.check()
                    if self.recording and self.archive.error:raise RuntimeError('장면 저장 실패: '+self.archive.error)
                    if not self.link.connected:
                        self.link.connect()
                        self.engine.observe_ego_link()
                    if command and command[0]=='replay':
                        self.select_replay(command[1],command[2]);initial=False
                    if command and command[0] in ('reset','apply'):
                        selection=(command[1] or self.engine.mode,command[2] or self.engine.section)
                        changed=(selection[0]!=self.engine.mode or
                                 selection[0]=='section' and selection[1]!=self.engine.section)
                        self.replay_done_reason=None
                        if command[0]=='apply' or changed:
                            self.engine.begin(self.monitor.reset_context(),selection=selection,reset_budget=True)
                        else:self.engine.begin(self.monitor.reset_context())
                        initial=False
                    if initial:
                        self.engine.begin(self.monitor.reset_context());initial=False
                        self.engine.read_states();self.snapshot()
                    revision=self.engine.state_revision
                    if self.engine.phase not in ('error','busy','replay_done','runs_done'):
                        reason=self.engine.step()
                        if reason:
                            if self.replay_runs:
                                self.replay_done_reason=reason;self.engine.phase='replay_done'
                                result='접촉 확인' if reason=='collision' else '구간 끝 도달'
                                self.engine.status=result+' · ego 정지 · 다시 재현 또는 다음 기록을 선택하세요'
                            else:self.engine.begin(self.monitor.reset_context())
                            self.engine.read_states();self.snapshot()
                    if self.engine.phase=='runs_done':
                        self.engine.read_states();self.snapshot()
                        # Completion already cleaned and returned ego while
                        # held. Calling stop() here would release it to an MPC
                        # that is still running in another terminal.
                        self.closing=True;return
                    # Include actual ego position even while static planning yields.
                    if self.engine.state_revision==revision:self.engine.read_states()
                    self.snapshot()
                except Exception as exc:
                    if self.close_requested.is_set():
                        if transport_unavailable(exc):
                            self.close_after_disconnect(exc);return
                        # A request arriving inside a blocked operation is handled
                        # on the next iteration, without another error-handler RPC.
                        continue
                    if isinstance(exc,ValidationBusy):
                        self.engine.phase='busy';self.engine.error=str(exc);self.engine.status=str(exc)
                        if self.speed_trace:
                            self.speed_trace.write(dict(event='busy',episode=self.engine.episode,phase='busy',error=str(exc)))
                        print('[통합 검증 중복 실행] '+str(exc),flush=True)
                        self.message.emit(str(exc));self.snapshot();initial=False
                        self.msleep(200);continue
                    if command is None and self.engine.cancelled.is_set() and isinstance(exc,SceneCancelled):
                        self.msleep(50);continue
                    self.engine.phase='error';self.engine.error=str(exc)
                    context=dict(self.engine.current_operation)
                    self.engine.record_lifecycle('runtime_error',operation=context,error=str(exc),
                                                 pending_sections=list(self.engine.pending))
                    section=LABELS.get(context.get('section'),'전체 씬')
                    action={'prepare':'준비','cleanup':'정리','tick':'주행 중 갱신','reset':'초기화',
                            'ready_check':'준비 상태 확인','ego_state':'ego 상태 조회',
                            'actor_states':'객체 상태 조회','final_reset':'최종 시작점 복귀',
                            'planning_stop':'판제 종료 확인',
                            'section_signals':'신호 유지',
                            'save_stagnation':'정체 사례 저장'}.get(context.get('action'),'처리')
                    prefix='ego 정지' if self.engine.lease.held else '검증 설정 전 실패'
                    self.engine.status=f'{prefix} · {section} {action} 실패 · '+str(exc)
                    if self.speed_trace:
                        self.speed_trace.write(dict(event='error',episode=self.engine.episode,seed=self.engine.seed,
                            phase='error',operation=context,error=str(exc),pending_sections=list(self.engine.pending)))
                    if self.engine.lease.held:
                        try:self.link.hold()
                        except Exception as hold_error:self.engine.status+=' · 정지 응답 실패: '+str(hold_error)
                    print('[통합 검증 오류] '+self.engine.status,flush=True)
                    self.message.emit(self.engine.status);self.snapshot();initial=False
                self.msleep(200)
        except Exception as exc:
            # Startup recording/archive failure must be visible and terminate
            # before begin() can release Ego. Preserve partial evidence.
            self.engine.phase='error';self.engine.error=str(exc)
            self.archive_message('[검증 시작 실패] '+str(exc))
            self.closing=True
        finally:
            self.close_resources()
            if self.closing:self.finished_clean.emit()


class ValidationWindow(QtWidgets.QMainWindow):
    def __init__(self,mode='scene',section='static',seed=None,sample_json=None,collision_debug=False,speed_debug=False,
                 replay=None,save_dir=None,save_collisions=True,replay_runs=(),run_counts=None,
                 with_planning=False,stack_config=None,recording_spec=None,tunnel_profile=None,
                 continue_on_static_contact=False):
        super().__init__();self.runner=ValidationRunner(mode,section,seed,sample_json,collision_debug,speed_debug,
                                                     replay,save_dir,save_collisions,replay_runs,run_counts,
                                                     with_planning,stack_config,recording_spec,tunnel_profile,
                                                     continue_on_static_contact=continue_on_static_contact)
        replay=self.runner.replay
        if replay:mode='section';section=replay['section']
        self.pending=None;self.pending_ego=None;self.last=None;self.allow_close=False;self.view='ego'
        self.setWindowTitle('scen_runner · 씬별/구간별 반복 검증')
        root=QtWidgets.QWidget();layout=QtWidgets.QVBoxLayout(root);self.setCentralWidget(root)
        controls=QtWidgets.QHBoxLayout();self.mode=QtWidgets.QComboBox()
        self.mode.addItem('씬별 검증','scene');self.mode.addItem('구간별 검증','section')
        self.mode.setCurrentIndex(0 if mode=='scene' else 1)
        self.section=QtWidgets.QComboBox()
        for name,label in LABELS.items():self.section.addItem(label,name)
        self.section.setCurrentIndex(list(LABELS).index(section))
        controls.addWidget(self.mode);controls.addWidget(self.section)
        apply=QtWidgets.QPushButton('선택 모드로 시작');apply.clicked.connect(self.apply_mode);controls.addWidget(apply)
        reset=QtWidgets.QPushButton('다음 반복 · 다시 준비');reset.clicked.connect(self.restart_selected);controls.addWidget(reset)
        if replay:
            self.mode.setEnabled(False);self.section.setEnabled(False);apply.setEnabled(False)
            kind={'trial':'장면','stagnation':'정체'}.get(replay.get('event_type'),'충돌')
            reset.setText('저장된 구간 다시 재현');self.setWindowTitle(f'scen_runner · {kind} 구간 재현')
        close=QtWidgets.QPushButton('객체 정리·조종 복귀 후 종료');close.clicked.connect(self.close);controls.addWidget(close)
        layout.addLayout(controls)
        self.replay_position=(0,0)
        if self.runner.replay_runs:
            navigation=QtWidgets.QHBoxLayout();navigation.addWidget(QtWidgets.QLabel('저장 실행'))
            self.replay_selector=QtWidgets.QComboBox()
            for run in self.runner.replay_runs:self.replay_selector.addItem(run.label)
            self.replay_selector.currentIndexChanged.connect(self.select_replay_run)
            navigation.addWidget(self.replay_selector,1)
            self.replay_previous=QtWidgets.QPushButton('이전 기록')
            self.replay_previous.clicked.connect(lambda:self.navigate_replay(-1))
            self.replay_next=QtWidgets.QPushButton('다음 기록')
            self.replay_next.clicked.connect(lambda:self.navigate_replay(1))
            navigation.addWidget(self.replay_previous);navigation.addWidget(self.replay_next)
            self.replay_previous.setEnabled(False)
            self.replay_next.setEnabled(len(self.runner.replay_runs[0].cases)>1)
            layout.addLayout(navigation)
            self.replay_info=QtWidgets.QLabel();self.replay_info.setWordWrap(True);layout.addWidget(self.replay_info)
        self.header=QtWidgets.QLabel('ego 초기화·정지 상태에서 준비합니다. 주행은 기존 주행 코드가 담당합니다.')
        self.planning_status=QtWidgets.QLabel('판제 연동: 켜짐 · 새 회차 준비마다 재시작' if with_planning else
                                             '판제 연동: 꺼짐 · 키보드/외부 제어기 사용')
        self.header.setWordWrap(True);layout.addWidget(self.header)
        layout.addWidget(self.planning_status)
        self.status=QtWidgets.QLabel('연결 준비 중');self.status.setWordWrap(True);layout.addWidget(self.status)
        self.contact=QtWidgets.QLabel('gRPC ego 접촉 조회 대기');layout.addWidget(self.contact)
        self.environment_status=QtWidgets.QLabel('날씨·시간 적용 대기');layout.addWidget(self.environment_status)
        self.archive_status=QtWidgets.QLabel('충돌/정체 기록 저장 준비 중' if save_collisions else '충돌/정체 기록 저장 꺼짐')
        self.archive_status.setWordWrap(True);layout.addWidget(self.archive_status)
        self.crossing_status=QtWidgets.QLabel('동적객체 상태 수신 대기')
        self.crossing_status.setWordWrap(True);layout.addWidget(self.crossing_status)
        views=QtWidgets.QHBoxLayout()
        for label,kind in [('ego 주변 확대','ego'),('선택 구간 보기','section'),('전체 경로','all')]:
            button=QtWidgets.QPushButton(label);button.clicked.connect(lambda checked=False,k=kind:self.change_view(k));views.addWidget(button)
        layout.addLayout(views)
        self.fig=Figure(figsize=(8,7));self.ax=self.fig.add_subplot(111);self.canvas=FigureCanvasQTAgg(self.fig)
        panel=QtWidgets.QHBoxLayout();left=QtWidgets.QVBoxLayout()
        left.addWidget(NavigationToolbar2QT(self.canvas,self));left.addWidget(self.canvas);panel.addLayout(left,3)
        self.table=QtWidgets.QTableWidget(5,3);self.table.setHorizontalHeaderLabels(['구간','준비 상태','관리 객체'])
        self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.table.setMaximumHeight(200)
        right=QtWidgets.QVBoxLayout()
        self.spawn_period_status=QtWidgets.QLabel('NPC 생성 주기: 회차 준비 중')
        self.spawn_period_status.setWordWrap(True)
        self.spawn_period_status.setVisible(mode=='scene' or section in ('roundabout','high_speed'))
        right.addWidget(self.spawn_period_status);right.addWidget(self.table)
        right.addWidget(QtWidgets.QLabel('NPC 설정/실측 km/h · 가속·브레이크는 MORAI 조회값'))
        self.vehicle_table=QtWidgets.QTableWidget(0,7)
        self.vehicle_table.setHorizontalHeaderLabels(['ID','모델','설정','실측','가속','제동','생성후 초'])
        self.vehicle_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.vehicle_table.verticalHeader().hide()
        self.vehicle_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)
        self.vehicle_table.horizontalHeader().setSectionResizeMode(1,QtWidgets.QHeaderView.Stretch)
        right.addWidget(self.vehicle_table)
        panel.addLayout(right,2);layout.addLayout(panel)
        doc=json.loads(repo_path(load_config()['map']['drivable_map']).read_text())
        self.polygons=[np.asarray(a['polygon'])[:,:2] for a in doc['drivable_areas']]
        self.polygon_bounds=np.asarray([(p[:,0].min(),p[:,1].min(),p[:,0].max(),p[:,1].max())
                                        for p in self.polygons]).reshape(-1,4)
        self.map_artists=None;self.visible_roads=None;self.map_labels={}
        self.map_limits=None;self.legend_ego=None
        self.runner.frame.connect(self.enqueue);self.runner.message.connect(self.status.setText)
        self.runner.finished_clean.connect(self.finish);self.runner.monitor.sample.connect(self.contacts)
        self.runner.monitor.actors.connect(self.enqueue_ego)
        self.timer=QtCore.QTimer(self);self.timer.timeout.connect(self.flush);self.timer.start(200)
        self.runner.start()

    def select_replay_run(self,index):
        if index>=0:self.runner.request('replay',index,0)

    def navigate_replay(self,delta):
        run_index,case_index=self.replay_position;target=case_index+delta
        if 0<=target<len(self.runner.replay_runs[run_index].cases):
            self.runner.request('replay',run_index,target)

    def apply_mode(self):self.runner.request('apply',self.mode.currentData(),self.section.currentData())
    def restart_selected(self):self.runner.request('reset',self.mode.currentData(),self.section.currentData())
    def change_view(self,kind):
        self.view=kind
        if self.last:self.draw(self.last)
    def enqueue(self,frame):self.pending=frame
    def enqueue_ego(self,sample):
        if sample['epoch']!=self.runner.monitor.context() or sample['error']:return
        self.pending_ego=next((s for s in sample['states'] if s['id']==self.runner.link.ego_id),None)

    def contacts(self,sample):
        if sample['epoch']!=self.runner.monitor.context():return
        if sample['error']:text='접촉 조회 실패 · 현재 판정 미확인';color='#777777'
        elif self.runner.engine.phase=='replay_done':
            collided=self.runner.replay_done_reason=='collision'
            text='접촉 확인 · 재현 종료' if collided else '구간 끝 도달 · 재현 종료'
            color='#d32f2f' if collided else '#1565c0'
        elif self.runner.engine.phase!='running':
            text='객체 준비/전환 중 · 자동 충돌 전환 판정 대기';color='#1565c0'
        elif sample.get('query_started_at',float('-inf'))<self.runner.engine.run_started:
            text='주행 시작 전 접촉 조회 응답 제외';color='#1565c0'
        else:
            observed=self.runner.engine.last_contact_evidence
            evidence=observed['records'] if observed and observed.get('query_started_at')==sample.get('query_started_at') else []
            ids=[r['id'] for r in evidence if r['verdict'] in ('accepted','ongoing')]
            unknown=[r['id'] for r in evidence if r['verdict']=='unknown']
            rejected=[r['id'] for r in evidence if r['verdict']=='rejected']
            stale=[r['id'] for r in evidence if r['verdict']=='stale']
            unowned=[r['id'] for r in evidence if r.get('unowned')]
            if ids:text='ego ↔ 생성 객체 접촉: '+','.join(ids);color='#d32f2f'
            elif unknown:text='접촉 ID 상대 상태 미확인: '+','.join(unknown);color='#777777'
            elif stale:text='기존 접촉 기록 잔류 · 새 접촉 미확인: '+','.join(stale);color='#777777'
            elif rejected:text='실제 먼 객체의 접촉 ID 제외: '+','.join(rejected);color='#1565c0'
            elif unowned:text='이번 회차 관리 대상이 아닌 객체 접촉: '+','.join(unowned);color='#777777'
            else:text='ego ↔ 생성 객체 접촉: 미관찰';color='#1565c0'
        self.contact.setText(text);self.contact.setStyleSheet('color:'+color)

    def flush(self):
        if self.pending is None:
            if self.last is None or self.pending_ego is None:return
            frame=dict(self.last)
        else:frame=self.pending
        self.pending=None
        if self.pending_ego is not None:
            frame['states']=[s for s in frame['states'] if s['id']!=self.runner.link.ego_id]+[self.pending_ego]
            self.pending_ego=None
        self.last=frame
        planning=frame.get('planning',{})
        if planning.get('enabled'):
            phase={'ready':'준비 완료','warming':'재시작 · 입력 대기','stopped':'종료 확인 · 대기',
                   'error':'준비 실패','offline':'실행기 미연결'}.get(planning.get('phase'),'상태 확인 중')
            pid=planning.get('worker_pid')
            detail=planning.get('detail','')
            self.planning_status.setText('판제 연동: 켜짐 · '+phase+
                (f' · 실행그룹 PID={pid}' if pid is not None else '')+(f' · {detail}' if detail else ''))
        case=frame.get('replay_case')
        if case:
            self.replay_position=(case['run_index'],case['case_index'])
            self.replay_selector.blockSignals(True);self.replay_selector.setCurrentIndex(case['run_index'])
            self.replay_selector.blockSignals(False)
            self.section.setCurrentIndex(list(LABELS).index(frame['section']))
            self.replay_previous.setEnabled(case['case_index']>0)
            self.replay_next.setEnabled(case['case_index']+1<case['count'])
            kind={'trial':'장면','stagnation':'정체'}.get(case.get('event_type'),'충돌')
            self.header.setText(f'{kind} 재현 {case["case_index"]+1}/{case["count"]} · {LABELS[frame["section"]]} · '
                                f'반복 {frame["episode"]}')
            stamp=case['time_kst'].replace('T',' ')[:19]
            self.replay_info.setText(f'원래 {kind} {stamp} · {case["models"] or LABELS[frame["section"]]} · '
                                    f'{case["weather"]} · {case["hour"]}:00')
            self.replay_info.setToolTip(case['path'])
        else:
            counts=frame.get('run_counts')
            count_text=f' · 주행 {counts["started"]}/{counts["limit"]}' if counts else ''
            self.header.setText(f'{"씬별" if frame["mode"]=="scene" else "구간별 · "+LABELS[frame["section"]]} 검증 · '
                                f'회차 {frame["episode"]}{count_text} · seed={frame["seed"]} · 상태={frame["phase"]}')
            if counts:self.header.setToolTip('주행 횟수 설정: '+str(counts['path']))
        status=frame['status']
        stagnant=frame.get('stagnation')
        if frame['phase']=='running' and stagnant and stagnant['enabled']:
            status+=f' · 정체 감시 {stagnant["elapsed_s"]:.0f}/{stagnant["threshold_s"]:g}초'
        if frame['mode']=='scene' and frame['phase']=='running' and frame.get('completion'):
            status+=' · 완주: '+('CP14 통과 · START/END 복귀 대기' if
                                 frame['completion']['checkpoint14_passed'] else 'CP14 통과 대기')
        self.status.setText(status)
        env=frame.get('environment',{})
        if env.get('error'):
            self.environment_status.setText('날씨·시간 적용 실패 · 실제 환경 미확인: '+env['error'])
        elif env.get('weather') is not None:
            trigger='저장된 환경 일정 재현' if frame.get('replay') else (
                '체크포인트 통과 시 변경' if frame['mode']=='scene' else '구간 반복 시작 시 변경')
            self.environment_status.setText(f'날씨 {env["weather"]} · 시간 {env["hour"]}:00 · '
                                            f'{env.get("reason","")} · {trigger}')
        else:self.environment_status.setText('날씨·시간 적용 대기')
        signals=frame.get('traffic_lights',{})
        if signals.get('colors'):
            color='직진·좌회전 초록' if 48 in signals['colors'].values() else '직진 초록'
            self.environment_status.setText(self.environment_status.text()+f' · 구간 신호: {color} 고정')
        archive=frame.get('archive')
        if archive:
            self.archive_status.setText('재현 기록 저장 실패: '+archive['error'] if archive['error'] else
                f'충돌/정체 저장 {archive["saved"]}건 · '+str(archive['last_file'] or archive['folder']))
        crossing_text=[]
        for r in frame.get('crossings',[]):
            d=r.get('ego_to_start_m')
            distance=f'{d:.2f}m' if d is not None else '미수신'
            crossing_text.append(f'{LABELS[r["section"]]} {r["model"]} ID={r["id"]}: {r["status"]} · '
                                 f'생성점 거리 {distance} / Activation {r["activation_distance_m"]:.2f}m')
        self.crossing_status.setText('\n'.join(crossing_text) or '동적객체 없음 / 준비 중')
        periods=[]
        for s in frame['sections']:
            if s['name'] not in ('roundabout','high_speed'):
                continue
            if frame['mode']=='section' and s['name']!=frame['section']:
                continue
            period=s.get('spawn_period_s')
            context='에고 출발 대기 중' if frame['phase']=='ego_wait' and s['name']=='high_speed' else '이번 회차'
            value=f'{period:g}초 ({context})' if period is not None else (
                '저장 기록 기준' if frame.get('replay') else '회차 준비 중')
            periods.append(f'{LABELS[s["name"]]} NPC 생성 주기: {value}')
        self.spawn_period_status.setText('\n'.join(periods))
        self.spawn_period_status.setVisible(bool(periods))
        self.spawn_period_status.setToolTip('생성 위치의 안전거리·TTC 조건 때문에 실제 생성은 늦어질 수 있습니다.')
        for row,s in enumerate(frame['sections']):
            for col,text in enumerate([LABELS[s['name']],s['status'],str(s['count'])]):
                self.set_cell(self.table,row,col,text,text)
        actual={r['id']:r for r in frame.get('vehicles',[])}
        details=frame.get('details',{})
        cars={aid:d for aid,d in details.items() if d['kind']==1}
        if self.vehicle_table.rowCount()!=len(cars):self.vehicle_table.setRowCount(len(cars))
        for row,(aid,info) in enumerate(cars.items()):
            sample=actual.get(aid)
            cells=[aid,info['model'],str(info['target'])]
            cells+=([f'{sample["speed"]:.1f}',f'{sample["throttle"]:.2f}',f'{sample["brake"]:.2f}',f'{sample["age_s"]:.0f}']
                    if sample else ['미수신','—','—','—'])
            tooltip=LABELS[info['section']]+f' · 생성점 {info["source"]}'
            if sample:
                front=sample['front_center_m']
                tooltip+=f'\n링크 {sample["link"]} · 앞쪽 차량 중심거리 '+(f'{front:.1f}m' if front is not None else '후보 없음')
            for col,text in enumerate(cells):
                self.set_cell(self.vehicle_table,row,col,text,tooltip)
        self.draw(frame)

    @staticmethod
    def set_cell(table,row,col,text,tooltip=None):
        item=table.item(row,col)
        if item is None:
            item=QtWidgets.QTableWidgetItem(text);table.setItem(row,col,item)
        elif item.text()!=text:item.setText(text)
        if tooltip is not None and item.toolTip()!=tooltip:item.setToolTip(tooltip)

    def draw(self,frame):
        route=self.runner.route;ego=next((s for s in frame['states'] if s['id']==self.runner.link.ego_id),None)
        if self.view=='all':points=route.curve.xyz[:,:2];lo=points.min(0)-15;hi=points.max(0)+15
        elif self.view=='section':
            points=route.section(self.section.currentData())['curve'].xyz[:,:2];lo=points.min(0)-20;hi=points.max(0)+20
        else:
            start=frame['start'] or route.start(frame['mode'],frame['section'])
            point=np.array([ego['x'],ego['y']]) if ego else np.array([start['x'],start['y']])
            lo=point-75;hi=point+75
        ax=self.ax
        if self.map_artists is None:
            ax.set_facecolor('#d8d3c4');ax.set_aspect('equal')
            roads=PolyCollection([],facecolors='#4b4f54',edgecolors='none');ax.add_collection(roads)
            line,=ax.plot(route.curve.xyz[:,0],route.curve.xyz[:,1],color='#1565c0',lw=1,label='ego 경로')
            ego_dot=ax.scatter([],[],marker='s',color='#1565c0',s=70,label='ego')
            dots=ax.scatter([],[],color='#ef6c00',s=40)
            ego_text=ax.text(0,0,'',fontsize=8)
            self.map_artists=roads,line,ego_dot,dots,ego_text
        roads,line,ego_dot,dots,ego_text=self.map_artists
        bounds=self.polygon_bounds
        visible=tuple(np.flatnonzero((bounds[:,2]>=lo[0]) & (bounds[:,0]<=hi[0]) &
                                    (bounds[:,3]>=lo[1]) & (bounds[:,1]<=hi[1])))
        if visible!=self.visible_roads:
            roads.set_verts([self.polygons[i] for i in visible]);self.visible_roads=visible
        ego_visible=False;positions=[];labels=set();occurrences={}
        for s in frame['states']:
            if not (lo[0]<=s['x']<=hi[0] and lo[1]<=s['y']<=hi[1]):continue
            if s['id']==self.runner.link.ego_id:
                ego_visible=True;ego_dot.set_offsets([[s['x'],s['y']]])
                ego_text.set_position((s['x']+1,s['y']+1));ego_text.set_text(f'ego {s["speed"]:.1f}km/h')
            elif s['id'] in frame['owned']:
                positions.append((s['x'],s['y']))
                model,speed=frame['owned'][s['id']]
                occurrence=occurrences.get(s['id'],0);occurrences[s['id']]=occurrence+1
                key=s['id'],occurrence;labels.add(key)
                text=self.map_labels.get(key)
                if text is None:text=ax.text(0,0,'',fontsize=7);self.map_labels[key]=text
                text.set_position((s['x']+1,s['y']+1))
                text.set_text(f'{s["id"]} · 설정 {speed} / 실측 {s["speed"]:.1f}km/h')
        for key in self.map_labels.keys()-labels:self.map_labels.pop(key).remove()
        ego_dot.set_visible(ego_visible);ego_text.set_visible(ego_visible)
        dots.set_offsets(np.asarray(positions).reshape(-1,2))
        limits=tuple(lo)+tuple(hi)
        if limits!=self.map_limits or ax.get_xlim()!=tuple((lo[0],hi[0])) or ax.get_ylim()!=tuple((lo[1],hi[1])):
            ax.set_xlim(lo[0],hi[0]);ax.set_ylim(lo[1],hi[1]);self.map_limits=limits
        if self.legend_ego!=ego_visible:
            ax.legend(handles=[line]+([ego_dot] if ego_visible else []),loc='upper left')
            self.legend_ego=ego_visible
        self.canvas.draw_idle()

    def finish(self):
        self.allow_close=True;self.timer.stop();self.runner.wait(1000);self.close()

    def closeEvent(self,event):
        if self.allow_close:event.accept();return
        event.ignore();self.status.setText('ego 정지 · 현재 반복 객체 정리 중');self.runner.request('close')


def main():
    parser=argparse.ArgumentParser(description='MORAI 씬별/구간별 반복 검증')
    parser.add_argument('--mode',choices=['scene','section'],default='scene')
    parser.add_argument('--section',choices=list(LABELS),default='static')
    parser.add_argument('--seed',type=int)
    parser.add_argument('--collision-debug',action='store_true',help='gRPC ego 접촉 목록이 바뀔 때만 출력')
    parser.add_argument('--continue-on-static-contact',action='store_true',
                        help='등록된 정적 미션 고정 장애물 접촉은 기록하고 주행 계속; 차량·보행자 접촉은 기존 종료')
    parser.add_argument('--speed-debug',action='store_true',help='NPC 목표/실측·가속·브레이크·링크를 JSONL에 저장')
    parser.add_argument('--sample-json',help='시작 위치를 읽을 JSON 경로; 기본은 사용자 예시의 저장된 시작 좌표')
    replay_mode=parser.add_mutually_exclusive_group()
    replay_mode.add_argument('--replay',help='저장된 충돌 JSON으로 해당 구간을 재현')
    replay_mode.add_argument('--replay-browser',action='store_true',help='최신 저장 실행부터 GUI로 충돌/정체 기록을 순서대로 재현')
    parser.add_argument('--replay-section',choices=list(LABELS),help='여러 구간이 저장된 파일의 재현 구간')
    parser.add_argument('--collision-log-dir',help='충돌 저장 폴더; 기본 tools/scen_runner/save_collision_log')
    parser.add_argument('--no-save-collision',action='store_true',help='이번 실행의 충돌/정체 재현 파일 저장을 끔')
    parser.add_argument('--run-counts',type=Path,help='총 주행 횟수 YAML; 기본 tools/scen_runner/config/run_counts.yaml')
    parser.add_argument('--with-planning',action='store_true',help='판제 실행기의 종료/새 위치 입력 준비를 확인하고 주행')
    parser.add_argument('--stack-config',type=Path,help='4개 실행기 연동 설정; 기본 config/stack.yaml')
    parser.add_argument('--recording-spec',type=Path,help='등록된 실험 run의 ROS 센서·판단 녹화 JSON')
    parser.add_argument('--tunnel-profile',type=Path,help='터널 반복 시험의 씬/정적 모델 선택 YAML; 배치 검사는 유지')
    args=parser.parse_args()
    try:require_morai_endpoint(load_config()['morai'])
    except (OSError,ValueError) as error:parser.error(str(error))
    replay=None;replay_runs=()
    if args.replay_section and not args.replay:parser.error('--replay-section은 --replay와 함께 사용합니다.')
    if args.replay:
        try:replay,source=load_collision(args.replay,args.replay_section)
        except (OSError,ValueError,KeyError) as exc:parser.error(str(exc))
        args.mode='section';args.section=replay['section'];args.seed=replay['seed']
        print('[충돌 재현 파일] '+args.replay,flush=True)
    elif args.replay_browser:
        from scen_runner.records.replay_browser import saved_runs
        try:
            replay_runs,warnings=saved_runs(args.collision_log_dir or SAVE_ROOT)
            replay=replay_runs[0].cases[0].load()
        except (OSError,ValueError,KeyError) as exc:parser.error(str(exc))
        args.mode='section';args.section=replay['section'];args.seed=replay['seed'];args.no_save_collision=True
        print(f'[충돌 재현 목록] {replay_runs[0].label} · 저장 실행 {len(replay_runs)}개',flush=True)
        if warnings:print(f'[재현 제외] 읽을 수 없는 기록 {len(warnings)}개',flush=True)
    runtime=os.environ.get('XDG_RUNTIME_DIR','')
    try:
        st=os.stat(runtime);usable=os.path.isdir(runtime) and st.st_uid==os.getuid() and st.st_mode&0o777==0o700
    except OSError:usable=False
    if not usable:
        runtime=tempfile.mkdtemp(prefix='scen-validation-qt-');os.chmod(runtime,0o700)
        os.environ['XDG_RUNTIME_DIR']=runtime;atexit.register(shutil.rmtree,runtime,ignore_errors=True)
    use_korean_font();app=QtWidgets.QApplication(sys.argv[:1])
    try:
        win=ValidationWindow(args.mode,args.section,args.seed,args.sample_json,args.collision_debug,args.speed_debug,
                             replay,args.collision_log_dir,not args.no_save_collision,replay_runs,args.run_counts,
                             args.with_planning,args.stack_config,args.recording_spec,args.tunnel_profile,
                             continue_on_static_contact=args.continue_on_static_contact)
    except (OSError,ValueError) as error:parser.error(str(error))
    show_on_current_screen(win)
    pending_close=[False]
    def request_signal_close(*_):pending_close[0]=True
    def poll_signal_close():
        if pending_close[0]:
            pending_close[0]=False;win.runner.request('close')
    # Python signal handlers never acquire Event/Queue locks. The Qt timer
    # delivers Ctrl+C/SIGTERM outside the interrupted callback.
    pulse=QtCore.QTimer(app);pulse.timeout.connect(poll_signal_close);pulse.start(100)
    previous={s:signal.signal(s,request_signal_close) for s in (signal.SIGINT,signal.SIGTERM)}
    try:return app.exec_()
    finally:
        for s,handler in previous.items():signal.signal(s,handler)


if __name__ == '__main__':
    raise SystemExit(main())
