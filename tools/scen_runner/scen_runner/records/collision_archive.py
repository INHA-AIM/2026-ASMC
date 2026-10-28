"""Capture replayable section entries before reset; write files off the GUI thread."""
import copy
import csv
import hashlib
import json
import math
import os
import queue
import subprocess
import threading
import uuid
from collections import deque
from datetime import datetime, timedelta, timezone
from pathlib import Path
import numpy as np

from scen_runner.config import REPO_ROOT, TOOL_ROOT, LINK_MAP, LOCAL_CONFIG, load_config, repo_path

SCHEMA='asmc.section_collision'
STAGNATION_SCHEMA='asmc.section_stagnation'
TRIAL_SCHEMA='asmc.section_trial'
VERSION=1
KST=timezone(timedelta(hours=9),'KST')
SAVE_ROOT=TOOL_ROOT/'save_collision_log'


def saved_event(record):
    kind='trial' if record.get('schema')==TRIAL_SCHEMA else 'stagnation' if record.get('schema')==STAGNATION_SCHEMA or 'stagnation' in record else 'collision'
    return kind,record[kind]


def json_scalar(value):
    """Timing calculations may contain numpy bool/int/float scalars or arrays."""
    if isinstance(value,np.generic):return value.item()
    if isinstance(value,np.ndarray):return value.tolist()
    raise TypeError('Unsupported collision-log value: '+type(value).__name__)


def wall_times(value=None):
    utc=datetime.fromisoformat(value) if value else datetime.now(timezone.utc)
    if utc.tzinfo is None:utc=utc.replace(tzinfo=timezone.utc)
    return dict(utc=utc.astimezone(timezone.utc).isoformat(),kst=utc.astimezone(KST).isoformat())


def actor_state(state):
    p=state.transform.location;r=state.transform.rotation;v=state.velocity
    vs=state.vehicle_state
    box=state.bounding_box
    return dict(id=state.actor_info.id.value,kind=int(state.actor_info.object_type),
        pose=dict(x=float(p.x),y=float(p.y),z=float(p.z),roll=float(r.x),pitch=float(r.y),yaw=float(r.z)),
        speed_kmh=float(math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)),velocity=[v.x,v.y,v.z],
        box_center=[box.location.x,box.location.y,box.location.z],
        box_extents=[box.extent.x,box.extent.y,box.extent.z],
        throttle=vs.throttle,brake=vs.brake,steer=vs.steer,
        link=vs.current_link_info.id.value,remaining_distance=vs.remaining_distance,
        native_contacts=list(vs.collision_objects))


def signal_settings(engine):
    signals=getattr(engine,'signals',None)
    return signals.snapshot() if signals is not None else dict(policy='unchanged',colors={})


def provenance():
    files=[p for p in (TOOL_ROOT/'config').glob('*.yaml') if p != LOCAL_CONFIG]
    files+=list((TOOL_ROOT/'scen_runner').rglob('*.py'))
    files+=list((TOOL_ROOT/'dll/src').glob('*.cs'))
    base=load_config()
    maps=[repo_path(value) for value in base['map'].values()]
    maps.append(repo_path(LINK_MAP))
    hashes={}
    for path in files+maps:
        if path.is_file():
            try:name=str(path.resolve().relative_to(REPO_ROOT))
            except ValueError:name=str(path.resolve())
            hashes[name]=hashlib.sha256(path.read_bytes()).hexdigest()
    try:
        commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=REPO_ROOT,text=True,timeout=2).strip()
        dirty=bool(subprocess.check_output(['git','status','--porcelain'],cwd=REPO_ROOT,text=True,timeout=2).strip())
    except (OSError,subprocess.SubprocessError):commit=None;dirty=None
    return dict(commit=commit,dirty=dirty,sha256=hashes,
                installed_dll='Not measured by the container; source fingerprints are recorded.')


class CollisionArchive:
    def __init__(self,root=None,notify=print,metadata=None):
        self.run_id=uuid.uuid4().hex[:12];self.started=wall_times()
        stamp=datetime.fromisoformat(self.started['kst']).strftime('%Y%m%d_%H%M%S')
        self.root=Path(root or SAVE_ROOT)
        self.folder=self.root/stamp
        self.file_owner=None
        self.notify=notify;self.metadata=metadata if metadata is not None else provenance()
        self.lock=threading.RLock();self.jobs=queue.Queue();self.writer=None
        self.episode=None;self.entries={};self.records={};self.current={};self.history=deque(maxlen=25)
        self.completed=set();self.saved=0;self.sequence=0;self.last_file=None;self.error=None

    def start(self):
        # Docker runs as root, but bind-mounted logs must remain manageable
        # by the workspace user. Derive that owner instead of assuming UID 1000.
        if os.geteuid()==0:
            owner_path=TOOL_ROOT if self.root==SAVE_ROOT else self.root
            while not owner_path.exists():owner_path=owner_path.parent
            owner=owner_path.stat();self.file_owner=(owner.st_uid,owner.st_gid)
        missing=[];parent=self.root
        while not parent.exists():missing.append(parent);parent=parent.parent
        self.root.mkdir(parents=True,exist_ok=True)
        for directory in reversed(missing):self._set_owner(directory)
        if self.root==SAVE_ROOT:self._set_owner(self.root)
        stem=self.folder.name;suffix=1
        while True:
            try:self.folder.mkdir();break
            except FileExistsError:
                suffix+=1;self.folder=self.root/f'{stem}_{suffix}'
        self._set_owner(self.folder)
        run_file=self.folder/'run.json'
        run_file.write_text(json.dumps(dict(schema=SCHEMA,version=VERSION,run_id=self.run_id,
            started=self.started,provenance=self.metadata),ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        self._set_owner(run_file)
        diagnostics=self.folder/'events.jsonl'
        diagnostics.touch();self._set_owner(diagnostics)
        self.writer=threading.Thread(target=self._write,daemon=True,name='collision-file-writer');self.writer.start()
        self.notify('[충돌 저장 폴더] '+str(self.folder))
        self.notify('[검증 진단 파일] '+str(diagnostics))

    def event(self,record):
        """Persist lifecycle/contact transitions without requiring speed debug."""
        self.jobs.put(('event',dict(time=wall_times(),**copy.deepcopy(record))))

    def _set_owner(self,path):
        if self.file_owner is not None:
            current=path.stat()
            if (current.st_uid,current.st_gid)!=self.file_owner:os.chown(path,*self.file_owner)

    def begin(self,engine,states):
        with self.lock:
            self.episode=engine.episode;self.entries={};self.records={};self.current={};self.history.clear()
        self.observe(engine,states,engine.run_started)

    def _definitions(self,engine):
        active={}
        for section in list(engine.adapters):
            for record in list(getattr(section,'replay_history',())):
                key=record['key']
                if key not in self.records:self.records[key]=dict(copy.deepcopy(record),section=section.name,seed=section.seed)
                elif record['native_id'] is not None:self.records[key]['native_id']=record['native_id']
            for actor in list(getattr(section,'actors',())):
                if actor.native_id and actor.replay_key in self.records and engine.id_owners.get(actor.native_id) is section:
                    active[actor.native_id]=actor.replay_key
        return active

    def observe(self,engine,states,now=None):
        if engine.phase!='running':return
        now=engine.clock() if now is None else now
        if self.episode!=engine.episode:return
        bodies={s.actor_info.id.value:actor_state(s) for s in states}
        ego=bodies.get(engine.link.ego_id)
        with self.lock:
            active=self._definitions(engine)
            self.current={key:bodies[aid] for aid,key in active.items()
                          if aid in bodies and bodies[aid]['kind']==self.records[key]['kind']}
            elapsed=max(0.,now-engine.run_started)
            environment=engine.environment.snapshot()
            if ego is None:return
            # actor_state creates fresh nested values, and observe replaces current
            # instead of mutating those values. Retain that snapshot; collision()
            # still takes an independent deep copy before queuing a file write.
            self.history.append(dict(elapsed_s=elapsed,ego=ego,actors=self.current))
            station=error=None
            for section in list(engine.adapters):
                name=section.name
                if name not in self.entries:
                    # Entry is captured once per section; later observations only
                    # need its actor/weather timeline, not another route projection.
                    if station is None:
                        station,error=engine.route.curve.project(ego['pose']['x'],ego['pose']['y'])
                    definition=engine.route.section(name)
                    entered=(engine.mode=='section' and name==engine.section) or (
                        error<=6. and definition['start']-1.<=station<=definition['end']+1.)
                    expected={key for key in active.values() if self.records[key]['section']==name}
                    if entered and expected and expected<=self.current.keys():
                        baseline=[]
                        for key,state in self.current.items():
                            if self.records[key]['section']==name:
                                baseline.append(dict(record=copy.deepcopy(self.records[key]),state=copy.deepcopy(state)))
                        self.entries[name]=dict(section=name,seed=section.seed,entry_elapsed_s=elapsed,
                            start_pose=dict(ego['pose']),configured_start_pose=dict(definition['pose']),
                            route_config=copy.deepcopy(engine.route.cfg),ego_at_entry=ego,
                            baseline=baseline,events=[],environment=copy.deepcopy(environment),weather_events=[],
                            traffic_lights=signal_settings(engine),
                            active_keys={x['record']['key'] for x in baseline},seen_keys=set(self.records))
                entry=self.entries.get(name)
                if entry is None:continue
                relative=elapsed-entry['entry_elapsed_s']
                for key,record in self.records.items():
                    if record['section']!=name or key in entry['seen_keys']:continue
                    if record['native_id'] is not None:
                        entry['seen_keys'].add(key)
                        birth=max(0.,record['born_at']-engine.run_started-entry['entry_elapsed_s'])
                        entry['events'].append(dict(action='spawn',time_s=birth,record=copy.deepcopy(record)))
                current_keys={key for key in active.values() if self.records[key]['section']==name}
                for key in entry['active_keys']-current_keys:
                    # Native deletion/ID reuse is captured by generation key, not by reused numeric ID.
                    entry['events'].append(dict(action='remove',time_s=relative,key=key))
                entry['active_keys']=current_keys
                last=entry['weather_events'][-1]['environment'] if entry['weather_events'] else entry['environment']
                if (environment.get('weather'),environment.get('hour'))!=(last.get('weather'),last.get('hour')):
                    entry['weather_events'].append(dict(time_s=relative,environment=copy.deepcopy(environment)))

    def checkpoint(self,engine,reason):
        """Save a non-collision scene too; do not consume its later failure event."""
        now=engine.clock()
        with self.lock:
            targets=sorted(self.entries)
            if not targets:return False
            replay=self._replay_entries(targets,now,engine.run_started,'trial')
            self.jobs.put(dict(schema=TRIAL_SCHEMA,version=VERSION,event_type='trial',
                run_id=self.run_id,run_started=self.started,mode=engine.mode,episode=engine.episode,
                seed=engine.seed,provenance=self.metadata,
                trial=dict(time=wall_times(),sections=targets,reason=reason,
                           elapsed_s=max(0.,now-engine.run_started),environment=engine.environment.snapshot()),
                replay=replay,replay_available=len(replay)==len(targets),
                all_section_settings=copy.deepcopy(list(self.records.values())),history=copy.deepcopy(list(self.history))))
        return True

    def collision(self,engine,evidence,sample,hits):
        now=sample.get('received_at',engine.clock())
        # Preserve the most recent section entry. The impact pose is never used as a new entry.
        bodies={s.actor_info.id.value:actor_state(s) for s in sample.get('actor_states',())}
        with self.lock:
            if engine.episode in self.completed:return
            self._definitions(engine)
            current_ids={}
            for aid in hits:
                candidates=[r for r in self.records.values() if r['native_id']==aid and r['born_at']<=now]
                if candidates:current_ids[aid]=max(candidates,key=lambda r:r['born_at'])['key']
            targets=sorted({self.records[key]['section'] for key in current_ids.values()})
            replay=self._replay_entries(targets,now,engine.run_started,'collision')
            document=dict(schema=SCHEMA,version=VERSION,run_id=self.run_id,run_started=self.started,
                mode=engine.mode,episode=engine.episode,seed=engine.seed,provenance=self.metadata,
                collision=dict(time=wall_times(sample.get('received_at_utc')),sections=targets,
                    counterpart_ids=sorted(hits),elapsed_s=max(0.,now-engine.run_started),
                    evidence=copy.deepcopy(evidence),actors=bodies,
                    environment=engine.environment.snapshot_at(now),
                    query_s=sample.get('query_s'),query_started_utc=sample.get('query_started_utc'),
                    native_timestamp=sample.get('native_timestamp'),native_tick_count=sample.get('native_tick_count')),
                replay=replay,replay_available=bool(targets) and len(replay)==len(targets),
                all_section_settings=copy.deepcopy([r for r in self.records.values() if r['born_at']<=now]),
                history=copy.deepcopy([r for r in self.history if r['elapsed_s']<=max(0.,now-engine.run_started)]))
            self.completed.add(engine.episode)
            self.jobs.put(document)

    def _replay_entries(self,targets,now,run_started,kind):
        replay={}
        for name in targets:
            entry=copy.deepcopy(self.entries.get(name))
            if entry is None or entry['entry_elapsed_s']>now-run_started:continue
            entry.pop('active_keys');entry.pop('seen_keys')
            event_time=max(0.,now-run_started-entry['entry_elapsed_s'])
            entry[kind+'_at_s']=event_time
            entry['events']=[e for e in entry['events'] if e['time_s']<=event_time]
            entry['weather_events']=[e for e in entry['weather_events'] if e['time_s']<=event_time]
            replay[name]=entry
        return replay

    def stagnation(self,engine,evidence,states):
        now=evidence['detected_at']
        self.observe(engine,states,now)
        bodies={s.actor_info.id.value:actor_state(s) for s in states}
        ego=bodies.get(engine.link.ego_id)
        if ego is None:raise RuntimeError('정체 저장에 필요한 ego 상태가 없습니다.')
        station,error=engine.route.curve.project(ego['pose']['x'],ego['pose']['y'])
        definitions={a.name:engine.route.section(a.name) for a in engine.adapters}
        if engine.mode=='section':
            name=engine.section;resolution='selected_section'
        else:
            inside=[name for name,d in definitions.items() if d['start']<=station<=d['end']]
            upcoming=[name for name,d in definitions.items() if d['start']>station]
            if inside:
                name=inside[0];resolution='inside_section'
            elif upcoming:
                name=min(upcoming,key=lambda n:definitions[n]['start']);resolution='outside_section_upcoming'
            else:
                name=min(definitions,key=lambda n:abs(station-definitions[n]['end']));resolution='outside_section_previous'
        with self.lock:
            if engine.episode in self.completed:return False
            active=self._definitions(engine)
            if name not in self.entries:
                # Before a section is entered, preserve the observed ego pose
                # and that section's CURRENT bodies. Do not invent missing states.
                expected={key for key in active.values() if self.records[key]['section']==name}
                if expected and expected<=self.current.keys():
                    baseline=[dict(record=copy.deepcopy(self.records[key]),state=copy.deepcopy(self.current[key]))
                              for key in sorted(expected)]
                    adapter=next(a for a in engine.adapters if a.name==name)
                    self.entries[name]=dict(section=name,seed=adapter.seed,entry_elapsed_s=now-engine.run_started,
                        start_pose=dict(ego['pose']),configured_start_pose=dict(definitions[name]['pose']),
                        route_config=copy.deepcopy(engine.route.cfg),ego_at_entry=copy.deepcopy(ego),
                        baseline=baseline,events=[],environment=engine.environment.snapshot_at(now),weather_events=[],
                        traffic_lights=signal_settings(engine),
                        active_keys=set(expected),seen_keys=set(self.records))
            replay=self._replay_entries([name],now,engine.run_started,'stagnation')
            document=dict(schema=STAGNATION_SCHEMA,version=VERSION,event_type='stagnation',
                run_id=self.run_id,run_started=self.started,mode=engine.mode,episode=engine.episode,
                seed=engine.seed,provenance=self.metadata,
                stagnation=dict(time=wall_times(),sections=[name],section_resolution=resolution,
                    route_station_m=station,route_error_m=error,elapsed_s=max(0.,now-engine.run_started),
                    detector=copy.deepcopy(evidence),actors=bodies,environment=engine.environment.snapshot_at(now)),
                replay=replay,replay_available=name in replay,
                all_section_settings=copy.deepcopy([r for r in self.records.values() if r['born_at']<=now]),
                history=copy.deepcopy([r for r in self.history if r['elapsed_s']<=max(0.,now-engine.run_started)]))
            self.completed.add(engine.episode);self.jobs.put(document)
            return True

    def _write(self):
        while True:
            record=self.jobs.get()
            try:
                if record is None:return
                if isinstance(record,tuple) and record[0]=='event':
                    payload=json.dumps(record[1],ensure_ascii=False,allow_nan=False,default=json_scalar)
                    with (self.folder/'events.jsonl').open('a',encoding='utf-8') as stream:
                        stream.write(payload+'\n')
                    continue
                self.sequence+=1
                kind,event=saved_event(record)
                section='-'.join(event['sections']) or 'unknown'
                prefix=kind+'_' if kind!='collision' else ''
                path=self.folder/f'{self.sequence:03d}_{prefix}{section}.json'
                temporary=path.with_suffix('.json.tmp')
                # Convert before opening a file: unsupported values must not
                # leave a partial document that looks like a pending save.
                payload=json.dumps(record,ensure_ascii=False,indent=2,allow_nan=False,default=json_scalar)
                with temporary.open('x',encoding='utf-8') as stream:
                    stream.write(payload+'\n');stream.flush()
                self._set_owner(temporary)
                temporary.replace(path)
                summary=self.folder/'summary.csv';new=not summary.exists()
                with summary.open('a',encoding='utf-8-sig',newline='') as stream:
                    writer=csv.writer(stream)
                    if new:writer.writerow(['time_kst','episode','section','ids','weather','hour','seed','file','event_type'])
                    env=event['environment']
                    writer.writerow([event['time']['kst'],record['episode'],section,','.join(event.get('counterpart_ids',())),
                                     env.get('weather'),env.get('hour'),record['seed'],path.name,kind])
                self._set_owner(summary)
                with self.lock:self.saved+=1;self.last_file=str(path)
                self.notify(({'stagnation':'[정체 저장 완료] ','trial':'[장면 저장 완료] '}.get(kind,'[충돌 저장 완료] '))+str(path))
            except Exception as exc:
                self.error=str(exc);self.notify('[재현 기록 저장 실패] '+str(exc))
            finally:self.jobs.task_done()

    def close(self):
        if self.writer is not None:
            self.jobs.put(None);self.writer.join(5.)
            if self.writer.is_alive():self.notify('[재현 기록 저장 실패] 종료 시 파일 쓰기 대기 시간 초과')


def load_collision(path,section=None,check_maps=True):
    record=json.loads(Path(path).read_text(encoding='utf-8'))
    if record.get('schema') not in (SCHEMA,STAGNATION_SCHEMA,TRIAL_SCHEMA) or record.get('version')!=VERSION:
        raise ValueError('지원하지 않는 구간 재현 파일 형식입니다.')
    choices=record.get('replay',{})
    if section is None:
        if len(choices)!=1:raise ValueError('재현 구간을 --replay-section으로 지정하세요. 후보: '+','.join(choices))
        section=next(iter(choices))
    if section not in choices:raise ValueError('저장된 진입 상태가 없는 구간입니다: '+str(section))
    selected=copy.deepcopy(choices[section])
    selected['event_type']=saved_event(record)[0]
    if selected.get('section')!=section or not isinstance(selected.get('seed'),int):
        raise ValueError('저장된 구간/seed 정보가 유효하지 않습니다.')
    route=selected.get('route_config',{})
    if section not in route.get('sections',{}) or 'end_radius_m' not in route or 'section_height_offset_m' not in route:
        raise ValueError('저장된 구간 경로 설정이 없습니다.')
    speed=selected.get('ego_at_entry',{}).get('speed_kmh')
    if not isinstance(speed,(int,float)) or not math.isfinite(speed) or speed<0:
        raise ValueError('재현 ego 진입 속도가 유효하지 않습니다.')
    if check_maps:
        map_files={str(repo_path(p).resolve()) for p in load_config()['map'].values()}
        map_files.add(str(repo_path(LINK_MAP).resolve()))
        for name,expected in record.get('provenance',{}).get('sha256',{}).items():
            source=repo_path(name)
            if str(source.resolve()) in map_files and (not source.is_file() or hashlib.sha256(source.read_bytes()).hexdigest()!=expected):
                raise ValueError('저장 당시와 현재 맵 파일이 다릅니다: '+name)
    if not selected.get('baseline'):
        # An explicitly authored empty control trial is different from a lost
        # collision snapshot. Missing baseline never silently becomes clear.
        if not (record.get('schema')==TRIAL_SCHEMA and selected.get('allow_empty_scene') is True
                and selected.get('baseline')==[] and selected.get('events',[])==[]):
            raise ValueError('재현할 객체의 구간 진입 상태가 없습니다.')
    for key in ('x','y','z','roll','pitch','yaw'):
        value=selected.get('start_pose',{}).get(key)
        if not isinstance(value,(int,float)) or not math.isfinite(value):raise ValueError('재현 ego 위치/방향 값이 유효하지 않습니다.')
    items=selected['baseline']+[e for e in selected.get('events',[]) if e['action']=='spawn']
    for item in items:
        d=item['record']['definition']
        if d.get('type') not in ('static','crossing','vehicle'):
            raise ValueError('지원하지 않는 재현 객체 유형')
        if not isinstance(d.get('model'),str) or not d['model']:raise ValueError('재현 객체 모델이 없습니다.')
        if d['type']=='vehicle' and d.get('method') not in ('direct','native'):raise ValueError('지원하지 않는 재현 차량 생성 방식')
    if selected['environment'].get('weather') not in ('Sunny','Foggy') or selected['environment'].get('hour') not in (11,13,15):
        raise ValueError('저장된 환경 상태가 유효하지 않습니다.')
    return selected,record
