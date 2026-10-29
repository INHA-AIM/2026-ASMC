"""Index saved collision/stagnation cases without simulator access."""
import json
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

from scen_runner.records.collision_archive import SAVE_ROOT,SCHEMA,STAGNATION_SCHEMA,TRIAL_SCHEMA,VERSION,load_collision,saved_event


@dataclass(frozen=True)
class ReplayCase:
    path: Path
    section: str
    time_kst: str
    episode: int
    models: str
    weather: str
    hour: int
    event_type: str='collision'

    def load(self):
        # Full validation, including map hashes, precedes any simulator mutation.
        return load_collision(self.path,self.section)[0]


@dataclass(frozen=True)
class ReplayRun:
    folder: Path
    started: str
    cases: tuple

    @property
    def label(self):
        stamp=datetime.fromisoformat(self.started).strftime('%Y-%m-%d %H:%M:%S')
        collisions=sum(c.event_type=='collision' for c in self.cases)
        trials=sum(c.event_type=='trial' for c in self.cases)
        stagnations=len(self.cases)-collisions-trials
        counts=f'충돌 {collisions}건' if not stagnations else f'충돌 {collisions}건 · 정체 {stagnations}건'
        return f'{stamp} · {counts}'+(f' · 장면 {trials}건' if trials else '')


def saved_runs(root=SAVE_ROOT):
    """Newest nonempty run first; cases ordered by recorded time, not names."""
    root=Path(root)
    if not root.is_dir():raise ValueError('저장된 충돌 기록/정체 기록이 없습니다: '+str(root))
    runs=[];warnings=[]
    for folder in root.iterdir():
        if not folder.is_dir():continue
        cases=[];started=None
        for path in folder.glob('*.json'):
            if path.name=='run.json':continue
            try:
                record=json.loads(path.read_text(encoding='utf-8'))
                if record.get('schema') not in (SCHEMA,STAGNATION_SCHEMA,TRIAL_SCHEMA) or record.get('version')!=VERSION:
                    raise ValueError('지원하지 않는 기록 형식')
                if not record.get('replay_available') or not record.get('replay'):continue
                kind,event=saved_event(record);stamp=event['time']['kst']
                datetime.fromisoformat(stamp)
                run_start=record['run_started']['kst'];datetime.fromisoformat(run_start)
                ids=set(event.get('counterpart_ids',()));environment=event['environment']
                for section,definition in sorted(record['replay'].items()):
                    if not definition.get('baseline') and not (record.get('schema')==TRIAL_SCHEMA and definition.get('allow_empty_scene') is True):continue
                    models=sorted({r['model'] for r in record.get('all_section_settings',())
                                   if r.get('section')==section and (kind!='collision' or r.get('native_id') in ids)})
                    cases.append(ReplayCase(path,section,stamp,record['episode'],', '.join(models),
                                            environment.get('weather'),environment.get('hour'),kind))
                started=run_start if started is None else min(started,run_start)
            except (OSError,ValueError,TypeError,KeyError,AttributeError) as exc:
                warnings.append(f'{path.name}: {exc}')
        if cases:
            cases.sort(key=lambda case:(datetime.fromisoformat(case.time_kst),case.episode,case.path.name,case.section))
            runs.append(ReplayRun(folder,started,tuple(cases)))
    runs.sort(key=lambda run:(datetime.fromisoformat(run.started),run.folder.name),reverse=True)
    if not runs:raise ValueError('재현 가능한 충돌 기록/정체 기록이 없습니다: '+str(root))
    return tuple(runs),warnings
