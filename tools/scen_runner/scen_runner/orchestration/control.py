"""Bounded, correlated local requests; the scenario itself has no ROS dependency."""
import fcntl
import json
import os
from pathlib import Path
import re
import time
import uuid

from scen_runner.config import TOOL_ROOT, load_yaml


class PlanningUnavailable(ConnectionError):
    connection_name = '판제 실행기'


def settings(path=None):
    data = load_yaml(Path(path) if path else TOOL_ROOT/'config/stack.yaml')
    if not re.fullmatch(r'[a-zA-Z0-9_-]+', str(data.get('instance', ''))):
        raise ValueError('stack instance must contain only letters, numbers, _ or -')
    for name in ('startup_timeout_s', 'stop_timeout_s', 'heartbeat_timeout_s', 'client_timeout_s'):
        value = float(data[name])
        if not 0 < value < 600:
            raise ValueError(f'invalid stack {name}')
        data[name] = value
    for name, value in data['readiness'].items():
        value = float(value)
        if not 0 < value < 600:
            raise ValueError(f'invalid readiness {name}')
        data['readiness'][name] = value
    return data


def runtime_dir(config):
    return TOOL_ROOT/'.runtime'/'stack'/config['instance']


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name+'.'+uuid.uuid4().hex+'.tmp')
    try:
        temporary.write_text(json.dumps(value, ensure_ascii=False), encoding='utf-8')
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


def read_json(path):
    try:
        return json.loads(Path(path).read_text(encoding='utf-8'))
    except (OSError, ValueError):
        return {}


def process_identity(pid=None):
    pid = os.getpid() if pid is None else int(pid)
    try:
        # comm can contain spaces/parentheses. Field 22 follows its final ')'.
        fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
        if fields[0] == 'Z':
            return None
        return dict(pid=pid, started=fields[19])
    except (OSError, IndexError):
        return None


def alive(identity):
    return bool(identity and process_identity(identity['pid']) == identity)


class RoleLease:
    def __init__(self, directory, role):
        Path(directory).mkdir(parents=True, exist_ok=True)
        self.stream = open(Path(directory)/(role+'.lock'), 'a+')
        try:
            fcntl.flock(self.stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            self.stream.close()
            raise RuntimeError(f'{role} 실행기가 이미 실행 중입니다.')

    def close(self):
        if not self.stream.closed:
            fcntl.flock(self.stream, fcntl.LOCK_UN)
            self.stream.close()


class PlanningClient:
    def __init__(self, config=None, directory=None, clock=time.time):
        self.config = settings() if config is None else config
        self.directory = runtime_dir(self.config) if directory is None else Path(directory)
        self.clock = clock
        self.identity = process_identity()
        self.client = uuid.uuid4().hex
        self.pending = None
        self.wait_started = None
        self.generation = None
        self.engaged = False
        self.stopped = False
        self.status = '판제 실행기 대기'

    def _server(self):
        state = read_json(self.directory/'server.json')
        age = self.clock()-state.get('heartbeat', 0)
        if not alive(state.get('process')) or not 0 <= age <= self.config['heartbeat_timeout_s']:
            raise PlanningUnavailable('판제 실행기 응답 없음 · run_planning.sh를 먼저 실행하세요.')
        return state

    def _request(self, operation, **payload):
        self._server()
        self.pending = dict(id=uuid.uuid4().hex, client=self.client, process=self.identity,
                            operation=operation, created=self.clock(), **payload)
        atomic_json(self.directory/'request.json', self.pending)
        self.wait_started = time.monotonic()
        self.engaged = True
        self.stopped = False
        return self.pending

    def _reply(self):
        self._server()
        reply = read_json(self.directory/'reply.json')
        if reply.get('id') != self.pending['id'] or reply.get('client') != self.client:
            return {}
        if reply.get('phase') == 'error':
            raise RuntimeError('판제 연동 실패 · '+reply.get('detail', '원인 미수신'))
        self.status = reply.get('detail', '판제 입력 준비 중')
        return reply

    def stop(self):
        if self.stopped:
            return
        self._request('stop')
        deadline = time.monotonic()+self.config['stop_timeout_s']
        while time.monotonic() < deadline:
            if self._reply().get('phase') == 'stopped':
                self.pending = None
                self.generation = None
                self.stopped = True
                return
            time.sleep(.05)
        raise PlanningUnavailable('판제 종료 확인 시간 초과 · ego 정지 유지')

    def close(self):
        if self.engaged and not self.stopped:
            self.stop()

    def ready(self, pose, episode):
        if self.pending is None:
            self._request('start', pose=dict(pose), episode=episode)
        if self.pending['operation'] != 'start':
            raise RuntimeError('판제 준비 요청 순서 오류')
        reply = self._reply()
        if reply.get('phase') == 'ready':
            self.generation = self.pending['id']
            self.check()
            return True
        if time.monotonic()-self.wait_started > self.config['startup_timeout_s']:
            raise RuntimeError('판제 준비 시간 초과 · '+self.status)
        return False

    def check(self):
        state = self._server()
        if state.get('generation') != self.generation or state.get('phase') != 'ready':
            raise RuntimeError('판제 실행 중단 · ego 정지 유지 · '+state.get('detail', '응답 불일치'))
        atomic_json(self.directory/'client.json', dict(client=self.client, process=self.identity,
                    generation=self.generation, heartbeat=self.clock()))

    def snapshot(self):
        state=read_json(self.directory/'server.json')
        age=self.clock()-state.get('heartbeat',0)
        online=alive(state.get('process')) and 0<=age<=self.config['heartbeat_timeout_s']
        return dict(enabled=True,online=bool(online),phase=state.get('phase','unknown') if online else 'offline',
                    worker_pid=state.get('worker_pid') if online else None,
                    detail=state.get('detail','판제 실행기 응답 대기') if online else '판제 실행기 응답 없음')

    def configuration(self):
        resolved=read_json(self.directory/'planning_resolved.json')
        worker=read_json(self.directory/'server.json').get('worker_pid')
        identity=resolved.get('process')
        if not identity or identity.get('pid')!=worker or not alive(identity):
            return {}
        return resolved

    def recovery_ready(self):
        state=self._server()
        return (state.get('generation')==self.generation and state.get('phase')=='ready' and
                state.get('selected_mode')==0)
