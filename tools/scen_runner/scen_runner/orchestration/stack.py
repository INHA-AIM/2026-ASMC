"""Four process groups; all source/config changes stay in scen_runner."""
import argparse
import http.client
import json
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import threading
import time
import xmlrpc.client

from scen_runner.config import REPO_ROOT, TOOL_ROOT
from .control import RoleLease, alive, atomic_json, process_identity, read_json, runtime_dir, settings
from .process import OwnedProcess
from .readiness import Readiness
from .camera_runtime import configure_camera_runtime
from .ros_registry import prune_stale_planning_nodes


GPU_NODES = {'semantic_inference_node.py', 'object_detection_node.py', 'traffic_light_node.py'}


class MasterTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        return http.client.HTTPConnection(host, timeout=2)


def master_graph():
    uri = os.environ.get('ROS_MASTER_URI', 'http://127.0.0.1:11311')
    try:
        with xmlrpc.client.ServerProxy(uri, transport=MasterTransport()) as master:
            code, detail, groups = master.getSystemState('/scen_runner_stack_check')
            if code != 1:
                raise RuntimeError(detail)
            return groups
    except (OSError, xmlrpc.client.Error) as error:
        raise RuntimeError('ROS master 응답 없음 · run_bridge.sh를 먼저 실행하세요.') from error


def master_nodes():
    return {name for rows in master_graph() for _, names in rows for name in names}


def check_conflicts(role, nodes, graph):
    existing = {name for rows in graph for _, names in rows for name in names}
    conflicts = sorted(node.namespace+node.name for node in nodes
                       if node.namespace+node.name in existing)
    topics = ({'/ctrl_cmd', '/planning/vehicle/state', '/planning/objects', '/planning/trajectory'}
              if role == 'planning' else
              {'/perception/camera/objects', '/perception/lane/boundary_family',
               '/traffic_light/state', '/last_lidar', '/YOLO/BBox',
               '/migration/lidar/objects_local', '/migration/lidar/planning_observations_map'})
    conflicts += [topic+' ('+', '.join(names)+')' for topic, names in graph[0] if topic in topics and names]
    if conflicts:
        raise RuntimeError('중복 노드/발행자가 이미 실행 중입니다: '+', '.join(conflicts))


def select_python(config):
    choice = config['gpu_python']
    if choice == 'auto':
        choice = os.environ.get('ASMC_YOLO_PYTHON', '/usr/bin/python3')
    path = Path(choice)
    if not path.is_file() or not os.access(path, os.X_OK):
        raise ValueError('GPU Python 실행 파일 없음: '+str(path))
    return str(path)


def configure_nodes(config, launch_config):
    """roslaunch resolves the original includes/params before these overrides."""
    python = select_python(config)
    for node in launch_config.nodes:
        if node.type in GPU_NODES:
            node.launch_prefix = shlex.quote(python)
        # A failed input/model must end this owned group instead of leaving
        # the remaining nodes silently alive with incomplete inputs.
        node.required = True
        node.respawn = False
    key = '/camera_semantic_inference/twinlite_root'
    if key in launch_config.params:
        choice = config.get('twinlite_root', 'auto')
        if choice == 'auto':
            candidates = (Path(launch_config.params[key].value), TOOL_ROOT/'.runtime/TwinLiteNetPlus')
            choice = next((str(path) for path in candidates if (path/'model/model.py').is_file()), '')
        if not choice or not (Path(choice)/'model/model.py').is_file():
            raise ValueError('TwinLiteNet+ 소스 없음 · README의 인지 실행 준비 절차를 확인하세요.')
        launch_config.params[key].value = str(Path(choice).resolve())
    configure_camera_runtime(config, launch_config)


def launch_configuration(role, config):
    import roslaunch.config
    import roslaunch.rlutil
    if role == 'bridge':
        file = REPO_ROOT/'src/asmc_bridge/launch/bridge.launch'
    else:
        file = TOOL_ROOT/'launch'/f'{role}.launch'
    # None preserves ROS_MASTER_URI, including isolated verification masters.
    launch_files = [str(file)] if role == 'bridge' else [(str(file),
        ['lidar_backend:=' + config.get('lidar_backend', 'main')])]
    launch_config = roslaunch.config.load_config_default(launch_files, None)
    if role == 'perception':
        configure_nodes(config, launch_config)
    else:
        for node in launch_config.nodes:
            node.required = True
            node.respawn = False
    import roslib.packages
    import rospkg
    rospack = rospkg.RosPack()
    for node in launch_config.nodes:
        if not roslib.packages.find_node(node.package, node.type, rospack=rospack):
            raise RuntimeError(f'실행 파일 없음: {node.package}/{node.type} · README의 빌드 절차를 확인하세요.')
    return launch_config


def guarded_command(command, owner, stopping):
    group = OwnedProcess(command)
    try:
        while not stopping[0] and (owner is None or alive(owner)):
            result = group.observe()
            if result is not None:
                return result
            time.sleep(.1)
        return 0
    finally:
        group.stop()


def worker(role, config, check=False, owner=None):
    stopping = [False]
    def stop(*_):
        stopping[0] = True
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, stop)
    if role == 'bridge' and not check:
        return guarded_command(['bash', str(REPO_ROOT/'scripts/udp_bridge.sh'), '--transport', 'local'], owner, stopping)
    import roslaunch
    import roslaunch.parent
    launch_config = launch_configuration(role, config)
    if check:
        print(json.dumps([dict(name=node.namespace+node.name, package=node.package,
                              executable=node.type, python=node.launch_prefix)
                          for node in launch_config.nodes], ensure_ascii=False, indent=2))
        return 0
    graph = master_graph()
    if role == 'planning':
        # A SIGTERM during node initialization can leave ROS registrations
        # behind even after OwnedProcess has confirmed all children exited.
        prune_stale_planning_nodes(launch_config.nodes, graph)
        graph = master_graph()
    check_conflicts(role, launch_config.nodes, graph)
    run_id = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(run_id)
    parent = roslaunch.parent.ROSLaunchParent(run_id, [], sigint_timeout=3, sigterm_timeout=2)
    # Noetic's start() uses a supplied config as-is. Includes/parameters above
    # are loaded by the same ROS loader as the team's ordinary roslaunch.
    parent.config = launch_config
    try:
        parent.start()
        while not stopping[0] and not parent.pm.is_shutdown and (owner is None or alive(owner)):
            parent.spin_once()
            time.sleep(.1)
        return 0 if stopping[0] else 1
    finally:
        parent.shutdown()


def child_command(role, config_path):
    command = [sys.executable, '-m', 'scen_runner.orchestration.stack', 'worker', '--role', role,
               '--owner', json.dumps(process_identity())]
    if config_path:
        command += ['--stack-config', str(config_path)]
    return command


class PlanningServer:
    def __init__(self, config, directory, probe, launch, ros_clock, clock=time.time):
        self.config, self.directory, self.probe = config, Path(directory), probe
        self.launch, self.ros_clock, self.clock = launch, ros_clock, clock
        self.process = process_identity()
        self.group = None
        self.active = None
        self.last_request = None
        self.phase = 'stopped'
        self.detail = '시나리오 요청 대기'

    def reply(self):
        self.publish()
        if self.active:
            atomic_json(self.directory/'reply.json', dict(id=self.active['id'], client=self.active['client'],
                        phase=self.phase, detail=self.detail))

    def publish(self):
        identity=getattr(self.group,'identity',None)
        worker_pid=identity.get('pid') if isinstance(identity,dict) else None
        atomic_json(self.directory/'server.json', dict(process=self.process, heartbeat=self.clock(),
                    phase=self.phase, detail=self.detail,
                    worker_pid=worker_pid,
                    generation=self.active['id'] if self.active and self.active['operation'] == 'start' else None))

    def stop(self):
        if self.group:
            identity=getattr(self.group,'identity',None)
            worker_pid=identity.get('pid') if isinstance(identity,dict) else None
            self.group.stop()
            self.group = None
            print(f'[판제 종료 확인] PID={worker_pid} · 판제·MPC 노드 정리 완료',flush=True)
        self.phase = 'stopped'
        self.detail = '판제·MPC·차량 상태 노드 종료 확인'

    def tick(self):
        request = read_json(self.directory/'request.json')
        if request and request.get('id') != self.last_request:
            self.last_request = request.get('id')
            age = self.clock()-request.get('created', 0)
            if not alive(request.get('process')) or not 0 <= age <= self.config['startup_timeout_s']:
                self.publish()
                return
            if (self.active and self.group and request.get('client') != self.active['client'] and
                    alive(self.active.get('process'))):
                atomic_json(self.directory/'reply.json', dict(id=request['id'], client=request['client'],
                    phase='error', detail='다른 시나리오가 판제 실행기를 사용 중입니다.'))
                return
            try:
                self.stop()
                self.active = request
                if request['operation'] == 'start':
                    self.probe.reset(request['pose'], self.ros_clock())
                    self.group = self.launch()
                    self.phase = 'warming'
                    self.detail = '새 회차 판제·MPC 시작 · 입력 대기'
                    identity=getattr(self.group,'identity',None)
                    worker_pid=identity.get('pid') if isinstance(identity,dict) else None
                    print(f'[판제 재시작] 회차 {request["episode"]} · PID={worker_pid} · 새 입력 준비 중', flush=True)
                elif request['operation'] != 'stop':
                    raise ValueError('알 수 없는 판제 요청')
            except Exception as error:
                self.phase = 'error'
                self.detail = str(error)
            self.reply()
        if self.group:
            try:
                if self.group.observe() is not None:
                    raise RuntimeError('판제 프로세스 종료 · 위 노드 오류 로그를 확인하세요.')
                if not alive(self.active.get('process')):
                    raise RuntimeError('시나리오 프로세스 종료 · 판제 정리')
                if self.phase == 'ready':
                    heartbeat = read_json(self.directory/'client.json')
                    if (heartbeat.get('generation') != self.active['id'] or
                            self.clock()-heartbeat.get('heartbeat', 0) > self.config['client_timeout_s']):
                        if self.clock()-self.ready_at > self.config['client_timeout_s']:
                            raise RuntimeError('시나리오 응답 중단 · 판제 정리')
                elif self.phase == 'warming':
                    if self.clock()-self.active['created'] > self.config['startup_timeout_s']:
                        raise RuntimeError('준비 시간 초과 · '+self.probe.detail)
                    self.detail = self.probe.detail
                    if self.probe.evaluate(time.monotonic(), self.ros_clock()):
                        self.phase = 'ready'
                        self.ready_at = self.clock()
                        self.detail = self.probe.detail
                        print('[판제 준비 완료] '+self.detail, flush=True)
                    self.reply()
            except Exception as error:
                self.stop()
                self.phase = 'error'
                self.detail = str(error)
                print('[판제 중단] '+self.detail, flush=True)
                self.reply()
        self.publish()


class RosProbe(Readiness):
    def __init__(self, config):
        self.lock = threading.RLock()
        super().__init__(config)
        import rospy
        from asmc_msgs.msg import Trajectory, VehicleState
        from planning_inputs.msg import DetectedObjectArray
        from morai_msgs.msg import CtrlCmd
        self.subscribers = []
        for name, topic, kind in (
                ('state', '/planning/vehicle/state', VehicleState),
                ('objects', '/planning/objects', DetectedObjectArray),
                ('trajectory', '/planning/trajectory', Trajectory),
                ('control', '/ctrl_cmd', CtrlCmd)):
            self.subscribers.append(rospy.Subscriber(topic, kind,
                lambda message, key=name: self.observe(key, message, time.monotonic()), queue_size=1))

    def reset(self, pose, cutoff):
        with self.lock:
            super().reset(pose, cutoff)

    def observe(self, name, message, received):
        with self.lock:
            super().observe(name, message, received)

    def evaluate(self, now, ros_now):
        with self.lock:
            return super().evaluate(now, ros_now)


def main(argv=None):
    parser = argparse.ArgumentParser(description='브릿지·인지·판제 실행 그룹')
    parser.add_argument('role', choices=['bridge', 'perception', 'planning', 'worker'])
    parser.add_argument('--role', dest='worker_role', choices=['bridge', 'perception', 'planning'])
    parser.add_argument('--stack-config', type=Path)
    parser.add_argument('--check', action='store_true', help='launch 구성만 확인; ROS 노드/제어 실행 없음')
    parser.add_argument('--owner', type=json.loads, help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    config = settings(args.stack_config)
    role = args.worker_role if args.role == 'worker' else args.role
    if not role:
        parser.error('worker requires --role')
    if args.role == 'worker' or args.check:
        return worker(role, config, args.check, args.owner)
    directory = runtime_dir(config)
    lease = RoleLease(directory, role)
    stopping = [False]
    def stop(*_):
        stopping[0] = True
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, stop)
    group = None
    server = None
    try:
        if role == 'planning':
            master_nodes()
            import rospy
            rospy.init_node('scen_runner_planning_supervisor', disable_signals=True)
            probe = RosProbe(config)
            server = PlanningServer(config, directory, probe,
                lambda: OwnedProcess(child_command(role, args.stack_config)), lambda: rospy.Time.now().to_sec())
            print('[판제 실행기] 시나리오 리스폰 요청 대기 · '+str(directory), flush=True)
            while not stopping[0] and not rospy.is_shutdown():
                server.tick()
                time.sleep(.1)
        else:
            if role == 'bridge':
                try:
                    existing = master_nodes()
                except RuntimeError:
                    existing = set()
                if existing & {'/morai_udp_bridge', '/morai_sensor_bridge'}:
                    raise RuntimeError('기존 UDP 브릿지가 이미 실행 중입니다. 그 터미널에서 종료 후 실행하세요.')
                group = OwnedProcess(child_command(role, args.stack_config))
            else:
                master_nodes()
                group = OwnedProcess(child_command(role, args.stack_config))
            while not stopping[0]:
                result = group.observe()
                if result is not None:
                    return result
                time.sleep(.1)
        return 0
    finally:
        try:
            if group:
                group.stop()
            if server:
                server.stop()
                server.publish()
        finally:
            lease.close()


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except Exception as error:
        print('[실행기 오류] '+str(error), file=sys.stderr, flush=True)
        raise SystemExit(1)
