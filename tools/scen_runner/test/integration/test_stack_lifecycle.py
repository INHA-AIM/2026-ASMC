import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
import time
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from scen_runner.orchestration.control import (PlanningClient, RoleLease, alive, atomic_json,
    process_identity, read_json, settings)
from scen_runner.orchestration.process import OwnedProcess
from scen_runner.orchestration.stack import (PlanningServer, configure_nodes, check_conflicts,
                                            guarded_command, planning_configuration)


class StackLifecycleTest(unittest.TestCase):
    def test_resolved_configuration_fingerprint_changes_with_runtime_settings(self):
        config=S(params={'/camera_bbox_lane_change/avoid_clearance':S(value=.05)},
                 nodes=[S(namespace='/',name='camera_bbox_lane_change',package='lane_boundary_adapter',
                          type='camera_bbox_lane_change_node')])
        first=planning_configuration(config)
        self.assertEqual(first['sha256'],planning_configuration(config)['sha256'])
        config.params['/camera_bbox_lane_change/avoid_clearance'].value=.5
        self.assertNotEqual(first['sha256'],planning_configuration(config)['sha256'])
        self.assertEqual(first['parameters']['/camera_bbox_lane_change/avoid_clearance'],.05)

    def test_previous_worker_configuration_is_not_attached_to_a_new_episode(self):
        with tempfile.TemporaryDirectory() as folder:
            client=PlanningClient(settings(),folder)
            atomic_json(Path(folder)/'planning_resolved.json',dict(process=process_identity(),sha256='first'))
            atomic_json(Path(folder)/'server.json',dict(worker_pid=99999999))
            self.assertEqual(client.configuration(),{})
            atomic_json(Path(folder)/'server.json',dict(worker_pid=os.getpid()))
            self.assertEqual(client.configuration()['sha256'],'first')

    def test_controller_with_another_node_name_cannot_share_control_topic(self):
        graph = [[('/ctrl_cmd', ['/external_pid'])], [], []]
        with self.assertRaisesRegex(RuntimeError, 'external_pid'):
            check_conflicts('planning', [S(namespace='/', name='mpc_controller')], graph)

    def test_planning_can_use_persistent_perception_publishers(self):
        graph = [[('/perception/lidar/tracks_map', ['/perception/lidar/lidar_pipeline']), ('/traffic_light/state', ['/traffic_light_infer'])], [], []]
        check_conflicts('planning', [S(namespace='/', name='mpc_controller')], graph)

    def test_two_scenario_generations_wait_for_ready_and_stop_their_own_group(self):
        # Real filesystem handshake; mocked launch cannot contact MORAI/ROS.
        config = settings(); config['stop_timeout_s'] = 2
        probe = Mock(detail='objects 미수신'); probe.evaluate.return_value = False
        groups = []
        def launch():
            group = Mock(); group.observe.return_value = None
            groups.append(group)
            return group
        with tempfile.TemporaryDirectory() as folder:
            server = PlanningServer(config, folder, probe, launch, lambda: 100.)
            server.publish()
            client = PlanningClient(config, folder)
            stopping = threading.Event()
            errors = []
            def serve():
                try:
                    while not stopping.wait(.01): server.tick()
                except Exception as error: errors.append(error)
            thread = threading.Thread(target=serve); thread.start()
            try:
                client.stop()
                self.assertFalse(client.ready(dict(x=10., y=20., yaw=90.), 1))
                time.sleep(.05)
                self.assertFalse(client.ready(dict(x=10., y=20., yaw=90.), 1))
                probe.evaluate.return_value = True
                deadline = time.monotonic()+2
                while not client.ready(dict(x=10., y=20., yaw=90.), 1):
                    self.assertLess(time.monotonic(), deadline); time.sleep(.01)
                first = client.generation
                client.check(); client.stop()
                groups[0].stop.assert_called_once()
                before = read_json(Path(folder)/'request.json')['id']
                client.close()
                self.assertEqual(read_json(Path(folder)/'request.json')['id'], before)
                probe.evaluate.return_value = False
                self.assertFalse(client.ready(dict(x=30., y=40., yaw=0.), 2))
                self.assertNotEqual(first, client.pending['id'])
                time.sleep(.05)
                self.assertFalse(client.ready(dict(x=30., y=40., yaw=0.), 2))
                self.assertEqual(probe.reset.call_args.args[0]['x'], 30.)
                client.stop()
                self.assertEqual(len(groups), 2)
                groups[1].stop.assert_called_once()
            finally:
                stopping.set(); thread.join(2)
            self.assertFalse(errors)

    def test_old_reply_cannot_acknowledge_new_request(self):
        with tempfile.TemporaryDirectory() as folder:
            client = PlanningClient(settings(), folder)
            atomic_json(Path(folder)/'server.json', dict(process=process_identity(), heartbeat=time.time()))
            client._request('start', pose={}, episode=2)
            atomic_json(Path(folder)/'reply.json', dict(id='previous', client=client.client, phase='ready'))
            self.assertEqual(client._reply(), {})

    def test_dead_or_stale_server_is_not_accepted_as_ready(self):
        with tempfile.TemporaryDirectory() as folder:
            client = PlanningClient(settings(), folder)
            for identity, heartbeat in ((dict(pid=99999999, started='0'), time.time()),
                                        (process_identity(), time.time()-100)):
                atomic_json(Path(folder)/'server.json', dict(process=identity, heartbeat=heartbeat))
                with self.assertRaisesRegex(ConnectionError, '응답 없음'): client._server()

    def test_missing_stop_ack_has_a_bounded_timeout_even_with_fixed_wall_clock(self):
        with tempfile.TemporaryDirectory() as folder:
            config = settings(); config['stop_timeout_s'] = .05
            atomic_json(Path(folder)/'server.json', dict(process=process_identity(), heartbeat=100))
            client = PlanningClient(config, folder, clock=lambda: 100)
            started = time.monotonic()
            with self.assertRaisesRegex(ConnectionError, '시간 초과'): client.stop()
            self.assertLess(time.monotonic()-started, .5)

    def test_failed_launch_is_reported_and_no_ready_reply_is_emitted(self):
        with tempfile.TemporaryDirectory() as folder:
            server = PlanningServer(settings(), folder, Mock(), Mock(side_effect=RuntimeError('launch failed')), lambda: 100)
            request = dict(id='one', client='scene', process=process_identity(),
                           created=time.time(), operation='start', pose={}, episode=1)
            atomic_json(Path(folder)/'request.json', request)
            server.tick()
            self.assertEqual(read_json(Path(folder)/'reply.json')['phase'], 'error')
            self.assertIsNone(server.group)

    def test_owned_worker_exit_or_scenario_death_stops_remaining_children(self):
        for failure in ('worker', 'scenario'):
            with tempfile.TemporaryDirectory() as folder:
                probe = Mock(detail='waiting'); probe.evaluate.return_value = False
                group = Mock(); group.observe.return_value = None
                server = PlanningServer(settings(), folder, probe, lambda: group, lambda: 100)
                request = dict(id='one', client='scene', process=process_identity(),
                    created=time.time(), operation='start', pose={}, episode=1)
                atomic_json(Path(folder)/'request.json', request); server.tick()
                if failure == 'worker': group.observe.return_value = 1
                else: server.active['process'] = dict(pid=99999999, started='0')
                server.tick()
                group.stop.assert_called_once()
                self.assertEqual(server.phase, 'error')

    def test_duplicate_supervisor_cannot_take_over_role_lock(self):
        with tempfile.TemporaryDirectory() as folder:
            first = RoleLease(folder, 'planning')
            try:
                with self.assertRaisesRegex(RuntimeError, '이미 실행'): RoleLease(folder, 'planning')
            finally: first.close()
            replacement = RoleLease(folder, 'planning'); replacement.close()

    def test_dead_parent_guard_cleans_up_real_owned_worker(self):
        from unittest.mock import patch
        owned = OwnedProcess([sys.executable, '-c', 'import time;time.sleep(60)'])
        try:
            with patch('scen_runner.orchestration.stack.OwnedProcess', return_value=owned):
                result = guarded_command([], dict(pid=99999999, started='0'), [False])
            self.assertEqual(result, 0)
            self.assertIsNotNone(owned.child.poll())
        finally:
            owned.stop(durations=(.1, .1, 1))

    def test_gpu_prefix_applies_to_all_three_models_and_not_cpu_postprocessing(self):
        with tempfile.TemporaryDirectory() as folder:
            (Path(folder)/'model').mkdir(); (Path(folder)/'model/model.py').touch()
            config = settings(); config.update(gpu_python=sys.executable, twinlite_root=folder, camera_runtime='off')
            nodes = [S(type=kind, launch_prefix=None, required=False, respawn=False, env_args=[]) for kind in
                     ('semantic_inference_node.py', 'object_detection_node.py', 'traffic_light_node.py', 'camera_bev_node.py')]
            launch = S(nodes=nodes, params={'/camera_semantic_inference/twinlite_root': S(value='missing')})
            configure_nodes(config, launch)
            self.assertEqual([node.launch_prefix for node in nodes[:3]], [sys.executable]*3)
            self.assertIsNone(nodes[3].launch_prefix)
            self.assertEqual(launch.params['/camera_semantic_inference/twinlite_root'].value, folder)

    def test_process_cleanup_handles_child_session_without_touching_unrelated_process(self):
        with tempfile.TemporaryDirectory() as folder:
            marker = Path(folder)/'child.pid'
            code = '''import pathlib,signal,subprocess,sys,time
signal.signal(signal.SIGINT,signal.SIG_IGN)
p=subprocess.Popen([sys.executable,'-c','import signal,time; signal.signal(signal.SIGINT,signal.SIG_IGN); time.sleep(60)'],start_new_session=True)
pathlib.Path(sys.argv[1]).write_text(str(p.pid))
time.sleep(60)
'''
            unrelated = subprocess.Popen([sys.executable, '-c', 'import time;time.sleep(60)'])
            owned = OwnedProcess([sys.executable, '-c', code, str(marker)])
            try:
                deadline = time.monotonic()+3
                while not marker.exists():
                    self.assertLess(time.monotonic(), deadline); time.sleep(.01)
                child = process_identity(int(marker.read_text()))
                owned.observe(); owned.stop(durations=(.1, .1, 1))
                self.assertFalse(alive(child))
                self.assertIsNone(unrelated.poll())
            finally:
                owned.stop(durations=(.1, .1, 1))
                unrelated.terminate(); unrelated.wait()


if __name__ == '__main__':
    unittest.main()
