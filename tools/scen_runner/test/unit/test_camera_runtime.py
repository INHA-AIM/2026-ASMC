import copy
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as S
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from scen_runner.orchestration.camera_runtime import configure_camera_runtime


class CameraRuntimeTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        (self.root/'.runtime').mkdir()
        (self.root/'python').mkdir()
        (self.root/'engine').write_bytes(b'test')
        self.profile = dict(schema_version=1, source_engine_sha256='a'*64,
                            engine_sha256='b'*64, engine_path='engine', runtime_pythonpath='python',
                            validation=dict(passed=True, engine_sha256='b'*64))
        self.node = S(type='object_detection_node.py', namespace='/', name='camera_object_detector',
                      env_args=[('PYTHONPATH', '/existing/ros'), ('KEEP', 'yes')])
        self.other = S(type='traffic_light_node.py', env_args=[])
        values = dict(weights='team.engine', expected_sha256='a'*64, runtime_backend='tensorrt',
                      conf_thres=.4, nms_iou=.7, target_hz=15., front_width=1280, batch_mode='2plus1')
        self.launch = S(nodes=[self.node, self.other],
                        params={'/camera_object_detector/'+k: S(value=v) for k,v in values.items()})

    def write_profile(self):
        (self.root/'.runtime/camera_runtime.json').write_text(json.dumps(self.profile))

    def test_verified_engine_keeps_team_thresholds_and_other_node_environments(self):
        self.write_profile()
        before = {k: v.value for k,v in self.launch.params.items()}
        configure_camera_runtime({}, self.launch, self.root)
        after = {k: v.value for k,v in self.launch.params.items()}
        changed = {k for k in before if before[k] != after[k]}
        self.assertEqual(changed, {'/camera_object_detector/weights', '/camera_object_detector/expected_sha256'})
        self.assertEqual(after['/camera_object_detector/weights'], str(self.root/'engine'))
        self.assertEqual(dict(self.node.env_args)['PYTHONPATH'], str(self.root/'python')+':/existing/ros')
        self.assertEqual(dict(self.node.env_args)['KEEP'], 'yes')
        self.assertEqual(self.other.env_args, [])

    def test_other_hosts_and_explicit_off_keep_upstream_configuration(self):
        before = copy.deepcopy(self.launch)
        configure_camera_runtime({}, self.launch, self.root)
        self.assertEqual(self.launch, before)
        self.write_profile()
        configure_camera_runtime({'camera_runtime': 'off'}, self.launch, self.root)
        self.assertEqual(self.launch, before)

    def test_new_team_model_cannot_silently_run_the_old_local_engine(self):
        self.write_profile()
        self.launch.params['/camera_object_detector/expected_sha256'].value = 'c'*64
        before = copy.deepcopy(self.launch)
        with self.assertRaisesRegex(ValueError, '모델 설정이 변경'):
            configure_camera_runtime({}, self.launch, self.root)
        self.assertEqual(self.launch, before)

    def test_unverified_engine_or_missing_runtime_is_rejected(self):
        for validation, dependencies in ((dict(passed=False, engine_sha256='b'*64), 'python'),
                                         (dict(passed=True, engine_sha256='c'*64), 'python'),
                                         (dict(passed=True, engine_sha256='b'*64), 'missing')):
            self.profile.update(validation=validation, runtime_pythonpath=dependencies)
            self.write_profile()
            with self.assertRaises(ValueError):
                configure_camera_runtime({}, self.launch, self.root)


if __name__ == '__main__':
    unittest.main()
