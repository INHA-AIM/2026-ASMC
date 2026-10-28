import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

SOURCE = Path(__file__).resolve().parents[2] / 'src/asmc_camera/src/asmc_camera/camera/backends/object_engine_selection.py'
spec = importlib.util.spec_from_file_location('object_selection', SOURCE)
selection = importlib.util.module_from_spec(spec)
spec.loader.exec_module(selection)


class ObjectSelectionTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.engine = self.root / 'local.engine'
        self.engine.write_bytes(b'validated engine')
        self.digest = hashlib.sha256(self.engine.read_bytes()).hexdigest()
        self.profile = self.root / 'camera_runtime.json'
        self.settings = dict(schema_version=1, source_engine_sha256='a'*64,
                             gpu_name='4060 Ti', tensorrt='10.13', engine_path=str(self.engine),
                             engine_sha256=self.digest,
                             validation=dict(passed=True, engine_sha256=self.digest))
        self.provenance = dict(engine_sha256=self.digest, tensorrt='10.13',
                               head_profile='mission15_one2one_nms')
        self.write()

    def write(self):
        self.profile.write_text(json.dumps(self.settings))
        self.engine.with_suffix('.engine.provenance.json').write_text(json.dumps(self.provenance))

    def select(self, source='a'*64, gpu='4060 Ti', trt='10.13'):
        return selection.select_object_engine('team.engine', source, gpu, trt,
                                              'mission15_one2one_nms', self.profile)

    def test_plain_runtime_selects_validated_export(self):
        self.assertEqual(self.select(), (self.engine, self.digest))

    def test_absent_profile_keeps_packaged_engine(self):
        self.profile.unlink()
        self.assertEqual(self.select(), (Path('team.engine'), 'a'*64))

    def test_other_model_is_not_overridden(self):
        self.assertEqual(self.select(source='b'*64), (Path('team.engine'), 'b'*64))

    def test_other_gpu_or_runtime_is_rejected(self):
        for values in [dict(gpu='4060'), dict(trt='10.14')]:
            with self.subTest(values=values), self.assertRaises(RuntimeError):
                self.select(**values)

    def test_modified_engine_is_rejected(self):
        self.engine.write_bytes(b'changed')
        with self.assertRaises(RuntimeError):
            self.select()

    def test_failed_smoke_is_rejected(self):
        self.settings['validation']['passed'] = False
        self.write()
        with self.assertRaises(RuntimeError):
            self.select()

    def test_wrong_head_is_rejected(self):
        self.provenance['head_profile'] = 'mission16_one2one_nms'
        self.write()
        with self.assertRaises(RuntimeError):
            self.select()


if __name__ == '__main__':
    unittest.main()
