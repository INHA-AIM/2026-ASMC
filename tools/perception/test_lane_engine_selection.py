import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

SOURCE = Path(__file__).resolve().parents[2] / 'src/asmc_camera/src/asmc_camera/camera/backends/engine_selection.py'
spec = importlib.util.spec_from_file_location('lane_engine_selection', SOURCE)
selection = importlib.util.module_from_spec(spec)
spec.loader.exec_module(selection)
builder_spec = importlib.util.spec_from_file_location(
    'prepare_local_lane_engine', Path(__file__).with_name('prepare_local_lane_engine.py'))
builder = importlib.util.module_from_spec(builder_spec)
builder_spec.loader.exec_module(builder)


class EngineSelectionTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.manifest = self.root / 'packaged.json'
        self.sha = 'a' * 64
        self.original = dict(checkpoint_sha256=self.sha, gpu_name='4060 Ti', tensorrt='10.13', completed=True)
        self.manifest.write_text(json.dumps(self.original))

    def tearDown(self):
        self.tmp.cleanup()

    def select(self, gpu='4060', trt='10.13'):
        return selection.select_engine_manifest(self.manifest, gpu, trt, self.root / 'cache')

    def local(self, **overrides):
        directory = selection.local_engine_directory(self.sha, '4060', '10.13', self.root / 'cache')
        directory.mkdir(parents=True)
        m = dict(self.original, gpu_name='4060')
        m.update(overrides)
        p = directory / 'manifest.json'
        p.write_text(json.dumps(m))
        return p

    def test_compatible_packaged_model_is_unchanged(self):
        self.assertEqual(self.select('4060 Ti'), self.manifest)

    def test_local_model_matches_gpu_and_runtime(self):
        p = self.local()
        self.assertEqual(self.select(), p)
        self.assertEqual(json.loads(self.manifest.read_text()), self.original)

    def test_absent_local_model_gives_exact_preparation_command(self):
        with self.assertRaisesRegex(RuntimeError, 'prepare_local_lane_engine.py'):
            self.select()

    def test_incomplete_model_is_never_selected(self):
        self.local(completed=False)
        with self.assertRaises(RuntimeError):
            self.select()

    def test_other_checkpoint_is_never_selected(self):
        self.local(checkpoint_sha256='b' * 64)
        with self.assertRaises(RuntimeError):
            self.select()

    def test_gpu_and_runtime_have_distinct_cache_keys(self):
        a = selection.local_engine_directory(self.sha, '4060', '10.13', self.root)
        self.assertNotEqual(a, selection.local_engine_directory(self.sha, '4060 Ti', '10.13', self.root))
        self.assertNotEqual(a, selection.local_engine_directory(self.sha, '4060', '10.14', self.root))

    def test_invalid_checkpoint_key_is_rejected(self):
        with self.assertRaises(ValueError):
            selection.local_engine_directory('../escape', '4060', '10.13', self.root)

    def build_fixture(self):
        directory = self.root / 'built'
        directory.mkdir()
        rows = []
        for name in ['front.engine', 'sides.engine']:
            raw = name.encode()
            (directory / name).write_bytes(raw)
            rows.append(dict(file=name, sha256=hashlib.sha256(raw).hexdigest()))
        m = dict(self.original, gpu_name='4060', model_config='large', engines=rows)
        (directory / 'manifest.json').write_text(json.dumps(m))
        return directory

    def test_builder_validates_both_engine_hashes(self):
        directory = self.build_fixture()
        builder.validate(directory, self.sha, '4060', '10.13')
        (directory / 'sides.engine').write_bytes(b'corrupt')
        with self.assertRaisesRegex(RuntimeError, 'SHA mismatch'):
            builder.validate(directory, self.sha, '4060', '10.13')

    def test_builder_rejects_wrong_gpu(self):
        directory = self.build_fixture()
        with self.assertRaises(RuntimeError):
            builder.validate(directory, self.sha, '4060 Ti', '10.13')

    def test_builder_rejects_missing_engine(self):
        directory = self.build_fixture()
        p = directory / 'manifest.json'
        m = json.loads(p.read_text())
        m['engines'].pop()
        p.write_text(json.dumps(m))
        with self.assertRaisesRegex(RuntimeError, 'both front and side'):
            builder.validate(directory, self.sha, '4060', '10.13')


if __name__ == '__main__':
    unittest.main()
