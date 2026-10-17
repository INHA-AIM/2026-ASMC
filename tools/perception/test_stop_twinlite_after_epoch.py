import importlib.util
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location('epoch_guard', Path(__file__).with_name('stop_twinlite_after_epoch.py'))
guard = importlib.util.module_from_spec(spec)
spec.loader.exec_module(guard)


class EpochGuardTest(unittest.TestCase):
    def test_only_atomic_target_checkpoint_requests_stop(self):
        with tempfile.TemporaryDirectory() as root:
            path = Path(root)
            (path / 'epoch007.pt').write_bytes(b'complete previous epoch')
            pending = path / 'epoch008.pt.part'
            pending.write_bytes(b'target checkpoint')
            self.assertFalse(guard.request_if_ready(path, 8, {'run_id': 'run'}, 'run'))
            self.assertFalse((path / 'STOP').exists())
            pending.replace(path / 'epoch008.pt')
            self.assertTrue(guard.request_if_ready(path, 8, {'run_id': 'run'}, 'run'))
            self.assertTrue((path / 'STOP').exists())

    def test_wrong_run_cannot_request_stop(self):
        with tempfile.TemporaryDirectory() as root:
            path = Path(root)
            (path / 'epoch008.pt').write_bytes(b'checkpoint')
            with self.assertRaises(ValueError):
                guard.request_if_ready(path, 8, {'run_id': 'other'}, 'run')
            self.assertFalse((path / 'STOP').exists())

    def test_exact_boundary_and_late_updates_are_distinguished(self):
        history = [{'epoch': 8, 'optimizer_steps': 100}]
        result = dict(completed_epochs=8, optimizer_steps=100, state='stopped')
        self.assertEqual(guard.check_result(result, history, 8)['state'], 'stopped_at_target')
        result['optimizer_steps'] += 1
        self.assertEqual(guard.check_result(result, history, 8)['state'], 'needs_review')
        self.assertEqual(guard.check_result(result, [], 8)['state'], 'ended_before_target')


if __name__ == '__main__':
    unittest.main()
