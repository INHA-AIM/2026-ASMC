"""Evidence ownership, no-overwrite and missing-data reporting contracts."""
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from scen_runner.records.sensor_recording import SensorRecording, recording_directory, REQUIRED_TOPICS


class RecordingTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)/'trial-01'
        self.root.mkdir()
        self.registration = dict(run_id='trial-01', status='active', kind='experiment', output='artifacts/trial-01')
        (self.root/'registration.json').write_text(json.dumps(self.registration))
        self.spec = dict(run_id='trial-01', run_root=str(self.root))
        self.path = self.root/'recording.json'
        self.path.write_text(json.dumps(self.spec))

    def recorder(self):
        return SensorRecording(self.path, notify=lambda _: None)

    def test_closed_or_mismatched_run_cannot_record(self):
        self.assertEqual(recording_directory(self.spec), self.root/'ros')
        self.registration['status'] = 'closed'
        (self.root/'registration.json').write_text(json.dumps(self.registration))
        with self.assertRaisesRegex(ValueError, 'active experiment'):
            self.recorder()

    def test_constructor_and_unstarted_close_do_not_create_output(self):
        recorder = self.recorder()
        recorder.close()
        self.assertFalse((self.root/'ros').exists())

    def test_existing_recording_is_never_overwritten(self):
        output = self.root/'ros'
        output.mkdir()
        sentinel = output/'sensors.bag'
        sentinel.write_bytes(b'preserved')
        recorder = self.recorder()
        with self.assertRaises(FileExistsError):
            recorder.start()
        recorder.close()
        self.assertEqual(sentinel.read_bytes(), b'preserved')
        self.assertFalse((output/'manifest.json').exists())

    def test_start_failure_preserves_incomplete_manifest(self):
        recorder = self.recorder()
        with patch('scen_runner.records.sensor_recording.subprocess.Popen', side_effect=FileNotFoundError('rosbag')):
            with self.assertRaises(FileNotFoundError):
                recorder.start()
        result = json.loads((self.root/'ros/manifest.json').read_text())
        self.assertEqual(result['status'], 'incomplete')
        self.assertEqual(result['missing_required_topics'], list(REQUIRED_TOPICS))
        self.assertIn('recorder_not_started', result['errors'])

    def test_partial_bag_and_overflow_are_never_success(self):
        recorder = self.recorder()
        recorder.directory.mkdir()
        recorder.started = True
        (recorder.directory/'sensors.bag.active').write_bytes(b'partial evidence')
        (recorder.directory/'rosbag.log').write_text('WARN buffer exceeded')
        result = recorder.close()
        self.assertEqual(result['status'], 'incomplete')
        self.assertIn('unclosed_bag', result['errors'])
        self.assertIn('rosbag_buffer_overflow', result['errors'])
        self.assertTrue((recorder.directory/'sensors.bag.active').exists())
        self.assertEqual(recorder.close(), result)


if __name__ == '__main__':
    unittest.main()
