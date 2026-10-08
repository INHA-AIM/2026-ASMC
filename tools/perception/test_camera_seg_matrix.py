import json
from pathlib import Path
import tempfile
import unittest
from analyze_camera_seg_matrix import analyze
from seg_measurements import make_protocol


class MatrixTest(unittest.TestCase):
    def fixture(self, duplicate=False, missing=False, failed=False, smoke=False, partial=False):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            jobs = make_protocol({'repeats': 3}, ('camera', 'debug', 'fusion'))
            if partial:
                jobs = [j for j in jobs if j['phase'] == 'camera']
            entries = []
            for n, job in enumerate(jobs):
                entries.append(dict(label=str(n), job=job))
                if missing and n == len(jobs)-1:
                    continue
                folder = root / str(n)
                folder.mkdir()
                (folder / 'report.json').write_text(json.dumps(dict(
                    passed=not failed,
                    fusion=dict(totals=dict(matched=10)),
                    measurements={'yolo': {'input_to_output_wall_ms': {'p95': 30}}})))
                (folder / 'observations.json').write_text(json.dumps(dict(tracks=[])))
            (root / 'protocol.json').write_text(json.dumps(jobs))
            (root / 'index.json').write_text(json.dumps(entries+[entries[-1]] if duplicate else entries))
            (root / 'manifest.json').write_text(json.dumps(dict(smoke_only=smoke)))
            accuracy = root / 'accuracy'
            (accuracy / 'yolo26').mkdir(parents=True)
            (accuracy / 'yolo26/summary.json').write_text(json.dumps(dict(selected_threshold=dict(
                confidence=.7, metrics=dict(macro_f1=.8, mask_iou=.7)))))
            return analyze(root, accuracy)

    def test_failure_preserved_without_selection(self):
        result = self.fixture(failed=True)
        self.assertTrue(result['protocol_complete'])
        self.assertEqual(54, len(result['failed_trials']))
        self.assertIsNone(result['training_candidate'])

    def test_missing_duplicate_smoke_or_partial_cannot_select(self):
        for kwargs in (dict(missing=True), dict(duplicate=True), dict(smoke=True), dict(partial=True)):
            result = self.fixture(**kwargs)
            self.assertFalse(result['protocol_complete'])
            self.assertIsNone(result['training_candidate'])

    def test_front_success_selects_training_only(self):
        result = self.fixture()
        self.assertTrue(result['protocol_complete'])
        self.assertEqual('yolo26', result['training_candidate']['model'])
        self.assertFalse(result['training_performed'])
        self.assertFalse(result['operational_defaults_changed'])


if __name__ == '__main__':
    unittest.main()
