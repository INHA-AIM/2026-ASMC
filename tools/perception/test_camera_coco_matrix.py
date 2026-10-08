import json
from pathlib import Path
import tempfile
import unittest

from analyze_camera_coco_matrix import analyze


class MatrixTest(unittest.TestCase):
    def run_fixture(self, duplicate=False, missing=False, failed=False):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            job = dict(phase='camera', bag_id='camera', classes='all', repeat=1, debug=False)
            (root/'protocol.json').write_text(json.dumps([job]))
            entry = dict(label='trial', job=job, return_code=1 if failed else 0)
            (root/'index.json').write_text(json.dumps([entry]* (2 if duplicate else 1)))
            if not missing:
                (root/'trial').mkdir()
                (root/'trial/report.json').write_text(json.dumps(dict(passed=not failed)))
            return analyze(root)

    def test_missing_report_is_not_complete(self):
        self.assertFalse(self.run_fixture(missing=True)['protocol_complete'])

    def test_duplicate_cannot_replace_missing_repeat(self):
        self.assertFalse(self.run_fixture(duplicate=True)['protocol_complete'])

    def test_performance_failure_is_preserved_in_complete_protocol(self):
        result = self.run_fixture(failed=True)
        self.assertTrue(result['protocol_complete'])
        self.assertEqual(result['performance_failed_trials'], ['trial'])
        self.assertFalse(result['all_classes_default_promoted'])


if __name__ == '__main__':
    unittest.main()
