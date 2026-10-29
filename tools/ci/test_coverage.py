import unittest
import shutil
import tempfile
from pathlib import Path
from tools.ci.check_coverage import audit, commands


class CoverageTest(unittest.TestCase):
    def test_each_option_variant_and_fixture_remains_distinct(self):
        result = commands('''python3 check.py --hood-mask
python3 check.py --hood-mask --debug-raw
for fixture_case in lane imu; do
  rostest adapter "$fixture_case.test"
done''')
        self.assertEqual(len(result), 4)
        self.assertIn(('rostest','adapter','imu.test'), result)
        self.assertIn(('python3','check.py','--hood-mask','--debug-raw'), result)

    def test_only_environment_and_evidence_location_are_normalized(self):
        self.assertEqual(commands('PYTHONPATH=src python3 check.py --output /tmp/a'),
                         commands('ASMC_STORAGE_UNMANAGED_OUTPUT=1 python3 check.py --output "$ASMC_CI_OUTPUT/b"'))
        self.assertNotEqual(commands('python3 check.py --batched'), commands('python3 check.py'))

    def test_current_workflows_preserve_prior_checks(self):
        errors, count = audit()
        self.assertEqual(errors, [])
        self.assertGreater(count, 100)

    def copy_ci(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        root = Path(temporary.name)
        original = Path(__file__).resolve().parents[2]
        shutil.copytree(original/'.github', root/'.github')
        return root

    def test_heavy_suite_cannot_run_automatically(self):
        root = self.copy_ci()
        path = root/'.github/workflows/full-regression.yml'
        path.write_text(path.read_text().replace('  workflow_dispatch:', '  pull_request:'))
        self.assertTrue(any('manual-only' in e for e in audit(root)[0]))

    def test_essential_budget_and_parallelism_are_enforced(self):
        root = self.copy_ci()
        path = root/'.github/workflows/ci.yml'
        path.write_text(path.read_text().replace('timeout-minutes: 6', 'timeout-minutes: 8', 1))
        self.assertTrue(any('six minutes' in e for e in audit(root)[0]))
        path.write_text(path.read_text().replace('    timeout-minutes: 8',
                                                '    needs: catkin-build\n    timeout-minutes: 6', 1))
        self.assertTrue(any('parallel' in e for e in audit(root)[0]))

    def test_missing_planner_runtime_regression_is_rejected(self):
        root = self.copy_ci()
        path = root/'.github/ci/planning.sh'
        path.write_text(path.read_text().replace('rostest asmc_planner highway_runtime.test', ''))
        self.assertTrue(any('missing manual planning regression' in e for e in audit(root)[0]))

    def test_critical_production_build_cannot_be_dropped(self):
        import json
        root = self.copy_ci()
        path = root/'.github/ci/essential.json'
        spec = json.loads(path.read_text())
        spec['jobs']['perception']['targets'].remove('lidar_pipeline_node')
        path.write_text(json.dumps(spec))
        self.assertTrue(any('targets missing' in e for e in audit(root)[0]))

    def test_handoff_fixture_must_be_built_before_direct_rostest(self):
        import json
        root = self.copy_ci()
        path = root/'.github/ci/essential.json'
        spec = json.loads(path.read_text())
        spec['jobs']['planning']['targets'].remove('gps_mode_source_test_node')
        path.write_text(json.dumps(spec))
        self.assertTrue(any('targets missing' in e for e in audit(root)[0]))

    def test_full_tensor_runtime_cannot_return_to_fast_ci(self):
        root = self.copy_ci()
        path = root/'.github/workflows/ci.yml'
        with path.open('a') as stream:
            stream.write('\n# cpu-requirements.txt\n')
        self.assertTrue(any('slow full suite' in e for e in audit(root)[0]))


if __name__ == '__main__':
    unittest.main()
