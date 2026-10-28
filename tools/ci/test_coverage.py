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

    def test_second_workspace_build_is_rejected(self):
        root = self.copy_ci()
        path = root/'.github/workflows/planning-integration.yml'
        text = path.read_text().replace('run: python3 tools/audit/check_lidar_canonicalization.py',
                                       'run: ./scripts/build_ws.sh')
        path.write_text(text)
        self.assertTrue(any('exactly once' in e for e in audit(root)[0]))

    def test_planning_gate_cannot_succeed_after_failed_shared_build(self):
        root = self.copy_ci()
        path = root/'.github/workflows/ci.yml'
        path.write_text(path.read_text().replace('test "$BUILD_RESULT" = success', 'echo passed'))
        self.assertTrue(any('must fail unless' in e for e in audit(root)[0]))

    def test_missing_planner_runtime_regression_is_rejected(self):
        root = self.copy_ci()
        path = root/'.github/ci/planning.sh'
        path.write_text(path.read_text().replace('rostest asmc_planner highway_runtime.test', ''))
        self.assertTrue(any('missing former planning check' in e for e in audit(root)[0]))

    def test_duplicate_input_execution_is_rejected(self):
        root = self.copy_ci()
        path = root/'.github/ci/planning.sh'
        with path.open('a') as stream:
            stream.write('devel/lib/planning_inputs/planning_gps_recovery_test\n')
        self.assertTrue(any('exactly once' in e for e in audit(root)[0]))


if __name__ == '__main__':
    unittest.main()
