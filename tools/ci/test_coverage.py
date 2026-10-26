import unittest
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


if __name__ == '__main__':
    unittest.main()
