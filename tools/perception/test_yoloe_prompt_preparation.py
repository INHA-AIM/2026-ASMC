import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from prepare_yoloe_l import profile_names


class PromptPreparationTest(unittest.TestCase):
    def test_invalid_profile_is_rejected_before_output_or_cuda(self):
        script = Path(__file__).with_name('prepare_yoloe_l.py')
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            policy = root / 'policy.json'
            policy.write_text(json.dumps({'prompt_profiles': {'candidate': ['person']}}))
            for profile in ['../escaped', 'candidate/child', 'missing']:
                output = root / 'prepared'
                result = subprocess.run(
                    [sys.executable, str(script), '--directory', str(output),
                     '--config', str(policy), '--profile', profile, '--stage', 'engine'],
                    capture_output=True, text=True,
                )
                self.assertNotEqual(result.returncode, 0)
                self.assertFalse(output.exists())
                self.assertNotIn('ModuleNotFoundError', result.stderr)
            self.assertFalse((root / 'escaped').exists())

    def test_duplicate_or_malformed_names_fail_instead_of_changing_class_ids(self):
        for names in [[], ['car', 'car'], ['person', ''], ['person', ' dog'], ['car', 1]]:
            with self.assertRaises(ValueError):
                profile_names({'prompt_profiles': {'candidate': names}}, 'candidate')

    def test_valid_vocabulary_retains_explicit_class_id_order(self):
        names = ['wooden box', 'person', 'car']
        self.assertEqual(profile_names({'prompt_profiles': {'obstacle17': names}},
                                      'obstacle17'), names)


if __name__ == '__main__':
    unittest.main()
