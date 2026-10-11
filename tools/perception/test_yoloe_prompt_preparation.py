import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from prepare_yoloe_l import export_profile, profile_names


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

    def test_x_teacher_rejects_export_before_creating_files_or_loading_gpu(self):
        script = Path(__file__).with_name('prepare_yoloe_l.py')
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'prepared'
            for stage in ('onnx', 'engine'):
                result = subprocess.run(
                    [sys.executable, str(script), '--directory', str(output),
                     '--config', str(Path(directory) / 'absent.yaml'),
                     '--profile', 'candidate', '--stage', stage, '--scale', 'x',
                     '--front-width', '640', '--variant'],
                    capture_output=True, text=True,
                )
                self.assertNotEqual(result.returncode, 0)
                self.assertIn('X teacher is PyTorch only', result.stderr)
                self.assertFalse(output.exists())

    def test_profiles_cover_front_and_side_shapes_after_integration(self):
        for width, height in ((640, 384), (960, 544), (1280, 736)):
            front, minimum, optimum, maximum = export_profile(width)
            self.assertEqual(front, (height, width))
            for shape in ((1, 3, height, width), (2, 3, 480, 640)):
                self.assertTrue(all(lo <= value <= hi for lo, value, hi
                                    in zip(minimum, shape, maximum)))
            self.assertEqual(optimum, (2, 3, 480, 640))

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
