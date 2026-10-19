"""Native/cache path preflight is read-only and has no personal drive defaults."""
import contextlib
import importlib.util
import io
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('verify_data_locations', ROOT/'scripts/verify_data_locations.py')
preflight = importlib.util.module_from_spec(spec)
spec.loader.exec_module(preflight)


class DataLocationPreflightTest(unittest.TestCase):
    def test_yolo_paths_labels_and_broken_links_are_checked_without_changes(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); dataset = root/'dataset'
            image = dataset/'images/train/view.png'; image.parent.mkdir(parents=True)
            image.write_bytes(b'RGB')
            label = dataset/'labels/train/view.txt'; label.parent.mkdir(parents=True)
            label.write_text('')
            result = preflight.check_dataset(dataset, root)
            self.assertEqual(result['missing_count'], 0)
            self.assertEqual(result['checked_splits'], ['train'])
            label.unlink()
            image.unlink(); image.symlink_to(root/'unavailable.png')
            result = preflight.check_dataset(dataset, root)
            self.assertEqual(result['missing_count'], 2)
            self.assertFalse((dataset/'images/val').exists())

    def test_jsonl_native_and_cached_targets_keep_source_and_sha(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); dataset = root/'version'; dataset.mkdir()
            for name in ('rgb.png', 'target.png', 'valid.png', 'aux.png'):
                (root/name).write_bytes(b'asset')
            row = dict(image='rgb.png', targets={'lane': 'target.png'},
                       valid_masks={'lane': 'valid.png'}, auxiliary_targets={'aux': 'aux.png'})
            manifest = dataset/'train.jsonl'; manifest.write_text(json.dumps(row)+'\n')
            before = manifest.read_bytes()
            result = preflight.check_dataset(dataset, root)
            self.assertEqual(result['samples']['train'], 1)
            self.assertEqual(result['missing_count'], 0)
            self.assertEqual(manifest.read_bytes(), before)
            self.assertEqual(result['manifest_sha256']['train.jsonl'], preflight.hashlib.sha256(before).hexdigest())
            (root/'target.png').unlink()
            self.assertEqual(preflight.check_dataset(dataset, root)['missing_count'], 1)

    def test_empty_missing_and_malformed_datasets_fail_instead_of_zero_sample_success(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.assertTrue(preflight.check_dataset(root/'missing', root)['error_count'])
            dataset = root/'version'; dataset.mkdir()
            (dataset/'train.jsonl').write_text('{bad json}\n{}\n')
            self.assertGreaterEqual(preflight.check_dataset(dataset, root)['error_count'], 2)

    def test_cli_optional_roots_do_not_become_cwd_or_required_drives(self):
        with tempfile.TemporaryDirectory() as tmp, patch.dict(os.environ, {}, clear=True):
            root = Path(tmp); dataset = root/'version'; dataset.mkdir()
            (root/'rgb.png').write_bytes(b'rgb')
            (dataset/'train.jsonl').write_text(json.dumps({'image': 'rgb.png'})+'\n')
            output = io.StringIO()
            with patch.object(preflight, 'default_data_root', return_value=str(root)), contextlib.redirect_stdout(output):
                self.assertEqual(preflight.main(['--dataset', str(dataset)]), 0)
            result = json.loads(output.getvalue())
            self.assertFalse(result['roots_available']['capture_archive_wsl'])
            self.assertFalse(result['roots_available']['aim_directory_wsl'])
            self.assertEqual(set(root.iterdir()), {root/'rgb.png', dataset})


if __name__ == '__main__':
    unittest.main()
