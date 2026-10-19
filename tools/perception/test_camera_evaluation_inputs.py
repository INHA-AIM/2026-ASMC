import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from evaluate_camera_bbox_pilot import evaluation_inputs


class EvaluationInputsTests(unittest.TestCase):
    def dataset(self, root):
        dataset = root / 'dataset_versions' / 'fixture'
        dataset.mkdir(parents=True)
        rows = [dict(id=role, split=role, image='/old/data/' + role + '.png')
                for role in ('train', 'selection', 'heldout')]
        (dataset / 'manifest.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in rows))
        (dataset / 'temporal-probe.jsonl').write_text('{"id":"temporal"}\n')
        return dataset

    def test_checkpoint_selection_excludes_heldout_and_temporal(self):
        with tempfile.TemporaryDirectory() as tmp:
            dataset = self.dataset(Path(tmp))
            rows, temporal = evaluation_inputs(dataset, selection_only=True)
            self.assertEqual([r['id'] for r in rows], ['selection'])
            self.assertEqual(temporal, [])

    def test_final_evaluation_retains_heldout_and_optional_temporal(self):
        with tempfile.TemporaryDirectory() as tmp:
            dataset = self.dataset(Path(tmp))
            rows, temporal = evaluation_inputs(dataset)
            self.assertEqual([r['id'] for r in rows], ['selection', 'heldout'])
            self.assertEqual(temporal, [dict(id='temporal')])
            self.assertEqual(evaluation_inputs(dataset, skip_temporal=True)[1], [])

    def test_relocated_selection_does_not_resolve_unused_heldout_image(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            dataset = self.dataset(root)
            (dataset / 'pretrain.json').write_text('{}')
            image = root / 'selection.png'
            image.write_bytes(b'fixture')
            with patch('camera_pretrain_policy.validate_dataset_contract',
                       return_value=dict(original_data_root='/old/data')):
                rows, _ = evaluation_inputs(dataset, selection_only=True)
                self.assertEqual(rows[0]['image'], str(image))
                with self.assertRaisesRegex(ValueError, 'missing relocated source'):
                    evaluation_inputs(dataset)
