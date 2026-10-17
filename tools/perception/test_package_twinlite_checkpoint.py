import copy
import unittest

import torch

from tools.perception.package_twinlite_checkpoint import deployment_state


class DeploymentCheckpointTest(unittest.TestCase):
    def source(self):
        return dict(schema_version='twinlite-managed-checkpoint-1.0.0',
                    epoch=8, completed_epochs=8, epoch_complete=True,
                    args={'config': 'large'}, upstream_commit='fixed-source',
                    validation={'sample_count': 1404},
                    task_spec={'heads': ['drivable', 'road_marking'],
                               'class_counts': {'drivable': 2, 'road_marking': 4},
                               'class_names': {
                        'drivable': ['background', 'drivable'],
                        'road_marking': ['background', 'white_lane', 'yellow_lane', 'stopline']}},
                    ema_state_dict={'weight': torch.tensor([1., 2.]),
                                    'batch_counter': torch.tensor(17, dtype=torch.int64)},
                    model_state_dict={'weight': torch.tensor([9., 9.])},
                    optimizer_state_dict={'training': 'only'})

    def test_preserves_selected_ema_buffers_and_excludes_training_state(self):
        source = self.source()
        result = deployment_state(source, 'original-sha')
        self.assertTrue(torch.equal(result['ema_state_dict']['weight'], torch.tensor([1., 2.])))
        self.assertEqual(result['ema_state_dict']['batch_counter'].dtype, torch.int64)
        self.assertNotIn('model_state_dict', result)
        self.assertNotIn('optimizer_state_dict', result)
        self.assertEqual(result['deployment_source']['checkpoint_sha256'], 'original-sha')
        self.assertEqual(result['epoch'], 8)
        self.assertEqual(result['args']['config'], 'large')

    def test_rejects_partial_unvalidated_or_wrong_epoch(self):
        for patch in ({'epoch_complete': False}, {'completed_epochs': 7},
                      {'validation': None}, {'ema_state_dict': {}},
                      {'schema_version': 'unknown'}):
            with self.subTest(patch=patch), self.assertRaises(ValueError):
                deployment_state(dict(self.source(), **patch), 'sha')

    def test_rejects_changed_class_order_or_scale(self):
        source = copy.deepcopy(self.source())
        source['task_spec']['class_names']['road_marking'][1:3] = ['yellow_lane', 'white_lane']
        with self.assertRaises(ValueError):
            deployment_state(source, 'sha')
        source = self.source()
        source['args'] = {'config': 'unknown'}
        with self.assertRaises(ValueError):
            deployment_state(source, 'sha')


if __name__ == '__main__':
    unittest.main()
