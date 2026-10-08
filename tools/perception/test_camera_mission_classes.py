from pathlib import Path
import unittest
import numpy as np
import yaml
from rescore_camera_mission_classes import filter_predictions, score_frame


class MissionClassesTest(unittest.TestCase):
    def test_unlearned_obstacle_gt_is_not_deleted_to_improve_score(self):
        truth = [dict(group='obstacle', xyxy=[0,0,10,10], source_class_name='wooden crate')]
        predictions = [dict(group='obstacle', xyxy=[0,0,10,10], source_class_name='bench', confidence=.9)]
        count, _, _, pairs = score_frame(truth, predictions, None, None, {'car'}, .7)
        self.assertEqual([], pairs)
        self.assertEqual(1, count['obstacle']['fn'])
        self.assertEqual(1, count['obstacle']['gt'])

    def test_original_mask_index_and_boundary_confidence_survive_filter(self):
        predictions = [dict(group='obstacle', xyxy=[0,0,10,10], source_class_name='bench', confidence=.9),
                       dict(group='vehicle', xyxy=[0,0,10,10], source_class_name='car', confidence=.7)]
        truth = [dict(group='vehicle', xyxy=[0,0,10,10], source_class_name='car')]
        correct = np.ones((10,10), bool)
        counts, values, _, _ = score_frame(truth, predictions, [correct],
                                           [np.zeros((10,10), bool), correct], {'car'}, .7)
        self.assertEqual(1, counts['vehicle']['tp'])
        self.assertEqual([1.], values)
        self.assertEqual([1], filter_predictions(predictions, {'car'}, .7))

    def test_profiles_keep_same_weights_heads_mask_and_source_vocabulary(self):
        root = Path(__file__).resolve().parents[2]
        base = root / 'src/asmc_camera/config/camera'
        policy = yaml.safe_load((root / 'config/experiments/camera_mission_classes_v1.yaml').read_text())
        self.assertEqual(['person','bicycle','car','motorcycle','bus','truck','dog'], policy['allowed_names'])
        coco = yaml.safe_load((base / 'yolo11m_coco_all.yaml').read_text())['allowed_names']
        self.assertEqual(policy['original_coco_ids'], [coco.index(n) for n in policy['allowed_names']])
        for model in ('yolo26s', 'yoloe26s'):
            original = yaml.safe_load((base / (model + '_seg_experiment.yaml')).read_text())
            mission = yaml.safe_load((base / (model + '_seg_mission_experiment.yaml')).read_text())
            self.assertEqual(policy['allowed_names'], mission.pop('allowed_names'))
            original.pop('allowed_names')
            self.assertEqual(original, mission)


if __name__ == '__main__':
    unittest.main()
