import tempfile
from pathlib import Path
import unittest
import numpy as np

from asmc_camera.detection.classes import load_class_groups
from asmc_camera.detection.segmentation import load_prompt_embeddings, select_result


class Box:
    def __init__(self, cls, score):
        self.cls, self.conf, self.xyxy = [cls], [score], [[1.2, 2.3, 4.5, 6.7]]


class SegmentationTest(unittest.TestCase):
    def test_optional_animals_do_not_change_old_mapping(self):
        old = load_class_groups(dict(groups=dict(person=['bicycle'], vehicle=['car'])))
        self.assertEqual(old.group_of('bicycle'), 'person')
        self.assertEqual(old.group_of('dog'), 'obstacle')
        new = load_class_groups(dict(groups=dict(person=['person'], animal=['dog'], vehicle=['bicycle'])))
        self.assertEqual(new.group_of('DOG'), 'animal')
        self.assertEqual(new.group_of('bicycle'), 'vehicle')
        with self.assertRaises(ValueError):
            load_class_groups(dict(groups=dict(person=['dog'], animal=['dog'], vehicle=[])))

    def test_box_and_mask_filtering_retains_correspondence(self):
        result = type('Result', (), dict(boxes=[Box(0,.69), Box(1,.7), Box(0,.9)]))()
        masks = np.array([[[0]], [[1]], [[0]]], dtype=bool)
        boxes, kept = select_result(result, {0:'car',1:'dog'}, ['car','dog'], .7, masks)
        self.assertEqual([b.source_class_name for b in boxes], ['dog','car'])
        self.assertEqual([m.item() for m in kept], [True,False])
        self.assertEqual(boxes[0].xmin, 1.2)
        with self.assertRaises(ValueError):
            select_result(result, {0:'car',1:'dog'}, ['car','dog'], .7, masks[:2])

    def test_empty_and_unselected_results(self):
        result = type('Result', (), dict(boxes=[]))()
        self.assertEqual(select_result(result, {}, [], .7), ([], []))

    def test_offline_embeddings_are_bound_to_checkpoint(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'prompt.npz'
            np.savez(path, names=np.array(['car']), embeddings=np.zeros((1,1,512)), weights_sha256=np.array('abc'))
            self.assertEqual(load_prompt_embeddings(path, 'abc')[0], ['car'])
            with self.assertRaises(ValueError):
                load_prompt_embeddings(path, 'different')
            with self.assertRaises(ValueError):
                load_prompt_embeddings('', 'abc')


if __name__ == '__main__':
    unittest.main()
