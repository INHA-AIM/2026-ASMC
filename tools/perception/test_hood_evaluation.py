import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from evaluate_hood_mask import matching


class HoodEvaluationTest(unittest.TestCase):
    def test_one_box_cannot_cover_two_ground_truth_objects(self):
        gt=[dict(label='person',xyxy=[0,0,10,10])]*2
        self.assertEqual(1,len(matching(gt,[dict(source_class_name='person',xyxy=[0,0,10,10])])))

    def test_labels_and_iou_boundary_are_required(self):
        gt=[dict(label='person',xyxy=[0,0,10,10])]
        self.assertEqual([],matching(gt,[dict(source_class_name='car',xyxy=[0,0,10,10])]))
        self.assertEqual([0],matching(gt,[dict(source_class_name='person',xyxy=[0,0,10,5])]))
        self.assertEqual([],matching(gt,[dict(source_class_name='person',xyxy=[0,0,10,4.9])]))

    def test_vehicle_group_matches_original_label_without_rewriting_it(self):
        detection=dict(source_class_name='truck',xyxy=[0,0,10,10])
        self.assertEqual([0],matching([dict(label='vehicle',xyxy=[0,0,10,10])],[detection]))
        self.assertEqual('truck',detection['source_class_name'])

    def test_empty_ground_truth_or_detection_is_not_a_match(self):
        self.assertEqual([],matching([],[]))
        self.assertEqual([],matching([dict(label='person',xyxy=[0,0,10,10])],[]))
